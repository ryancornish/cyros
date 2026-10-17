#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
# Copyright (C) 2026 Ryan Cornish
"""Read a cyros trace stream, and turn it into a Perfetto trace.

    cyros_trace.py dump a.cytrace                    every record in time order, one a line
    cyros_trace.py perfetto a.cytrace [-o a.pftrace]  for ui.perfetto.dev or trace_processor_shell

A stream is what cyros::trace::stream_writer writes, and cyros/trace/stream.hpp
is where its format is specified. tests/unit/common/trace_file.hpp writes one
from a host test or program.

What the Perfetto trace shows:

- one CPU track per core, with the thread each core ran, from sched_switch
  events, so Perfetto's scheduling views work: run, runnable and sleeping
  spans per thread, and who woke whom
- a process per core holding the threads pinned to it, since a cyros thread
  never migrates. Every core's idle thread is cyros id 0, which Perfetto
  already treats as the idle task
- on each thread, a "boosted" slice wherever it ran more urgent than its
  base priority, which is priority inheritance (or a ceiling) at work. Each
  switch-in's urgency is also the sched slice's priority. Lower is more
  urgent, as in cyros
- a track per mutex, a slice per hold named for the holder, and on each
  thread a "wait" slice from blocking on a mutex to being handed it
- trace::user() records as instants on the thread's track
- "records lost" instants wherever the kernel dropped records

The protobuf is written by hand, so this needs nothing but Python. The field
numbers are Perfetto's own, from its protos at v58.2.
"""

import argparse
import struct
import sys
from dataclasses import dataclass, field

# ---------------------------------------------------------------------------
# The stream

EVENTS = [
    "thread_created", "thread_ready", "thread_running", "thread_blocked",
    "thread_terminated", "remote_wake", "mutex_acquired", "mutex_released", "user",
]

PACKET_HEADER, PACKET_RECORDS, PACKET_DROPPED = 1, 2, 3
PACKET_THREAD_NAME, PACKET_OBJECT_NAME, PACKET_TAG_NAME = 4, 5, 6

RECORD = struct.Struct("<QQIBBBB")
assert RECORD.size == 24


@dataclass
class Record:
    timestamp: int
    object: int
    thread: int
    event: str
    core: int        # the thread's core
    priority: int    # base priority, urgency for thread_running, the tag for user
    recorded_on: int  # the core whose buffer it came from
    index: int = 0    # its place in the stream


@dataclass
class Drop:
    core: int
    total: int        # trace::dropped(core) when reported
    after: int        # how many records the stream held when it was reported


@dataclass
class Stream:
    version: int = 0
    cores: int = 0
    timestamp_hz: int = 0
    records: list = field(default_factory=list)   # stream order
    drops: list = field(default_factory=list)
    thread_names: dict = field(default_factory=dict)
    object_names: dict = field(default_factory=dict)
    tag_names: dict = field(default_factory=dict)
    skipped_kinds: dict = field(default_factory=dict)
    truncated: bool = False


class FormatError(Exception):
    pass


def decode(data):
    """Every packet of a stream, as stream.hpp lays them out."""
    stream = Stream()
    at = 0
    first = True
    while at < len(data):
        if len(data) - at < 3:
            stream.truncated = True
            break
        kind = data[at]
        (length,) = struct.unpack_from("<H", data, at + 1)
        if len(data) - at - 3 < length:
            stream.truncated = True
            break
        payload = data[at + 3:at + 3 + length]
        at += 3 + length

        if first:
            if kind != PACKET_HEADER or payload[:4] != b"CYTR":
                raise FormatError("not a cyros trace stream: it does not begin with a header")
            first = False

        if kind == PACKET_HEADER:
            stream.version, stream.cores, stream.timestamp_hz = struct.unpack_from("<HHQ", payload, 4)
            if stream.version != 1:
                raise FormatError(f"stream version {stream.version}, and this reads version 1")
        elif kind == PACKET_RECORDS:
            on = payload[0]
            if (length - 1) % RECORD.size:
                raise FormatError("a records packet that does not hold whole records")
            for offset in range(1, length, RECORD.size):
                ts, obj, thread, event, core, priority, _ = RECORD.unpack_from(payload, offset)
                name = EVENTS[event] if event < len(EVENTS) else f"event_{event}"
                stream.records.append(Record(ts, obj, thread, name, core, priority, on, len(stream.records)))
        elif kind == PACKET_DROPPED:
            core, total = struct.unpack_from("<BI", payload)
            stream.drops.append(Drop(core, total, len(stream.records)))
        elif kind == PACKET_THREAD_NAME:
            (thread,) = struct.unpack_from("<I", payload)
            stream.thread_names[thread] = payload[4:].decode("utf-8", "replace")
        elif kind == PACKET_OBJECT_NAME:
            (obj,) = struct.unpack_from("<Q", payload)
            stream.object_names[obj] = payload[8:].decode("utf-8", "replace")
        elif kind == PACKET_TAG_NAME:
            stream.tag_names[payload[0]] = payload[1:].decode("utf-8", "replace")
        else:
            stream.skipped_kinds[kind] = stream.skipped_kinds.get(kind, 0) + 1
    if first:
        raise FormatError("empty stream")
    return stream


class Names:
    def __init__(self, stream):
        self.stream = stream

    def thread(self, tid, core=None):
        if tid == 0:
            return "idle" if core is None else f"idle {core}"
        return self.stream.thread_names.get(tid, f"thread {tid}")

    def object(self, obj):
        return self.stream.object_names.get(obj, f"0x{obj:x}")

    def tag(self, tag):
        return self.stream.tag_names.get(tag, f"user {tag}")


def in_time_order(stream):
    """Every core's records merged by timestamp. A core's own records are
    already in order, and a stable sort keeps them so where stamps tie."""
    return sorted(stream.records, key=lambda r: r.timestamp)


class Clock:
    """Record timestamps to nanoseconds from the first record."""

    def __init__(self, stream, hz_override):
        self.hz = hz_override or stream.timestamp_hz
        if not self.hz:
            warn("the stream does not say what a second is in its timestamps, so they are taken as "
                 "nanoseconds. Pass --hz to say")
            self.hz = 1_000_000_000
        self.origin = min((r.timestamp for r in stream.records), default=0)
        if stream.records and all(r.timestamp == self.origin for r in stream.records):
            warn("every record has the same timestamp (QEMU's Arm stamps are all zero), so time shows nothing")

    def ns(self, timestamp):
        return (timestamp - self.origin) * 1_000_000_000 // self.hz


def warn(text):
    print(f"cyros_trace: {text}", file=sys.stderr)


def report_losses(stream):
    if stream.truncated:
        warn("the stream ends partway through a packet, which was ignored")
    for kind, count in sorted(stream.skipped_kinds.items()):
        warn(f"skipped {count} packet(s) of unknown kind {kind}")
    for core in sorted({d.core for d in stream.drops}):
        total = max(d.total for d in stream.drops if d.core == core)
        warn(f"core {core} dropped {total} record(s): the drain did not keep up, so its timeline has gaps")


# ---------------------------------------------------------------------------
# dump

def dump(stream, clock, out):
    names = Names(stream)
    drops = {}
    for d in stream.drops:
        drops.setdefault(d.after, []).append(d)

    base = {}
    for r in stream.records:
        if r.event == "thread_created":
            base[r.thread] = r.priority

    print(f"cyros trace, version {stream.version}, {stream.cores} core(s), "
          f"{clock.hz} ticks a second, {len(stream.records)} records", file=out)

    order = sorted(range(len(stream.records)), key=lambda i: stream.records[i].timestamp)
    for i in order:
        r = stream.records[i]
        who = names.thread(r.thread, r.core)
        what = {
            "thread_created": f"created on core {r.core}, priority {r.priority}",
            "thread_ready": "ready",
            "thread_running": f"running at urgency {r.priority}"
                              + (f" (boosted from {base[r.thread]})"
                                 if r.thread in base and r.priority < base[r.thread] else ""),
            "thread_blocked": "blocked" + (f" on {names.object(r.object)}" if r.object else ""),
            "thread_terminated": "terminated",
            "remote_wake": f"woken from core {r.recorded_on}",
            "mutex_acquired": f"owns {names.object(r.object)}",
            "mutex_released": f"releases {names.object(r.object)}",
            "user": f"{names.tag(r.priority)} {r.object} (0x{r.object:x})",
        }.get(r.event, r.event)
        print(f"{clock.ns(r.timestamp) / 1e6:14.6f} ms  [{r.recorded_on}]  {who:<20} {what}", file=out)
        for d in drops.get(i + 1, ()):
            print(f"{'':17}  [{d.core}]  ---- core {d.core} had dropped {d.total} record(s) in all by here", file=out)


# ---------------------------------------------------------------------------
# Protobuf, by hand

def _varint(n):
    if n < 0:
        n += 1 << 64
    out = bytearray()
    while True:
        low = n & 0x7F
        n >>= 7
        if n:
            out.append(low | 0x80)
        else:
            out.append(low)
            return bytes(out)


def pb_int(number, value):
    return _varint(number << 3) + _varint(value)


def pb_bytes(number, value):
    return _varint(number << 3 | 2) + _varint(len(value)) + value


def pb_str(number, value):
    return pb_bytes(number, value.encode())


def pb_message(number, *fields):
    return pb_bytes(number, b"".join(fields))


# Field numbers, from perfetto/protos at v58.2
class F:
    trace_packet = 1                    # Trace

    packet_ftrace_events = 1            # TracePacket
    packet_process_tree = 2
    packet_timestamp = 8
    packet_sequence_id = 10
    packet_track_event = 11
    packet_sequence_flags = 13
    packet_track_descriptor = 60

    bundle_cpu = 1                      # FtraceEventBundle
    bundle_event = 2

    ftrace_timestamp = 1                # FtraceEvent
    ftrace_pid = 2
    ftrace_sched_switch = 4
    ftrace_sched_waking = 20

    switch_prev_comm, switch_prev_pid, switch_prev_prio, switch_prev_state = 1, 2, 3, 4
    switch_next_comm, switch_next_pid, switch_next_prio = 5, 6, 7

    waking_comm, waking_pid, waking_prio, waking_success, waking_target_cpu = 1, 2, 3, 4, 5

    tree_processes, tree_threads = 1, 2            # ProcessTree
    process_pid = 1                                # ProcessTree.Process
    thread_tid, thread_name, thread_tgid = 1, 2, 3  # ProcessTree.Thread

    track_uuid, track_name, track_process, track_thread = 1, 2, 3, 4  # TrackDescriptor
    process_descriptor_pid, process_descriptor_name = 1, 6            # ProcessDescriptor
    thread_descriptor_pid, thread_descriptor_tid, thread_descriptor_name = 1, 2, 5  # ThreadDescriptor

    event_annotations, event_type, event_track, event_name = 4, 9, 11, 23  # TrackEvent
    annotation_uint, annotation_name = 3, 10       # DebugAnnotation


SLICE_BEGIN, SLICE_END, INSTANT = 1, 2, 3
SEQ_INCREMENTAL_STATE_CLEARED = 1

# sched_switch's prev_state, as Perfetto reads it with no kernel version given
STATE_RUNNABLE, STATE_SLEEPING, STATE_DEAD = 0, 1, 16

PROCESS_BASE = 100_000   # a core's process id is this plus the core
SEQUENCE = 1


class Perfetto:
    """Builds the packets. Times are nanoseconds from the first record."""

    def __init__(self, names, offset_ns):
        self.names = names
        self.offset = offset_ns
        self.packets = []
        self.cpu_events = {}          # cpu -> [(ns, encoded FtraceEvent)]
        self.thread_tracks = {}       # tid -> uuid
        self.object_tracks = {}       # object -> uuid
        self.next_uuid = 1
        self.first_on_sequence = True

    def _uuid(self):
        self.next_uuid += 1
        return self.next_uuid

    def _packet(self, ns, *fields):
        extra = []
        if self.first_on_sequence:
            extra.append(pb_int(F.packet_sequence_flags, SEQ_INCREMENTAL_STATE_CLEARED))
            self.first_on_sequence = False
        self.packets.append(b"".join((pb_int(F.packet_timestamp, ns + self.offset),
                                      pb_int(F.packet_sequence_id, SEQUENCE), *extra, *fields)))

    # ftrace

    def _ftrace(self, cpu, ns, pid, *fields):
        event = b"".join((pb_int(F.ftrace_timestamp, ns + self.offset), pb_int(F.ftrace_pid, pid), *fields))
        self.cpu_events.setdefault(cpu, []).append((ns, event))

    def switch(self, cpu, ns, prev, prev_prio, prev_state, nxt, next_prio):
        self._ftrace(cpu, ns, prev, pb_message(
            F.ftrace_sched_switch,
            pb_str(F.switch_prev_comm, self.names.thread(prev)),
            pb_int(F.switch_prev_pid, prev),
            pb_int(F.switch_prev_prio, prev_prio),
            pb_int(F.switch_prev_state, prev_state),
            pb_str(F.switch_next_comm, self.names.thread(nxt)),
            pb_int(F.switch_next_pid, nxt),
            pb_int(F.switch_next_prio, next_prio)))

    def waking(self, cpu, ns, waker, wakee, prio, target_cpu):
        self._ftrace(cpu, ns, waker, pb_message(
            F.ftrace_sched_waking,
            pb_str(F.waking_comm, self.names.thread(wakee)),
            pb_int(F.waking_pid, wakee),
            pb_int(F.waking_prio, prio),
            pb_int(F.waking_success, 1),
            pb_int(F.waking_target_cpu, target_cpu)))

    # process tree

    def process_tree(self, ns, thread_cores):
        """A process per core, holding the threads pinned to it. The tree says
        which thread is in which process. A descriptor per process names it,
        because a tree names a process only by its command line."""
        cores = sorted(set(thread_cores.values()))
        for core in cores:
            self._packet(ns, pb_message(
                F.packet_track_descriptor,
                pb_int(F.track_uuid, self._uuid()),
                pb_message(F.track_process,
                           pb_int(F.process_descriptor_pid, PROCESS_BASE + core),
                           pb_str(F.process_descriptor_name, f"cyros core {core}"))))
        processes = [pb_message(F.tree_processes, pb_int(F.process_pid, PROCESS_BASE + core))
                     for core in cores]
        threads = [pb_message(F.tree_threads,
                              pb_int(F.thread_tid, tid),
                              pb_str(F.thread_name, self.names.thread(tid)),
                              pb_int(F.thread_tgid, PROCESS_BASE + core))
                   for tid, core in sorted(thread_cores.items())]
        self._packet(ns, pb_message(F.packet_process_tree, *processes, *threads))

    # tracks

    def thread_track(self, ns, tid, core):
        if tid not in self.thread_tracks:
            uuid = self._uuid()
            self.thread_tracks[tid] = uuid
            self._packet(ns, pb_message(
                F.packet_track_descriptor,
                pb_int(F.track_uuid, uuid),
                pb_message(F.track_thread,
                           pb_int(F.thread_descriptor_pid, PROCESS_BASE + core),
                           pb_int(F.thread_descriptor_tid, tid),
                           pb_str(F.thread_descriptor_name, self.names.thread(tid)))))
        return self.thread_tracks[tid]

    def object_track(self, ns, key, name):
        if key not in self.object_tracks:
            uuid = self._uuid()
            self.object_tracks[key] = uuid
            self._packet(ns, pb_message(
                F.packet_track_descriptor,
                pb_int(F.track_uuid, uuid),
                pb_str(F.track_name, name)))
        return self.object_tracks[key]

    def track_event(self, ns, track, kind, name=None, annotations=()):
        fields = [pb_int(F.event_type, kind), pb_int(F.event_track, track)]
        if name is not None:
            fields.append(pb_str(F.event_name, name))
        for key, value in annotations:
            fields.append(pb_message(F.event_annotations,
                                     pb_str(F.annotation_name, key),
                                     pb_int(F.annotation_uint, value)))
        self._packet(ns, pb_message(F.packet_track_event, *fields))

    def serialise(self):
        out = [pb_bytes(F.trace_packet, p) for p in self.packets]
        for cpu in sorted(self.cpu_events):
            events = sorted(self.cpu_events[cpu], key=lambda e: e[0])
            for start in range(0, len(events), 1000):
                bundle = pb_message(F.packet_ftrace_events,
                                    pb_int(F.bundle_cpu, cpu),
                                    *(pb_bytes(F.bundle_event, e) for _, e in events[start:start + 1000]))
                out.append(pb_bytes(F.trace_packet, bundle))
        return b"".join(out)


def to_perfetto(stream, clock):
    names = Names(stream)
    out = Perfetto(names, offset_ns=1_000)

    thread_cores = {r.thread: r.core for r in stream.records if r.thread != 0}
    out.process_tree(0, thread_cores)

    running = {}          # core -> the thread on it, 0 for idle
    state = {}            # thread -> created, ready, running, blocked, terminated
    woken = set()         # threads whose wake is already shown and that have not run since
    base = {}             # thread -> base priority, which never changes
    urgency = {}          # thread -> its urgency when it last ran, its base priority before
    boosted = {}          # thread -> the urgency of its open boosted slice
    waiting = {}          # thread -> the object of an open wait slice
    holder = {}           # mutex -> the thread whose hold slice is open
    last_on_core = {}     # recording core -> ns of its latest record, for placing losses

    prev_state = {"ready": STATE_RUNNABLE, "running": STATE_RUNNABLE,
                  "blocked": STATE_SLEEPING, "terminated": STATE_DEAD}

    def end_wait(ns, tid):
        if tid in waiting:
            del waiting[tid]
            out.track_event(ns, out.thread_track(ns, tid, thread_cores[tid]), SLICE_END)

    def show_boost(ns, tid, value):
        """value is the urgency it runs at now, or None once it stops running."""
        want = value if value is not None and value < base.get(tid, value) else None
        if boosted.get(tid) == want:
            return
        track = out.thread_track(ns, tid, thread_cores[tid])
        if tid in boosted:
            del boosted[tid]
            out.track_event(ns, track, SLICE_END)
        if want is not None:
            boosted[tid] = want
            out.track_event(ns, track, SLICE_BEGIN, name=f"boosted {base[tid]} to {want}")

    def mutex_track(ns, obj):
        return out.object_track(ns, obj, f"mutex {names.object(obj)}")

    drops = {}
    for d in stream.drops:
        drops.setdefault(d.after, []).append(d)
    reported = {}

    def show_losses(after, ns):
        for d in drops.get(after, ()):
            lost = d.total - reported.get(d.core, 0)
            reported[d.core] = d.total
            at = last_on_core.get(d.core, ns)
            out.track_event(at, out.object_track(at, ("lost", d.core), f"core {d.core} records lost"),
                            INSTANT, name=f"{lost} record(s) lost by here")

    show_losses(0, 0)  # Reported before any record
    for r in in_time_order(stream):
        ns = clock.ns(r.timestamp)
        t = r.thread
        current = running.get(r.recorded_on, 0)
        last_on_core[r.recorded_on] = ns

        if r.event == "thread_created":
            state[t] = "created"
            base[t] = r.priority
            urgency[t] = r.priority

        elif r.event == "thread_ready" and t != 0:  # An idle thread is never woken, only preempted
            if state.get(t) != "running":
                if t not in woken:
                    out.waking(r.recorded_on, ns, current, t, urgency.get(t, r.priority), r.core)
                    woken.add(t)
                end_wait(ns, t)
            state[t] = "ready"

        elif r.event == "remote_wake":
            if state.get(t) in ("created", "blocked") and t not in woken:
                out.waking(r.recorded_on, ns, current, t, urgency.get(t, r.priority), r.core)
                woken.add(t)

        elif r.event == "thread_running":
            previous = running.get(r.core, 0)
            if previous != t:
                out.switch(r.core, ns, previous, urgency.get(previous, 0),
                           STATE_RUNNABLE if previous == 0 else prev_state.get(state.get(previous), STATE_RUNNABLE),
                           t, r.priority)
                if previous != 0:
                    show_boost(ns, previous, None)
            running[r.core] = t
            if t != 0:
                # A reschedule that picks the running thread again records it
                # again, so a boost that lands mid-run shows here too
                state[t] = "running"
                urgency[t] = r.priority
                woken.discard(t)
                end_wait(ns, t)
                show_boost(ns, t, r.priority)

        elif r.event == "thread_blocked":
            state[t] = "blocked"
            if r.object:
                waiting[t] = r.object
                out.track_event(ns, out.thread_track(ns, t, r.core), SLICE_BEGIN,
                                name=f"wait {names.object(r.object)}")

        elif r.event == "thread_terminated":
            state[t] = "terminated"

        elif r.event == "mutex_acquired":
            track = mutex_track(ns, r.object)
            # A release is recorded before the handover's acquire, so a hold
            # still open here means the release record was lost to a drop
            if r.object in holder:
                out.track_event(ns, track, SLICE_END)
            holder[r.object] = t
            out.track_event(ns, track, SLICE_BEGIN, name=names.thread(t))
            if waiting.get(t) == r.object:
                end_wait(ns, t)

        elif r.event == "mutex_released":
            if holder.get(r.object) == t:
                del holder[r.object]
                out.track_event(ns, mutex_track(ns, r.object), SLICE_END)

        elif r.event == "user":
            out.track_event(ns, out.thread_track(ns, t, r.core), INSTANT,
                            name=names.tag(r.priority), annotations=[("value", r.object)])

        show_losses(r.index + 1, ns)

    return out.serialise()


# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("command", choices=["dump", "perfetto"])
    parser.add_argument("stream", help="a stream written by cyros::trace::stream_writer")
    parser.add_argument("-o", "--output", help="perfetto: where to write, by default the stream's name with .pftrace")
    parser.add_argument("--hz", type=int, default=0,
                        help="record timestamps per second, when the stream's header has 0 or is wrong")
    args = parser.parse_args()

    with open(args.stream, "rb") as f:
        data = f.read()
    try:
        stream = decode(data)
    except FormatError as e:
        sys.exit(f"cyros_trace: {args.stream}: {e}")

    report_losses(stream)
    clock = Clock(stream, args.hz)

    if args.command == "dump":
        dump(stream, clock, sys.stdout)
    else:
        output = args.output or (args.stream.rsplit(".", 1)[0] + ".pftrace")
        with open(output, "wb") as f:
            f.write(to_perfetto(stream, clock))
        print(f"wrote {output}: open it at https://ui.perfetto.dev, or query it with trace_processor_shell")


if __name__ == "__main__":
    main()

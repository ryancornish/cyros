import gdb

def format_atomic(val):
    """Delegates formatting to GDB's built-in printer to guarantee accuracy."""
    try:
        raw_str = str(val)
        if "{" in raw_str and "}" in raw_str:
            return raw_str.split("{")[1].split("}")[0].strip()
        return raw_str
    except Exception:
        return "Unreadable"

def get_tcb_summary(tcb_ptr):
    """Safely dereferences a TCB pointer to grab its ID or returns nullptr/error."""
    try:
        addr = int(tcb_ptr)
        if addr == 0:
            return "nullptr"
        # Safely extract the thread ID from the pointed-to TCB structure
        tcb_val = tcb_ptr.dereference()
        t_id = int(tcb_val['id'])
        return f"ID {t_id} (Addr: 0x{addr:x})"
    except Exception:
        return f"0x{int(tcb_ptr):x} [ID unreadable]"

class ThreadControlBlockPrinter:
    def __init__(self, val):
        self.val = val

    def _get_state_string(self):
        try:
            state_val = int(self.val['state'])
            mapping = {0: "ready", 1: "running", 2: "blocked", 3: "terminated"}
            return mapping.get(state_val, f"unknown({state_val})")
        except Exception:
            return "unknown"

    def to_string(self):
        t_id = int(self.val['id'])
        state = self._get_state_string()
        base_pri = int(self.val['base_priority'])
        eff_pri = int(self.val['effective_priority'])
        core = int(self.val['pinned_core'])

        this_addr = self.val.address
        next_addr = self.val['next']
        enqueued = "No (Sentinel)" if this_addr == next_addr else "Yes"

        return (
            f"TCB [ID: {t_id}] (State: {state})\n"
            f"    ├── Priority (Eff/Base): {eff_pri} / {base_pri}\n"
            f"    ├── Pinned Core:         {core}\n"
            f"    └── Enqueued Status:     {enqueued}"
        )


class SchedulerPrinter:
    def __init__(self, val):
        self.val = val

    def to_string(self):
        core_id = int(self.val['core_id'])
        pinned = format_atomic(self.val['pinned_thread_counter'])
        poke = "Pending" if "true" in format_atomic(self.val['inbox_poke_pending']).lower() else "No"

        curr_str = get_tcb_summary(self.val['current_thread'])
        idle_str = get_tcb_summary(self.val['idle_thread'])

        try:
            bitmap = int(self.val['ready_matrix']['bitmap'])
            matrix_str = f"0x{bitmap:x} (Raw: {bin(bitmap)})"
        except Exception:
            matrix_str = "Unknown"

        # Explicitly prepend a newline \n to the start of every core block!
        # This completely overrides GDB's attempts to paste them side-by-side.
        return (
            f"\n  Scheduler [Core {core_id}]\n"
            f"    ├── Active TCB:          {curr_str}\n"
            f"    ├── Idle TCB:            {idle_str}\n"
            f"    ├── Pinned Threads:      {pinned}\n"
            f"    ├── Ready Matrix Bitmap: {matrix_str}\n"
            f"    └── Inbox Poke Status:   {poke}"
        )


class KernelStatePrinter:
    def __init__(self, val):
        self.val = val

    def to_string(self):
        init = format_atomic(self.val['initialised'])
        run = format_atomic(self.val['running'])
        threads = format_atomic(self.val['active_threads'])
        next_id = format_atomic(self.val['thread_id_generator'])

        return (
            f"KernelState Layout\n"
            f"    ├── Initialised:         {init}\n"
            f"    ├── Running:             {run}\n"
            f"    ├── Active Threads:      {threads}\n"
            f"    └── Next Thread ID Gen:  {next_id}"
        )

    def children(self):
        schedulers = self.val['schedulers']
        try:
            elems = schedulers['_M_elems']
            array_type = elems.type
            num_cores = int(array_type.sizeof / array_type.target().sizeof)
            for i in range(num_cores):
                yield f'Core {i}', elems[i]
        except Exception:
            yield 'schedulers_raw', schedulers


def lookup_cyros_types(val):
    type_tag = val.type.unqualified().strip_typedefs().tag
    if not type_tag:
        return None

    if type_tag.endswith("kernel_state"):
        return KernelStatePrinter(val)
    elif type_tag.endswith("scheduler"):
        return SchedulerPrinter(val)
    elif type_tag.endswith("thread_control_block"):
        return ThreadControlBlockPrinter(val)

    return None

gdb.pretty_printers.append(lookup_cyros_types)

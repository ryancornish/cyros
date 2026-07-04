# Cyros

Cyros is a small, modern C++ real-time kernel for embedded systems.

I'm building it around one idea: an RTOS should be something you can *read*
before you *trust*. The core is intentionally tiny, the layers are cleanly
separated, and almost everything beyond the bare minimum is opt-in.

> **Status: early work-in-progress.** What follows is the design I'm building
> toward. Not all of it exists yet, and the parts that do are still moving.
> See [Project Status](#project-status) for an honest breakdown of what's
> actually there today. APIs *will* change.

## Why I'm building this

Most RTOSes make two choices I wanted to avoid:

1. **They bake time into the scheduler.** "Real-time" gets read as "the
   scheduler counts ticks," and timekeeping ends up tangled through the core.
2. **They ship a kitchen sink.** One giant config header tries to anticipate
   every use case, and you pay (in code size and in cognitive load) for
   features you never enable.

Cyros keeps a tight, time-agnostic core instead, and layers everything else on
top as components and features you choose to include.

## Architecture

Cyros is organised into four layers. I take the boundaries between them
seriously: each layer depends only on the ones it's allowed to, and I've picked
the seams deliberately so behaviour stays consistent across very different
targets.

```
        +-------------------------------------------------+
        |  userlib                                        |
        |  Mutex, and other opt-in primitives & features  |
        +-------------------------------------------------+
                 |                          |
                 v                          v
        +-------------------+      +-------------------+
        |  kernel           |      |  time driver      |
        |  schedulers,      |      |  monotonic time,  |
        |  threads,         |      |  scheduled        |
        |  waitable         |      |  callbacks        |
        +-------------------+      +-------------------+
                 |                          |
                 v                          v
        +-------------------------------------------------+
        |  port  (C ABI)                                  |
        |  context switching, IRQ control, cores, time    |
        +-------------------------------------------------+
```

### Port layer

The port is the hardware (or host) abstraction layer. I expose it as a plain
**C ABI** so it can be implemented in C or assembly. It provides context
creation/switching, critical-section (interrupt) control, multi-core startup
and inter-core signalling, and the low-level time source.

The part I care most about here is *where the cut is made*. I slice the port at
exactly the depth where the scheduler and all higher-level kernel policy run
identically whether the port is:

- on **Linux**, using [Boost.Context](https://www.boost.org/doc/libs/release/libs/context/)
  to create and switch cooperative contexts, or
- on **bare metal** - an ARM Cortex-M port is my intended first target.

The kernel never sees a fiber or a thread, only an opaque context. So an
application built on Cyros gets *nearly* reproducible behaviour between a
hosted unit test and the real device. That's useful to me while developing the
kernel, and it should be just as useful to anyone building on top of it.

There's one asymmetry I can't design away: a real bare-metal port can preempt a
task from an external interrupt, while the Linux/Boost.Context port is
cooperative by nature. So ports declare their scheduling type (preemptive or
cooperative) and environment (bare-metal or simulation), and the kernel adapts.

### Kernel layer

The kernel is, more than anything, a set of **per-core schedulers** in an SMP
arrangement.

It has **no concept of time.** This is the deliberately radical choice - an
RTOS core that doesn't know what a second is. The kernel only cares whether a
thread is `ready`, `running`, or `blocked`, and what it's blocked *on*: an
abstract object I call a **`waitable`**.

Each core owns its own scheduler state. Core-local structures are only ever
mutated by their owning core; cross-core operations go through an explicit
message inbox plus an inter-core interrupt. The kernel gives you thread creation
and the blocking/unblocking machinery, and not much else.

> The `waitable` abstraction works well as a notification-style primitive, but
> I'm not happy with it yet for *conditional acquisition* - the mutex-locking
> case, where waking is contingent on a predicate. Expect this area to change.

### Time driver layer

The time driver sits **parallel to** the kernel, not underneath it. It depends
on the port, but the kernel and the time driver don't depend on each other.

Because I keep time outside the core, it's genuinely optional. If you want
nothing to do with time, you can omit the time driver entirely (along with any
feature that transitively needs it) and still have a valid Cyros instance. If
you already have your own timer infrastructure, you can implement the time
driver interface yourself and sidestep mine completely.

I ship a few time driver implementations to pick between:

- a **periodic (tickful)** driver,
- a **tickless** driver, and
- a **simulation** driver for deterministic host-side testing.

### userlib layer

userlib is where the convenient, user-facing primitives live - the things built
*on top of* the kernel primitives (thread creation and `waitable` blocking).

My guiding principle here is **pick-and-choose**. userlib is a set of features,
and a downstream project includes only the ones it wants:

- Don't want a heap? Omit the allocator feature.
- Already have your own allocator? Omit mine and use yours.
- Want synchronization primitives but no time-based variants? Take a `sync`
  feature without timed methods; a separate feature can combine `sync` with the
  time driver to provide the timed variants.

Features can compose with each other and with the time driver, but I don't
force anything on you.

## Building

Cyros is assembled with **Cyros-Builder**, a separate Python-based tool I'm
writing alongside it. It reads component and profile descriptions, selects the
port, time driver, and feature set you want, and produces a packaged library
plus headers.

The component layout in this repo (the `component.toml` files, public header
mappings, and build profiles) is tailored for that builder. For build
instructions, the profile format, and how feature opt-in works mechanically,
see the Cyros-Builder project.

## Project Status

Cyros is in early development. Roughly where things stand:

**Implemented and under test**

- Per-core SMP scheduler with fixed-priority selection and core affinity.
- Thread creation, joining, and the `waitable` blocking model
  (`wait_for`, `wait_for_any`).
- Linux / Boost.Context simulation port.
- Time driver implementations (periodic, tickless, simulation).
- A `Mutex` in `userlib`.

**In flux / being redesigned**

- Reschedule semantics that behave identically on cooperative (Linux) and
  preemptive (bare-metal) ports.
- Spinlock and cross-core synchronisation details.
- The `waitable` interface, particularly conditional acquisition.
- The time driver public interface (moving away from a singleton-style API).

**Planned**

- A bare-metal port (ARM Cortex-M first).
- More `userlib` features - additional sync primitives, an optional heap, and
  so on.
- Example projects, including on-device demos.

If you're poking at the scheduler or thinking about a port, I'd genuinely like
the feedback.
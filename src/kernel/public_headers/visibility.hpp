/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file visibility.hpp
 * @brief Marks declarations that form the public ABI of libcyros.
 *
 * Only relevant to the `lto_merged` archive strategy (see gcc-release.toml).
 * That strategy merges every object into one relocatable and then runs
 * `objcopy --localize-hidden`, so any symbol the compiler marked hidden becomes
 * local and cannot be referenced from outside the archive. The release toolchain
 * compiles with `-fvisibility=hidden`, which hides everything by default, so a
 * symbol stays reachable only if its declaration carries CYROS_PUBLIC.
 *
 * The rule is simply "everything declared in a public_headers file is public".
 * If you add a public entry point, annotate it. Forgetting produces a loud
 * undefined reference when something links against a release build.
 */

#ifndef CYROS_VISIBILITY_HPP
#define CYROS_VISIBILITY_HPP

#if defined(__GNUC__) || defined(__clang__)
#  define CYROS_PUBLIC __attribute__((visibility("default")))
#else
#  define CYROS_PUBLIC
#endif

/**
 * @def CYROS_HOT
 * @brief Places a function on the path every switch, wait or wake runs in one
 *        section, `.text.cyros.hot`, so that path is contiguous in the image.
 *
 * Not about ABI, but it lives beside CYROS_PUBLIC because both say something
 * to the build about where a function goes. Contiguous, the path (about 2 KB
 * on the Cortex-M33) cannot evict itself from an instruction cache, where
 * scattered by the linker it could: on the U575's 8 KB 2-way cache one
 * layout put three of its lines in each of five sets, and the blocking
 * benchmarks lost 4 percent to misses another layout did not take
 * (~/cyros-claude/performance-investigation.md). Any linker script whose text
 * rule matches `.text*` keeps the section whole, and one that wants the path
 * in RAM can place it there. The port's twin is CYROS_PORT_HOT.
 */
#if defined(__GNUC__) || defined(__clang__)
#  define CYROS_HOT __attribute__((section(".text.cyros.hot")))
#else
#  define CYROS_HOT
#endif

#endif  // CYROS_VISIBILITY_HPP

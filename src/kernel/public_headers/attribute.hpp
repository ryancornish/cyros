/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/**
 * @file attribute.hpp
 * @brief Compiler attributes used by the kernel and userlib
 */

#ifndef CYROS_ATTRIBUTE_HPP
#define CYROS_ATTRIBUTE_HPP

/**
 * @def CYROS_PUBLIC
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
 * On targets like the Cortex-M33, this helps cache locality
 */
#if defined(__GNUC__) || defined(__clang__)
#  define CYROS_HOT __attribute__((section(".text.cyros.hot")))
#else
#  define CYROS_HOT
#endif

#endif  // CYROS_ATTRIBUTE_HPP

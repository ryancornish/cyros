/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

#ifndef CYROS_MUTEX_HPP
#define CYROS_MUTEX_HPP

#include <cyros/kernel/base_mutex.hpp>
#include <cyros/kernel/attribute.hpp>

namespace cyros::sync
{

/**
 * @brief Mutual exclusion with priority inheritance.
 *
 * A zero-cost facade over the kernel's base_mutex, which is where the type has
 * to live because the scheduler folds it at every pick. This header is the one
 * users include, so every synchronisation primitive stays grouped here.
 *
 * Naming the protocol at the declaration is the point: a future ceiling_mutex is
 * a sibling facade over the same base, so which inversion protocol a lock runs
 * is visible where the lock is declared rather than in a config flag. See
 * mutex-first-class-plan.md D6.
 *
 * Not a waitable, so this_thread::wait_on cannot take it, with a cancel signal
 * or without. That ban is structural and deliberate, see base_mutex.
 */
class CYROS_PUBLIC mutex : public base_mutex
{
public:
   constexpr mutex() noexcept = default;
};

}  // namespace cyros::sync

namespace cyros
{
using sync::mutex;
}

#endif // CYROS_MUTEX_HPP

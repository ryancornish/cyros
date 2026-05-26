#ifndef CORTOS_WAITABLE_UTILITIES_HPP
#define CORTOS_WAITABLE_UTILITIES_HPP

#include <cortos/kernel/waitable.hpp>
#include <cortos/config/config.hpp>
#include <cortos/port/port.h>

#include <array>
#include <algorithm>

namespace cortos
{

template<std::size_t Max>
class waitable_ref_vector
{
private:
   std::array<waitable*, Max> store{};
   std::size_t count = 0;

public:
   constexpr waitable_ref_vector() = default;
   using iterator = waitable**;
   using const_iterator = waitable* const*;

   iterator begin()
   {
      return store.data();
   }

   iterator end()
   {
      return store.data() + count;
   }

   [[nodiscard]] bool empty() const noexcept
   {
      return count == 0;
   }

   [[nodiscard]] size_t size() const noexcept
   {
      return count;
   }

   [[nodiscard]] const_iterator begin() const noexcept
   {
      return store.data();
   }

   [[nodiscard]] const_iterator end() const noexcept
   {
     return store.data() + count;
   }

   [[nodiscard]] constexpr size_t capacity() const noexcept
   {
     return store.size();
   }

   waitable* const& operator[](std::size_t index) const noexcept
   {
      CORTOS_ASSERT(index < count);
      return store[index];
   }

   void push(waitable* waitable)
   {
      CORTOS_ASSERT(count < store.size());
      store[count++] = waitable;
   }

   void push_range(std::span<waitable* const> waitables)
   {
      CORTOS_ASSERT(count + waitables.size() <= store.size());
      std::ranges::copy(waitables, store.data() + count);
      count += waitables.size();
   }

   void pop()
   {
      CORTOS_ASSERT(!empty());
      --count;
   }

   void sort_by_address()
   {
      if (count < 2) return;
      std::ranges::sort(begin(), end());
   }

   void clear() noexcept
   {
      count = 0;
   }
};


class waitable_group_lock
{
private:
   waitable_ref_vector<config::max_wait_nodes> group;

public:
   waitable_group_lock(waitable_group_lock const&) = delete;
   waitable_group_lock(waitable_group_lock&&) = delete;
   waitable_group_lock& operator=(waitable_group_lock const&) = delete;
   waitable_group_lock& operator=(waitable_group_lock&&) = delete;

   explicit waitable_group_lock(std::span<waitable* const> waitables)
   {
      group.push_range(waitables);
      group.sort_by_address();
      // Enforce uniqueness
      for (std::size_t i = 1; i < group.size(); ++i) {
         CORTOS_ASSERT(group[i] != group[i - 1]);
      }

      // Lock in order
      for (auto& waitable : group) {
         waitable->wait_lock.lock();
      }
   }

   ~waitable_group_lock()
   {
      auto n = group.size();
      while (n--) {
         group[n]->wait_lock.unlock();
      }
   }
};

} // namespace cortos

#endif // CORTOS_WAITABLE_UTILITIES_HPP

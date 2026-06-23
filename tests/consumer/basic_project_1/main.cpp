#include <cyros/kernel/kernel.hpp>
#include <cyros/port/port.h>
#include <cyros/port/port_traits.h>
#include <cyros/time/time.hpp>

#include <print>
#include <cstddef>
#include <array>

alignas(CYROS_PORT_STACK_ALIGN)
static std::array<std::byte, cyros::thread::min_stack_size + 4096> user_stack;

int main()
{
   cyros::kernel::initialise();

   std::println("Cyros initialised.");

   cyros::thread user_thread(
      [](){
         std::println("User thread executed.");
      },
      user_stack,
      cyros::thread::priority(0),
      cyros::any_core
   );

   cyros::kernel::start();

   std::println("Cyros Finished.");

   cyros::kernel::finalise();
}

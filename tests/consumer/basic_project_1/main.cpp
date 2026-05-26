#include <cortos/kernel/kernel.hpp>
#include <cortos/port/port.h>
#include <cortos/port/port_traits.h>
#include <cortos/time/time.hpp>

#include <print>
#include <cstddef>
#include <array>

alignas(CORTOS_PORT_STACK_ALIGN) static std::array<std::byte, 4096> user_stack;

int main()
{
   cortos::kernel::initialise();

   std::println("CoRTOS initialised.");

   cortos::thread user_thread(
      [](){
         std::println("User thread executed.");
      },
      user_stack,
      cortos::thread::priority(0),
      cortos::any_core
   );

   cortos::kernel::start();

   std::println("CoRTOS Finished.");

   cortos::kernel::finalise();
}

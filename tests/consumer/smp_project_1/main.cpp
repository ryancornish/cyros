#include <cyros/kernel/kernel.hpp>
#include <cyros/time/time.hpp>

#include "logging.hpp"

static constexpr auto CLOCK_FREQ_HZ = 1'000'000u; // 1MHz


int main()
{
   log::printf("\n---SMP_PROJECT_1---\n\n");

   cyros::kernel::initialise();
   cyros::time::initialise(CLOCK_FREQ_HZ);


}
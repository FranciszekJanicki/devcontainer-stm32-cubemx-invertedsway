#include "balance_sway.hpp"
#include "system_clock.h"
#include "tests.hpp"

int main()
{
    HAL_Init();
    SystemClock_Config();

    Tests::MPU_DMP_TEST();

    return 0;
}

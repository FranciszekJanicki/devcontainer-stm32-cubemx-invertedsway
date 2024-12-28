#include "system_clock.h"
#include "tests.hpp"

int main()
{
    using namespace Tests;

    HAL_Init();
    SystemClock_Config();

    MPU_TEST();

    return 0;
}

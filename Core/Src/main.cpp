#include "system_clock.h"
#include "tests.hpp"

using namespace Tests;

int main()
{
    HAL_Init();
    SystemClock_Config();

    MPU_TEST();

    return 0;
}
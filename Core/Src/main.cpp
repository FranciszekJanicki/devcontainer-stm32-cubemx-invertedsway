#include "balance_sway.hpp"
#include "gpio.h"
#include "i2c.h"
#include "system_clock.h"
#include "tim.h"
#include "usart.h"

int main()
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();

    balance_sway();

    return 0;
}

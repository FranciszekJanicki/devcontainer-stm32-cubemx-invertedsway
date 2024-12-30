#ifndef COMMON_HPP
#define COMMON_HPP

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_gpio.h"
#include "stm32l4xx_hal_i2c.h"
#include "stm32l4xx_hal_tim.h"
#include "stm32l4xx_hal_uart.h"
#include "usart.h"
#include <array>
#include <bit>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace InvertedSway {

    using TimerHandle = TIM_HandleTypeDef*;
    using GPIOHandle = GPIO_TypeDef*;
    using UARTHandle = UART_HandleTypeDef*;
    using I2CBusHandle = I2C_HandleTypeDef*;

}; // namespace InvertedSway

#endif // COMMON_HPP
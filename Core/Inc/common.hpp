#ifndef COMMON_HPP
#define COMMON_HPP

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_gpio.h"
#include "stm32l4xx_hal_i2c.h"
#include "stm32l4xx_hal_tim.h"
#include "stm32l4xx_hal_uart.h"
#include "usart.h"
#include <bit>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace InvertedSway {

    using TimerHandle = TIM_HandleTypeDef*;
    using GpioHandle = GPIO_TypeDef*;
    using UartHandle = UART_HandleTypeDef*;
    using I2cHandle = I2C_HandleTypeDef*;

}; // namespace InvertedSway

#endif // COMMON_HPP
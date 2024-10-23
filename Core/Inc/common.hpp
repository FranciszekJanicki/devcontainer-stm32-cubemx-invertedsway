#ifndef COMMON_HPP
#define COMMON_HPP

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_gpio.h"
#include "stm32l4xx_hal_i2c.h"
#include "stm32l4xx_hal_tim.h"
#include "stm32l4xx_hal_uart.h"
#include <bit>
#include <cstdint>
#include <cstdio>
#include <cstring>

using TimerHandle = TIM_HandleTypeDef*;
using GpioHandle = GPIO_TypeDef*;
using UartHandle = UART_HandleTypeDef*;
using I2cHandle = I2C_HandleTypeDef*;

static void uart_send_string(UartHandle huart, char* string) noexcept
{
    HAL_UART_Transmit(huart, std::bit_cast<const std::uint8_t*>(string), strlen(string), 1000);
    memset(string, 0, strlen(string));
}

#endif // COMMON_HPP
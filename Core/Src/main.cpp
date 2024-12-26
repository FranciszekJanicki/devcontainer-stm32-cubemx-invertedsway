#include "filters.hpp"
#include "gpio.h"
#include "i2c.h"
#include "regulators.hpp"
#include "system_clock.h"
#include "tests.hpp"
#include "tim.h"
#include "usart.h"
#include <cstdio>
#include <utility>

using namespace Tests;

static void test_motor()
{
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_TIM4_Init();

    MOTOR_TEST();
}

static void test_motor_boost()
{
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_TIM4_Init();

    MOTOR_BOOST_TEST();
}

static void test_encoder()
{
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();

    ENCODER_TEST();
}

static void test_mpu()
{
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();

    MPU_TEST();
}

static void test_mpu_dmp()
{
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();

    MPU_DMP_TEST();
}

static void test_kalman()
{
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();

    KALMAN_TEST();
}

static void test_dutkiewicz()
{
    MX_USART2_UART_Init();

    DUTKIEWICZ_TEST();
}

int main()
{
    HAL_Init();
    SystemClock_Config();

    test_mpu();

    return 0;
}
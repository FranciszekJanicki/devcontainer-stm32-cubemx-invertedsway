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

using namespace InvertedSway;
using namespace Regulators;
using namespace Filters;
using namespace Tests;

void test_motor()
{
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_TIM4_Init();

    MOTOR_TEST(Motor{&htim4, TIM_CHANNEL_1, L298N_IN1_GPIO_Port, L298N_IN1_Pin, L298N_IN3_Pin});
}

void test_motor_boost()
{
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_TIM4_Init();

    MOTOR_BOOST_TEST(Motor{&htim4, TIM_CHANNEL_1, L298N_IN1_GPIO_Port, L298N_IN1_Pin, L298N_IN3_Pin}, 2.0f);
}

void test_encoder()
{
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();

    ENCODER_TEST(Encoder{&htim3}, Motor{&htim4, TIM_CHANNEL_1, L298N_IN1_GPIO_Port, L298N_IN1_Pin, L298N_IN3_Pin});
}

void test_kalman()
{
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();

    KALMAN_TEST(MPU6050{&hi2c1,
                        MPU6050::DevAddress::AD0_LOW,
                        MPU6050::GyroRange::GYRO_FS_250,
                        MPU6050::AccelRange::ACCEL_FS_2,
                        8000U},
                make_kalman(0.0f, 0.0f, 0.1f, 0.3f, 0.03f),
                8000U);
}

void test_dutkiewicz()
{
    MX_USART2_UART_Init();

    DUTKIEWICZ_TEST();
}

int main()
{
    HAL_Init();
    SystemClock_Config();

    /* most important test */
    test_kalman();

    return 0;
}
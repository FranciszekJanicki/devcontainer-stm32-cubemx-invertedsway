#include "main.h"
#include "encoder.hpp"
#include "filters.hpp"
#include "gpio.h"
#include "i2c.h"
#include "l298n.hpp"
#include "mpu6050.hpp"
#include "regulators.hpp"
#include "system.hpp"
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
    MX_TIM2_Init();

    MOTOR_TEST(Motor{&htim2, TIM_CHANNEL_1, GPIOB, IN1_Pin, IN3_Pin});
}

void test_motor_boost()
{
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_TIM2_Init();

    MOTOR_BOOST_TEST(Motor{&htim2, TIM_CHANNEL_1, GPIOB, IN1_Pin, IN3_Pin}, 2.0f);
}

void test_encoder()
{
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();

    ENCODER_TEST(Encoder{&htim1}, Motor{&htim2, TIM_CHANNEL_1, GPIOB, IN1_Pin, IN3_Pin});
}

void test_kalman()
{
    MX_I2C1_Init();
    MX_USART2_UART_Init();

    KALMAN_TEST(MPU6050{&hi2c1,
                        MPU6050::DeviceAddress::AD0_LOW,
                        MPU6050::GyroRange::GYRO_FS_250,
                        MPU6050::AccelRange::ACCEL_FS_2,
                        8000U},
                make_kalman(0.0f, 0.0f, 0.1f, 0.3f, 0.03f),
                1.0f / static_cast<float>(8000U));
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
    test_dutkiewicz();

    return 0;
}

void Error_Handler()
{
    __disable_irq();
    while (1) {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{}
#endif /* USE_FULL_ASSERT */

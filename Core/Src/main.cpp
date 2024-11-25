#include "main.h"
#include "encoder.hpp"
#include "filters.hpp"
#include "gpio.h"
#include "i2c.h"
#include "l298n.hpp"
#include "mpu6050.hpp"
#include "regulator.hpp"
#include "system.hpp"
#include "system_clock.h"
#include "tim.h"
#include "usart.h"
#include <cstdio>
#include <utility>

static bool timer_elapsed{false};

int main()
{
    using namespace InvertedSway;
    using namespace InvertedSway::Regulator;

    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_TIM2_Init();
    MX_I2C1_Init();
    MX_TIM1_Init();

    L298N l298n{L298N::MotorChannels{
        L298N::make_motor_channel(L298N::Channel::CHANNEL1,
                                  &htim2,
                                  TIM_CHANNEL_1,
                                  GPIOB,
                                  IN1_Pin,
                                  IN3_Pin,
                                  OUT1_Pin,
                                  OUT3_Pin),
        L298N::make_motor_channel(L298N::Channel::CHANNEL2),
    }};

    MPU6050 mpu6050{&hi2c1, MPU6050::ADDRESS, MPU6050::GYRO_FS_250, MPU6050::ACCEL_FS_2};

    auto kalman{make_kalman<float>(0.0f, 0.0f, 0.1f, 0.3f, 0.03f)};

    auto regulator{make_regulator<float>(Algorithm::PID, 0.0f, 0.0f, 0.0f, 0.0f)};

    Encoder encoder{&htim1};

    System system{std::move(mpu6050), std::move(l298n), std::move(kalman), std::move(regulator), std::move(encoder)};

    const float angle{0.0f};
    const float sampling_time{MPU6050::SAMPLING_TIME_S};

    HAL_TIM_Base_Start_IT(&htim3);

    while (true) {
        if (timer_elapsed) {
            system.balance_sway(angle, sampling_time);
            timer_elapsed = false;
        }
    }

    return 0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim == nullptr) {
        return;
    }
    if (htim->Instance == TIM3) {
        bool timer_elapsed = true;
        HAL_TIM_Base_Start_IT(htim);
    }
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

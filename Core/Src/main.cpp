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

static bool sampling_timer_elapsed{false};

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM2) {
        sampling_timer_elapsed = true;
    }
    HAL_TIM_Base_Start_IT(htim);
}

static void balance_sway()
{
    using namespace InvertedSway;
    using namespace Regulators;
    using namespace Filters;

    auto const sampling_rate_hz{8000U};
    auto const sampling_time{1.0f / static_cast<float>(sampling_rate_hz)};
    auto const angle{0.0f};

    L298N l298n{L298N::MotorChannels{L298N::make_motor_channel(L298N::Channel::CHANNEL1,
                                                               &htim4,
                                                               TIM_CHANNEL_1,
                                                               L298N_IN1_GPIO_Port,
                                                               L298N_IN1_Pin,
                                                               L298N_IN3_Pin),
                                     L298N::make_motor_channel(L298N::Channel::CHANNEL2)}};

    MPU6050 mpu6050{&hi2c1,
                    MPU6050::DeviceAddress::AD0_LOW,
                    MPU6050::GyroRange::GYRO_FS_250,
                    MPU6050::AccelRange::ACCEL_FS_2,
                    sampling_rate_hz};

    auto kalman{make_kalman(0.0f, 0.0f, 0.1f, 0.3f, 0.03f)};

    auto regulator{make_regulator<Algorithm::PID>(0.0f, 0.0f, 0.0f, 0.0f)};

    Encoder encoder{&htim3};

    System system{std::move(mpu6050), std::move(l298n), std::move(kalman), std::move(regulator), std::move(encoder)};

    HAL_TIM_Base_Start_IT(&htim2);

    while (true) {
        if (sampling_timer_elapsed) {
            system(angle, sampling_time);
            sampling_timer_elapsed = false;
        }
    }
}

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

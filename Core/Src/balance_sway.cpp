#include "balance_sway.hpp"
#include "encoder.hpp"
#include "filters.hpp"
#include "gpio.h"
#include "i2c.h"
#include "i2c_driver_sigma.hpp"
#include "l298n.hpp"
#include "main.h"
#include "mpu6050.hpp"
#include "regulators.hpp"
#include "sway.hpp"
#include "tim.h"
#include "usart.h"
#include <utility>

static bool sampling_timer_elapsed{false};

// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
// {
//     if (GPIO_Pin == MPU6050_INTR_Pin) {
//         sampling_timer_elapsed = true;
//     }
// }

void balance_sway()
{
    using namespace InvertedSway;
    using namespace Regulators;
    using namespace Filters;

    auto const sampling_rate_hz{8000U};
    auto const sampling_time{1.0f / static_cast<float>(sampling_rate_hz)};
    auto const angle{0.0f};

    L298N::MotorChannels motor_channels{
        L298N::MotorChannel{L298N::Channel::CHANNEL1,
                            Motor{&htim4, TIM_CHANNEL_1, L298N_IN1_GPIO_Port, L298N_IN1_Pin, L298N_IN3_Pin}},
        L298N::MotorChannel{L298N::Channel::CHANNEL2}};

    L298N l298n{std::move(motor_channels)};

    MPU6050 mpu6050{&hi2c1,
                    MPU6050::DevAddress::AD0_LOW,
                    MPU6050::GyroRange::GYRO_FS_250,
                    MPU6050::AccelRange::ACCEL_FS_2,
                    sampling_rate_hz};

    auto kalman{make_kalman(0.0f, 0.0f, 0.1f, 0.3f, 0.03f)};

    auto regulator{make_regulator<Algorithm::PID>(0.1f, 0.0f, 0.0f, 0.0f)};

    Encoder encoder{&htim3};

    Sway sway{std::move(mpu6050), std::move(l298n), std::move(kalman), std::move(regulator), std::move(encoder)};

    while (true) {
        if (sampling_timer_elapsed) {
            sway(angle, sampling_time);
            sampling_timer_elapsed = false;
        }
    }
}

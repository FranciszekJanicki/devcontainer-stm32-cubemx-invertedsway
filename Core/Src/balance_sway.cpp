#include "balance_sway.hpp"
#include "encoder.hpp"
#include "filters.hpp"
#include "gpio.h"
#include "i2c.h"
#include "i2c_device.hpp"
#include "l298n.hpp"
#include "main.h"
#include "mpu6050.hpp"
#include "regulators.hpp"
#include "sway.hpp"
#include "tim.h"
#include "usart.h"
#include <utility>

namespace {

    bool sampling_timer_elapsed{false};

};

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == MPU6050_INTR_Pin) {
        sampling_timer_elapsed = true;
    }
}

void balance_sway()
{
    using namespace InvertedSway;
    using namespace Regulators;
    using Kalman = Filters::Kalman<float>;
    using Regulator = Regulators::PID<float>;

    auto const sampling_rate_hz{200U};
    auto const sampling_time{1.0F / static_cast<float>(sampling_rate_hz)};
    auto input_angle{-0.04F};

    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();

    L298N::MotorChannels motor_channels{
        L298N::MotorChannel{.channel = L298N::Channel::CHANNEL1,
                            .motor = Motor{&htim4, TIM_CHANNEL_1, L298N_IN1_GPIO_Port, L298N_IN1_Pin, L298N_IN3_Pin}},
        L298N::MotorChannel{.channel = L298N::Channel::CHANNEL2}};

    L298N l298n{std::move(motor_channels)};

    I2CDevice i2c_mpu_device{.i2c_bus = &hi2c1, .device_address = std::to_underlying(MPU6050::DevAddress::AD0_LOW)};

    MPU6050 mpu6050{i2c_mpu_device,
                    sampling_rate_hz,
                    MPU6050::GyroRange::GYRO_FS_2000,
                    MPU6050::AccelRange::ACCEL_FS_2,
                    MPU6050::DLPF::BW_42,
                    MPU6050::DHPF::DHPF_RESET};

    MPU_DMP mpu_dmp{std::move(mpu6050)};

    Kalman kalman{.k_angle = 0.0F, .k_bias = 0.0F, .Q_angle = 0.1F, .Q_bias = 0.3F, .R = 0.03F};

    Regulator regulator{.kp = 400.0F, .ki = 500.0F, .kd = 10.0F, .windup = 6.0F};

    Encoder encoder{&htim3};

    Sway sway{std::move(mpu_dmp), std::move(l298n), std::move(kalman), std::move(regulator), std::move(encoder)};

    auto prev_val = 0.0F;
    PID<float> regulator_encoder{.kp = 1.0F, .ki = 0.0F, .kd = 0.0F, .windup = 1.0F};

    while (true) {
        if (sampling_timer_elapsed) {
            // auto angle_error = encoder_angle - encoder.get_angle().value();
            // auto balance_angle = regulator_encoder(angle_error, sampling_time);
            sway(input_angle, sampling_time);
            sampling_timer_elapsed = false;
        }
    }
}

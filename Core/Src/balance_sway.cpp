#include "balance_sway.hpp"
#include "encoder.hpp"
#include "filters.hpp"
#include "gpio.h"
#include "i2c.h"
#include "i2c_device.hpp"
#include "l298n.hpp"
#include "main.h"
#include "mpu6050.hpp"
#include "pwm_device.hpp"
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

    constexpr auto SAMPLING_RATE{200U};
    constexpr auto SAMPLING_TIME{1.0F / static_cast<float>(SAMPLING_RATE)};
    constexpr auto INPUT_ANGLE{-0.04F};

    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();

    PWMDevice pwm_device{.timer = &htim4,
                         .timer_channel = TIM_CHANNEL_1,
                         .counter_period = 39999U,
                         .min_voltage = 0.0F,
                         .max_voltage = 6.0F};

    Motor motor1{pwm_device, L298N_IN1_GPIO_Port, L298N_IN1_Pin, L298N_IN3_Pin};

    Motor motor2{};

    L298N l298n{
        .motor_channels = {L298N::MotorChannel{.channel = L298N::Channel::CHANNEL1, .motor = std::move(motor1)},
                           L298N::MotorChannel{.channel = L298N::Channel::CHANNEL2, .motor = std::move(motor2)}}};

    I2CDevice i2c_device{.i2c_bus = &hi2c1, .device_address = std::to_underlying(MPU6050::DevAddress::AD0_LOW)};

    MPU6050 mpu6050{i2c_device,
                    SAMPLING_RATE,
                    MPU6050::GyroRange::GYRO_FS_2000,
                    MPU6050::AccelRange::ACCEL_FS_2,
                    MPU6050::DLPF::BW_42,
                    MPU6050::DHPF::DHPF_RESET};

    MPU_DMP mpu_dmp{std::move(mpu6050)};

    Kalman kalman{.k_angle = 0.0F, .k_bias = 0.0F, .Q_angle = 0.1F, .Q_bias = 0.3F, .R = 0.03F};

    Regulator regulator{.kp = 400.0F, .ki = 500.0F, .kd = 10.0F, .windup = 6.0F};

    Encoder encoder{&htim3, 360U, 1U, 65535U};

    Sway sway{std::move(mpu_dmp), std::move(l298n), std::move(kalman), std::move(regulator), std::move(encoder)};

    while (true) {
        if (sampling_timer_elapsed) {
            sway(INPUT_ANGLE, SAMPLING_TIME);
            sampling_timer_elapsed = false;
        }
    }
}

#include "tests.hpp"
#include "balance_sway.hpp"
#include "encoder.hpp"
#include "filters.hpp"
#include "gpio.h"
#include "i2c.h"
#include "i2c_device.hpp"
#include "l298n.hpp"
#include "main.h"
#include "motor.hpp"
#include "motor_driver.hpp"
#include "mpu6050.hpp"
#include "mpu6050_dmp.hpp"
#include "regulators.hpp"
#include "sway.hpp"
#include "tim.h"
#include "usart.h"
#include <cmath>
#include <cstdio>
#include <utility>

using namespace InvertedSway;
using namespace Filters;
using namespace Regulators;

static bool sampling_timer_elapsed{false};

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == MPU6050_INTR_Pin) {
        // sampling_timer_elapsed = true;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM2) {
        sampling_timer_elapsed = true;
    }
    HAL_TIM_Base_Start_IT(htim);
}

namespace Tests {

    void MOTOR_TEST() noexcept
    {
        MX_GPIO_Init();
        MX_USART2_UART_Init();
        MX_TIM4_Init();

        Motor motor{&htim4, TIM_CHANNEL_1, L298N_IN1_GPIO_Port, L298N_IN1_Pin, L298N_IN3_Pin};

        while (true) {
            for (auto const voltage : {0.0f, 3.0f, 6.0f, 3.0f, 0.0f, -3.0f, -6.0f, -3.0f}) {
                if (voltage > 0.0f) {
                    motor.set_forward();
                } else if (voltage < 0.0f) {
                    motor.set_backward();
                } else {
                    motor.set_fast_stop();
                }

                motor.set_compare_voltage(std::abs(voltage));
                HAL_Delay(1000);
            }
        }
    }

    void MOTOR_BOOST_TEST() noexcept
    {
        MX_GPIO_Init();
        MX_USART2_UART_Init();
        MX_TIM4_Init();

        Motor motor{&htim4, TIM_CHANNEL_1, L298N_IN1_GPIO_Port, L298N_IN1_Pin, L298N_IN3_Pin};

        float const voltage_start_threshold{1.0f};

        while (true) {
            auto last_voltage{0.0f};
            for (auto const voltage : {0.0f, 3.0f, 6.0f, 3.0f, 0.0f, -3.0f, -6.0f, -3.0f}) {
                if (voltage > 0.0f) {
                    motor.set_forward();
                } else if (voltage < 0.0f) {
                    motor.set_backward();
                } else {
                    motor.set_fast_stop();
                }

                if (std::abs(voltage) >= voltage_start_threshold && std::abs(last_voltage) < voltage_start_threshold) {
                    motor.set_compare_max();
                    HAL_Delay(10);
                }
                motor.set_compare_voltage(std::abs(voltage));

                last_voltage = voltage;
                HAL_Delay(1000);
            }
        }
    }

    void MOTOR_DRIVER_TEST() noexcept
    {
        MX_GPIO_Init();
        MX_USART2_UART_Init();
        MX_TIM3_Init();
        MX_TIM4_Init();

        auto regulator{make_regulator<Algorithm::PID>(0.0f, 0.0f, 0.0f, 0.0f)};

        Motor motor{&htim4, TIM_CHANNEL_1, L298N_IN1_GPIO_Port, L298N_IN1_Pin, L298N_IN3_Pin};

        Encoder encoder{&htim3};

        MotorDriver motor_driver{std::move(regulator), std::move(motor), std::move(encoder)};

        constexpr auto MAX_SPEED{MotorDriver::MAX_SPEED_RPM};
        constexpr auto MIN_SPEED{MotorDriver::MIN_SPEED_RPM};

        while (true) {
            for (auto const speed : {MIN_SPEED,
                                     MAX_SPEED / 2,
                                     MAX_SPEED,
                                     MAX_SPEED / 2,
                                     MIN_SPEED,
                                     -MAX_SPEED / 2,
                                     -MAX_SPEED,
                                     -MAX_SPEED / 2}) {
                motor_driver(speed, 1);
                HAL_Delay(1000);
            }
        }
    }

    void MPU_TEST() noexcept
    {
        MX_GPIO_Init();
        MX_USART2_UART_Init();
        MX_I2C1_Init();
        MX_TIM2_Init();

        using RegAddress = MPU6050::RegAddress;

        std::uint8_t buf = 1 << 7;
        HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, std::to_underlying(RegAddress::PWR_MGMT_1), 1, &buf, 1, 1000);

        buf = 0;
        HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, std::to_underlying(RegAddress::PWR_MGMT_1), 1, &buf, 1, 1000);

        buf = 39;
        HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, std::to_underlying(RegAddress::SMPLRT_DIV), 1, &buf, 1, 1000);

        buf = 0;
        HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, std::to_underlying(RegAddress::CONFIG), 1, &buf, 1, 1000);

        buf = 0;
        HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, std::to_underlying(RegAddress::GYRO_CONFIG), 1, &buf, 1, 1000);

        buf = 0;
        HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, std::to_underlying(RegAddress::ACCEL_CONFIG), 1, &buf, 1, 1000);

        buf = (0 << 7) | (0 << 5) | (1 << 4);
        HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, std::to_underlying(RegAddress::INT_PIN_CFG), 1, &buf, 1, 1000);

        buf = 1;
        HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, std::to_underlying(RegAddress::INT_ENABLE), 1, &buf, 1, 1000);

        HAL_TIM_Base_Start_IT(&htim2);

        std::uint8_t buffer[2];

        while (true) {
            if (sampling_timer_elapsed) {
                std::uint8_t buffer[2];
                HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, std::to_underlying(RegAddress::GYRO_XOUT_H), 1, buffer, 2, 1000);

                printf("accel %d\n\r", ((int16_t)buffer[0] << 8) | (int16_t)buffer[1]);

                HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, std::to_underlying(RegAddress::GYRO_XOUT_H), 1, buffer, 2, 1000);

                printf("gyro %d\n\r", ((int16_t)buffer[0] << 8) | (int16_t)buffer[1]);
                sampling_timer_elapsed = false;
            }
        }
    }

    void MPU_DMP_TEST() noexcept
    {
        MX_GPIO_Init();
        MX_USART2_UART_Init();
        MX_I2C1_Init();

        I2CDevice i2c_mpu_device{&hi2c1, std::to_underlying(MPU6050::DevAddress::AD0_LOW)};

        MPU6050 mpu6050{i2c_mpu_device,
                        8000U,
                        MPU6050::GyroRange::GYRO_FS_250,
                        MPU6050::AccelRange::ACCEL_FS_2,
                        MPU6050::DLPF::BW_256,
                        MPU6050::DHPF::DHPF_RESET};

        MPU6050_DMP mpu6050_dmp{std::move(mpu6050)};

        while (true) {
            if (sampling_timer_elapsed) {
                auto const& [roll, pitch, yaw]{mpu6050_dmp.get_roll_pitch_yaw()};
                printf("roll: %f, pitch: %f, yaw: %f\n\r", roll, pitch, yaw);
                sampling_timer_elapsed = false;
            }
        }
    }

    void KALMAN_TEST() noexcept
    {
        MX_GPIO_Init();
        MX_USART2_UART_Init();
        MX_I2C1_Init();

        I2CDevice i2c_mpu_device{&hi2c1, std::to_underlying(MPU6050::DevAddress::AD0_LOW)};

        MPU6050 mpu6050{i2c_mpu_device,
                        8000U,
                        MPU6050::GyroRange::GYRO_FS_250,
                        MPU6050::AccelRange::ACCEL_FS_2,
                        MPU6050::DLPF::BW_256,
                        MPU6050::DHPF::DHPF_RESET};

        auto kalman{make_kalman(0.0f, 0.0f, 0.1f, 0.3f, 0.03f)};

        auto const sampling_time{1.0f / 8000.0f};

        while (true) {
            if (sampling_timer_elapsed) {
                float const roll = mpu6050.get_roll();
                float const gx = mpu6050.get_rotation_x_scaled();
                printf("mpu angle: %f, %f\n\r", gx, roll);
                printf("kalman angle: %f\n\r", kalman(gx, roll, sampling_time));
                sampling_timer_elapsed = false;
            }
        }
    }

    void ENCODER_TEST() noexcept
    {
        MX_GPIO_Init();
        MX_USART2_UART_Init();
        MX_TIM3_Init();
        MX_TIM4_Init();

        Encoder encoder{&htim3};
        Motor motor{&htim4, TIM_CHANNEL_1, L298N_IN1_GPIO_Port, L298N_IN1_Pin, L298N_IN3_Pin};

        while (true) {
            float const angle = encoder.get_angle().value();

            if (angle >= 350.0f) {
                motor.set_backward();
            } else if (angle <= 10.0f) {
                motor.set_forward();
            }

            printf("encoder angle: %f\n\r", angle);
            HAL_Delay(100);
        }
    }

    void DUTKIEWICZ_TEST() noexcept
    {
        MX_USART2_UART_Init();

        while (true) {
            for (auto i{0}; i < 8; ++i)
                printf("DUTKIEWICZ SIGMA\t");
            printf("\n\r");
        }
    }

}; // namespace Tests

#include "tests.hpp"
#include "balance_sway.hpp"
#include "encoder.hpp"
#include "filters.hpp"
#include "gpio.h"
#include "i2c.h"
#include "kalman.hpp"
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
        sampling_timer_elapsed = true;
    }
}

namespace Tests {

    void MOTOR_TEST() noexcept
    {
        Motor motor{&htim4, TIM_CHANNEL_1, L298N_IN1_GPIO_Port, L298N_IN1_Pin, L298N_IN3_Pin};

        using Direction = Motor::Direction;

        motor.set_compare_min();
        motor.set_fast_stop();

        while (true) {
            for (auto const voltage : {0.0f, 3.0f, 6.0f, 3.0f, 0.0f, -3.0f, -6.0f, -3.0f}) {
                if (voltage > 0.0f) {
                    printf("setting direction forward\n\r");
                    motor.set_direction(Direction::FORWARD);
                } else if (voltage < 0.0f) {
                    printf("setting direction backward\n\r");
                    motor.set_direction(Direction::BACKWARD);
                } else {
                    printf("setting direction fast stop\n\r");
                    motor.set_direction(Direction::FAST_STOP);
                }

                printf("setting compare %f\n\r", std::abs(voltage));
                motor.set_compare_voltage(std::abs(voltage));
                HAL_Delay(1000);
            }
        }

        motor.set_compare_min();
        motor.set_fast_stop();
    }

    void MOTOR_BOOST_TEST() noexcept
    {
        Motor motor{&htim4, TIM_CHANNEL_1, L298N_IN1_GPIO_Port, L298N_IN1_Pin, L298N_IN3_Pin};
        float const voltage_start_threshold{1.0f};

        using Direction = Motor::Direction;

        motor.set_compare_min();
        motor.set_fast_stop();

        while (true) {
            auto last_voltage{0.0f};
            for (auto const voltage : {0.0f, 3.0f, 6.0f, 3.0f, 0.0f, -3.0f, -6.0f, -3.0f}) {
                if (voltage > 0.0f) {
                    printf("setting direction forward\n\r");
                    motor.set_direction(Direction::FORWARD);
                } else if (voltage < 0.0f) {
                    printf("setting direction backward\n\r");
                    motor.set_direction(Direction::BACKWARD);
                } else {
                    printf("setting direction fast stop\n\r");
                    motor.set_direction(Direction::FAST_STOP);
                }

                if (std::abs(voltage) >= voltage_start_threshold && std::abs(last_voltage) < voltage_start_threshold) {
                    motor.set_compare_max();
                    HAL_Delay(10);
                } else {
                    printf("setting compare %f\n\r", std::abs(voltage));
                    motor.set_compare_voltage(std::abs(voltage));
                }

                last_voltage = voltage;
                HAL_Delay(1000);
            }
        }

        motor.set_compare_min();
        motor.set_fast_stop();
    }

    void MOTOR_DRIVER_TEST() noexcept
    {
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
                //    motor_driver(speed, 1);
                HAL_Delay(1000);
            }
        }
    }

    void MPU_TEST() noexcept
    {
        MPU6050 mpu6050{&hi2c1,
                        MPU6050::DevAddress::AD0_LOW,
                        MPU6050::GyroRange::GYRO_FS_250,
                        MPU6050::AccelRange::ACCEL_FS_2,
                        8000U};

        while (true) {
            // if (/*sampling_timer_elapsed*/ HAL_GPIO_ReadPin(MPU6050_INTR_GPIO_Port, MPU6050_INTR_Pin) ==
            //   GPIO_PinState::GPIO_PIN_SET) {
            auto const& [ax, ay, az]{mpu6050.get_acceleration_scaled()};
            auto const& [gx, gy, gz]{mpu6050.get_rotation_scaled()};
            printf("accel x: %f, y: %f, z: %f\n\r", ax, ay, az);
            printf("gyro x: %f, y: %f, z: %f\n\r", gx, gy, gz);
            /*sampling_timer_elapsed = false;*/
            //}
        }
    }

    void MPU_DMP_TEST() noexcept
    {
        MX_GPIO_Init();
        MX_USART2_UART_Init();
        MX_I2C1_Init();

        MPU6050 mpu6050{&hi2c1,
                        MPU6050::DevAddress::AD0_LOW,
                        MPU6050::GyroRange::GYRO_FS_250,
                        MPU6050::AccelRange::ACCEL_FS_2,
                        8000U};

        MPU6050_DMP mpu6050_dmp{std::move(mpu6050)};

        while (true) {
            // if (/*sampling_timer_elapsed*/ HAL_GPIO_ReadPin(MPU6050_INTR_GPIO_Port, MPU6050_INTR_Pin) ==
            //   GPIO_PinState::GPIO_PIN_SET) {
            auto const& [ax, ay, az]{mpu6050_dmp.get_acceleration_scaled()};
            auto const& [gx, gy, gz]{mpu6050_dmp.get_rotation_scaled()};
            printf("accel x: %f, y: %f, z: %f\n\r", ax, ay, az);
            printf("gyro x: %f, y: %f, z: %f\n\r", gx, gy, gz);
            /*sampling_timer_elapsed = false;*/
            //}
        }
    }

    void KALMAN_TEST() noexcept
    {
        MPU6050 mpu6050{&hi2c1,
                        MPU6050::DevAddress::AD0_LOW,
                        MPU6050::GyroRange::GYRO_FS_250,
                        MPU6050::AccelRange::ACCEL_FS_2,
                        8000U};

        auto kalman{make_kalman(0.0f, 0.0f, 0.1f, 0.3f, 0.03f)};

        auto const sampling_time{1.0f / 8000.0f};

        while (true) {
            if (/*sampling_timer_elapsed*/ HAL_GPIO_ReadPin(MPU6050_INTR_GPIO_Port, MPU6050_INTR_Pin) ==
                GPIO_PinState::GPIO_PIN_SET) {
                float const roll = mpu6050.get_roll();
                float const gx = mpu6050.get_rotation_x_scaled();
                printf("mpu angle: %f, %f\n\r", gx, roll);
                printf("kalman angle: %f\n\r", kalman(gx, roll, sampling_time));
                /*sampling_timer_elapsed = false;*/
            }
        }
    }

    void ENCODER_TEST() noexcept
    {
        Encoder encoder{&htim3};
        Motor motor{&htim4, TIM_CHANNEL_1, L298N_IN1_GPIO_Port, L298N_IN1_Pin, L298N_IN3_Pin};

        using Direction = Motor::Direction;

        motor.set_compare_min();
        motor.set_fast_stop();

        while (true) {
            float const angle = encoder.get_angle().value();

            if (angle >= 350.0f) {
                motor.set_direction(Direction::BACKWARD);
            } else if (angle <= 10.0f) {
                motor.set_direction(Direction::FORWARD);
            }

            printf("encoder angle: %f\n\r", angle);
            HAL_Delay(100);
        }

        motor.set_compare_min();
        motor.set_fast_stop();
    }

    void DUTKIEWICZ_TEST() noexcept
    {
        while (true) {
            for (auto i{0}; i < 8; ++i)
                printf("DUTKIEWICZ SIGMA\t");
            printf("\n\r");
        }
    }

}; // namespace Tests

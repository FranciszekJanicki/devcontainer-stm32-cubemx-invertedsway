#include "tests.hpp"
#include "encoder.hpp"
#include "kalman.hpp"
#include "motor.hpp"
#include "mpu6050.hpp"
#include "regulators.hpp"
#include "system.hpp"
#include <cmath>
#include <cstdio>

using namespace InvertedSway;
using Kalman = Filters::Kalman<float>;

namespace Tests {

    void MOTOR_TEST(Motor motor) noexcept
    {
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

    void MOTOR_TEST_STARTING(Motor motor, float const motor_start_threshold) noexcept
    {
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

                if (std::abs(voltage) >= motor_start_threshold && std::abs(previous_voltage) < motor_start_threshold) {
                    motor.set_compare_max();
                    HAL_Delay(10);
                } else {
                    printf("setting compare %f\n\r", std::abs(voltage));
                    motor.set_compare_voltage(std::abs(voltage));
                }

                previous_voltage = voltage;
                HAL_Delay(1000);
            }
        }

        motor.set_compare_min();
        motor.set_fast_stop();
    }

    void KALMAN_TEST(MPU6050 mpu6050, Kalman kalman, float dt) noexcept
    {
        while (true) {
            if (HAL_GPIO_ReadPin(INTR_GPIO_Port, INTR_Pin) == GPIO_PinState::GPIO_PIN_SET) {
                float const roll = mpu6050.get_roll();
                float const gx = mpu6050.get_rotation_x_scaled();
                printf("mpu angle: %f, %f\n\r", gx, roll);
                printf("kalman angle: %f\n\r", kalman(gx, roll, dt));
            }
        }
    }

    void ENCODER_TEST(Encoder encoder, Motor motor) noexcept
    {
        using Direction = Motor::Direction;

        motor.set_compare_min();
        motor.set_fast_stop();

        while (true) {
            float const angle = encoder.get_angle();

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
            printf("DUTKIEWICZ\n\r");
        }
    }

}; // namespace Tests

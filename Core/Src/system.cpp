#include "system.hpp"
#include "kalman.hpp"
#include "l298n.hpp"
#include "main.h"
#include "mpu6050.hpp"
#include "regulators.hpp"
#include <cstdio>
#include <functional>
#include <memory>
#include <utility>
#include <variant>

using namespace InvertedSway;
using Value = System::Value;
using Kalman = System::Kalman;
using Regulator = System::Regulator;

namespace InvertedSway {

    Value System::angle_to_voltage(Value const angle) noexcept
    {
        return SWAY_MASS_KG * EARTH_ACCELERATION * MOTOR_RESISTANCE * std::sin(angle) / MOTOR_VELOCITY_CONSTANT;
    }

    Value System::voltage_to_angle(Value const voltage) noexcept
    {
        return std::asin((voltage * MOTOR_VELOCITY_CONSTANT) / (SWAY_MASS_KG * EARTH_ACCELERATION * MOTOR_RESISTANCE));
    }

    System::System(MPU6050&& mpu6050, L298N&& l298n, Kalman&& kalman, Regulator&& regulator, Encoder&& encoder) noexcept
        :
        mpu6050_{std::forward<MPU6050>(mpu6050)},
        l298n_{std::forward<L298N>(l298n)},
        kalman_{std::forward<Kalman>(kalman)},
        regulator_{std::forward<Regulator>(regulator)},
        encoder_{std::forward<Encoder>(encoder)}
    {
        this->initialize();
    }

    System::~System() noexcept
    {
        this->deinitialize();
    }

    void System::operator()(Value const input_angle, Value const dt) noexcept
    {
        this->set_angle(this->get_control_angle(this->get_error_angle(input_angle, dt), dt));
    }

    Value System::get_measured_angle(Value const dt) noexcept
    {
        if (HAL_GPIO_ReadPin(MPU6050_INTR_GPIO_Port, MPU6050_INTR_Pin) == GPIO_PinState::GPIO_PIN_SET) {
            this->roll_ = this->mpu6050_.get_roll();
            this->gx_ = this->mpu6050_.get_rotation_x_scaled();
        }
        // printf("mpu angle: %f, %f\n\r", this->gx_, this->roll_);
        // printf("kalman angle: %f\n\r", this->output_signal_);
        // printf("encoder angle: %f\n\r", this->encoder_.get_angle().value());
        return this->kalman_(this->gx_, this->roll_, dt);
    }

    Value System::get_error_angle(Value const input_angle, Value const dt) noexcept
    {
        return input_angle - this->get_measured_angle(dt);
    }

    Value System::get_control_angle(Value const error_angle, Value const dt) noexcept
    {
#if defined(REGULATOR_PTR)
        if (this->regulator_ != nullptr) {
            return std::invoke(*this->regulator_, this->error_signal_, this->dt_);
        }
        std::unreachable();
#elif defined(REGULATOR_VARIANT)
        if (!this->regulator_.valueless_by_exception()) {
            return std::visit([error_angle, dt]<typename Regulator>(
                                  Regulator&& regulator) { return std::invoke(regulator, error_angle, dt); },
                              this->regulator_);
        }
        std::unreachable();
#elif defined(REGULATOR_LAMBDA)
        if (this->regulator_) {
            return std::invoke(this->regulator_, error_angle, dt);
        }
        std::unreachable();
#endif
    }

    void System::set_angle(Value const control_angle) noexcept
    {
        this->try_motor_boost(control_angle);
        this->set_direction(control_angle);
        this->set_voltage(control_angle);
    }

    void System::set_direction(Value const control_angle) const noexcept
    {
        if (control_angle > 0.0f) {
            this->l298n_.set_forward(L298N::Channel::CHANNEL1);
            // printf("setting motor forward\n\r");
        } else if (control_angle < 0.0f) {
            this->l298n_.set_backward(L298N::Channel::CHANNEL1);
            // printf("setting motor backward\n\r");
        } else {
            this->l298n_.set_soft_stop(L298N::Channel::CHANNEL1);
            // printf("setting motor stop\n\r");
        }
    }

    void System::set_voltage(Value const control_angle) const noexcept
    {
        this->l298n_.set_compare_voltage(L298N::Channel::CHANNEL1, angle_to_voltage(std::abs(control_angle)));
    }

    void System::try_motor_boost(Value const control_angle) noexcept
    {
        if (std::exchange(this->last_control_voltage_, angle_to_voltage(control_angle)) < MOTOR_START_THRESHOLD_V &&
            angle_to_voltage(control_angle) >= MOTOR_START_THRESHOLD_V) {
            this->set_voltage(MAX_CONTROL_SIGNAL_V);
            HAL_Delay(10);
        }
    }

    void System::initialize() noexcept
    {
        this->l298n_.set_fast_stop(L298N::Channel::CHANNEL1);
        this->l298n_.set_compare_min(L298N::Channel::CHANNEL1);
    }

    void System::deinitialize() noexcept
    {
        this->l298n_.set_fast_stop(L298N::Channel::CHANNEL1);
        this->l298n_.set_compare_min(L298N::Channel::CHANNEL1);
    }

}; // namespace InvertedSway
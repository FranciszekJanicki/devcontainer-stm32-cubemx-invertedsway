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

    void System::balance_sway(const Value angle, const Value dt) noexcept
    {
        this->update_dt(dt);
        this->update_input_signal(angle);
        this->update_output_signal();
        this->update_error_signal();
        this->update_control_signal();
        this->update_direction();
        this->update_compare();
    }

    void System::operator()(const Value angle, const Value dt) noexcept
    {
        this->balance_sway(angle, dt);
    }

    void System::update_dt(const Value dt) noexcept
    {
        this->dt_ = dt;
        printf("sampling time: %f", this->dt_);
    }

    void System::update_input_signal(const Value input_signal) noexcept
    {
        this->input_signal_ = input_signal;
        printf("input angle: %f\n\r", this->input_signal_);
    }

    void System::update_output_signal() noexcept
    {
        if (HAL_GPIO_ReadPin(INTR_GPIO_Port, INTR_Pin) == GPIO_PinState::GPIO_PIN_SET) {
            this->roll_ = this->mpu6050_.get_roll();
            this->gx_ = this->mpu6050_.get_rotation_x_scaled();
        }
        printf("mpu angle: %f, %f\n\r", this->gx_, this->roll_);

        this->output_signal_ = this->kalman_(this->gx_, this->roll_, this->dt_);
        printf("kalman angle: %f\n\r", this->output_signal_);

        printf("encoder angle: %f\n\r", this->encoder_.get_angle());
    }

    void System::update_error_signal() noexcept
    {
        this->error_signal_ = this->input_signal_ - this->output_signal_;
        printf("error angle: %f\n\r", this->error_signal_);
    }

    void System::update_control_signal() noexcept
    {
#if defined(REGULATOR_PTR)

        if (this->regulator_ != nullptr) {
            this->control_signal_ = std::invoke(*this->regulator_, this->error_signal_, this->dt_);
        }
#elif defined(REGULATOR_VARIANT)
        if (!this->regulator_.valueless_by_exception()) {
            this->control_signal_ = std::visit(
                [this]<typename Regulator>(Regulator&& regulator) {
                    return std::invoke(regulator, this->error_signal_, this->dt_);
                },
                this->regulator_);
        }
#elif defined(REGULATOR_LAMBDA)
        if (this->regulator_) {
            this->control_signal_ = std::invoke(this->regulator_, this->error_signal_, this->dt_);
        }
#endif
        printf("regulated angle: %f\n\r", this->control_signal_);
    }

    Value System::angle_to_voltage(const Value angle) noexcept
    {
        return SWAY_MASS_KG * EARTH_ACCELERATION * MOTOR_RESISTANCE * std::sin(angle) / MOTOR_VELOCITY_CONSTANT;
    }

    Value System::voltage_to_angle(const Value voltage) noexcept
    {
        return std::asin((voltage * MOTOR_VELOCITY_CONSTANT) / (SWAY_MASS_KG * EARTH_ACCELERATION * MOTOR_RESISTANCE));
    }

    void System::update_direction() noexcept
    {
        if (this->error_signal_ >= 0) {
            this->l298n_.set_forward(L298N::Channel::CHANNEL1);
            printf("setting motor forward\n\r");
        } else if (this->error_signal_ <= 0) {
            this->l298n_.set_backward(L298N::Channel::CHANNEL1);
            printf("setting motor backward\n\r");
        } else {
            this->l298n_.set_fast_stop(L298N::Channel::CHANNEL1);
            printf("setting motor stop\n\r");
        }
    }

    void System::update_compare() noexcept
    {
        if (this->last_control_signal_ < voltage_to_angle(MOTOR_START_THRESHOLD_V) &&
            angle_to_voltage(this->control_signal_) > voltage_to_angle(MOTOR_START_THRESHOLD_V)) {
            this->set_angle(voltage_to_angle(MAX_CONTROL_SIGNAL_V));
        }
        this->set_angle(this->control_signal_);
        this->last_control_signal_ = this->control_signal_;
    }

    void System::initialize() noexcept
    {
        this->set_angle(0.0);
    }

    void System::deinitialize() noexcept
    {
        this->set_angle(0.0);
    }

    void System::set_angle(const Value angle) noexcept
    {
        this->l298n_.set_compare_voltage(L298N::Channel::CHANNEL1, angle_to_voltage(angle));
    }

}; // namespace InvertedSway
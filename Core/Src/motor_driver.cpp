#include "motor_driver.hpp"
#include "main.h"
#include "regulators.hpp"
#include <cstdio>
#include <functional>
#include <memory>
#include <utility>
#include <variant>

using namespace InvertedSway;
using Value = MotorDriver::Value;
using Regulator = MotorDriver::Regulator;

namespace InvertedSway {

    Value MotorDriver::clamp_speed(Value const speed) noexcept
    {
        return std::clamp(speed, MIN_SPEED_RPM, MAX_SPEED_RPM);
    }

    Value MotorDriver::speed_to_voltage(Value const speed) noexcept
    {
        return (clamp_speed(speed) - MIN_SPEED_RPM) * (MAX_VOLTAGE_V - MIN_VOLTAGE_V) /
                   (MAX_SPEED_RPM - MIN_SPEED_RPM) +
               MIN_VOLTAGE_V;
    }

    MotorDriver::MotorDriver(Regulator&& regulator, Motor&& motor, Encoder&& encoder) noexcept
    {
        this->initialize();
    }

    MotorDriver::~MotorDriver() noexcept
    {
        this->deinitialize();
    }

    void MotorDriver::operator()(Value const input_speed, Value const dt) noexcept
    {
        this->set_speed(this->get_control_speed(this->get_error_speed(input_speed, dt), dt));
    }

    void MotorDriver::set_speed(Value const control_speed) const noexcept
    {
        this->set_direction(control_speed);
        this->set_voltage(control_speed);
    }

    void MotorDriver::set_direction(Value const control_speed) const noexcept
    {
        if (control_speed > 0.0f) {
            this->motor_.set_forward();
        } else if (control_speed < 0.0f) {
            this->motor_.set_backward();
        }
        this->motor_.set_soft_stop();
    }

    void MotorDriver::set_voltage(Value const control_speed) const noexcept
    {
        this->motor_.set_compare_voltage(speed_to_voltage(std::abs(control_speed)));
    }

    Value MotorDriver::get_measured_speed(Value const dt) noexcept
    {
        if (auto speed{this->encoder_.get_angular_speed(dt)}; speed.has_value()) {
            return speed.value();
        }
        std::unreachable();
    }

    Value MotorDriver::get_control_speed(Value const error_speed, Value const dt) noexcept
    {
        if (!this->regulator_.valueless_by_exception()) {
            return std::visit(
                [error_speed, dt]<typename Regulator>(Regulator& regulator) { return regulator(error_speed, dt); },
                this->regulator_);
        }
        std::unreachable();
    }

    Value MotorDriver::get_error_speed(Value const input_speed, Value const dt) noexcept
    {
        return input_speed - this->get_measured_speed(dt);
    }

    void MotorDriver::initialize() const noexcept
    {
        this->motor_.set_fast_stop();
        this->motor_.set_compare_min();
    }

    void MotorDriver::deinitialize() const noexcept
    {
        this->motor_.set_fast_stop();
        this->motor_.set_compare_min();
    }

}; // namespace InvertedSway

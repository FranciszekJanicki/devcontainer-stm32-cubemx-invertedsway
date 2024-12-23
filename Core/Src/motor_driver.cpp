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
using Direction = MotorDriver::Direction;
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

    Direction MotorDriver::speed_to_direction(Value const speed) noexcept
    {
        if (speed > 0.0f) {
            return Direction::FORWARD;
        } else if (speed < 0.0f) {
            return Direction::BACKWARD;
        } else {
            return Direction::SOFT_STOP;
        }
    }

    MotorDriver::MotorDriver(Regulator&& regulator, Motor&& motor, Encoder&& encoder) noexcept :
        regulator_{std::forward<Regulator>(regulator)},
        motor_{std::forward<Motor>(motor)},
        encoder_{std::forward<Encoder>(encoder)}
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
        this->motor_.set_direction(speed_to_direction(control_speed));
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

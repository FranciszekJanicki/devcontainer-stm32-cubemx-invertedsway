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
using Error = MotorDriver::Error;
using Expected = MotorDriver::Expected;
using Unexpected = MotorDriver::Unexpected;

namespace InvertedSway {

    Direction MotorDriver::speed_to_direction(Value const speed) noexcept
    {
        if (speed > 0.0f) {
            return Direction::FORWARD;
        } else if (speed < 0.0f) {
            return Direction::BACKWARD;
        }
        return Direction::SOFT_STOP;
    }

    MotorDriver::MotorDriver(Regulator&& regulator, Motor&& motor, Encoder&& encoder) noexcept
    {
        this->initialize();
    }

    MotorDriver::~MotorDriver() noexcept
    {
        this->deinitialize();
    }

    Error MotorDriver::operator()(Direction const input_direction, Value const input_speed, Value const dt) noexcept
    {
        auto const error_speed{this->get_error_speed(input_speed, dt)};
        auto const control_speed{this->get_control_speed(error_speed, dt)};

        if (this->set_direction(control_speed) != Error::OK) {
            return Error::MOTOR_FAIL;
        }
        if (this->set_speed(control_speed) != Error::OK) {
            return Error::MOTOR_FAIL;
        }
        return Error::OK;
    }

    Error MotorDriver::set_direction(Value const control_speed) noexcept
    {
        this->motor_.set_direction(speed_to_direction(control_speed));
    }

    Error MotorDriver::set_speed(Value const control_speed) noexcept
    {
        this->motor_.set_compare_speed(std::abs(control_speed));
    }

    Expected MotorDriver::get_speed(Value const dt) noexcept
    {
        return this->encoder_.get_angular_speed(dt);
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
        if (auto speed{this->get_speed(dt)}; speed.has_value()) {
            return input_speed - speed.value();
        }
        std::unreachable();
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

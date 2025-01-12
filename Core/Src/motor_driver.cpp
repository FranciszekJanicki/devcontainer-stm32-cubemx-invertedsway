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

    MotorDriver::MotorDriver(Regulator&& regulator,
                             Motor&& motor,
                             Encoder&& encoder,
                             SpeedToVoltage const speed_to_voltage,
                             SpeedToDirection const speed_to_direction) noexcept :
        regulator_{std::forward<Regulator>(regulator)},
        motor_{std::forward<Motor>(motor)},
        encoder_{std::forward<Encoder>(encoder)},
        speed_to_voltage_{speed_to_voltage},
        speed_to_direction_{speed_to_direction}
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
        this->motor_.set_direction(std::invoke(this->speed_to_direction_, control_speed));
    }

    void MotorDriver::set_voltage(Value const control_speed) const noexcept
    {
        this->motor_.set_compare_voltage(std::invoke(this->speed_to_voltage_, std::abs(control_speed)));
    }

    Value MotorDriver::get_measured_speed(Value const dt) noexcept
    {
        return this->encoder_.get_speed_degrees(dt);
    }

    Value MotorDriver::get_control_speed(Value const error_speed, Value const dt) noexcept
    {
        return std::invoke(this->regulator_, error_speed, dt);
    }

    Value MotorDriver::get_error_speed(Value const input_speed, Value const dt) noexcept
    {
        return input_speed - this->get_measured_speed(dt);
    }

    void MotorDriver::initialize() const noexcept
    {
        this->motor_.set_fast_stop();
        this->motor_.set_voltage_min();
    }

    void MotorDriver::deinitialize() const noexcept
    {
        this->motor_.set_fast_stop();
        this->motor_.set_voltage_min();
    }

}; // namespace InvertedSway

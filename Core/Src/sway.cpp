#include "sway.hpp"
#include "kalman.hpp"
#include "l298n.hpp"
#include "main.h"
#include "mpu_dmp.hpp"
#include "regulators.hpp"
#include <cstdio>
#include <functional>
#include <memory>
#include <utility>
#include <variant>

using namespace InvertedSway;
using Value = Sway::Value;
using Kalman = Sway::Kalman;
using Direction = Sway::Direction;
using Regulator = Sway::Regulator;

namespace InvertedSway {

    Value Sway::angle_to_voltage(Value const angle) noexcept
    {
        // return SWAY_MASS_KG * EARTH_ACCELERATION * MOTOR_RESISTANCE * std::sin(angle) / MOTOR_VELOCITY_CONSTANT;
        return angle;
    }

    Direction Sway::angle_to_direction(Value const angle) noexcept
    {
        if (angle < 0.0F) {
            return Direction::FORWARD;
        } else {
            return Direction::BACKWARD;
        }
    }

    Sway::Sway(MPU_DMP&& mpu_dmp, L298N&& l298n, Kalman&& kalman, Regulator&& regulator, Encoder&& encoder) noexcept :
        mpu_dmp_{std::forward<MPU_DMP>(mpu_dmp)},
        l298n_{std::forward<L298N>(l298n)},
        kalman_{std::forward<Kalman>(kalman)},
        regulator_{std::forward<Regulator>(regulator)},
        encoder_{std::forward<Encoder>(encoder)}
    {
        this->initialize();
    }

    Sway::~Sway() noexcept
    {
        this->deinitialize();
    }

    void Sway::operator()(Value const input_angle, Value const dt) noexcept
    {
        this->set_angle(this->get_control_angle(this->get_error_angle(input_angle, dt), dt));
    }

    Value Sway::get_measured_angle(Value const dt) noexcept
    {
        return this->mpu_dmp_.get_pitch();
    }

    Value Sway::get_error_angle(Value const input_angle, Value const dt) noexcept
    {
        return input_angle - this->get_measured_angle(dt);
    }

    Value Sway::get_control_angle(Value const error_angle, Value const dt) noexcept
    {
        return std::invoke(this->regulator_, error_angle, dt);
    }

    void Sway::set_angle(Value const control_angle) noexcept
    {
        this->set_direction(control_angle);
        // this->try_motor_boost(control_angle);
        this->set_voltage(control_angle);
    }

    void Sway::set_direction(Value const control_angle) const noexcept
    {
        this->l298n_.set_direction(L298N::Channel::CHANNEL1, angle_to_direction(control_angle));
    }

    void Sway::set_voltage(Value const control_angle) const noexcept
    {
        this->l298n_.set_voltage(L298N::Channel::CHANNEL1, angle_to_voltage(std::abs(control_angle)));
    }

    void Sway::try_motor_boost(Value const control_angle) noexcept
    {
        auto const control_voltage{angle_to_voltage(std::abs(control_angle))};
        if (std::exchange(this->last_control_voltage_, control_voltage) < MOTOR_START_THRESHOLD_V &&
            control_voltage >= MOTOR_START_THRESHOLD_V) {
            this->l298n_.set_voltage_max(L298N::Channel::CHANNEL1);
            HAL_Delay(10);
        }
    }

    void Sway::initialize() noexcept
    {
        this->l298n_.reset();
    }

    void Sway::deinitialize() noexcept
    {
        this->l298n_.reset();
    }

}; // namespace InvertedSway
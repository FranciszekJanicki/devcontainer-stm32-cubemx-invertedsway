#include "system.hpp"
#include "kalman.hpp"
#include "l298n.hpp"
#include "mpu6050.hpp"
#include "regulator.hpp"
#include <functional>
#include <memory>
#include <utility>
#include <variant>

namespace InvertedSway {

    using Value = System::Value;
    System::System(MPU6050&& mpu6050, L298N&& l298n, Kalman<Value>&& kalman, Regulator&& regulator, Encoder&& encoder) :
        mpu6050_{std::forward<MPU6050>(mpu6050)},
        l298n_{std::forward<L298N>(l298n)},
        kalman_{std::forward<Kalman<Value>>(kalman)},
        regulator_{std::forward<Regulator>(regulator)},
        encoder_{std::forward<Encoder>(encoder)}
    {
        // do setup
    }

    System::~System() noexcept
    {
        // undo setup
    }

    void System::operator()(const Value input_angle) noexcept
    {
        this->update_output_signal();
        this->update_input_signal(input_angle);
        this->update_error_signal();
        this->update_control_signal();
    }

    void System::update_output_signal() noexcept
    {
        this->output_signal_ = this->kalman_(this->mpu6050_.get_gyroscope_scaled().x,
                                             this->mpu6050_.get_accelerometer_scaled().x,
                                             this->dt_);
        [[maybe_unused]] const auto encoder_value{this->encoder_.get_angle()};
    }

    void System::update_input_signal(const Value input_signal) noexcept
    {
        this->input_signal_ = input_signal;
    }

    void System::update_error_signal() noexcept
    {
        this->update_output_signal();
        this->error_signal_ = this->input_signal_ - this->output_signal_;
    }

    void System::update_control_signal() noexcept
    {
#if defined(REGULATOR_PTR)

        if (this->regulator_ != nullptr) {
            this->control_signal_ = *this->regulator_(this->error_signal_, this->dt_);
        }
#elif defined(REGULATOR_VARIANT)
        if (!this->regulator_.valueless_by_exception()) {
            this->control_signal_ =
                std::visit([this](auto& regulator) { return regulator(this->error_signal_, this->dt_) },
                           this->regulator_);
        }
#elif defined(REGULATOR_LAMBDA)
        if (this->regulator_) {
            this->control_signal_ = this->regulator_(this->error_signal_, this->dt_);
        }
#endif
    }

    Value System::angle_to_voltage(const Value angle) noexcept
    {
        return SWAY_MASS * EARTH_ACCELERATION * MOTOR_RESISTANCE * std::sin(angle) / MOTOR_VELOCITY_CONSTANT;
    }

    void System::update_direction() noexcept
    {
        if (this->error_signal_ >= 0) {
            this->l298n_.set_direction(L298N::Channel::CHANNEL1, L298N::Direction::FORWARD);
        } else if (this->error_signal_ <= 0) {
            this->l298n_.set_direction(L298N::Channel::CHANNEL1, L298N::Direction::BACKWARD);
        } else {
            this->l298n_.set_direction(L298N::Channel::CHANNEL1, L298N::Direction::FAST_STOP);
        }
    }

    void System::update_compare() noexcept
    {
        this->l298n_.set_compare_voltage(L298N::Channel::CHANNEL1, angle_to_voltage(this->control_signal_));
    }

}; // namespace InvertedSway
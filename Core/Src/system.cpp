#include "system.hpp"
#include "kalman.hpp"
#include "l298n.hpp"
#include "main.h"
#include "mpu6050.hpp"
#include "regulator.hpp"
#include <functional>
#include <memory>
#include <utility>
#include <variant>

namespace InvertedSway {

    using Value = System::Value;
    using KalmanFilter = System::KalmanFilter;
    using RegulatorBlock = System::RegulatorBlock;

    System::System(MPU6050&& mpu6050,
                   L298N&& l298n,
                   KalmanFilter&& kalman,
                   RegulatorBlock&& regulator,
                   Encoder&& encoder) noexcept :
        mpu6050_{std::forward<MPU6050>(mpu6050)},
        l298n_{std::forward<L298N>(l298n)},
        kalman_{std::forward<KalmanFilter>(kalman)},
        regulator_{std::forward<RegulatorBlock>(regulator)},
        encoder_{std::forward<Encoder>(encoder)}
    {
        // do stuff with kalman
    }

    System::System(const MPU6050& mpu6050,
                   const L298N& l298n,
                   const KalmanFilter& kalman,
                   const RegulatorBlock& regulator,
                   const Encoder& encoder) :
        mpu6050_{mpu6050}, l298n_{l298n}, kalman_{kalman}, regulator_{regulator}, encoder_{encoder}
    {
        // do stuff with kalman
    }

    System::~System() noexcept
    {
        // do stuff with kalman
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
    }

    void System::update_input_signal(const Value input_signal) noexcept
    {
        this->input_signal_ = input_signal;
    }

    void System::update_output_signal() noexcept
    {
        if (HAL_GPIO_ReadPin(INTR_GPIO_Port, INTR_Pin) == GPIO_PinState::GPIO_PIN_SET) {
            this->ax_ = this->mpu6050_.get_accelerometer_scaled().x;
            this->gx_ = this->mpu6050_.get_gyroscope_scaled().x;
        }
        [[maybe_unused]] const auto encoder_angle{this->encoder_.get_angle()};
        this->output_signal_ = this->kalman_(this->gx_, this->ax_, this->dt_);
    }

    void System::update_error_signal() noexcept
    {
        this->error_signal_ = this->input_signal_ - this->output_signal_;
    }

    void System::update_control_signal() noexcept
    {
#if defined(REGULATOR_PTR)

        if (this->regulator_ != nullptr) {
            this->control_signal_ = std::invoke(*this->regulator_, this->error_signal_, this->dt_);
        }
#elif defined(REGULATOR_VARIANT)
        if (!this->regulator_.valueless_by_exception()) {
            this->control_signal_ =
                std::visit([this](auto& regulator) { return regulator(this->error_signal_, this->dt_); },
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
            this->l298n_.set_forward(L298N::Channel::CHANNEL1);
        } else if (this->error_signal_ <= 0) {
            this->l298n_.set_backward(L298N::Channel::CHANNEL1);
        } else {
            this->l298n_.set_fast_stop(L298N::Channel::CHANNEL1);
        }
    }

    void System::update_compare() noexcept
    {
        this->l298n_.set_compare_voltage(L298N::Channel::CHANNEL1, angle_to_voltage(this->control_signal_));
    }

}; // namespace InvertedSway
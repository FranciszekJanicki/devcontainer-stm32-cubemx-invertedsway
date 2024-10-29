#ifndef INVERTED_SWAY_HPP
#define INVERTED_SWAY_HPP

#include "filters.hpp"
#include "kalman_filter.hpp"
#include "l298n.hpp"
#include "mpu6050.hpp"
#include "regulator.hpp"
#include <functional>
#include <memory>
#include <utility>
#include <variant>

#define REGULATOR_LAMBDA

struct InvertedSway {
public:
    using Value = MPU6050::Scaled;
    using Kalman = Kalman<Value, Value>;
#if defined(REGULATOR_PTR)
    using Regulator = RegulatorPtr<Value>;
#elif defined(REGULATOR_VARIANT)
    using Regulator = RegulatorVariant<Value>;
#elif defined(REGULATOR_LAMBDA)
    using Regulator = RegulatorFunction<Value>;
#endif

    InvertedSway(MPU6050&& mpu6050, L298N&& l298n, Kalman&& kalman, Regulator&& regulator) :
        mpu6050_{std::forward<MPU6050>()},
        l298n_{std::forward<L298N>(l298n)},
        kalman_{std::forward<Kalman>(kalman)},
        regulator_{std::forward<Regulator>(regulator)}
    {
        // do setup
    }

    InvertedSway(const InvertedSway&) = delete;
    InvertedSway(InvertedSway&&) noexcept = delete;

    InvertedSway& operator=(const InvertedSway&) = delete;
    InvertedSway& operator=(InvertedSway&&) noexcept = delete;

    ~InvertedSway() noexcept
    {
        // undo setup
    }

    void operator()(const Value angle) noexcept
    {
        update_output_signal();
        update_input_signal(angle);
        update_error_signal();
        update_control_signal();
    }

    void update_output_signal() noexcept
    {
        output_signal_ = kalman_filter_(mpu6050_.get_gyroscope_scaled(), mpu6050_.get_accelerometer_scaled(), dt_);
    }

    void update_input_signal(const Value input_signal) noexcept
    {
        input_signal_ = input_signal;
    }

    void update_error_signal() noexcept
    {
        update_output_signal();
        update_input_signal();
        error_signal_ = input_signal_ - output_signal_;
    }

    void update_control_signal() noexcept
    {
#if defined(REGULATOR_PTR)

        if (regulator_ != nullptr) {
            control_signal_ = *regulator_(error_signal_, dt_);
        }
#elif defined(REGULATOR_VARIANT)
        if (!regulator_.valueless_by_exception()) {
            control_signal_ =
                std::visit([this](auto& regulator) { return regulator(this->error_signal_, this->dt_) }, regulator_);
        }
#elif defined(REGULATOR_LAMBDA)
        if (regulator_) {
            control_signal_ = regulator_(error_signal_, dt_);
        }
#endif
    }

private:
    static angle_to_voltage(const Value angle) noexcept
    {
        return SWAY_MASS * EARTH_ACCELERATION * MOTOR_RESISTANCE * std::sin(angle) / MOTOR_VELOCITY_CONSTANT;
    }

    static constexpr Value MOTOR_RESISTANCE{0};
    static constexpr Value EARTH_ACCELERATION{9.81};
    static constexpr Value SWAY_MASS{0};
    static constexpr Value MOTOR_VELOCITY_CONSTANT{0};

    void update_direction() noexcept
    {
        if (error_signal_ >= 0) {
            l298n_.set_direction(L298N::Direction::FORWARD);
        } else if (error_signal_ <= 0) {
            l298n_.set_direction(L298N::Direction::BACKWARD);
        } else {
            l298n_.set_direction(L298N::Direction::FAST_STOP);
        }
    }

    void update_compare() noexcept
    {
        l298n_.set_compare_voltage(angle_to_voltage(control_signal_));
    }

    bool initialized_{false};

    Value dt_{1};

    Value error_signal_{};
    Value input_signal_{};
    Value output_signal_{};
    Value control_signal_{};

    MPU6050 mpu6050_{};
    L298N l298n_{};
    Kalman kalman_{};
    Regulator regulator_{};
};

#endif // INVERTED_SWAY_HPP
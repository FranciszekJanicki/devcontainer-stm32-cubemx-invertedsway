#ifndef INVERTED_SWAY_HPP
#define INVERTED_SWAY_HPP

#include "filters.hpp"
#include "kalman_filter.hpp"
#include "l298n.hpp"
#include "mpu6050.hpp"
#include "regulator.hpp"
#include <memory>
#include <utility>

class InvertedSway {
public:
    using Time = MPU6050::Scaled;
    using Angle = MPU6050::Scaled;
    using Kalman = Kalman<Angle, Time>;
    using Regulator = std::function<Angle(Angle, Time)>;

    template <typename CallableRegulator>
    InvertedSway(MPU6050&& mpu6050, L298N&& l298n, Kalman&& kalman, CallableRegulator&& callable_regulator) noexcept :
        mpu6050_{std::forward<MPU6050>()},
        l298n_{std::forward<L298N>(l298n)},
        kalman_{std::forward<Kalman>(kalman)},
        regulator_{std::forward<CallableRegulator>(callable_regulator)}
    {
        // do setup
    }

    ~InvertedSway() noexcept
    {
        // undo setup
    }

    void update_output_signal() noexcept
    {
        output_signal_ = kalman_filter_(mpu6050_.get_gyroscope_scaled(), mpu6050_.get_accelerometer_scaled(), dt_);
    }

    void update_input_signal(const Angle input_signal) noexcept
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
        if (regulator_) {
            control_signal_ = regulator_(error_signal_, dt_);
        }
    }

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

private:
    static angle_to_voltage(const Angle angle) noexcept
    {
        return angle;
    }

    bool initialized_{false};

    Time dt_{1};

    Angle error_signal_{};
    Angle input_signal_{};
    Angle output_signal_{};
    Angle control_signal_{};

    MPU6050 mpu6050_{};
    L298N l298n_{};
    Kalman kalman_{};
    Regulator regulator_{};
};

#endif // INVERTED_SWAY_HPP
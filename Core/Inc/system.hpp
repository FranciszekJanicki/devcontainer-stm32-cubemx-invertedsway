#ifndef SYSTEM_HPP
#define SYSTEM_HPP

#include "encoder.hpp"
#include "kalman.hpp"
#include "l298n.hpp"
#include "mpu6050.hpp"
#include "regulator.hpp"
#include <functional>
#include <memory>
#include <utility>
#include <variant>

namespace InvertedSway {

    struct System {
    public:
        using Value = MPU6050::Scaled;

#if defined(REGULATOR_PTR)
        using Regulator = RegulatorPtr<Value>;
#elif defined(REGULATOR_VARIANT)
        using Regulator = RegulatorVariant<Value>;
#elif defined(REGULATOR_LAMBDA)
        using Regulator = RegulatorFunction<Value>;
#endif

        System(MPU6050&& mpu6050, L298N&& l298n, Kalman<Value>&& kalman, Regulator&& regulator, Encoder&& encoder);

        System(const System& other) = delete;
        System(System&& other) noexcept = delete;

        System& operator=(const System& other) = delete;
        System& operator=(System&& other) noexcept = delete;

        ~System() noexcept;

        void operator()(const Value angle) noexcept;

    private:
        static Value angle_to_voltage(const Value angle) noexcept;

        void update_output_signal() noexcept;
        void update_input_signal(const Value input_signal) noexcept;
        void update_error_signal() noexcept;
        void update_control_signal() noexcept;

        static constexpr Value MOTOR_RESISTANCE{0};
        static constexpr Value EARTH_ACCELERATION{9.81};
        static constexpr Value SWAY_MASS{0};
        static constexpr Value MOTOR_VELOCITY_CONSTANT{0};

        void update_direction() noexcept;
        void update_compare() noexcept;

        bool initialized_{false};

        Value dt_{1};

        Value error_signal_{};
        Value input_signal_{};
        Value output_signal_{};
        Value control_signal_{};

        MPU6050 mpu6050_{};
        L298N l298n_{};
        Kalman<Value> kalman_{};
        Regulator regulator_{};
        Encoder encoder_{};
    };

}; // namespace InvertedSway

#endif // SYSTEM_HPP
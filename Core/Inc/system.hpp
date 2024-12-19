#ifndef SYSTEM_HPP
#define SYSTEM_HPP

#include "encoder.hpp"
#include "kalman.hpp"
#include "l298n.hpp"
#include "mpu6050.hpp"
#include "regulators.hpp"
#include <functional>
#include <memory>
#include <utility>
#include <variant>

namespace InvertedSway {

    struct System {
    public:
        using Value = std::float_t;
        using Kalman = Filters::Kalman<Value>;
        using Regulator = Regulators::Regulator<Value>;

        System() = delete;
        System(MPU6050&& mpu6050, L298N&& l298n, Kalman&& kalman, Regulator&& regulator, Encoder&& encoder) noexcept;

        System(System const& other) = delete;
        System(System&& other) noexcept = default;

        System& operator=(System const& other) = delete;
        System& operator=(System&& other) noexcept = default;

        ~System() noexcept;

        void balance_sway(Value const angle, Value const dt) noexcept;
        void operator()(Value const angle, Value const dt) noexcept;

    private:
        static Value voltage_to_angle(Value const voltage) noexcept;
        static Value angle_to_voltage(Value const angle) noexcept;

        static constexpr Value MOTOR_RESISTANCE{10.0f};
        static constexpr Value EARTH_ACCELERATION{9.81f};
        static constexpr Value SWAY_MASS_KG{0.1f};
        static constexpr Value MOTOR_VELOCITY_CONSTANT{1.0f};
        static constexpr Value MOTOR_START_THRESHOLD_V{2.0f};
        static constexpr Value MIN_CONTROL_SIGNAL_V{Motor::MIN_VOLTAGE_V};
        static constexpr Value MAX_CONTROL_SIGNAL_V{Motor::MAX_VOLTAGE_V};

        void update_dt(Value const dt) noexcept;
        void update_input_signal(Value const input_signal) noexcept;
        void update_output_signal() noexcept;
        void update_error_signal() noexcept;
        void update_control_signal() noexcept;

        void update_direction() noexcept;
        void update_compare() noexcept;

        void deinitialize() noexcept;
        void initialize() noexcept;

        void set_angle(Value const angle) noexcept;

        Value dt_{};
        Value gx_{};
        Value roll_{};
        Value error_signal_{};
        Value input_signal_{};
        Value output_signal_{};
        Value control_signal_{};
        Value last_control_signal_{};

        MPU6050 mpu6050_{};
        L298N l298n_{};
        Kalman kalman_{};
        Regulator regulator_{};
        Encoder encoder_{};
    };
}; // namespace InvertedSway

#endif // SYSTEM_HPP
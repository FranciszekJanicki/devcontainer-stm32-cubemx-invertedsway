#ifndef SWAY_HPP
#define SWAY_HPP

#include "encoder.hpp"
#include "kalman.hpp"
#include "l298n.hpp"
#include "mpu_dmp.hpp"
#include "regulators.hpp"
#include <functional>
#include <memory>
#include <utility>
#include <variant>

namespace InvertedSway {

    struct Sway {
    public:
        using Value = std::float_t;
        using Direction = L298N::Direction;
        using Kalman = Filters::Kalman<Value>;
        using Regulator = Regulators::PID<Value>;

        Sway() = delete;
        Sway(MPU_DMP&& mpu_dmp, L298N&& l298n, Kalman&& kalman, Regulator&& regulator, Encoder&& encoder) noexcept;

        Sway(Sway const& other) = delete;
        Sway(Sway&& other) noexcept = delete;

        Sway& operator=(Sway const& other) = delete;
        Sway& operator=(Sway&& other) noexcept = delete;

        ~Sway() noexcept;

        void operator()(Value const input_angle, Value const dt) noexcept;

    private:
        static Value angle_to_voltage(Value const angle) noexcept;
        static Direction angle_to_direction(Value const angle) noexcept;

        static constexpr Value MOTOR_RESISTANCE{10.0F};
        static constexpr Value EARTH_ACCELERATION{9.81F};
        static constexpr Value SWAY_MASS_KG{0.1F};
        static constexpr Value MOTOR_VELOCITY_CONSTANT{1.0F};
        static constexpr Value MOTOR_START_THRESHOLD_V{2.0F};

        Value get_measured_angle(Value const dt) noexcept;
        Value get_control_angle(Value const error_angle, Value const dt) noexcept;
        Value get_error_angle(Value const input_angle, Value const dt) noexcept;

        void set_angle(Value const control_angle) noexcept;
        void set_direction(Value const control_angle) const noexcept;
        void set_voltage(Value const control_angle) const noexcept;
        void try_motor_boost(Value const control_angle) noexcept;

        void deinitialize() noexcept;
        void initialize() noexcept;

        Value last_control_voltage_{};

        MPU_DMP mpu_dmp_{};
        L298N l298n_{};
        Kalman kalman_{};
        Regulator regulator_{};
        Encoder encoder_{};
    };

}; // namespace InvertedSway

#endif // SWAY_HPP
#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include "encoder.hpp"
#include "motor.hpp"
#include "regulators.hpp"
#include <utility>

namespace InvertedSway {

    struct MotorDriver {
    public:
        using Value = Motor::Voltage;
        using Direction = Motor::Direction;
        using Regulator = Regulators::Regulator<Value>;

        static constexpr Value MAX_VOLTAGE_V{Motor::MAX_VOLTAGE_V};
        static constexpr Value MIN_VOLTAGE_V{Motor::MIN_VOLTAGE_V};

        static constexpr Value MIN_SPEED_RPM{0};
        static constexpr Value MAX_SPEED_RPM{1000};

        MotorDriver() noexcept = default;

        MotorDriver(Regulator&& regulator, Motor&& motor, Encoder&& encoder) noexcept;

        MotorDriver(MotorDriver const& other) noexcept = delete;
        MotorDriver(MotorDriver&& other) noexcept = default;

        MotorDriver& operator=(MotorDriver const& other) noexcept = delete;
        MotorDriver& operator=(MotorDriver&& other) noexcept = default;

        ~MotorDriver() noexcept;

        void operator()(Value const input_speed, Value const dt) noexcept;

    private:
        static Value speed_to_voltage(Value const speed) noexcept;
        static Value clamp_speed(Value const speed) noexcept;
        static Direction speed_to_direction(Value const speed) noexcept;

        void set_voltage(Value const control_speed) const noexcept;
        void set_speed(Value const control_speed) const noexcept;
        void set_direction(Value const control_speed) const noexcept;

        Value get_measured_speed(Value const dt) noexcept;
        Value get_control_speed(Value const error_speed, Value const dt) noexcept;
        Value get_error_speed(Value const input_speed, Value const dt) noexcept;

        void initialize() const noexcept;
        void deinitialize() const noexcept;

        Regulator regulator_{};
        Motor motor_{};
        Encoder encoder_{};
    };

}; // namespace InvertedSway

#endif // MOTOR_DRIVER_HPP
#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include "encoder.hpp"
#include "motor.hpp"
#include "regulators.hpp"
#include <utility>

namespace InvertedSway {

    struct MotorDriver {
    public:
        enum struct Error {
            MOTOR_FAIL,
            ENCODER_FAIL,
            OK,
            FAIL,
            INIT,
            DEINIT,
        };

        using Value = float;
        using Direction = Motor::Direction;
        using Regulator = Regulators::Regulator<Value>;
        using Expected = Encoder::ExpectedSpeed;
        using Unexpected = Encoder::Unexpected;

        MotorDriver() noexcept = default;

        MotorDriver(Regulator&& regulator, Motor&& motor, Encoder&& encoder) noexcept;

        MotorDriver(MotorDriver const& other) noexcept = delete;
        MotorDriver(MotorDriver&& other) noexcept = default;

        MotorDriver& operator=(MotorDriver const& other) noexcept = delete;
        MotorDriver& operator=(MotorDriver&& other) noexcept = default;

        ~MotorDriver() noexcept;

        [[nodiscard]] Error
        operator()(Direction const input_direction, Value const input_speed, Value const dt) noexcept;

    private:
        static Direction speed_to_direction(Value const speed) noexcept;

        Error set_speed(Value const control_speed) noexcept;
        Error set_direction(Value const control_speed) noexcept;

        Expected get_speed(Value const dt) noexcept;

        Value get_control_speed(Value const error_speed, Value const dt) noexcept;
        Value get_error_speed(Value const input_speed, Value const dt) noexcept;

        void initialize() const noexcept;
        void deinitialize() const noexcept;

        Direction last_direction_{};

        Regulator regulator_{};
        Motor motor_{};
        Encoder encoder_{};
    };

}; // namespace InvertedSway

#endif // MOTOR_DRIVER_HPP
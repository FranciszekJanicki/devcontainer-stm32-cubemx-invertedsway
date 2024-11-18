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
        using KalmanFilter = Kalman<Value>;
        using RegulatorBlock = Regulator<Value>;

        System(MPU6050&& mpu6050,
               L298N&& l298n,
               KalmanFilter&& kalman,
               RegulatorBlock&& regulator,
               Encoder&& encoder) noexcept;
        System(const MPU6050& mpu6050,
               const L298N& l298n,
               const KalmanFilter& kalman,
               const RegulatorBlock& regulator,
               const Encoder& encoder);

        System(const System& other) = delete;
        System(System&& other) noexcept = delete;

        System& operator=(const System& other) = delete;
        System& operator=(System&& other) noexcept = delete;

        ~System() noexcept;

        void balance_sway(const Value angle, const Value dt = MPU6050::SAMPLING_TIME_S) noexcept;
        void operator()(const Value angle, const Value dt = MPU6050::SAMPLING_TIME_S) noexcept;

    private:
        static Value angle_to_voltage(const Value angle) noexcept;

        void update_dt(const Value dt) noexcept;
        void update_input_signal(const Value input_signal) noexcept;
        void update_output_signal() noexcept;
        void update_error_signal() noexcept;
        void update_control_signal() noexcept;

        static constexpr Value MOTOR_RESISTANCE{0};
        static constexpr Value EARTH_ACCELERATION{9.81};
        static constexpr Value SWAY_MASS{0};
        static constexpr Value MOTOR_VELOCITY_CONSTANT{0};

        void update_direction() noexcept;
        void update_compare() noexcept;

        Value dt_{MPU6050::SAMPLING_TIME_S};

        Value error_signal_{};
        Value input_signal_{};
        Value output_signal_{};
        Value control_signal_{};

        MPU6050 mpu6050_{};
        L298N l298n_{};
        KalmanFilter kalman_{};
        RegulatorBlock regulator_{};
        Encoder encoder_{};

        Value gx_{};
        Value ax_{};
    };

}; // namespace InvertedSway

#endif // SYSTEM_HPP
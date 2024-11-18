#ifndef L298N_HPP
#define L298N_HPP

#include "common.hpp"
#include "motor.hpp"
#include <array>
#include <cmath>
#include <cstdint>
#include <expected>
#include <tuple>
#include <utility>

namespace InvertedSway {

    struct L298N {
    public:
        enum struct Channel {
            CHANNEL1,
            CHANNEL2,
        };

        using Raw = Motor::Raw;
        using Speed = std::double_t;
        using Voltage = std::double_t;
        using Torque = std::double_t;
        using Direction = Motor::Direction;
        using Error = Motor::Error;
        using ExpectedDirection = Motor::ExpectedDirection;
        using ExpectedRaw = Motor::ExpectedRaw;
        using ExpectedVoltage = Motor::ExpectedVoltage;
        using ExpectedSpeed = Motor::ExpectedSpeed;
        using ExpectedTorque = Motor::ExpectedTorque;
        using Unexpected = Motor::Unexpected;
        using MotorChannel = std::pair<Channel, Motor>;
        using MotorChannels = std::array<MotorChannel, 2>;

        template <typename... MotorArgs>
        static MotorChannel make_motor_channel(const Channel channel, MotorArgs... motor_args) noexcept
        {
            return MotorChannel{std::piecewise_construct,
                                std::forward_as_tuple(channel),
                                std::forward_as_tuple(motor_args...)};
        }

        static MotorChannel make_motor_channel(const Channel channel) noexcept
        {
            return MotorChannel{std::piecewise_construct, std::forward_as_tuple(channel), std::forward_as_tuple()};
        }

        L298N() noexcept = default;

        L298N(const MotorChannels& motor_channels) noexcept;
        L298N(MotorChannels&& motor_channels) noexcept;

        L298N(const L298N& other) noexcept = default;
        L298N(L298N&& other) noexcept = default;

        L298N& operator=(const L298N& other) noexcept = default;
        L298N& operator=(L298N&& other) noexcept = default;

        ~L298N() noexcept;

        const MotorChannels& motor_channels() const& noexcept;
        MotorChannels&& motor_channels() && noexcept;

        void motor_channels(const MotorChannels& motors) noexcept;
        void motor_channels(MotorChannels&& motors) noexcept;

        [[nodiscard]] ExpectedRaw get_compare_raw(const Channel channel) const noexcept;
        [[nodiscard]] Error set_compare_raw(const Channel channel, const Raw raw) const noexcept;

        [[nodiscard]] ExpectedVoltage get_compare_voltage(const Channel channel) const noexcept;
        [[nodiscard]] Error set_compare_voltage(const Channel channel, const Voltage voltage) const noexcept;

        [[nodiscard]] ExpectedSpeed get_compare_speed(const Channel channel) const noexcept;
        [[nodiscard]] Error set_compare_speed(const Channel channel, const Speed speed) const noexcept;

        [[nodiscard]] ExpectedTorque get_compare_torque(const Channel channel) const noexcept;
        [[nodiscard]] Error set_compare_torque(const Channel channel, const Torque torque) const noexcept;

        [[nodiscard]] Error set_direction(const Channel channel, const Direction direction) const noexcept;
        [[nodiscard]] ExpectedDirection get_direction(const Channel channel) const noexcept;

        [[nodiscard]] Error set_forward(const Channel channel) const noexcept;
        [[nodiscard]] Error set_backward(const Channel channel) const noexcept;
        [[nodiscard]] Error set_soft_stop(const Channel channel) const noexcept;
        [[nodiscard]] Error set_fast_stop(const Channel channel) const noexcept;
        [[nodiscard]] Error toggle_direction(const Channel channel) const noexcept;

    private:
        Error initialize() noexcept;
        Error deinitialize() noexcept;

        const Motor& get_motor(const Channel channel) const noexcept;
        Motor& get_motor(const Channel channel) noexcept;

        MotorChannels motor_channels_{};
    };

}; // namespace InvertedSway

#endif // L298N_HPP
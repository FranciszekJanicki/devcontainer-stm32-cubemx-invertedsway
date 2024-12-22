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
        using Error = Motor::Error;
        using ExpectedRaw = Motor::ExpectedRaw;
        using ExpectedVoltage = Motor::ExpectedVoltage;
        using ExpectedSpeed = Motor::ExpectedSpeed;
        using Direction = Motor::Direction;
        using Unexpected = Motor::Unexpected;
        using MotorChannel = std::pair<Channel, Motor>;
        using MotorChannels = std::array<MotorChannel, 2>;

        template <typename... MotorArgs>
        static MotorChannel make_motor_channel(Channel const channel, MotorArgs... motor_args) noexcept
        {
            return MotorChannel{std::piecewise_construct,
                                std::forward_as_tuple(channel),
                                std::forward_as_tuple(motor_args...)};
        }

        L298N() noexcept = default;
        L298N(MotorChannels&& motor_channels) noexcept;

        L298N(L298N const& other) noexcept = delete;
        L298N(L298N&& other) noexcept = default;

        L298N& operator=(L298N const& other) noexcept = delete;
        L298N& operator=(L298N&& other) noexcept = default;

        ~L298N() noexcept;

        MotorChannels&& motor_channels() && noexcept;
        void motor_channels(MotorChannels&& motors) noexcept;

        [[nodiscard]] Error set_compare_raw(Channel const channel, Raw const raw) const noexcept;
        [[nodiscard]] Error set_compare_voltage(Channel const channel, Voltage const voltage) const noexcept;
        [[nodiscard]] Error set_compare_speed(Channel const channel, Speed const speed) const noexcept;

        [[nodiscard]] ExpectedRaw get_compare_raw(Channel const channel) const noexcept;
        [[nodiscard]] ExpectedVoltage get_compare_voltage(Channel const channel) const noexcept;
        [[nodiscard]] ExpectedSpeed get_compare_speed(Channel const channel) const noexcept;

        [[nodiscard]] Error set_direction(Channel const channel, Direction const direction) const noexcept;
        [[nodiscard]] Error set_forward(Channel const channel) const noexcept;
        [[nodiscard]] Error set_backward(Channel const channel) const noexcept;
        [[nodiscard]] Error set_soft_stop(Channel const channel) const noexcept;
        [[nodiscard]] Error set_fast_stop(Channel const channel) const noexcept;

    private:
        void initialize() noexcept;
        void deinitialize() noexcept;

        const Motor& get_motor(Channel const channel) const noexcept;
        Motor& get_motor(Channel const channel) noexcept;

        MotorChannels motor_channels_{};
    };

}; // namespace InvertedSway

#endif // L298N_HPP
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

        struct MotorChannel {
            Channel channel{};
            Motor motor{};
        };

        using Raw = Motor::Raw;
        using Voltage = Motor::Voltage;
        using Direction = Motor::Direction;
        using MotorChannels = std::array<MotorChannel, 2>;

        void reset() const noexcept;

        void set_voltage(Channel const channel, Voltage const voltage) const noexcept;
        void set_voltage_max(Channel const channel) const noexcept;
        void set_voltage_min(Channel const channel) const noexcept;

        void set_direction(Channel const channel, Direction const direction) const noexcept;
        void set_forward(Channel const channel) const noexcept;
        void set_backward(Channel const channel) const noexcept;
        void set_soft_stop(Channel const channel) const noexcept;
        void set_fast_stop(Channel const channel) const noexcept;

        MotorChannels motor_channels{};

    private:
        const Motor& get_motor(Channel const channel) const noexcept;
        Motor& get_motor(Channel const channel) noexcept;
    };

}; // namespace InvertedSway

#endif // L298N_HPP
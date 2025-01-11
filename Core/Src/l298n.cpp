#include "l298n.hpp"
#include "common.hpp"
#include "motor.hpp"
#include <algorithm>
#include <cassert>
#include <expected>
#include <ranges>
#include <utility>

using namespace InvertedSway;
using Direction = L298N::Direction;
using Channel = L298N::Channel;
using MotorChannel = L298N::MotorChannel;
using MotorChannels = L298N::MotorChannels;
using Raw = L298N::Raw;
using Voltage = L298N::Voltage;
using Direction = L298N::Direction;

namespace InvertedSway {

    void L298N::reset() const noexcept
    {
        std::ranges::for_each(this->motor_channels, [](auto& motor_channel) {
            motor_channel.motor.set_fast_stop();
            motor_channel.motor.set_voltage_min();
        });
    }

    void L298N::set_voltage(Channel const channel, Voltage const voltage) const noexcept
    {
        this->get_motor(channel).set_voltage(voltage);
    }

    void L298N::set_voltage_max(Channel const channel) const noexcept
    {
        this->get_motor(channel).set_voltage_max();
    }

    void L298N::set_voltage_min(Channel const channel) const noexcept
    {
        this->get_motor(channel).set_voltage_min();
    }

    void L298N::set_direction(Channel const channel, Direction const direction) const noexcept
    {
        this->get_motor(channel).set_direction(direction);
    }

    void L298N::set_forward(Channel const channel) const noexcept
    {
        this->get_motor(channel).set_direction(Direction::FORWARD);
    }

    void L298N::set_backward(Channel const channel) const noexcept
    {
        this->get_motor(channel).set_direction(Direction::BACKWARD);
    }

    void L298N::set_soft_stop(Channel const channel) const noexcept
    {
        this->get_motor(channel).set_direction(Direction::SOFT_STOP);
    }

    void L298N::set_fast_stop(Channel const channel) const noexcept
    {
        this->get_motor(channel).set_direction(Direction::FAST_STOP);
    }

    const Motor& L298N::get_motor(Channel const channel) const noexcept
    {
        if (const auto motor_channel{std::ranges::find_if(
                std::as_const(this->motor_channels),
                [channel](auto const& motor_channel) { return motor_channel.channel == channel; })};
            motor_channel != this->motor_channels.cend()) {
            return motor_channel->motor;
        }
        std::unreachable();
    }

    Motor& L298N::get_motor(Channel const channel) noexcept
    {
        if (auto motor_channel{std::ranges::find_if(
                this->motor_channels,
                [channel](auto const& motor_channel) { return motor_channel.channel == channel; })};
            motor_channel != this->motor_channels.cend()) {
            return motor_channel->motor;
        }
        std::unreachable();
    }
}; // namespace InvertedSway
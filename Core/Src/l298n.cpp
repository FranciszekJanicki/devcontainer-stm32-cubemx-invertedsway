#include "l298n.hpp"
#include "common.hpp"
#include "motor.hpp"
#include <algorithm>
#include <cassert>
#include <expected>
#include <ranges>
#include <utility>

namespace InvertedSway {

    using Error = L298N::Error;
    using Direction = L298N::Direction;
    using Channel = L298N::Channel;
    using MotorChannels = L298N::MotorChannels;
    using Raw = L298N::Raw;
    using Speed = L298N::Speed;
    using Voltage = L298N::Voltage;
    using Torque = L298N::Torque;
    using ExpectedDirection = L298N::ExpectedDirection;
    using ExpectedRaw = L298N::ExpectedRaw;
    using ExpectedVoltage = L298N::ExpectedVoltage;
    using ExpectedSpeed = L298N::ExpectedSpeed;
    using ExpectedTorque = L298N::ExpectedTorque;
    using Unexpected = L298N::Unexpected;

    template <typename... MotorArgs>
    MotorChannel L298N::make_motor_channel(const Channel channel, MotorArgs... motor_args) noexcept
    {
        return std::pair<Channel, Motor>{std::piecewise_construct,
                                         std::forward_as_tuple(channel),
                                         std::forward_as_tuple(motor_args...)};
    }

    MotorChannel L298N::make_motor_channel(const Channel channel) noexcept
    {
        return std::pair<Channel, Motor>{std::piecewise_construct,
                                         std::forward_as_tuple(channel),
                                         std::forward_as_tuple()};
    }

    L298N::L298N(const MotorChannels& motor_channels) noexcept : motor_channels_{motor_channels}
    {
        std::ranges::for_each(this->motor_channels_, [](auto& motor_channel) { motor_channel.second.initialize(); });
    }

    L298N::L298N(MotorChannels&& motor_channels) noexcept : motor_channels_{std::forward<MotorChannels>(motor_channels)}
    {
        std::ranges::for_each(this->motor_channels_, [](auto& motor_channel) { motor_channel.second.initialize(); });
    }

    L298N::~L298N() noexcept
    {
        std::ranges::for_each(this->motor_channels_, [](auto& motor_channel) { motor_channel.second.deinitialize(); });
    }

    const MotorChannels& L298N::motor_channels() const& noexcept
    {
        return this->motor_channels_;
    }

    MotorChannels&& L298N::motor_channels() && noexcept
    {
        return std::forward<L298N>(*this).motor_channels_;
    }

    void L298N::motor_channels(const MotorChannels& motor_channels) noexcept
    {
        this->motor_channels_ = motor_channels;
    }

    void L298N::motor_channels(MotorChannels&& motor_channels) noexcept
    {
        this->motor_channels_ = std::forward<MotorChannels>(motor_channels);
    }

    ExpectedRaw L298N::get_compare_raw(const Channel channel) const noexcept
    {
        return this->get_motor(channel).get_compare_raw();
    }

    Error L298N::set_compare_raw(const Channel channel, const Raw raw) const noexcept
    {
        return this->get_motor(channel).set_compare_raw(raw);
    }

    ExpectedVoltage L298N::get_compare_voltage(const Channel channel) const noexcept
    {
        return this->get_motor(channel).get_compare_voltage();
    }

    Error L298N::set_compare_voltage(const Channel channel, const Voltage voltage) const noexcept
    {
        return this->get_motor(channel).set_compare_voltage(voltage);
    }

    ExpectedSpeed L298N::get_compare_speed(const Channel channel) const noexcept
    {
        return this->get_motor(channel).get_compare_speed();
    }

    Error L298N::set_compare_speed(const Channel channel, const Speed speed) const noexcept
    {
        return this->get_motor(channel).set_compare_speed(speed);
    }

    ExpectedTorque L298N::get_compare_torque(const Channel channel) const noexcept
    {
        return this->get_motor(channel).get_compare_torque();
    }

    Error L298N::set_compare_torque(const Channel channel, const Torque torque) const noexcept
    {
        return this->get_motor(channel).set_compare_torque(torque);
    }

    Error L298N::set_direction(const Channel channel, const Direction direction) const noexcept
    {
        return this->get_motor(channel).set_direction(direction);
    }

    ExpectedDirection L298N::get_direction(const Channel channel) const noexcept
    {
        return this->get_motor(channel).get_direction();
    }

    Error L298N::set_forward(const Channel channel) const noexcept
    {
        return this->get_motor(channel).set_direction(Direction::FORWARD);
    }

    Error L298N::set_backward(const Channel channel) const noexcept
    {
        return this->get_motor(channel).set_direction(Direction::BACKWARD);
    }

    Error L298N::set_soft_stop(const Channel channel) const noexcept
    {
        return this->get_motor(channel).set_direction(Direction::SOFT_STOP);
    }

    Error L298N::set_fast_stop(const Channel channel) const noexcept
    {
        return this->get_motor(channel).set_direction(Direction::FAST_STOP);
    }

    Error L298N::toggle_direction(const Channel channel) const noexcept
    {
        return this->get_motor(channel).toggle_direction();
    }

    const Motor& L298N::get_motor(const Channel channel) const noexcept
    {
        if (const auto motor_channel{
                std::ranges::find_if(std::as_const(this->motor_channels_),
                                     [channel](const auto& item) { return item.first == channel; })};
            motor_channel != this->motor_channels_.cend()) {
            return motor_channel->second;
        }
        std::unreachable();
    }

    Motor& L298N::get_motor(const Channel channel) noexcept
    {
        if (auto motor_channel{std::ranges::find_if(this->motor_channels_,
                                                    [channel](const auto& item) { return item.first == channel; })};
            motor_channel != this->motor_channels_.cend()) {
            return motor_channel->second;
        }
        std::unreachable();
    }

}; // namespace InvertedSway
#include "l298n.hpp"
#include "common.hpp"
#include "motor.hpp"
#include <algorithm>
#include <cassert>
#include <expected>
#include <ranges>
#include <utility>

using namespace InvertedSway;
using Error = L298N::Error;
using Direction = L298N::Direction;
using Channel = L298N::Channel;
using MotorChannel = L298N::MotorChannel;
using MotorChannels = L298N::MotorChannels;
using Raw = L298N::Raw;
using Speed = L298N::Speed;
using Voltage = L298N::Voltage;
using Direction = L298N::Direction;
using ExpectedRaw = L298N::ExpectedRaw;
using ExpectedVoltage = L298N::ExpectedVoltage;
using ExpectedSpeed = L298N::ExpectedSpeed;
using Unexpected = L298N::Unexpected;

namespace InvertedSway {

    L298N::L298N(MotorChannels&& motor_channels) noexcept : motor_channels_{std::forward<MotorChannels>(motor_channels)}
    {
        // this->initialize();
    }

    L298N::~L298N() noexcept
    {
        // this->deinitialize();
    }

    MotorChannels&& L298N::motor_channels() && noexcept
    {
        return std::forward<L298N>(*this).motor_channels_;
    }

    void L298N::motor_channels(MotorChannels&& motor_channels) noexcept
    {
        this->motor_channels_ = std::forward<MotorChannels>(motor_channels);
    }

    ExpectedRaw L298N::get_compare_raw(Channel const channel) const noexcept
    {
        return this->get_motor(channel).get_compare_raw();
    }

    Error L298N::set_compare_raw(Channel const channel, Raw const raw) const noexcept
    {
        return this->get_motor(channel).set_compare_raw(raw);
    }

    ExpectedVoltage L298N::get_compare_voltage(Channel const channel) const noexcept
    {
        return this->get_motor(channel).get_compare_voltage();
    }

    Error L298N::set_compare_voltage(Channel const channel, Voltage const voltage) const noexcept
    {
        return this->get_motor(channel).set_compare_voltage(voltage);
    }

    ExpectedSpeed L298N::get_compare_speed(Channel const channel) const noexcept
    {
        return this->get_motor(channel).get_compare_speed();
    }

    Error L298N::set_compare_speed(Channel const channel, Speed const speed) const noexcept
    {
        return this->get_motor(channel).set_compare_speed(speed);
    }

    Error L298N::set_direction(Channel const channel, Direction const direction) const noexcept
    {
        return this->get_motor(channel).set_direction(direction);
    }

    Error L298N::set_forward(Channel const channel) const noexcept
    {
        return this->get_motor(channel).set_direction(Direction::FORWARD);
    }

    Error L298N::set_backward(Channel const channel) const noexcept
    {
        return this->get_motor(channel).set_direction(Direction::BACKWARD);
    }

    Error L298N::set_soft_stop(Channel const channel) const noexcept
    {
        return this->get_motor(channel).set_direction(Direction::SOFT_STOP);
    }

    Error L298N::set_fast_stop(Channel const channel) const noexcept
    {
        return this->get_motor(channel).set_direction(Direction::FAST_STOP);
    }

    void L298N::initialize() noexcept
    {
        std::ranges::for_each(this->motor_channels_, [](auto& motor_channel) {
            motor_channel.motor.set_fast_stop();
            motor_channel.motor.set_compare_min();
        });
    }

    void L298N::deinitialize() noexcept
    {
        std::ranges::for_each(this->motor_channels_, [](auto& motor_channel) {
            motor_channel.motor.set_fast_stop();
            motor_channel.motor.set_compare_min();
        });
    }

    const Motor& L298N::get_motor(Channel const channel) const noexcept
    {
        if (const auto motor_channel{std::ranges::find_if(
                std::as_const(this->motor_channels_),
                [channel](auto const& motor_channel) { return motor_channel.channel == channel; })};
            motor_channel != this->motor_channels_.cend()) {
            return motor_channel->motor;
        }
        std::unreachable();
    }

    Motor& L298N::get_motor(Channel const channel) noexcept
    {
        if (auto motor_channel{std::ranges::find_if(
                this->motor_channels_,
                [channel](auto const& motor_channel) { return motor_channel.channel == channel; })};
            motor_channel != this->motor_channels_.cend()) {
            return motor_channel->motor;
        }
        std::unreachable();
    }

}; // namespace InvertedSway
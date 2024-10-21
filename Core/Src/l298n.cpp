#include "l298n.hpp"
#include <cassert>
#include <cmath>
#include <expected>

L298N::L298N(const Channel& channel1, const Channel& channel2) noexcept : channel1_{channel1}, channel2_{channel2}
{
    if (initialize(channel1_.channel_num) != Error::OK) {
    }
    if (initialize(channel2_.channel_num) != Error::OK) {
    }
}

L298N::L298N(Channel&& channel1, Channel&& channel2) noexcept :
    channel1_{std::forward<Channel>(channel1)}, channel2_{std::forward<Channel>(channel2)}
{
    if (initialize(channel1_.channel_num) != Error::OK) {
    }
    if (initialize(channel2_.channel_num) != Error::OK) {
    }
}

L298N::~L298N() noexcept
{
    if (deinitialize(channel1_.channel_num) != Error::OK) {
    }
    if (deinitialize(channel2_.channel_num) != Error::OK) {
    }
}

const Channel& L298N::channel1() const& noexcept
{
    return channel1_;
}

Channel&& L298N::channel1() && noexcept
{
    return std::forward<L298N>(*this).channel1_;
}

const Channel& L298N::channel2() const& noexcept
{
    return channel2_;
}

Channel&& L298N::channel2() && noexcept
{
    return std::forward<L298N>(*this).channel2_;
}

void L298N::channel1(const Channel& channel1) noexcept
{
    channel1_ = channel1;
}

void L298N::channel1(Channel&& channel1) noexcept
{
    channel1_ = std::forward<Channel>(channel1);
}

void L298N::channel2(const Channel& channel2) noexcept
{
    channel2_ = channel2;
}

void L298N::channel2(Channel&& channel2) noexcept
{
    channel2_ = std::forward<Channel>(channel2);
}

Error L298N::initialize(const ChannelNum channel_num) noexcept
{
    if (initialized_) {
        return Error::INIT;
    }
    auto& channel{get_channel(channel_num)};
    if (channel.initialized) {
        return Error::INIT;
    }
    for (const auto option : channel.timer_it_options) {
        __HAL_TIM_ENABLE_IT(channel.timer, option);
    }
    if (HAL_TIM_PWM_Start_IT(channel.timer, channel.timer_channel) != HAL_OK) {
        return channel_num_to_error(channel_num);
    }
    channel.initialized = true;
    return Error::OK;
}

Error L298N::deinitialize(const ChannelNum channel_num) noexcept
{
    Channel& channel{get_channel(channel_num)};
    if (!channel.initialized) {
        return Error::DEINIT;
    }
    for (const auto option : channel.timer_it_options) {
        __HAL_TIM_DISABLE_IT(channel.timer, option);
    }
    if (HAL_TIM_PWM_Stop_IT(channel.timer, channel.timer_channel) != HAL_OK) {
        return channel_num_to_error(channel_num);
    }
    channel.initialized = false;
    return Error::OK;
}

ExpectedRaw L298N::get_compare_raw(const ChannelNum channel_num) const noexcept
{
    const Channel& channel{get_channel(channel_num)};
    if (!channel.initialized) {
        return Unexpected{channel_num_to_error(channel_num)};
    }
    return ExpectedRaw{__HAL_TIM_GetCompare(channel.timer, channel.timer_channel)};
}

Error L298N::set_compare_raw(const ChannelNum channel_num, const Raw raw) const noexcept
{
    assert(raw <= MAX_RAW && raw >= MIN_RAW);
    const auto& channel{get_channel(channel_num)};
    if (!channel.initialized) {
        return channel_num_to_error(channel_num);
    }
    __HAL_TIM_SetCompare(channel.timer, channel.timer_channel, raw);
    return Error::OK;
}

ExpectedVoltage L298N::get_compare_voltage(const ChannelNum channel_num) const noexcept
{
    const auto& channel{get_channel(channel_num)};
    if (!channel.initialized) {
        return Unexpected{channel_num_to_error(channel_num)};
    }
    return ExpectedVoltage{raw_to_voltage(__HAL_TIM_GetCompare(channel.timer, channel.timer_channel))};
}

Error L298N::set_compare_voltage(const ChannelNum channel_num, const Voltage voltage) const noexcept
{
    assert(voltage <= MAX_VOLTAGE_MV && voltage >= MIN_VOLTAGE_MV);
    const auto& channel{get_channel(channel_num)};
    if (!channel.initialized) {
        return channel_num_to_error(channel_num);
    }
    __HAL_TIM_SetCompare(channel.timer, channel.timer_channel, voltage_to_raw(voltage));
    return Error::OK;
}

ExpectedSpeed L298N::get_compare_speed(const ChannelNum channel_num) const noexcept
{
    const auto& channel{get_channel(channel_num)};
    if (!channel.initialized) {
        return Unexpected{channel_num_to_error(channel_num)};
    }
    return ExpectedSpeed{raw_to_speed(__HAL_TIM_GetCompare(channel.timer, channel.timer_channel))};
}

Error L298N::set_compare_speed(const ChannelNum channel_num, const Speed speed) const noexcept
{
    assert(speed <= MAX_SPEED_RPM && speed >= MIN_SPEED_RPM);
    const auto& channel{get_channel(channel_num)};
    if (!channel.initialized) {
        return channel_num_to_error(channel_num);
    }
    __HAL_TIM_SetCompare(channel.timer, channel.timer_channel, speed_to_raw(speed));
    return Error::OK;
}

Error L298N::set_direction(const ChannelNum channel_num, const Direction direction) const noexcept
{
    const auto& channel{get_channel(channel_num)};
    if (!channel.initialized) {
        return channel_num_to_error(channel_num);
    }
    switch (direction) {
        case Direction::SOFT_STOP:
            HAL_GPIO_WritePin(channel.gpio, channel.pin_IN1, GPIO_PinState::GPIO_PIN_RESET);
            HAL_GPIO_WritePin(channel.gpio, channel.pin_IN2, GPIO_PinState::GPIO_PIN_RESET);
            return set_compare_raw(channel_num, MAX_RAW);
        case Direction::FAST_STOP:
            HAL_GPIO_WritePin(channel.gpio, channel.pin_IN1, GPIO_PinState::GPIO_PIN_SET);
            HAL_GPIO_WritePin(channel.gpio, channel.pin_IN2, GPIO_PinState::GPIO_PIN_SET);
            return set_compare_raw(channel_num, MIN_RAW);
        case Direction::FORWARD:
            HAL_GPIO_WritePin(channel.gpio, channel.pin_IN1, GPIO_PinState::GPIO_PIN_SET);
            HAL_GPIO_WritePin(channel.gpio, channel.pin_IN2, GPIO_PinState::GPIO_PIN_RESET);
            return Error::OK;
        case Direction::BACKWARD:
            HAL_GPIO_WritePin(channel.gpio, channel.pin_IN1, GPIO_PinState::GPIO_PIN_RESET);
            HAL_GPIO_WritePin(channel.gpio, channel.pin_IN2, GPIO_PinState::GPIO_PIN_SET);
            return Error::OK;
        default:
            return channel_num_to_error(channel_num);
    }
    return Error::OK;
}

Error L298N::set_forward(const ChannelNum channel_num) const noexcept
{
    return set_direction(channel_num, Direction::FORWARD);
}

Error L298N::set_backward(const ChannelNum channel_num) const noexcept
{
    return set_direction(channel_num, Direction::BACKWARD);
}

Error L298N::set_soft_stop(const ChannelNum channel_num) const noexcept
{
    return set_direction(channel_num, Direction::SOFT_STOP);
}

Error L298N::set_fast_stop(const ChannelNum channel_num) const noexcept
{
    return set_direction(channel_num, Direction::FAST_STOP);
}

const Channel& L298N::get_channel(const ChannelNum channel) const noexcept
{
    switch (channel) {
        case ChannelNum::CHANNEL1:
            return channel1_;
        case ChannelNum::CHANNEL2:
            return channel2_;
        default:
            std::unreachable();
    }
}

Channel& L298N::get_channel(const ChannelNum channel_num) noexcept
{
    switch (channel_num) {
        case ChannelNum::CHANNEL1:
            return channel1_;
        case ChannelNum::CHANNEL2:
            return channel2_;
        default:
            std::unreachable();
    }
}

static const char* L298N::error_to_string(const Error error) noexcept
{
    switch (error) {
        case Error::INIT:
            return "INIT ERROR";
        case Error::DEINIT:
            return "DEINIT ERROR";
        case Error::CHANNEL1:
            return "CHANNEL1 ERROR";
        case Error::CHANNEL2:
            return "CHANNEL2 ERROR";
        case Error::OK:
            return "OK ERROR";
        default:
            return "NONE ERROR";
    }
}

static Error L298N::channel_num_to_error(const ChannelNum channel_num) noexcept
{
    switch (channel_num) {
        case ChannelNum::CHANNEL1:
            return Error::CHANNEL1;
        case ChannelNum::CHANNEL2:
            return Error::CHANNEL2;
        default:
            std::unreachable();
    }
}

static Speed L298N::raw_to_speed(const Raw raw) noexcept
{
    return raw * (MAX_SPEED_RPM - MIN_SPEED_RPM) / (MAX_RAW - MIN_RAW);
}

static Raw L298N::speed_to_raw(const Speed speed) noexcept
{
    return speed * (MAX_RAW - MIN_RAW) / (MAX_SPEED_RPM - MIN_SPEED_RPM);
}

static Voltage L298N::raw_to_voltage(const Raw raw) noexcept
{
    return raw * (MAX_VOLTAGE_MV - MIN_VOLTAGE_MV) / (MAX_RAW - MIN_RAW);
}

static Raw L298N::voltage_to_raw(const Voltage voltage) noexcept
{
    return voltage * (MAX_RAW - MIN_RAW) / (MAX_VOLTAGE_MV - MIN_VOLTAGE_MV);
}
#include "l298n.hpp"
#include "common.hpp"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_gpio.h"
#include "stm32l4xx_hal_tim.h"
#include <algorithm>
#include <cassert>
#include <expected>
#include <ranges>
#include <utility>

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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim)
{
}

const char* Motor::error_to_string(const Error error) noexcept
{
    switch (error) {
        case Error::OK:
            return "OK";
        case Error::FAIL:
            return "FAIL";
        case Error::INIT:
            return "INIT";
        case Error::DEINIT:
            return "DEINIT";
        default:
            return "NONE";
    }
}

Error Motor::initialize() noexcept
{
    if (this->initialized) {
        return Error::INIT;
    }
    if (this->timer == nullptr || this->gpio == nullptr) {
        return Error::INIT;
    }
    if (HAL_TIM_PWM_Start(this->timer, this->timer_channel) != HAL_OK) {
        return Error::INIT;
    }
    this->initialized = true;
    return Error::OK;
}

Error Motor::deinitialize() noexcept
{
    if (!this->initialized) {
        return Error::DEINIT;
    }
    if (this->timer == nullptr || this->gpio == nullptr) {
        return Error::DEINIT;
    }
    if (HAL_TIM_PWM_Stop(this->timer, this->timer_channel) != HAL_OK) {
        return Error::DEINIT;
    }
    this->initialized = false;
    return Error::OK;
}

ExpectedRaw Motor::get_compare_raw() const noexcept
{
    if (!this->initialized) {
        return Unexpected{Error::FAIL};
    }
    return ExpectedRaw{__HAL_TIM_GetCompare(this->timer, this->timer_channel)};
}

Error Motor::set_compare_raw(const Raw raw) const noexcept
{
    if (!this->initialized) {
        return Error::FAIL;
    }
    __HAL_TIM_SetCompare(this->timer, this->timer_channel, raw);
    return Error::OK;
}

Error Motor::set_direction(const Direction direction) const noexcept
{
    if (!this->initialized) {
        return Error::FAIL;
    }
    switch (direction) {
        case Direction::SOFT_STOP:
            HAL_GPIO_WritePin(this->gpio, this->pin_in1, GPIO_PinState::GPIO_PIN_RESET);
            HAL_GPIO_WritePin(this->gpio, this->pin_in2, GPIO_PinState::GPIO_PIN_RESET);
            return this->set_compare_raw(MAX_RAW);
        case Direction::FAST_STOP:
            HAL_GPIO_WritePin(this->gpio, this->pin_in1, GPIO_PinState::GPIO_PIN_SET);
            HAL_GPIO_WritePin(this->gpio, this->pin_in2, GPIO_PinState::GPIO_PIN_SET);
            return this->set_compare_raw(MIN_RAW);
        case Direction::FORWARD:
            HAL_GPIO_WritePin(this->gpio, this->pin_in1, GPIO_PinState::GPIO_PIN_SET);
            HAL_GPIO_WritePin(this->gpio, this->pin_in2, GPIO_PinState::GPIO_PIN_RESET);
            return Error::OK;
        case Direction::BACKWARD:
            HAL_GPIO_WritePin(this->gpio, this->pin_in1, GPIO_PinState::GPIO_PIN_RESET);
            HAL_GPIO_WritePin(this->gpio, this->pin_in2, GPIO_PinState::GPIO_PIN_SET);
            return Error::OK;
        default:
            return Error::FAIL;
    }
    return Error::OK;
}

ExpectedDirection Motor::get_direction() const noexcept
{
    if (!this->initialized) {
        return Unexpected{Error::FAIL};
    }
    const auto pin_out1{HAL_GPIO_ReadPin(this->gpio, this->pin_out1)};
    const auto pin_out2{HAL_GPIO_ReadPin(this->gpio, this->pin_out2)};
    if (pin_out1 == GPIO_PinState::GPIO_PIN_SET && pin_out2 == GPIO_PinState::GPIO_PIN_RESET) {
        return ExpectedDirection{Direction::FORWARD};
    }
    if (pin_out1 == GPIO_PinState::GPIO_PIN_RESET && pin_out2 == GPIO_PinState::GPIO_PIN_SET) {
        return ExpectedDirection{Direction::BACKWARD};
    }
    const auto raw{__HAL_TIM_GetCompare(this->timer, this->timer_channel)};
    if ((pin_out1 == pin_out2) == GPIO_PinState::GPIO_PIN_RESET && raw >= MAX_RAW) {
        return ExpectedDirection{Direction::SOFT_STOP};
    }
    if ((pin_out1 == pin_out2) == GPIO_PinState::GPIO_PIN_SET && raw <= MIN_RAW) {
        return ExpectedDirection{Direction::FAST_STOP};
    }
    return Unexpected{Error::OK};
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
    assert(raw <= Motor::MAX_RAW && raw >= Motor::MIN_RAW);
    if (raw >= Motor::MAX_RAW || raw <= Motor::MIN_RAW) {
        return Error::FAIL;
    }
    return this->get_motor(channel).set_compare_raw(raw);
}

ExpectedVoltage L298N::get_compare_voltage(const Channel channel) const noexcept
{
    if (auto raw{this->get_motor(channel).get_compare_raw()}; !raw.has_value()) {
        return Unexpected{Error::FAIL};
    } else {
        return ExpectedVoltage{raw_to_voltage(std::move(raw).value())};
    }
}

Error L298N::set_compare_voltage(const Channel channel, const Voltage voltage) const noexcept
{
    assert(voltage <= MAX_VOLTAGE_V && voltage >= MIN_VOLTAGE_V);
    if (voltage >= MAX_VOLTAGE_V || voltage <= MIN_VOLTAGE_V) {
        return Error::FAIL;
    }
    return this->get_motor(channel).set_compare_raw(voltage_to_raw(voltage));
}

ExpectedSpeed L298N::get_compare_speed(const Channel channel) const noexcept
{
    if (auto raw{this->get_motor(channel).get_compare_raw()}; !raw.has_value()) {
        return Unexpected{std::move(raw).error()};
    } else {
        return ExpectedSpeed{raw_to_speed(std::move(raw).value())};
    }
}

Error L298N::set_compare_speed(const Channel channel, const Speed speed) const noexcept
{
    assert(speed <= MAX_SPEED_RPM && speed >= MIN_SPEED_RPM);
    if (speed >= MAX_SPEED_RPM || speed <= MIN_SPEED_RPM) {
        return Error::FAIL;
    }
    return this->get_motor(channel).set_compare_raw(speed_to_raw(speed));
}

ExpectedTorque L298N::get_compare_torque(const Channel channel) const noexcept
{
    if (auto raw{this->get_motor(channel).get_compare_raw()}; !raw.has_value()) {
        return Unexpected{std::move(raw).error()};
    } else {
        return ExpectedSpeed{raw_to_torque(std::move(raw).value())};
    }
}

Error L298N::set_compare_torque(const Channel channel, const Torque torque) const noexcept
{
    assert(torque <= MAX_TORQUE_NM && torque >= MIN_TORQUE_NM);
    if (torque >= MAX_TORQUE_NM || torque <= MIN_TORQUE_NM) {
        return Error::FAIL;
    }
    return this->get_motor(channel).set_compare_raw(torque_to_raw(torque));
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
    const auto& motor{get_motor(channel)};
    if (!motor.initialized) {
        return Error::FAIL;
    }
    if (auto direction{motor.get_direction()}; !direction.has_value()) {
        return std::move(direction).error();
    } else {
        switch (direction.value()) {
            case Direction::FORWARD:
                return motor.set_direction(Direction::BACKWARD);
            case Direction::BACKWARD:
                return motor.set_direction(Direction::FORWARD);
            case Direction::FAST_STOP:
                return Error::OK;
            case Direction::SOFT_STOP:
                return Error::OK;
        }
    }
}

const Motor& L298N::get_motor(const Channel channel) const noexcept
{
    if (const auto motor_channel{std::ranges::find_if(std::as_const(this->motor_channels_),
                                                      [channel](const auto& item) {
                                                          const auto& [key, value]{item};
                                                          return key == channel;
                                                      })};
        motor_channel != this->motor_channels_.cend()) {
        return motor_channel->second;
    }
    std::unreachable();
}

Motor& L298N::get_motor(const Channel channel) noexcept
{
    if (auto motor_channel{std::ranges::find_if(this->motor_channels_,
                                                [channel](const auto& item) {
                                                    const auto& [key, value]{item};
                                                    return key == channel;
                                                })};
        motor_channel != this->motor_channels_.cend()) {
        return motor_channel->second;
    }
    std::unreachable();
}

Speed L298N::raw_to_speed(const Raw raw) noexcept
{
    assert(raw <= Motor::MAX_RAW && raw >= Motor::MIN_RAW);
    return std::clamp(
        Speed{(raw - Motor::MIN_RAW) * (MAX_SPEED_RPM - MIN_SPEED_RPM) / (Motor::MAX_RAW - Motor::MIN_RAW) +
              MAX_SPEED_RPM},
        MIN_SPEED_RPM,
        MAX_SPEED_RPM);
}

Raw L298N::speed_to_raw(const Speed speed) noexcept
{
    assert(speed <= MAX_SPEED_RPM && speed >= MIN_SPEED_RPM);
    return std::clamp(
        Raw{(speed - MIN_SPEED_RPM) * (Motor::MAX_RAW - Motor::MIN_RAW) / (MAX_SPEED_RPM - MIN_SPEED_RPM) +
            Motor::MIN_RAW},
        Motor::MIN_RAW,
        Motor::MAX_RAW);
}

Voltage L298N::raw_to_voltage(const Raw raw) noexcept
{
    assert(raw <= Motor::MAX_RAW && raw >= Motor::MIN_RAW);
    return std::clamp(
        Voltage{(raw - Motor::MIN_RAW) * (MAX_VOLTAGE_V - MIN_VOLTAGE_V) / (Motor::MAX_RAW - Motor::MIN_RAW) +
                MIN_VOLTAGE_V},
        MIN_VOLTAGE_V,
        MAX_VOLTAGE_V);
}

Raw L298N::voltage_to_raw(const Voltage voltage) noexcept
{
    assert(voltage <= MAX_VOLTAGE_V && voltage >= MIN_VOLTAGE_V);
    return std::clamp(
        Raw{(voltage - MIN_VOLTAGE_V) * (Motor::MAX_RAW - Motor::MIN_RAW) / Raw(MAX_VOLTAGE_V - MIN_VOLTAGE_V) +
            Motor::MIN_RAW},
        Motor::MIN_RAW,
        Motor::MAX_RAW);
}

Torque L298N::raw_to_torque(const Raw raw) noexcept
{
    assert(raw <= Motor::MAX_RAW && raw >= Motor::MIN_RAW);
    return std::clamp(
        Torque{(raw - Motor::MIN_RAW) * (MAX_TORQUE_NM - MIN_TORQUE_NM) / Torque(Motor::MAX_RAW - Motor::MIN_RAW) +
               MIN_TORQUE_NM},
        MIN_TORQUE_NM,
        MAX_TORQUE_NM);
}

Raw L298N::torque_to_raw(const Torque torque) noexcept
{
    assert(torque <= MAX_TORQUE_NM && torque >= MIN_TORQUE_NM);
    return std::clamp(
        Raw{(torque - MIN_TORQUE_NM) * (Motor::MAX_RAW - Motor::MIN_RAW) / (MAX_TORQUE_NM - MIN_TORQUE_NM) +
            Motor::MIN_RAW},
        Motor::MIN_RAW,
        Motor::MAX_RAW);
}
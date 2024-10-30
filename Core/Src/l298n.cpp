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
using MotorChannel = L298N::MotorChannel;
using Motor = L298N::Motor;
using Motors = L298N::Motors;
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

const char* L298N::error_to_string(const Error error) noexcept
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

L298N::L298N(const Motors& motors) noexcept : motors_{motors}
{
    std::ranges::for_each(motors_, [this](Motor& motor) { this->initialize(motor); });
}

L298N::L298N(Motors&& motors) noexcept : motors_{std::forward<Motors>(motors)}
{
    std::ranges::for_each(motors_, [this](Motor& motor) { this->initialize(motor); });
}

L298N::~L298N() noexcept
{
    std::ranges::for_each(motors_, [this](Motor& motor) { this->deinitialize(motor); });
}

const Motors& L298N::motors() const& noexcept
{
    return motors_;
}

Motors&& L298N::motors() && noexcept
{
    return std::forward<L298N>(*this).motors_;
}

void L298N::motors(const Motors& motors) noexcept
{
    motors_ = motors;
}

void L298N::motors(Motors&& motors) noexcept
{
    motors_ = std::forward<Motors>(motors);
}

Error L298N::initialize(Motor& motor) noexcept
{
    if (motor.initialized) {
        return Error::INIT;
    }
    if (motor.timer == nullptr || motor.gpio == nullptr) {
        return Error::INIT;
    }
    if (HAL_TIM_PWM_Start(motor.timer, motor.timer_channel) != HAL_OK) {
        return motor_channel_to_error(motor.motor_channel);
    }
    motor.initialized = true;
    return Error::OK;
}

Error L298N::deinitialize(Motor& motor) noexcept
{
    if (!motor.initialized) {
        return Error::DEINIT;
    }
    if (motor.timer == nullptr || motor.gpio == nullptr) {
        return Error::DEINIT;
    }
    if (HAL_TIM_PWM_Stop(motor.timer, motor.timer_channel) != HAL_OK) {
        return motor_channel_to_error(motor.motor_channel);
    }
    motor.initialized = false;
    return Error::OK;
}

ExpectedRaw L298N::get_compare_raw(const MotorChannel motor_channel) const noexcept
{
    const auto& motor{get_motor(motor_channel)};
    if (!motor.initialized) {
        return Unexpected{motor_channel_to_error(motor_channel)};
    }
    return ExpectedRaw{__HAL_TIM_GetCompare(motor.timer, motor.timer_channel)};
}

Error L298N::set_compare_raw(const MotorChannel motor_channel, const Raw raw) const noexcept
{
    assert(raw <= MAX_RAW && raw >= MIN_RAW);
    const auto& motor{get_motor(motor_channel)};
    if (!motor.initialized) {
        return motor_channel_to_error(motor_channel);
    }
    __HAL_TIM_SetCompare(motor.timer, motor.timer_channel, raw);
    return Error::OK;
}

ExpectedVoltage L298N::get_compare_voltage(const MotorChannel motor_channel) const noexcept
{
    const auto& motor{get_motor(motor_channel)};
    if (!motor.initialized) {
        return Unexpected{motor_channel_to_error(motor_channel)};
    }
    return ExpectedVoltage{raw_to_voltage(__HAL_TIM_GetCompare(motor.timer, motor.timer_channel))};
}

Error L298N::set_compare_voltage(const MotorChannel motor_channel, const Voltage voltage) const noexcept
{
    assert(voltage <= MAX_VOLTAGE_V && voltage >= MIN_VOLTAGE_V);
    const auto& motor{get_motor(motor_channel)};
    if (!motor.initialized) {
        return motor_channel_to_error(motor_channel);
    }
    __HAL_TIM_SetCompare(motor.timer, motor.timer_channel, voltage_to_raw(voltage));
    return Error::OK;
}

ExpectedSpeed L298N::get_compare_speed(const MotorChannel motor_channel) const noexcept
{
    const auto& motor{get_motor(motor_channel)};
    if (!motor.initialized) {
        return Unexpected{motor_channel_to_error(motor_channel)};
    }
    return ExpectedSpeed{raw_to_speed(__HAL_TIM_GetCompare(motor.timer, motor.timer_channel))};
}

Error L298N::set_compare_speed(const MotorChannel motor_channel, const Speed speed) const noexcept
{
    assert(speed <= MAX_SPEED_RPM && speed >= MIN_SPEED_RPM);
    const auto& motor{get_motor(motor_channel)};
    if (!motor.initialized) {
        return motor_channel_to_error(motor_channel);
    }
    __HAL_TIM_SetCompare(motor.timer, motor.timer_channel, speed_to_raw(speed));
    return Error::OK;
}

ExpectedTorque L298N::get_compare_torque(const MotorChannel motor_channel) const noexcept
{
    const auto& motor{get_motor(motor_channel)};
    if (!motor.initialized) {
        return Unexpected{motor_channel_to_error(motor_channel)};
    }
    return ExpectedTorque{raw_to_torque(__HAL_TIM_GetCompare(motor.timer, motor.timer_channel))};
}

Error L298N::set_compare_torque(const MotorChannel motor_channel, const Torque torque) const noexcept
{
    assert(torque >= MIN_TORQUE_NM && torque <= MAX_TORQUE_NM);
    const auto& motor{get_motor(motor_channel)};
    if (!motor.initialized) {
        return motor_channel_to_error(motor_channel);
    }
    if (__HAL_TIM_SetCompare(motor.timer, motor.timer_channel, torque_to_raw(torque)) != HAL_OK) {
        return motor_channel_to_error(motor_channel);
    }
    return Error::OK;
}

Error L298N::set_direction(const MotorChannel motor_channel, const Direction direction) const noexcept
{
    const auto& motor{get_motor(motor_channel)};
    if (!motor.initialized) {
        return motor_channel_to_error(motor_channel);
    }
    switch (direction) {
        case Direction::SOFT_STOP:
            HAL_GPIO_WritePin(motor.gpio, motor.pin_in1, GPIO_PinState::GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motor.gpio, motor.pin_in2, GPIO_PinState::GPIO_PIN_RESET);
            return set_compare_raw(motor_channel, MAX_RAW);
        case Direction::FAST_STOP:
            HAL_GPIO_WritePin(motor.gpio, motor.pin_in1, GPIO_PinState::GPIO_PIN_SET);
            HAL_GPIO_WritePin(motor.gpio, motor.pin_in2, GPIO_PinState::GPIO_PIN_SET);
            return set_compare_raw(motor_channel, MIN_RAW);
        case Direction::FORWARD:
            HAL_GPIO_WritePin(motor.gpio, motor.pin_in1, GPIO_PinState::GPIO_PIN_SET);
            HAL_GPIO_WritePin(motor.gpio, motor.pin_in2, GPIO_PinState::GPIO_PIN_RESET);
            return Error::OK;
        case Direction::BACKWARD:
            HAL_GPIO_WritePin(motor.gpio, motor.pin_in1, GPIO_PinState::GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motor.gpio, motor.pin_in2, GPIO_PinState::GPIO_PIN_SET);
            return Error::OK;
        default:
            return motor_channel_to_error(motor_channel);
    }
    return Error::OK;
}

ExpectedDirection L298N::get_direction(const MotorChannel motor_channel) const noexcept
{
    const auto& motor{get_motor(motor_channel)};
    if (!motor.initialized) {
        return Unexpected{motor_channel_to_error(motor_channel)};
    }
    const auto pin_out1{HAL_GPIO_ReadPin(motor.gpio, motor.pin_out1)};
    const auto pin_out2{HAL_GPIO_ReadPin(motor.gpio, motor.pin_out2)};
    if (pin_out1 == GPIO_PinState::GPIO_PIN_SET && pin_out2 == GPIO_PinState::GPIO_PIN_RESET) {
        return ExpectedDirection{Direction::FORWARD};
    }
    if (pin_out1 == GPIO_PinState::GPIO_PIN_RESET && pin_out2 == GPIO_PinState::GPIO_PIN_SET) {
        return ExpectedDirection{Direction::BACKWARD};
    }
    const auto raw{__HAL_TIM_GetCompare(motor.timer, motor.timer_channel)};
    if ((pin_out1 == pin_out2) == GPIO_PinState::GPIO_PIN_RESET && raw >= MAX_RAW) {
        return ExpectedDirection{Direction::SOFT_STOP};
    }
    if ((pin_out1 == pin_out2) == GPIO_PinState::GPIO_PIN_SET && raw <= MIN_RAW) {
        return ExpectedDirection{Direction::FAST_STOP};
    }
    return Unexpected{Error::OK};
}

Error L298N::set_forward(const MotorChannel motor_channel) const noexcept
{
    return set_direction(motor_channel, Direction::FORWARD);
}

Error L298N::set_backward(const MotorChannel motor_channel) const noexcept
{
    return set_direction(motor_channel, Direction::BACKWARD);
}

Error L298N::set_soft_stop(const MotorChannel motor_channel) const noexcept
{
    return set_direction(motor_channel, Direction::SOFT_STOP);
}

Error L298N::set_fast_stop(const MotorChannel motor_channel) const noexcept
{
    return set_direction(motor_channel, Direction::FAST_STOP);
}

const Motor& L298N::get_motor(const MotorChannel motor_channel) const noexcept
{
    if (const auto motor{
            std::ranges::find_if(std::as_const(motors_),
                                 [motor_channel](const Motor& item) { return item.motor_channel == motor_channel; })};
        motor != motors_.cend()) {
        return *motor;
    }
    std::unreachable();
}

Motor& L298N::get_motor(const MotorChannel motor_channel) noexcept
{
    if (auto motor{
            std::ranges::find_if(motors_,
                                 [motor_channel](const Motor& item) { return item.motor_channel == motor_channel; })};
        motor != motors_.cend()) {
        return *motor;
    }
    std::unreachable();
}

Error L298N::motor_channel_to_error(const MotorChannel motor_channel) noexcept
{
    switch (motor_channel) {
        case MotorChannel::CHANNEL1:
            return Error::CHANNEL1;
        case MotorChannel::CHANNEL2:
            return Error::CHANNEL2;
        default:
            std::unreachable();
    }
}

Speed L298N::raw_to_speed(const Raw raw) noexcept
{
    assert(raw <= MAX_RAW && raw >= MIN_RAW);
    return std::clamp(Speed{(raw - MIN_RAW) * (MAX_SPEED_RPM - MIN_SPEED_RPM) / (MAX_RAW - MIN_RAW) + MAX_SPEED_RPM},
                      MIN_SPEED_RPM,
                      MAX_SPEED_RPM);
}

Raw L298N::speed_to_raw(const Speed speed) noexcept
{
    assert(speed <= MAX_SPEED_RPM && speed >= MIN_SPEED_RPM);
    return std::clamp(Raw{(speed - MIN_SPEED_RPM) * (MAX_RAW - MIN_RAW) / (MAX_SPEED_RPM - MIN_SPEED_RPM) + MIN_RAW},
                      MIN_RAW,
                      MAX_RAW);
}

Voltage L298N::raw_to_voltage(const Raw raw) noexcept
{
    assert(raw <= MAX_RAW && raw >= MIN_RAW);
    return std::clamp(Voltage{(raw - MIN_RAW) * (MAX_VOLTAGE_V - MIN_VOLTAGE_V) / (MAX_RAW - MIN_RAW) + MIN_VOLTAGE_V},
                      MIN_VOLTAGE_V,
                      MAX_VOLTAGE_V);
}

Raw L298N::voltage_to_raw(const Voltage voltage) noexcept
{
    assert(voltage <= MAX_VOLTAGE_V && voltage >= MIN_VOLTAGE_V);
    return std::clamp(
        Raw{(voltage - MIN_VOLTAGE_V) * (MAX_RAW - MIN_RAW) / Raw(MAX_VOLTAGE_V - MIN_VOLTAGE_V) + MIN_RAW},
        MIN_RAW,
        MAX_RAW);
}

Torque L298N::raw_to_torque(const Raw raw) noexcept
{
    assert(raw <= MAX_RAW && raw >= MIN_RAW);
    return std::clamp(
        Torque{(raw - MIN_RAW) * (MAX_TORQUE_NM - MIN_TORQUE_NM) / Torque(MAX_RAW - MIN_RAW) + MIN_TORQUE_NM},
        MIN_TORQUE_NM,
        MAX_TORQUE_NM);
}

Raw L298N::torque_to_raw(const Torque torque) noexcept
{
    assert(torque <= MAX_TORQUE_NM && torque >= MIN_TORQUE_NM);
    return std::clamp(Raw{(torque - MIN_TORQUE_NM) * (MAX_RAW - MIN_RAW) / (MAX_TORQUE_NM - MIN_TORQUE_NM) + MIN_RAW},
                      MIN_RAW,
                      MAX_RAW);
}

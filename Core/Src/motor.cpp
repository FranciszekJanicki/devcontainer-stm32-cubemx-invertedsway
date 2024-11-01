#include "motor.hpp"
#include "stm32l4xx_hal_gpio.h"
#include "stm32l4xx_hal_tim.h"
#include <algorithm>
#include <cassert>
#include <expected>
#include <utility>

namespace InvertedSway {

    using Error = Motor::Error;
    using Direction = Motor::Direction;
    using Raw = Motor::Raw;
    using Speed = Motor::Speed;
    using Voltage = Motor::Voltage;
    using Speed = Motor::Speed;
    using Torque = Motor::Torque;
    using ExpectedDirection = Motor::ExpectedDirection;
    using ExpectedRaw = Motor::ExpectedRaw;
    using ExpectedSpeed = Motor::ExpectedSpeed;
    using ExpectedTorque = Motor::ExpectedTorque;
    using ExpectedVoltage = Motor::ExpectedVoltage;
    using Unexpected = Motor::Unexpected;

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

    Speed Motor::raw_to_speed(const Raw raw) noexcept
    {
        assert(raw <= MAX_RAW && raw >= MIN_RAW);
        return std::clamp(
            Speed{(raw - MIN_RAW) * (MAX_SPEED_RPM - MIN_SPEED_RPM) / (MAX_RAW - MIN_RAW) + MAX_SPEED_RPM},
            MIN_SPEED_RPM,
            MAX_SPEED_RPM);
    }

    Raw Motor::speed_to_raw(const Speed speed) noexcept
    {
        assert(speed <= MAX_SPEED_RPM && speed >= MIN_SPEED_RPM);
        return std::clamp(
            Raw{(speed - MIN_SPEED_RPM) * (MAX_RAW - MIN_RAW) / (MAX_SPEED_RPM - MIN_SPEED_RPM) + MIN_RAW},
            MIN_RAW,
            MAX_RAW);
    }

    Voltage Motor::raw_to_voltage(const Raw raw) noexcept
    {
        assert(raw <= MAX_RAW && raw >= MIN_RAW);
        return std::clamp(
            Voltage{(raw - MIN_RAW) * (MAX_VOLTAGE_V - MIN_VOLTAGE_V) / (MAX_RAW - MIN_RAW) + MIN_VOLTAGE_V},
            MIN_VOLTAGE_V,
            MAX_VOLTAGE_V);
    }

    Raw Motor::voltage_to_raw(const Voltage voltage) noexcept
    {
        assert(voltage <= MAX_VOLTAGE_V && voltage >= MIN_VOLTAGE_V);
        return std::clamp(
            Raw{(voltage - MIN_VOLTAGE_V) * (MAX_RAW - MIN_RAW) / Raw(MAX_VOLTAGE_V - MIN_VOLTAGE_V) + MIN_RAW},
            MIN_RAW,
            MAX_RAW);
    }

    Torque Motor::raw_to_torque(const Raw raw) noexcept
    {
        assert(raw <= MAX_RAW && raw >= MIN_RAW);
        return std::clamp(
            Torque{(raw - MIN_RAW) * (MAX_TORQUE_NM - MIN_TORQUE_NM) / Torque(MAX_RAW - MIN_RAW) + MIN_TORQUE_NM},
            MIN_TORQUE_NM,
            MAX_TORQUE_NM);
    }

    Raw Motor::torque_to_raw(const Torque torque) noexcept
    {
        assert(torque <= MAX_TORQUE_NM && torque >= MIN_TORQUE_NM);
        return std::clamp(
            Raw{(torque - MIN_TORQUE_NM) * (MAX_RAW - MIN_RAW) / (MAX_TORQUE_NM - MIN_TORQUE_NM) + MIN_RAW},
            MIN_RAW,
            MAX_RAW);
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
        assert(raw <= MAX_RAW && raw >= MIN_RAW);
        if (raw > MAX_RAW || raw < MIN_RAW) {
            return Error::FAIL;
        }
        __HAL_TIM_SetCompare(this->timer, this->timer_channel, raw);
        return Error::OK;
    }

    ExpectedVoltage Motor::get_compare_voltage() const noexcept
    {
        if (auto raw{this->get_compare_raw()}; !raw.has_value()) {
            return Unexpected{std::move(raw).error()};
        } else {
            return ExpectedVoltage{raw_to_voltage(std::move(raw).value())};
        }
    }

    Error Motor::set_compare_voltage(const Voltage voltage) const noexcept
    {
        assert(voltage <= MAX_VOLTAGE_V && voltage >= MIN_VOLTAGE_V);
        if (voltage >= MAX_VOLTAGE_V || voltage <= MIN_VOLTAGE_V) {
            return Error::FAIL;
        }
        return this->set_compare_raw(voltage_to_raw(voltage));
    }

    ExpectedSpeed Motor::get_compare_speed() const noexcept
    {
        if (auto raw{this->get_compare_raw()}; !raw.has_value()) {
            return Unexpected{std::move(raw).error()};
        } else {
            return ExpectedSpeed{raw_to_speed(std::move(raw).value())};
        }
    }

    Error Motor::set_compare_speed(const Speed speed) const noexcept
    {
        assert(speed <= MAX_SPEED_RPM && speed >= MIN_SPEED_RPM);
        if (speed >= MAX_SPEED_RPM || speed <= MIN_SPEED_RPM) {
            return Error::FAIL;
        }
        return this->set_compare_raw(speed_to_raw(speed));
    }

    ExpectedTorque Motor::get_compare_torque() const noexcept
    {
        if (auto raw{this->get_compare_raw()}; !raw.has_value()) {
            return Unexpected{std::move(raw).error()};
        } else {
            return ExpectedSpeed{raw_to_torque(std::move(raw).value())};
        }
    }

    Error Motor::set_compare_torque(const Torque torque) const noexcept
    {
        assert(torque <= MAX_TORQUE_NM && torque >= MIN_TORQUE_NM);
        if (torque >= MAX_TORQUE_NM || torque <= MIN_TORQUE_NM) {
            return Error::FAIL;
        }
        return this->set_compare_raw(torque_to_raw(torque));
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
        const auto out1{HAL_GPIO_ReadPin(this->gpio, this->pin_out1)};
        const auto out2{HAL_GPIO_ReadPin(this->gpio, this->pin_out2)};
        if (out1 == GPIO_PinState::GPIO_PIN_SET && out2 == GPIO_PinState::GPIO_PIN_RESET) {
            return ExpectedDirection{Direction::FORWARD};
        }
        if (out1 == GPIO_PinState::GPIO_PIN_RESET && out2 == GPIO_PinState::GPIO_PIN_SET) {
            return ExpectedDirection{Direction::BACKWARD};
        }
        const auto raw{__HAL_TIM_GetCompare(this->timer, this->timer_channel)};
        if ((out1 == out2) == GPIO_PinState::GPIO_PIN_RESET && raw >= MAX_RAW) {
            return ExpectedDirection{Direction::SOFT_STOP};
        }
        if ((out1 == out2) == GPIO_PinState::GPIO_PIN_SET && raw <= MIN_RAW) {
            return ExpectedDirection{Direction::FAST_STOP};
        }
        return Unexpected{Error::OK};
    }

    Error Motor::set_forward() const noexcept
    {
        if (!this->initialized) {
            return Error::FAIL;
        }
        return this->set_direction(Direction::FORWARD);
    }

    Error Motor::set_backward() const noexcept
    {
        if (!this->initialized) {
            return Error::FAIL;
        }
        return this->set_direction(Direction::BACKWARD);
    }

    Error Motor::set_fast_stop() const noexcept
    {
        if (!this->initialized) {
            return Error::FAIL;
        }
        return this->set_direction(Direction::FAST_STOP);
    }

    Error Motor::set_soft_stop() const noexcept
    {
        if (!this->initialized) {
            return Error::FAIL;
        }
        return this->set_direction(Direction::SOFT_STOP);
    }

    Error Motor::toggle_direction() const noexcept
    {
        if (!this->initialized) {
            return Error::FAIL;
        }
        if (auto direction{this->get_direction()}; !direction.has_value()) {
            return std::move(direction).error();
        } else {
            if (auto err{this->set_direction(Direction::FAST_STOP)}; err != Error::OK) {
                return err;
            }
            if (direction.value() == Direction::FORWARD) {
                return this->set_direction(Direction::BACKWARD);
            } else if (direction.value() == Direction::BACKWARD) {
                return this->set_direction(Direction::FORWARD);
            }
        }
    }

}; // namespace InvertedSway
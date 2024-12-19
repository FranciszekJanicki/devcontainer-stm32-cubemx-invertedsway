#include "motor.hpp"
#include "stm32l4xx_hal_tim.h"
#include <algorithm>
#include <expected>
#include <utility>

using namespace InvertedSway;
using Error = Motor::Error;
using Direction = Motor::Direction;
using Raw = Motor::Raw;
using Speed = Motor::Speed;
using Voltage = Motor::Voltage;
using Speed = Motor::Speed;
using ExpectedRaw = Motor::ExpectedRaw;
using ExpectedSpeed = Motor::ExpectedSpeed;
using ExpectedVoltage = Motor::ExpectedVoltage;
using Unexpected = Motor::Unexpected;

namespace InvertedSway {

    const char* Motor::error_to_string(Error const error) noexcept
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

    Speed Motor::raw_to_speed(Raw const raw) noexcept
    {
        return (std::clamp(raw, MIN_RAW, MAX_RAW) - MIN_RAW) * (MAX_SPEED_RPM - MIN_SPEED_RPM) / (MAX_RAW - MIN_RAW) +
               MAX_SPEED_RPM;
    }

    Raw Motor::speed_to_raw(Speed const speed) noexcept
    {
        return (std::clamp(speed, MIN_SPEED_RPM, MAX_SPEED_RPM) - MIN_SPEED_RPM) * (MAX_RAW - MIN_RAW) /
                   (MAX_SPEED_RPM - MIN_SPEED_RPM) +
               MIN_RAW;
    }

    Voltage Motor::raw_to_voltage(Raw const raw) noexcept
    {
        return (std::clamp(raw, MIN_RAW, MAX_RAW) - MIN_RAW) * (MAX_VOLTAGE_V - MIN_VOLTAGE_V) / (MAX_RAW - MIN_RAW) +
               MIN_VOLTAGE_V;
    }

    Raw Motor::voltage_to_raw(Voltage const voltage) noexcept
    {
        return (std::clamp(voltage, MIN_VOLTAGE_V, MAX_VOLTAGE_V) - MIN_VOLTAGE_V) * (MAX_RAW - MIN_RAW) /
                   (MAX_VOLTAGE_V - MIN_VOLTAGE_V) +
               MIN_RAW;
    }

    Motor::Motor(TimerHandle timer,
                 std::uint32_t const timer_channel,
                 GpioHandle gpio,
                 std::uint16_t const pin_in1,
                 std::uint16_t const pin_in2) noexcept :
        timer_{timer}, timer_channel_{timer_channel}, gpio_{gpio}, pin_in1_{pin_in1}, pin_in2_{pin_in2}
    {
        this->initialize();
    }

    Motor::~Motor() noexcept
    {
        this->deinitialize();
    }

    Error Motor::initialize() noexcept
    {
        if (this->timer_ == nullptr || this->gpio_ == nullptr) {
            return Error::INIT;
        }
        if (HAL_TIM_PWM_Start(this->timer_, this->timer_channel_) != HAL_OK) {
            return Error::INIT;
        }
        this->initialized_ = true;
        return Error::OK;
    }

    Error Motor::deinitialize() noexcept
    {
        if (this->timer_ == nullptr || this->gpio_ == nullptr) {
            return Error::DEINIT;
        }
        if (HAL_TIM_PWM_Stop(this->timer_, this->timer_channel_) != HAL_OK) {
            return Error::DEINIT;
        }
        this->initialized_ = false;
        return Error::OK;
    }

    ExpectedRaw Motor::get_compare_raw() const noexcept
    {
        if (!this->initialized_) {
            return Unexpected{Error::FAIL};
        }
        return ExpectedRaw{__HAL_TIM_GetCompare(this->timer_, this->timer_channel_)};
    }

    Error Motor::set_compare_raw(Raw const raw) const noexcept
    {
        if (!this->initialized_) {
            return Error::FAIL;
        }
        if (raw > MAX_RAW || raw < MIN_RAW) {
            return Error::FAIL;
        }
        __HAL_TIM_SetCompare(this->timer_, this->timer_channel_, raw);
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

    Error Motor::set_compare_voltage(Voltage const voltage) const noexcept
    {
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

    Error Motor::set_compare_speed(Speed const speed) const noexcept
    {
        if (speed >= MAX_SPEED_RPM || speed <= MIN_SPEED_RPM) {
            return Error::FAIL;
        }
        return this->set_compare_raw(speed_to_raw(speed));
    }

    Error Motor::set_direction(Direction const direction) const noexcept
    {
        if (!this->initialized_) {
            return Error::FAIL;
        }
        switch (direction) {
            case Direction::SOFT_STOP:
                HAL_GPIO_WritePin(this->gpio_, this->pin_in1_, GPIO_PinState::GPIO_PIN_RESET);
                HAL_GPIO_WritePin(this->gpio_, this->pin_in2_, GPIO_PinState::GPIO_PIN_RESET);
                return this->set_compare_raw(MAX_RAW);
            case Direction::FAST_STOP:
                HAL_GPIO_WritePin(this->gpio_, this->pin_in1_, GPIO_PinState::GPIO_PIN_SET);
                HAL_GPIO_WritePin(this->gpio_, this->pin_in2_, GPIO_PinState::GPIO_PIN_SET);
                return this->set_compare_raw(MIN_RAW);
            case Direction::FORWARD:
                HAL_GPIO_WritePin(this->gpio_, this->pin_in1_, GPIO_PinState::GPIO_PIN_SET);
                HAL_GPIO_WritePin(this->gpio_, this->pin_in2_, GPIO_PinState::GPIO_PIN_RESET);
                return Error::OK;
            case Direction::BACKWARD:
                HAL_GPIO_WritePin(this->gpio_, this->pin_in1_, GPIO_PinState::GPIO_PIN_RESET);
                HAL_GPIO_WritePin(this->gpio_, this->pin_in2_, GPIO_PinState::GPIO_PIN_SET);
                return Error::OK;
            default:
                return Error::FAIL;
        }
        return Error::OK;
    }

    Error Motor::set_forward() const noexcept
    {
        if (!this->initialized_) {
            return Error::FAIL;
        }
        return this->set_direction(Direction::FORWARD);
    }

    Error Motor::set_backward() const noexcept
    {
        if (!this->initialized_) {
            return Error::FAIL;
        }
        return this->set_direction(Direction::BACKWARD);
    }

    Error Motor::set_fast_stop() const noexcept
    {
        if (!this->initialized_) {
            return Error::FAIL;
        }
        return this->set_direction(Direction::FAST_STOP);
    }

    Error Motor::set_soft_stop() const noexcept
    {
        if (!this->initialized_) {
            return Error::FAIL;
        }
        return this->set_direction(Direction::SOFT_STOP);
    }

    Error Motor::set_compare_max() const noexcept
    {
        if (!this->initialized_) {
            return Error::FAIL;
        }
        return this->set_compare_raw(MAX_RAW);
    }

    Error Motor::set_compare_min() const noexcept
    {
        if (!this->initialized_) {
            return Error::FAIL;
        }
        return this->set_compare_raw(MIN_RAW);
    }
}; // namespace InvertedSway
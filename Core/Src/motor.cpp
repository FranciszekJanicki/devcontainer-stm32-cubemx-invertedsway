#include "motor.hpp"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_gpio.h"
#include "stm32l4xx_hal_tim.h"
#include <cassert>
#include <expected>

using Error = Motor::Error;
using Direction = Motor::Direction;
using Raw = Motor::Raw;
using ExpectedDirection = Motor::ExpectedDirection;
using ExpectedRaw = Motor::ExpectedRaw;
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
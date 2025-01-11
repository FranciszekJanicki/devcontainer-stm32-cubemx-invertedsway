#include "motor.hpp"
#include "stm32l4xx_hal_tim.h"
#include <algorithm>
#include <expected>
#include <utility>

using namespace InvertedSway;
using Direction = Motor::Direction;
using Raw = Motor::Raw;
using Voltage = Motor::Voltage;

namespace InvertedSway {

    Motor::Motor(PWMDevice const& pwm_device,
                 GPIOHandle const gpio,
                 std::uint16_t const pin_in1,
                 std::uint16_t const pin_in2) noexcept :
        pwm_device_{pwm_device}, gpio_{gpio}, pin_in1_{pin_in1}, pin_in2_{pin_in2}
    {
        this->initialize();
    }

    Motor::~Motor() noexcept
    {
        this->deinitialize();
    }

    void Motor::set_voltage(Voltage const voltage) const noexcept
    {
        return this->pwm_device_.set_compare_voltage(voltage);
    }

    void Motor::set_voltage_max() const noexcept
    {
        this->pwm_device_.set_compare_max();
    }

    void Motor::set_voltage_min() const noexcept
    {
        this->pwm_device_.set_compare_min();
    }

    void Motor::set_direction(Direction const direction) const noexcept
    {
        if (!this->initialized_) {
            return;
        }

        if (direction == Direction::SOFT_STOP) {
            HAL_GPIO_WritePin(this->gpio_, this->pin_in1_, GPIO_PinState::GPIO_PIN_RESET);
            HAL_GPIO_WritePin(this->gpio_, this->pin_in2_, GPIO_PinState::GPIO_PIN_RESET);
            this->set_voltage_max();
        } else if (direction == Direction::FAST_STOP) {
            HAL_GPIO_WritePin(this->gpio_, this->pin_in1_, GPIO_PinState::GPIO_PIN_SET);
            HAL_GPIO_WritePin(this->gpio_, this->pin_in2_, GPIO_PinState::GPIO_PIN_SET);
            this->set_voltage_min();
        } else if (direction == Direction::FORWARD) {
            HAL_GPIO_WritePin(this->gpio_, this->pin_in1_, GPIO_PinState::GPIO_PIN_SET);
            HAL_GPIO_WritePin(this->gpio_, this->pin_in2_, GPIO_PinState::GPIO_PIN_RESET);
        } else if (direction == Direction::BACKWARD) {
            HAL_GPIO_WritePin(this->gpio_, this->pin_in1_, GPIO_PinState::GPIO_PIN_RESET);
            HAL_GPIO_WritePin(this->gpio_, this->pin_in2_, GPIO_PinState::GPIO_PIN_SET);
        }
    }

    void Motor::set_forward() const noexcept
    {
        return this->set_direction(Direction::FORWARD);
    }

    void Motor::set_backward() const noexcept
    {
        return this->set_direction(Direction::BACKWARD);
    }

    void Motor::set_fast_stop() const noexcept
    {
        return this->set_direction(Direction::FAST_STOP);
    }

    void Motor::set_soft_stop() const noexcept
    {
        return this->set_direction(Direction::SOFT_STOP);
    }

    void Motor::initialize() noexcept
    {
        if (this->gpio_ != nullptr) {
            this->pwm_device_.initialize();
            this->initialized_ = true;
        }
    }

    void Motor::deinitialize() noexcept
    {
        if (this->gpio_ != nullptr) {
            this->pwm_device_.deinitialize();
            this->initialized_ = false;
        }
    }

}; // namespace InvertedSway
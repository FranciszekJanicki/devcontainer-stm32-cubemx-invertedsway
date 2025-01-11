#include "pwm_device.hpp"
#include "common.hpp"
#include "stm32l4xx_hal.h"
#include <algorithm>
#include <utility>

using namespace InvertedSway;
using Raw = PWMDevice::Raw;
using Voltage = PWMDevice::Voltage;

namespace InvertedSway {

    void PWMDevice::set_compare_raw(Raw const raw) const noexcept
    {
        if (this->initialized) {
            __HAL_TIM_SetCompare(this->timer, this->timer_channel, raw);
        }
    }

    void PWMDevice::set_compare_voltage(Voltage const voltage) const noexcept
    {
        this->set_compare_raw(this->voltage_to_raw(voltage));
    }

    void PWMDevice::set_compare_max() const noexcept
    {
        this->set_compare_raw(this->counter_period);
    }

    void PWMDevice::set_compare_min() const noexcept
    {
        this->set_compare_raw(0UL);
    }

    void PWMDevice::initialize() noexcept
    {
        if (this->timer != nullptr) {
            HAL_TIM_PWM_Start(this->timer, this->timer_channel);
            this->initialized = true;
        }
    }

    void PWMDevice::deinitialize() noexcept
    {
        if (this->timer != nullptr) {
            HAL_TIM_PWM_Stop(this->timer, this->timer_channel);
            this->initialized = false;
        }
    }

    Raw PWMDevice::voltage_to_raw(Voltage const voltage) const noexcept
    {
        return (std::clamp(voltage, this->min_voltage, this->max_voltage) - this->min_voltage) *
               (this->counter_period) / (this->max_voltage - this->min_voltage);
    }

    Voltage PWMDevice::raw_to_voltage(Raw const raw) const noexcept
    {
        return std::clamp(raw, Raw{0}, this->counter_period) * (this->max_voltage - this->min_voltage) /
                   (this->counter_period) +
               this->min_voltage;
    }

}; // namespace InvertedSway
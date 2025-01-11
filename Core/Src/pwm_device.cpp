#include "pwm_device.hpp"
#include "common.hpp"
#include "stm32l4xx_hal.h"
#include <algorithm>
#include <utility>

using namespace InvertedSway;
using Raw = PWMDevice::Raw;
using Voltage = PWMDevice::Voltage;

namespace InvertedSway {

    PWMDevice::PWMDevice(TimerHandle const timer,
                         std::uint32_t const timer_channel,
                         Raw const counter_period,
                         Voltage const min_voltage,
                         Voltage const max_voltage) noexcept :
        timer_{timer},
        timer_channel_{timer_channel},
        counter_period_{counter_period},
        min_voltage_{min_voltage},
        max_voltage_{max_voltage}
    {
        this->initialize();
    }

    PWMDevice::~PWMDevice() noexcept
    {
        this->deinitialize();
    }

    void PWMDevice::set_compare_raw(Raw const raw) const noexcept
    {
        if (this->initialized_) {
            __HAL_TIM_SetCompare(this->timer_, this->timer_channel_, raw);
        }
    }

    void PWMDevice::set_compare_voltage(Voltage const voltage) const noexcept
    {
        this->set_compare_raw(this->voltage_to_raw(voltage));
    }

    void PWMDevice::set_compare_max() const noexcept
    {
        this->set_compare_raw(this->counter_period_);
    }

    void PWMDevice::set_compare_min() const noexcept
    {
        this->set_compare_raw(0UL);
    }

    void PWMDevice::initialize() noexcept
    {
        if (this->timer_ != nullptr) {
            HAL_TIM_PWM_Start(this->timer_, this->timer_channel_);
            this->initialized_ = true;
        }
    }

    void PWMDevice::deinitialize() noexcept
    {
        if (this->timer_ != nullptr) {
            HAL_TIM_PWM_Stop(this->timer_, this->timer_channel_);
            this->initialized_ = false;
        }
    }

    Raw PWMDevice::voltage_to_raw(Voltage const voltage) const noexcept
    {
        return (std::clamp(voltage, this->min_voltage_, this->max_voltage_) - this->min_voltage_) *
               (this->counter_period_) / (this->max_voltage_ - this->min_voltage_);
    }

    Voltage PWMDevice::raw_to_voltage(Raw const raw) const noexcept
    {
        return std::clamp(raw, Raw{0}, this->counter_period_) * (this->max_voltage_ - this->min_voltage_) /
                   (this->counter_period_) +
               this->min_voltage_;
    }

}; // namespace InvertedSway
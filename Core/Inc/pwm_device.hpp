#ifndef PWM_DEVICE_HPP
#define PWM_DEVICE_HPP

#include "common.hpp"
#include "stm32l4xx_hal.h"
#include <cstdint>

namespace InvertedSway {

    struct PWMDevice {
    public:
        using Voltage = float;
        using Raw = std::uint32_t;

        PWMDevice() noexcept = default;
        PWMDevice(TimerHandle const timer,
                  std::uint32_t const timer_channel,
                  Raw const counter_period,
                  Voltage const min_voltage,
                  Voltage const max_voltage) noexcept;

        PWMDevice(PWMDevice const& other) noexcept = delete;
        PWMDevice(PWMDevice&& other) noexcept = default;

        PWMDevice& operator=(PWMDevice const& other) noexcept = delete;
        PWMDevice& operator=(PWMDevice&& other) noexcept = default;

        ~PWMDevice() noexcept;

        void set_compare_raw(Raw const raw) const noexcept;
        void set_compare_voltage(Voltage const voltage) const noexcept;
        void set_compare_max() const noexcept;
        void set_compare_min() const noexcept;

        void initialize() noexcept;
        void deinitialize() noexcept;

        Raw voltage_to_raw(Voltage const voltage) const noexcept;
        Voltage raw_to_voltage(Raw const raw) const noexcept;

        bool initialized_{false};

        TimerHandle timer_{nullptr};
        std::uint32_t timer_channel_{};

        Raw counter_period_{};
        Voltage min_voltage_{};
        Voltage max_voltage_{};
    };

}; // namespace InvertedSway

#endif // PWM_DEVICE_HPP
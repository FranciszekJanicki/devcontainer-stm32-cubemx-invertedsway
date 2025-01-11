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

        void set_compare_raw(Raw const raw) const noexcept;
        void set_compare_voltage(Voltage const voltage) const noexcept;
        void set_compare_max() const noexcept;
        void set_compare_min() const noexcept;

        void initialize() noexcept;
        void deinitialize() noexcept;

        Raw voltage_to_raw(Voltage const voltage) const noexcept;
        Voltage raw_to_voltage(Raw const raw) const noexcept;

        TimerHandle timer{nullptr};
        std::uint32_t timer_channel{};

        Raw counter_period{};
        Voltage min_voltage{};
        Voltage max_voltage{};

        bool initialized{false};
    };

}; // namespace InvertedSway

#endif // PWM_DEVICE_HPP
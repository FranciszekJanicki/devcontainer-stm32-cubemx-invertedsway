#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "common.hpp"
#include "pwm_device.hpp"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_tim.h"
#include <cstdint>

namespace InvertedSway {

    struct Motor {
    public:
        enum struct Direction {
            FORWARD,
            BACKWARD,
            FAST_STOP,
            SOFT_STOP,
        };

        using Raw = PWMDevice::Raw;
        using Voltage = PWMDevice::Voltage;

        void set_voltage(Voltage const voltage) const noexcept;
        void set_voltage_max() const noexcept;
        void set_voltage_min() const noexcept;

        void set_direction(Direction const direction) const noexcept;
        void set_forward() const noexcept;
        void set_backward() const noexcept;
        void set_soft_stop() const noexcept;
        void set_fast_stop() const noexcept;

        PWMDevice pwm_device{};

        GPIOHandle gpio{nullptr};
        std::uint16_t pin_left{};
        std::uint16_t pin_right{};
    };

}; // namespace InvertedSway

#endif // MOTOR_HPP
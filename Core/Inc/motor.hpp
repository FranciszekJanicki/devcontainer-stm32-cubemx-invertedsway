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

        Motor() noexcept = default;
        Motor(PWMDevice const& pwm_device,
              GPIOHandle const gpio,
              std::uint16_t const pin_in1,
              std::uint16_t const pin_in2) noexcept;

        Motor(Motor const& motor) = delete;
        Motor(Motor&& motor) noexcept = default;

        Motor& operator=(Motor const& motor) = delete;
        Motor& operator=(Motor&& motor) noexcept = default;

        ~Motor() noexcept;

        void set_voltage(Voltage const voltage) const noexcept;
        void set_voltage_max() const noexcept;
        void set_voltage_min() const noexcept;

        void set_direction(Direction const direction) const noexcept;
        void set_forward() const noexcept;
        void set_backward() const noexcept;
        void set_soft_stop() const noexcept;
        void set_fast_stop() const noexcept;

    private:
        void initialize() noexcept;
        void deinitialize() noexcept;

        bool initialized_{false};

        PWMDevice pwm_device_{};

        GPIOHandle gpio_{nullptr};
        std::uint16_t pin_in1_{};
        std::uint16_t pin_in2_{};
    };

}; // namespace InvertedSway

#endif // MOTOR_HPP
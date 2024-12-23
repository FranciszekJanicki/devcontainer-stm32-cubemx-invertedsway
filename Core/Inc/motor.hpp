#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "common.hpp"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_tim.h"
#include <cstdint>
#include <expected>

namespace InvertedSway {

    struct Motor {
    public:
        enum struct Error {
            OK,
            INIT,
            DEINIT,
            FAIL,
        };

        enum struct Direction {
            FORWARD,
            BACKWARD,
            FAST_STOP,
            SOFT_STOP,
        };

        using Raw = std::uint16_t;
        using Speed = double;
        using Voltage = double;
        using ExpectedVoltage = std::expected<Voltage, Error>;
        using ExpectedSpeed = std::expected<Speed, Error>;
        using ExpectedRaw = std::expected<Raw, Error>;
        using Unexpected = std::unexpected<Error>;

        static const char* error_to_string(const Error error) noexcept;

        static constexpr Voltage MAX_VOLTAGE_V{6};
        static constexpr Voltage MIN_VOLTAGE_V{0};

        static constexpr Raw COUNTER_PERIOD{39999};
        static constexpr Raw MAX_RAW{COUNTER_PERIOD};
        static constexpr Raw MIN_RAW{0};

        Motor() noexcept = default;

        Motor(TimerHandle const timer,
              std::uint32_t const timer_channel,
              GPIOHandle const gpio,
              std::uint16_t const pin_in1,
              std::uint16_t const pin_in2) noexcept;

        Motor(Motor const& motor) = delete;
        Motor(Motor&& motor) noexcept = default;

        Motor& operator=(Motor const& motor) = delete;
        Motor& operator=(Motor&& motor) noexcept = default;

        ~Motor() noexcept;

        [[nodiscard]] Error set_compare_raw(Raw const raw) const noexcept;
        [[nodiscard]] Error set_compare_voltage(Voltage const voltage) const noexcept;

        [[nodiscard]] ExpectedRaw get_compare_raw() const noexcept;
        [[nodiscard]] ExpectedVoltage get_compare_voltage() const noexcept;

        [[nodiscard]] Error set_direction(Direction const direction) const noexcept;
        [[nodiscard]] Error set_forward() const noexcept;
        [[nodiscard]] Error set_backward() const noexcept;
        [[nodiscard]] Error set_soft_stop() const noexcept;
        [[nodiscard]] Error set_fast_stop() const noexcept;
        [[nodiscard]] Error set_compare_max() const noexcept;
        [[nodiscard]] Error set_compare_min() const noexcept;

    private:
        static Raw clamp_raw(Raw const raw) noexcept;
        static Voltage clamp_voltage(Voltage const voltage) noexcept;
        static Speed clamp_speed(Speed const speed) noexcept;

        static Speed raw_to_speed(Raw const raw) noexcept;
        static Voltage raw_to_voltage(Raw const raw) noexcept;

        static Raw speed_to_raw(Speed const speed) noexcept;
        static Raw voltage_to_raw(Voltage const voltage) noexcept;

        Error initialize() noexcept;
        Error deinitialize() noexcept;

        TimerHandle timer_{nullptr};
        std::uint32_t timer_channel_{};

        GPIOHandle gpio_{nullptr};
        std::uint16_t pin_in1_{};
        std::uint16_t pin_in2_{};

        bool initialized_{false};
    };

}; // namespace InvertedSway

#endif // MOTOR_HPP
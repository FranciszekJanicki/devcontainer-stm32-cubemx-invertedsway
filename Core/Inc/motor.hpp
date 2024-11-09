#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "common.hpp"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_gpio.h"
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
        using Torque = double;
        using ExpectedVoltage = std::expected<Voltage, Error>;
        using ExpectedSpeed = std::expected<Speed, Error>;
        using ExpectedTorque = std::expected<Torque, Error>;
        using ExpectedRaw = std::expected<Raw, Error>;
        using ExpectedDirection = std::expected<Direction, Error>;
        using Unexpected = std::unexpected<Error>;

        static const char* error_to_string(const Error error) noexcept;

        [[nodiscard]] ExpectedRaw get_compare_raw() const noexcept;
        [[nodiscard]] Error set_compare_raw(const Raw raw) const noexcept;

        [[nodiscard]] ExpectedVoltage get_compare_voltage() const noexcept;
        [[nodiscard]] Error set_compare_voltage(const Voltage voltage) const noexcept;

        [[nodiscard]] ExpectedSpeed get_compare_speed() const noexcept;
        [[nodiscard]] Error set_compare_speed(const Speed speed) const noexcept;

        [[nodiscard]] ExpectedTorque get_compare_torque() const noexcept;
        [[nodiscard]] Error set_compare_torque(const Torque torque) const noexcept;

        [[nodiscard]] Error set_direction(const Direction direction) const noexcept;
        [[nodiscard]] ExpectedDirection get_direction() const noexcept;

        [[nodiscard]] Error set_forward() const noexcept;
        [[nodiscard]] Error set_backward() const noexcept;
        [[nodiscard]] Error set_soft_stop() const noexcept;
        [[nodiscard]] Error set_fast_stop() const noexcept;
        [[nodiscard]] Error toggle_direction() const noexcept;

        [[nodiscard]] Error initialize() noexcept;
        [[nodiscard]] Error deinitialize() noexcept;

        TimerHandle timer{nullptr};
        std::uint32_t timer_channel{};

        GpioHandle gpio{nullptr};
        std::uint16_t pin_in1{};
        std::uint16_t pin_in2{};
        std::uint16_t pin_out1{};
        std::uint16_t pin_out2{};

        bool initialized{false};

    private:
        static Speed raw_to_speed(const Raw raw) noexcept;
        static Raw speed_to_raw(const Speed speed) noexcept;

        static Voltage raw_to_voltage(const Raw raw) noexcept;
        static Raw voltage_to_raw(const Voltage voltage) noexcept;

        static Torque raw_to_torque(const Raw raw) noexcept;
        static Raw torque_to_raw(const Torque torque) noexcept;

        static constexpr Raw COUNTER_PERIOD{62499};
        static constexpr Raw MAX_RAW{COUNTER_PERIOD};
        static constexpr Raw MIN_RAW{0};

        static inline Voltage MAX_VOLTAGE_V{12};
        static inline Voltage MIN_VOLTAGE_V{0};

        static constexpr Speed MIN_SPEED_RPM{0};
        static constexpr Speed MAX_SPEED_RPM{1000};

        static constexpr Speed MIN_TORQUE_NM{0};
        static constexpr Speed MAX_TORQUE_NM{1000};
    };

}; // namespace InvertedSway

#endif // MOTOR_HPP
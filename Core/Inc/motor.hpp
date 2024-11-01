#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "common.hpp"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_gpio.h"
#include "stm32l4xx_hal_tim.h"
#include <cstdint>
#include <expected>

struct Motor {
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
    using ExpectedRaw = std::expected<Raw, Error>;
    using ExpectedDirection = std::expected<Direction, Error>;
    using Unexpected = std::unexpected<Error>;

    static const char* error_to_string(const Error error) noexcept;

    [[nodiscard]] ExpectedRaw get_compare_raw() const noexcept;
    [[nodiscard]] Error set_compare_raw(const Raw raw) const noexcept;

    [[nodiscard]] Error set_direction(const Direction direction) const noexcept;
    [[nodiscard]] ExpectedDirection get_direction() const noexcept;

    [[nodiscard]] Error initialize() noexcept;
    [[nodiscard]] Error deinitialize() noexcept;

    static constexpr Raw MAX_RAW{static_cast<Raw>(std::pow(2, 16) - 1)};
    static constexpr Raw MIN_RAW{0};

    TimerHandle timer{nullptr};
    std::uint32_t timer_channel{};

    GpioHandle gpio{nullptr};
    std::uint16_t pin_in1{};
    std::uint16_t pin_in2{};
    std::uint16_t pin_out1{};
    std::uint16_t pin_out2{};

    bool initialized{false};
};

#endif // MOTOR_HPP
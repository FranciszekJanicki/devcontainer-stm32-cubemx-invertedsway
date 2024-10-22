#ifndef L298N_HPP
#define L298N_HPP

#include "common.hpp"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_gpio.h"
#include "stm32l4xx_hal_tim.h"
#include <array>
#include <cmath>
#include <cstdint>
#include <expected>
#include <functional>
#include <optional>
#include <span>
#include <utility>

class L298N {
public:
    enum struct Error {
        OK,
        INIT,
        DEINIT,
        CHANNEL1,
        CHANNEL2,
    };

    enum struct Direction {
        FORWARD,
        BACKWARD,
        FAST_STOP,
        SOFT_STOP,
    };

    enum struct MotorChannel {
        CHANNEL1,
        CHANNEL2,
    };

    struct Motor {
        MotorChannel motor_channel{};

        TimerHandle timer{nullptr};
        std::uint32_t timer_channel{};

        GpioHandle gpio{nullptr};
        std::uint16_t pin_in1{};
        std::uint16_t pin_in2{};
        std::uint16_t pin_out1{};
        std::uint16_t pin_out2{};

        bool initialized{false};
    };

    using Raw = std::uint16_t;
    using Speed = std::double_t;
    using Voltage = std::double_t;
    using ExpectedDirection = std::expected<Direction, Error>;
    using ExpectedRaw = std::expected<Raw, Error>;
    using ExpectedVoltage = std::expected<Voltage, Error>;
    using ExpectedSpeed = std::expected<Speed, Error>;
    using Unexpected = std::unexpected<Error>;
    using Motors = std::array<Motor, 2>;

    static const char* error_to_string(const Error error) noexcept;

    L298N(UartHandle uart, const Motors& motors) noexcept;
    L298N(UartHandle uart, Motors&& motors) noexcept;

    L298N(const L298N& other) noexcept = default;
    L298N(L298N&& other) noexcept = default;

    L298N& operator=(const L298N& other) noexcept = default;
    L298N& operator=(L298N&& other) noexcept = default;

    ~L298N() noexcept;

    const Motors& motors() const& noexcept;
    Motors&& motors() && noexcept;

    void motors(const Motors& motors) noexcept;
    void motors(Motors&& motors) noexcept;

    ExpectedRaw get_compare_raw(const MotorChannel motor_channel) const noexcept;
    Error set_compare_raw(const MotorChannel motor_channel, const Raw raw) const noexcept;

    ExpectedVoltage get_compare_voltage(const MotorChannel motor_channel) const noexcept;
    Error set_compare_voltage(const MotorChannel motor_channel, const Voltage voltage) const noexcept;

    ExpectedSpeed get_compare_speed(const MotorChannel motor_channel) const noexcept;
    Error set_compare_speed(const MotorChannel motor_channel, const Speed speed) const noexcept;

    Error set_direction(const MotorChannel motor_channel, const Direction direction) const noexcept;
    ExpectedDirection get_direction(const MotorChannel motor_channel) const noexcept;

    Error set_forward(const MotorChannel motor_channel) const noexcept;
    Error set_backward(const MotorChannel motor_channel) const noexcept;
    Error set_soft_stop(const MotorChannel motor_channel) const noexcept;
    Error set_fast_stop(const MotorChannel motor_channel) const noexcept;

    const Motor& get_motor(const MotorChannel channel) const noexcept;
    Motor& get_motor(const MotorChannel motor_channel) noexcept;

private:
    static Error motor_channel_to_error(const MotorChannel motor_channel) noexcept;

    static Speed raw_to_speed(const Raw raw) noexcept;
    static Raw speed_to_raw(const Speed speed) noexcept;

    static Voltage raw_to_voltage(const Raw raw) noexcept;
    static Raw voltage_to_raw(const Voltage voltage) noexcept;

    static constexpr std::uint8_t PWM_RESOLUTION_BITS{16};
    static constexpr std::uint64_t CLOCK_RESOLUTION_HZ{16000000};
    static constexpr std::uint64_t TIMER_PRESCALER{40};
    static constexpr std::uint64_t TIMER_RESOLUTION_HZ{CLOCK_RESOLUTION_HZ / TIMER_PRESCALER};

    static constexpr Voltage MAX_VOLTAGE_V{12};
    static constexpr Voltage MIN_VOLTAGE_V{0};

    static constexpr Raw MAX_RAW{static_cast<Raw>(std::pow(2, PWM_RESOLUTION_BITS) - 1)};
    static constexpr Raw MIN_RAW{0};

    static constexpr Speed MIN_SPEED_RPM{0};
    static constexpr Speed MAX_SPEED_RPM{100000};

    Error initialize(Motor& motor) noexcept;
    Error deinitialize(Motor& motor) noexcept;

    Error print_and_return(const Error error) const noexcept;

    UartHandle uart_{nullptr};

    Motors motors_{};

    mutable char uart_buffer_[100];

    bool initialized_{false};
};

#endif // L298N_HPP
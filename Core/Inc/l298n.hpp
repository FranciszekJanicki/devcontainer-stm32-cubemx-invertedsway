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
    using Torque = std::double_t;
    using ExpectedDirection = std::expected<Direction, Error>;
    using ExpectedRaw = std::expected<Raw, Error>;
    using ExpectedVoltage = std::expected<Voltage, Error>;
    using ExpectedSpeed = std::expected<Speed, Error>;
    using ExpectedTorque = std::expected<Torque, Error>;
    using Unexpected = std::unexpected<Error>;
    using Motors = std::array<Motor, 2>;

    static const char* error_to_string(const Error error) noexcept;

    L298N(UartHandle uart, const Motors& motors) noexcept;
    L298N(UartHandle uart, Motors&& motors) noexcept;

    L298N(const L298N& other) noexcept = delete;
    L298N(L298N&& other) noexcept = delete;

    L298N& operator=(const L298N& other) noexcept = delete;
    L298N& operator=(L298N&& other) noexcept = delete;

    ~L298N() noexcept;

    const Motors& motors() const& noexcept;
    Motors&& motors() && noexcept;

    void motors(const Motors& motors) noexcept;
    void motors(Motors&& motors) noexcept;

    [[nodiscard]] ExpectedRaw get_compare_raw(const MotorChannel motor_channel) const noexcept;
    [[nodiscard]] Error set_compare_raw(const MotorChannel motor_channel, const Raw raw) const noexcept;

    [[nodiscard]] ExpectedVoltage get_compare_voltage(const MotorChannel motor_channel) const noexcept;
    [[nodiscard]] Error set_compare_voltage(const MotorChannel motor_channel, const Voltage voltage) const noexcept;

    [[nodiscard]] ExpectedSpeed get_compare_speed(const MotorChannel motor_channel) const noexcept;
    [[nodiscard]] Error set_compare_speed(const MotorChannel motor_channel, const Speed speed) const noexcept;

    [[nodiscard]] ExpectedTorque get_compare_torque(const MotorChannel motor_channel) const noexcept;
    [[nodiscard]] Error set_compare_torque(const MotorChannel motor_channel, const Torque torque) const noexcept;

    [[nodiscard]] Error set_direction(const MotorChannel motor_channel, const Direction direction) const noexcept;
    [[nodiscard]] ExpectedDirection get_direction(const MotorChannel motor_channel) const noexcept;

    [[nodiscard]] Error set_forward(const MotorChannel motor_channel) const noexcept;
    [[nodiscard]] Error set_backward(const MotorChannel motor_channel) const noexcept;
    [[nodiscard]] Error set_soft_stop(const MotorChannel motor_channel) const noexcept;
    [[nodiscard]] Error set_fast_stop(const MotorChannel motor_channel) const noexcept;

private:
    static Error motor_channel_to_error(const MotorChannel motor_channel) noexcept;

    static Speed raw_to_speed(const Raw raw) noexcept;
    static Raw speed_to_raw(const Speed speed) noexcept;

    static Voltage raw_to_voltage(const Raw raw) noexcept;
    static Raw voltage_to_raw(const Voltage voltage) noexcept;

    static Torque raw_to_torque(const Raw raw) noexcept;
    static Raw torque_to_raw(const Torque torque) noexcept;

    static constexpr auto BIT_RESOLUTION{16};
    static constexpr auto PRESCALER{0};
    static constexpr auto CLK_DIVISION{1};
    static constexpr auto CLK_FREQ_HZ{80000000 / CLK_DIVISION};
    static constexpr auto TICKS_PER_PERIOD{std::pow(2, BIT_RESOLUTION) - 1};
    static constexpr auto TICK_FREQ_HZ{CLK_FREQ_HZ / (PRESCALER + 1)};
    static constexpr auto TIMER_FREQ_HZ{TICK_FREQ_HZ / (TICKS_PER_PERIOD + 1)};

    static constexpr Voltage MAX_VOLTAGE_V{12};
    static constexpr Voltage MIN_VOLTAGE_V{0};

    static constexpr Raw MAX_RAW{static_cast<Raw>(std::pow(2, BIT_RESOLUTION) - 1)};
    static constexpr Raw MIN_RAW{0};

    static constexpr Speed MIN_SPEED_RPM{0};
    static constexpr Speed MAX_SPEED_RPM{1000};

    static constexpr Speed MIN_TORQUE_NM{0};
    static constexpr Speed MAX_TORQUE_NM{1000};

    Error initialize(Motor& motor) noexcept;
    Error deinitialize(Motor& motor) noexcept;

    const Motor& get_motor(const MotorChannel channel) const noexcept;
    Motor& get_motor(const MotorChannel motor_channel) noexcept;

    UartHandle uart_{nullptr};

    Motors motors_{};

    mutable char uart_buffer_[100];

    bool initialized_{false};
};

#endif // L298N_HPP
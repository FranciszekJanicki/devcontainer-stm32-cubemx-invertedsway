#ifndef L298N_HPP
#define L298N_HPP

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_gpio.h"
#include "stm32l4xx_hal_tim.h"
#include <cmath>
#include <cstdint>
#include <expected>
#include <utility>

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim)
{
}

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

    enum struct ChannelNum {
        CHANNEL1,
        CHANNEL2,
    };

    using Raw = std::uint16_t;
    using Speed = std::double_t;
    using Voltage = std::double_t;

    using ExpectedRaw = std::expected<Raw, Error>;
    using ExpectedVoltage = std::expected<Voltage, Error>;
    using ExpectedSpeed = std::expected<Speed, Error>;

    using Unexpected = std::unexpected<Error>;

    using TIM_Handle = TIM_HandleTypeDef*;
    using GPIO_Handle = GPIO_TypeDef*;

    struct Channel {
        ChannelNum channel_num{};

        std::uint32_t timer_channel{};
        std::uint32_t timer_it_options[5]{};

        std::uint16_t pin_IN1{};
        std::uint16_t pin_IN2{};
        std::uint16_t pin_OUT1{};
        std::uint16_t pin_OUT2{};
        std::uint16_t pin_EN{};

        TIM_Handle timer{nullptr};
        GPIO_Handle gpio{nullptr};
        bool initialized{false};
    };

    static const char* error_to_string(const Error error) noexcept;

    L298N(const Channel& channel1, const Channel& channel2) noexcept;
    L298N(Channel&& channel1, Channel&& channel2) noexcept;

    L298N(const L298N& other) noexcept = default;
    L298N(L298N&& other) noexcept = default;

    L298N& operator=(const L298N& other) noexcept = default;
    L298N& operator=(L298N&& other) noexcept = default;

    ~L298N() noexcept;

    const Channel& channel1() const& noexcept;
    Channel&& channel1() && noexcept;

    const Channel& channel2() const& noexcept;
    Channel&& channel2() && noexcept;

    void channel1(const Channel& channel1) noexcept;
    void channel1(Channel&& channel1) noexcept;

    void channel2(const Channel& channel2) noexcept;
    void channel2(Channel&& channel2) noexcept;

    ExpectedRaw get_compare_raw(const ChannelNum channel_num) const noexcept;
    Error set_compare_raw(const ChannelNum channel_num, const Raw raw) const noexcept;

    ExpectedVoltage get_compare_voltage(const ChannelNum channel_num) const noexcept;
    Error set_compare_voltage(const ChannelNum channel_num, const Voltage voltage) const noexcept;

    ExpectedSpeed get_compare_speed(const ChannelNum channel_num) const noexcept;
    Error set_compare_speed(const ChannelNum channel_num, const Speed speed) const noexcept;

    Error set_direction(const ChannelNum channel_num, const Direction direction) const noexcept;

    Error set_forward(const ChannelNum channel_num) const noexcept;
    Error set_backward(const ChannelNum channel_num) const noexcept;
    Error set_soft_stop(const ChannelNum channel_num) const noexcept;
    Error set_fast_stop(const ChannelNum channel_num) const noexcept;

    const Channel& get_channel(const ChannelNum channel) const noexcept;
    Channel& get_channel(const ChannelNum channel_num) noexcept;

private:
    static Error channel_num_to_error(const ChannelNum channel_num) noexcept;

    static Speed raw_to_speed(const Raw raw) noexcept;
    static Raw speed_to_raw(const Speed speed) noexcept;

    static Voltage raw_to_voltage(const Raw raw) noexcept;
    static Raw voltage_to_raw(const Voltage voltage) noexcept;

    static constexpr std::uint8_t PWM_RESOLUTION_BITS{16};
    static constexpr std::uint64_t CLOCK_RESOLUTION_HZ{16000000};
    static constexpr std::uint64_t TIMER_PRESCALER{40};
    static constexpr std::uint64_t TIMER_RESOLUTION_HZ{CLOCK_RESOLUTION_HZ / PRESCALER};

    static constexpr Voltage MAX_VOLTAGE_V{12};
    static constexpr Voltage MIN_VOLTAGE_V{0};

    static constexpr Raw MAX_RAW{std::pow(2, PWM_RESOLUTION_BITS) - 1};
    static constexpr Raw MIN_RAW{0};

    static constexpr Speed MIN_SPEED_RPM{0};
    static constexpr Speed MAX_SPEED_RPM{100000};

    Error initialize(const ChannelNum channel_num) noexcept;
    Error deinitialize(const ChannelNum channel_num) noexcept;

    Channel channel1_{};
    Channel channel2_{};

    bool initialized_{false};
};

#endif // L298N_HPP
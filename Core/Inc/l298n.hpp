#ifndef L298N_HPP
#define L298N_HPP

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_gpio.h"
#include "stm32l4xx_hal_tim.h"
#include <cassert>
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

struct L298N {
public:
    enum struct Error {
        OK,
        INIT,
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
    using UnexpectedRaw = std::unexpected<Error>;
    using UnexpectedVoltage = std::unexpected<Error>;

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

    L298N(const Channel& channel1, const Channel& channel2) noexcept : channel1_{channel1}, channel2_{channel2}
    {
        if (initialize(channel1_.channel_num) != Error::OK) {
        }
        if (initialize(channel2_.channel_num) != Error::OK) {
        }
    }

    L298N(Channel&& channel1, Channel&& channel2) noexcept :
        channel1_{std::forward<Channel>(channel1)}, channel2_{std::forward<Channel>(channel2)}
    {
        if (initialize(channel1_.channel_num) != Error::OK) {
        }
        if (initialize(channel2_.channel_num) != Error::OK) {
        }
    }

    L298N(const L298N& other) noexcept = default;
    L298N(L298N&& other) noexcept = default;

    L298N& operator=(const L298N& other) noexcept = default;
    L298N& operator=(L298N&& other) noexcept = default;

    ~L298N() noexcept
    {
        if (deinitialize(channel1_.channel_num) != Error::OK) {
        }
        if (deinitialize(channel2_.channel_num) != Error::OK) {
        }
    }

    Error initialize(const ChannelNum channel_num) noexcept
    {
        auto& channel{get_channel(channel_num)};
        if (channel.initialized) {
            return channel_num_to_error(channel_num);
        }
        for (const auto option : channel.timer_it_options) {
            __HAL_TIM_ENABLE_IT(channel.timer, option);
        }
        if (HAL_TIM_PWM_Start_IT(channel.timer, channel.timer_channel) != HAL_OK) {
            return channel_num_to_error(channel_num);
        }
        channel.initialized = true;
        return Error::OK;
    }

    Error deinitialize(const ChannelNum channel_num) noexcept
    {
        auto& channel{get_channel(channel)};
        if (!channel.initialized) {
            return channel_num_to_error(channel_num);
        }
        for (const auto option : channel.timer_it_options) {
            __HAL_TIM_DISABLE_IT(channel.timer, option);
        }
        if (HAL_TIM_PWM_Stop_IT(channel.timer, channel.timer_channel) != HAL_OK) {
            return channel_num_to_error(channel_num);
        }
        channel.initialized = false;
        return Error::OK;
    }

    ExpectedRaw get_compare_raw(const ChannelNum channel_num) const noexcept
    {
        const auto& channel{get_channel(channel)};
        if (!channel.initialized) {
            return UnexpectedRaw{channel_num_to_error(channel_num)};
        }
        return ExpectedRaw{__HAL_TIM_GetCompare(channel.timer, channel.timer_channel)};
    }

    Error set_compare_raw(const ChannelNum channel_num, const Raw raw) const noexcept
    {
        assert(raw <= MAX_RAW && raw >= MIN_RAW);
        const auto& channel{get_channel(channel_num)};
        if (!channel.initialized) {
            return channel_num_to_error(channel_num);
        }
        __HAL_TIM_SetCompare(channel.timer, channel.timer_channel, raw);
        return Error::OK;
    }

    ExpectedVoltage get_compare_voltage(const ChannelNum channel_num) const noexcept
    {
        const auto& channel{get_channel(channel_num)};
        if (!channel.initialized) {
            return UnexpectedRaw{channel_num_to_error(channel_num)};
        }
        return ExpectedRaw{raw_to_voltage(__HAL_TIM_GetCompare(channel.timer, channel.timer_channel))};
    }

    Error set_compare_voltage(const ChannelNum channel_num, const Voltage voltage) const noexcept
    {
        assert(voltage <= MAX_VOLTAGE_MV && voltage > MIN_VOLTAGE_MV);
        const auto& channel{get_channel(channel_num)};
        if (!channel.initialized) {
            return channel_num_to_error(channel_num);
        }
        __HAL_TIM_SetCompare(channel.timer, channel.timer_channel, voltage_to_raw(voltage));
        return Error::OK;
    }

    Error set_direction(const ChannelNum channel_num, const Direction direction) const noexcept
    {
        const auto& channel{get_channel(channel_num)};
        if (!channel.initialized) {
            return channel_num_to_error(channel_num);
        }
        switch (direction) {
            case Direction::SOFT_STOP:
                HAL_GPIO_WritePin(channel.gpio, channel.pin_IN1, GPIO_PinState::GPIO_PIN_RESET);
                HAL_GPIO_WritePin(channel.gpio, channel.pin_IN2, GPIO_PinState::GPIO_PIN_RESET);
                return set_compare_raw(channel_num, MAX_RAW);
            case Direction::FAST_STOP:
                HAL_GPIO_WritePin(channel.gpio, channel.pin_IN1, GPIO_PinState::GPIO_PIN_SET);
                HAL_GPIO_WritePin(channel.gpio, channel.pin_IN2, GPIO_PinState::GPIO_PIN_SET);
                return set_compare_raw(channel_num, MIN_RAW);
            case Direction::FORWARD:
                HAL_GPIO_WritePin(channel.gpio, channel.pin_IN1, GPIO_PinState::GPIO_PIN_SET);
                HAL_GPIO_WritePin(channel.gpio, channel.pin_IN2, GPIO_PinState::GPIO_PIN_RESET);
                return Error::OK;
            case Direction::BACKWARD:
                HAL_GPIO_WritePin(channel.gpio, channel.pin_IN1, GPIO_PinState::GPIO_PIN_RESET);
                HAL_GPIO_WritePin(channel.gpio, channel.pin_IN2, GPIO_PinState::GPIO_PIN_SET);
                return Error::OK;
            default:
                return channel_num_to_error(channel_num);
        }
        return Error::OK;
    }

    Error set_forward(const ChannelNum channel_num) const noexcept
    {
        return set_direction(channel_num, Direction::FORWARD);
    }

    Error set_backward(const ChannelNum channel_num) const noexcept
    {
        return set_direction(channel_num, Direction::BACKWARD);
    }

    Error set_soft_stop(const ChannelNum channel_num) const noexcept
    {
        return set_direction(channel_num, Direction::SOFT_STOP);
    }

    Error set_fast_stop(const ChannelNum channel_num) const noexcept
    {
        return set_direction(channel_num, Direction::FAST_STOP);
    }

    const Channel& get_channel(const ChannelNum channel) const noexcept
    {
        switch (channel) {
            case ChannelNum::CHANNEL1:
                return channel1_;
            case ChannelNum::CHANNEL2:
                return channel2_;
            default:
                std::unreachable();
        }
    }

    Channel& get_channel(const ChannelNum channel_num) noexcept
    {
        switch (channel_num) {
            case ChannelNum::CHANNEL1:
                return channel1_;
            case ChannelNum::CHANNEL2:
                return channel2_;
            default:
                std::unreachable();
        }
    }

    Channel channel1_{};
    Channel channel2_{};

    static constexpr const char* error_to_string(const Error error) noexcept
    {
        switch (error) {
            case Error::INIT:
                return "initialization error";
            case Error::CHANNEL1:
                return "channel one error";
            case Error::CHANNEL2:
                return "channel two error";
            case Error::OK:
                return "no error";
            default:
                return "None";
        }
    }

    static constexpr Error channel_num_to_error(const ChannelNum channel_num) noexcept
    {
        switch (channel_num) {
            case ChannelNum::CHANNEL1:
                return Error::CHANNEL1;
            case ChannelNum::CHANNEL2:
                return Error::CHANNEL2;
            default:
                std::unreachable();
        }
    }

    static constexpr Voltage raw_to_speed(const Raw raw) noexcept
    {
        return raw * (MAX_SPEED_RPM - MIN_SPEED_RPM) / (MAX_RAW - MIN_RAW);
    }
    static constexpr Voltage speed_to_raw(const Speed speed) noexcept
    {
        return speed * (MAX_RAW - MIN_RAW) / (MAX_SPEED_RPM - MIN_SPEED_RPM);
    }
    static constexpr Voltage raw_to_voltage(const Raw raw) noexcept
    {
        return raw * (MAX_VOLTAGE_MV - MIN_VOLTAGE_MV) / (MAX_RAW - MIN_RAW);
    }
    static constexpr Raw voltage_to_raw(const Voltage voltage) noexcept
    {
        return voltage * (MAX_RAW - MIN_RAW) / (MAX_VOLTAGE_MV - MIN_VOLTAGE_MV);
    }

    static constexpr std::uint8_t PWM_RESOLUTION_BITS{16};
    static constexpr std::uint64_t TIMER_RESOLUTION_HZ{};
    static constexpr Voltage MAX_VOLTAGE_MV{12};
    static constexpr Voltage MIN_VOLTAGE_MV{0};
    static constexpr Raw MAX_RAW{std::pow(2, PWM_RESOLUTION_BITS) - 1};
    static constexpr Raw MIN_RAW{0};
    static constexpr Speed MIN_SPEED_RPM{0};
    static constexpr Speed MAX_SPEED_RPM{0};
};

#endif // L298N_HPP
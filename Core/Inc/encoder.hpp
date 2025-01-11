#ifndef ENCODER_HPP
#define ENCODER_HPP

#include "common.hpp"
#include "stm32l4xx_hal_tim.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <utility>

namespace InvertedSway {

    struct Encoder {
    public:
        using Count = std::uint32_t;
        using Angle = std::float_t;
        using Speed = std::float_t;
        using OptionalAngle = std::optional<Angle>;
        using OptionalSpeed = std::optional<Speed>;

        Encoder() noexcept = default;
        Encoder(TimerHandle const timer,
                Count const pulses_per_360,
                Count const counts_per_pulse,
                Count const counter_period) noexcept;

        Encoder(Encoder const& other) noexcept = delete;
        Encoder(Encoder&& other) noexcept = default;

        Encoder& operator=(Encoder const& other) noexcept = delete;
        Encoder& operator=(Encoder&& other) noexcept = default;

        ~Encoder() noexcept;

        [[nodiscard]] OptionalAngle get_angle() noexcept;
        [[nodiscard]] OptionalSpeed get_angular_speed(float const dt) noexcept;

    private:
        Angle count_to_angle(Count const count) const noexcept;
        Angle count_to_angle_diff(Count const count_diff) const noexcept;

        void initialize() noexcept;
        void deinitialize() noexcept;

        bool initialized_{false};

        TimerHandle timer_{nullptr};

        Count last_count_{};
        Count count_{};

        Count counts_per_360_{};
        Count counter_period_{};
    };

}; // namespace InvertedSway

#endif // ENCODER_HPP
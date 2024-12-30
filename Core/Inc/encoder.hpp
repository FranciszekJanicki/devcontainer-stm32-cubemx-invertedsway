#ifndef ENCODER_HPP
#define ENCODER_HPP

#include "common.hpp"
#include "stm32l4xx_hal_tim.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <expected>
#include <utility>

namespace InvertedSway {

    struct Encoder {
    public:
        enum struct Error {
            OK,
            FAIL,
            INIT,
            DEINIT,
        };

        using Count = std::uint32_t;
        using Angle = std::float_t;
        using Speed = std::float_t;
        using ExpectedAngle = std::expected<Angle, Error>;
        using ExpectedSpeed = std::expected<Speed, Error>;
        using Unexpected = std::unexpected<Error>;

        Encoder() noexcept = default;

        Encoder(TimerHandle const timer) noexcept;

        Encoder(Encoder const& other) noexcept = delete;
        Encoder(Encoder&& other) noexcept = default;

        Encoder& operator=(Encoder const& other) noexcept = delete;
        Encoder& operator=(Encoder&& other) noexcept = default;

        ~Encoder() noexcept;

        [[nodiscard]] ExpectedAngle get_angle() noexcept;
        [[nodiscard]] ExpectedSpeed get_angular_speed(float const dt) noexcept;

    private:
        static Angle count_to_angle(Count const count) noexcept;
        static Angle get_angle_difference(Count const count, Count const last_count) noexcept;

        void initialize() noexcept;
        void deinitialize() noexcept;

        static constexpr Count COUNTS_PER_PULSE{1};
        static constexpr Count PULSES_PER_REVOLUTION{52};
        static constexpr Count COUNTS_PER_REVOLUTION{PULSES_PER_REVOLUTION * COUNTS_PER_PULSE};
        static constexpr Count COUNTER_PERIOD{65535};

        TimerHandle timer_{nullptr};

        Count last_count_{};
        Count count_{};

        bool initialized_{false};
    };

}; // namespace InvertedSway

#endif // ENCODER_HPP
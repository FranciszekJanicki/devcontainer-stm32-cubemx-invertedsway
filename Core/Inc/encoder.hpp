#ifndef ENCODER_HPP
#define ENCODER_HPP

#include "common.hpp"
#include "stm32l4xx_hal_tim.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <utility>

namespace InvertedSway {

    struct Encoder {
    public:
        using Count = std::uint16_t;
        using Angle = std::float_t;

        Encoder() noexcept = default;
        Encoder(TimerHandle timer) noexcept;

        Encoder(Encoder const& other) noexcept = delete;
        Encoder(Encoder&& other) noexcept = default;

        Encoder& operator=(Encoder const& other) noexcept = delete;
        Encoder& operator=(Encoder&& other) noexcept = default;

        ~Encoder() noexcept;

        [[nodiscard]] Angle get_angle() noexcept;

    private:
        static Angle count_to_angle(Count const count) noexcept;

        void initialize() noexcept;
        void deinitialize() noexcept;

        static constexpr Count COUNTS_PER_PULSE{1};
        static constexpr Count PULSES_PER_REVOLUTION{52};
        static constexpr Count COUNTS_PER_REVOLUTION{PULSES_PER_REVOLUTION * COUNTS_PER_PULSE};
        static constexpr Count COUNTER_PERIOD{65535};

        static constexpr Angle MIN_ANGLE_DEG{0};
        static constexpr Angle MAX_ANGLE_DEG{360};

        TimerHandle timer_{nullptr};

        Count last_count_{};
        Count count_{};

        bool initialized_{false};
    };

}; // namespace InvertedSway

#endif // ENCODER_HPP
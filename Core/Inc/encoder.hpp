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
        using uCount = std::uint16_t; // uCount and Count should be the same size
        using Count = std::int16_t;
        using Angle = double;

        Encoder() noexcept = default;
        Encoder(TimerHandle timer) noexcept;

        Encoder(const Encoder& other) noexcept = default;
        Encoder(Encoder&& other) noexcept = default;

        Encoder& operator=(const Encoder& other) noexcept = default;
        Encoder& operator=(Encoder&& other) noexcept = default;

        ~Encoder() noexcept;

        [[nodiscard]] Angle get_angle() noexcept;

    private:
        static Angle pulses_to_angle(const uCount pulse) noexcept;

        uCount get_previous_count() noexcept;

        void initialize() noexcept;
        void deinitialize() noexcept;

        static constexpr uCount COUNTS_PER_REVOLUTION{52};
        static constexpr uCount COUNTER_PERIOD{65535};

        static constexpr Angle MIN_ANGLE_DEG{0};
        static constexpr Angle MAX_ANGLE_DEG{360};

        TimerHandle timer_{nullptr};

        bool initialized_{false};
    };

}; // namespace InvertedSway

#endif // ENCODER_HPP
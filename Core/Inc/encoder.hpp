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
        using Angle = double;

        Encoder() noexcept = default;
        Encoder(TimerHandle timer, const Angle starting_angle) noexcept;

        Encoder(const Encoder& other) noexcept = default;
        Encoder(Encoder&& other) noexcept = default;

        Encoder& operator=(const Encoder& other) noexcept = default;
        Encoder& operator=(Encoder&& other) noexcept = default;

        ~Encoder() noexcept;

        [[nodiscard]] Angle get_angle() noexcept;

    private:
        static Angle encoder_count_to_angle(const Count encoder_count) noexcept;
        static Count count_to_encoder_count(const Count count) noexcept;

        Count get_previous_count() noexcept;

        void initialize() noexcept;
        void deinitialize() noexcept;

        static constexpr Count COUNT_PER_ENCODER_COUNT{4};
        static constexpr Count ENCODER_COUNT_PER_REVOLUTION{10};

        static constexpr Count MIN_COUNT{0};
        static constexpr Count MAX_COUNT{static_cast<Count>(std::pow(2, 16) - 1)};

        static constexpr Angle MIN_ANGLE_DEG{0};
        static constexpr Angle MAX_ANGLE_DEG{360};

        TimerHandle timer_{nullptr};

        Angle angle_{};
        std::uint64_t last_count_{};

        bool initialized_{false};
    };

}; // namespace InvertedSway

#endif // ENCODER_HPP
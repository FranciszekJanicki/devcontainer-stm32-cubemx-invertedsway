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
        enum struct Error {
            OK,
            FAIL,
            INIT,
            DEINIT,
        };

        using Count = std::uint16_t;
        using Angle = std::int16_t;

        static const char* error_to_string(const Error error) noexcept;

        Encoder() noexcept = default;
        Encoder(TimerHandle timer) noexcept;

        Encoder(const Encoder& other) noexcept = default;
        Encoder(Encoder&& other) noexcept = default;

        Encoder& operator=(const Encoder& other) noexcept = default;
        Encoder& operator=(Encoder&& other) noexcept = default;

        ~Encoder() noexcept;

        [[nodiscard]] Count get_count() noexcept;
        [[nodiscard]] Angle get_angle() noexcept;

    private:
        [[nodiscard]] Error initialize() noexcept;
        [[nodiscard]] Error deinitialize() noexcept;

        static constexpr Count COUNT_PER_ENCODER_COUNT{4};

        static constexpr Count MIN_ENCODER_COUNT{0};
        static constexpr Count MAX_ENCODER_COUNT{10};

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
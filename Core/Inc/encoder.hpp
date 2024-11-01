#ifndef HAL_SENSOR_HPP
#define HAL_SENSOR_HPP

#include "common.hpp"
#include "stm32l4xx_hal_tim.h"
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <expected>
#include <utility>

namespace InvertedSway {

    struct Encoder {
        enum struct Direction {
            CLOCKWISE,
            COUNTER_CLOCKWISE,
        };

        enum struct Error {
            OK,
            FAIL,
            INIT,
            DEINIT,
        };

        using Count = std::uint8_t;
        using Angle = std::int16_t;
        using ExpectedCount = std::expected<Count, Error>;
        using ExpectedAngle = std::expected<Angle, Error>;
        using Unexpected = std::unexpected<Error>;

        static const char* error_to_string(const Error error) noexcept
        {
            switch (error) {
                case Error::OK:
                    return "OK";
                case Error::FAIL:
                    return "FAIL";
                case Error::INIT:
                    return "INIT";
                case Error::DEINIT:
                    return "DEINIT";
                default:
                    return "NONE";
            }
        }

        [[nodiscard]] Error initialize() noexcept
        {
            if (this->initialized) {
                return Error::INIT;
            }
            if (HAL_TIM_Encoder_Start(timer, TIM_CHANNEL_ALL) != HAL_OK) {
                return Error::INIT;
            }
            return Error::OK;
        }

        [[nodiscard]] Error deinitialize() noexcept
        {
            if (!this->initialized) {
                return Error::DEINIT;
            }
            if (HAL_TIM_Encoder_Stop(timer, TIM_CHANNEL_ALL) != HAL_OK) {
                return Error::DEINIT;
            }
            return Error::OK;
        }

        [[nodiscard]] ExpectedCount get_count() noexcept
        {
            if (!this->initialized) {
                return Unexpected{Error::FAIL};
            }
            return ExpectedCount{this->last_count = static_cast<Count>(__HAL_TIM_GetCounter(this->timer))};
        }

        [[nodiscard]] ExpectedAngle get_angle() noexcept
        {
            if (!this->initialized) {
                return Unexpected{Error::FAIL};
            }
            const auto count_difference{
                static_cast<Count>(__HAL_TIM_GetCounter(this->timer)) -
                std::exchange(this->last_count, static_cast<Count>(__HAL_TIM_GetCounter(this->timer)))};
            if (count_difference >= COUNT_PER_ENCODER_COUNT || count_difference <= -COUNT_PER_ENCODER_COUNT) {
                this->angle = std::clamp(
                    this->angle + static_cast<Angle>((count_difference / COUNT_PER_ENCODER_COUNT) % MAX_COUNT),
                    MIN_ANGLE_DEG,
                    MAX_ANGLE_DEG);
            }
            return ExpectedAngle{this->angle};
        }

        static constexpr Count COUNT_PER_ENCODER_COUNT{4};

        static constexpr Count MIN_ENCODER_COUNT{0};
        static constexpr Count MAX_ENCODER_COUNT{10};

        static constexpr Count MIN_COUNT{0};
        static constexpr Count MAX_COUNT{static_cast<Count>(std::pow(2, 16) - 1)};

        static constexpr Angle MIN_ANGLE_DEG{0};
        static constexpr Angle MAX_ANGLE_DEG{360};

        TimerHandle timer{nullptr};

        Angle angle{};
        std::uint64_t last_count{};

        bool initialized{false};
    };

}; // namespace InvertedSway

#endif // HAL_SENSOR_HPP
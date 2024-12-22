#include "encoder.hpp"

using namespace InvertedSway;
using Count = Encoder::Count;
using Error = Encoder::Error;
using Angle = Encoder::Angle;
using Speed = Encoder::Speed;
using ExpectedAngle = Encoder::ExpectedAngle;
using ExpectedSpeed = Encoder::ExpectedSpeed;
using Unexpected = Encoder::Unexpected;

namespace InvertedSway {

    Angle Encoder::count_to_angle(Count const count) noexcept
    {
        return static_cast<Angle>(std::clamp(count, Count{0}, COUNTS_PER_REVOLUTION)) * 360.0f /
               static_cast<Angle>(COUNTS_PER_REVOLUTION);
    }

    Encoder::Encoder(TimerHandle const timer) noexcept : timer_{timer}
    {
        this->initialize();
    }

    Encoder::~Encoder() noexcept
    {
        this->deinitialize();
    }

    void Encoder::initialize() noexcept
    {
        if (this->initialized_) {
            return;
        }
        if (HAL_TIM_Encoder_Start(this->timer_, TIM_CHANNEL_ALL) == HAL_OK) {
            this->initialized_ = true;
        }
    }

    void Encoder::deinitialize() noexcept
    {
        if (!this->initialized_) {
            return;
        }
        if (HAL_TIM_Encoder_Stop(this->timer_, TIM_CHANNEL_ALL) == HAL_OK) {
            this->initialized_ = false;
        }
    }

    ExpectedAngle Encoder::get_angle() noexcept
    {
        if (!this->initialized_) {
            return Unexpected{Error::INIT};
        }

        // this->count_ = (this->count_ - this->last_count_ + static_cast<Count>(__HAL_TIM_GetCounter(this->timer_))) %
        //                COUNTER_PERIOD;
        // this->last_count_ = this->count_;

        // return ExpectedAngle{count_to_angle(this->count_)};
        return ExpectedAngle{count_to_angle(static_cast<Count>(__HAL_TIM_GetCounter(this->timer_)))};
    }

    ExpectedSpeed Encoder::get_angular_speed(float const dt) noexcept
    {
        if (!this->initialized_) {
            return Unexpected{Error::INIT};
        }

        auto const get_angle_difference{[](Count const count, Count const last_count) {
            auto const difference{count_to_angle(count - last_count)};
            if (difference > 0.0f) {
                return difference - 360.0f;
            } else if (difference < 0.0f) {
                return difference + 360.0f;
            }
            return difference;
        }};

        this->count_ = (this->count_ - this->last_count_ + static_cast<Count>(__HAL_TIM_GetCounter(this->timer_))) %
                       COUNTER_PERIOD;
        auto const angle_difference{get_angle_difference(this->count_, this->last_count_)};
        this->last_count_ = this->count_;

        return ExpectedSpeed{angle_difference / static_cast<Angle>(dt)};
    }

}; // namespace InvertedSway
#include "encoder.hpp"
#include "utility.hpp"

using namespace InvertedSway;
using namespace Utility;
using Count = Encoder::Count;
using Angle = Encoder::Angle;
using Speed = Encoder::Speed;
using OptionalAngle = Encoder::OptionalAngle;
using OptionalSpeed = Encoder::OptionalSpeed;

namespace InvertedSway {

    Encoder::Encoder(TimerHandle const timer,
                     Count const pulses_per_360,
                     Count const counts_per_pulse,
                     Count const counter_period) noexcept :
        timer_{timer}, counts_per_360_{pulses_per_360 * counts_per_pulse}, counter_period_{counter_period}
    {
        this->initialize();
    }

    Encoder::~Encoder() noexcept
    {
        this->deinitialize();
    }

    void Encoder::initialize() noexcept
    {
        if (this->timer_ != nullptr) {
            if (HAL_TIM_Encoder_Start(this->timer_, TIM_CHANNEL_ALL) == HAL_OK) {
                this->initialized_ = true;
            }
        }
    }

    void Encoder::deinitialize() noexcept
    {
        if (this->timer_ != nullptr) {
            if (HAL_TIM_Encoder_Stop(this->timer_, TIM_CHANNEL_ALL) == HAL_OK) {
                this->initialized_ = false;
            }
        }
    }

    OptionalAngle Encoder::get_angle() noexcept
    {
        if (!this->initialized_) {
            return OptionalAngle{std::nullopt};
        }

        // this->count_ = (this->count_ - std::exchange(this->last_count_, this->count_) +
        //                static_cast<Count>(__HAL_TIM_GetCounter(this->timer_))) %
        //               this->counter_period_;
        // // return OptionalAngle{count_to_angle(this->count_)};
        return OptionalAngle{degrees_to_radians(static_cast<Angle>(__HAL_TIM_GetCounter(this->timer_)))};
    }

    OptionalSpeed Encoder::get_angular_speed(float const dt) noexcept
    {
        if (!this->initialized_) {
            return OptionalAngle{std::nullopt};
        }

        this->count_ = (this->count_ - this->last_count_ + static_cast<Count>(__HAL_TIM_GetCounter(this->timer_))) %
                       this->counter_period_;
        return OptionalSpeed{count_to_angle_diff(this->count_ - std::exchange(this->last_count_, this->count_)) /
                             static_cast<Angle>(dt)};
    }

    Angle Encoder::count_to_angle(Count const count) const noexcept
    {
        return std::clamp(count, 0UL, this->counter_period_) * 360.0F / this->counts_per_360_;
    }

    Angle Encoder::count_to_angle_diff(Count const count_diff) const noexcept
    {
        auto const angle_diff{this->count_to_angle(count_diff)};
        if (angle_diff > 0.0F) {
            return angle_diff - 360.0F;
        } else if (angle_diff < 0.0F) {
            return angle_diff + 360.0F;
        }
        return angle_diff;
    }

}; // namespace InvertedSway
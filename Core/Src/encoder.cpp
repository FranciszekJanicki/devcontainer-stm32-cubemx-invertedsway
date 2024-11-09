#include "encoder.hpp"

namespace InvertedSway {

    using Count = Encoder::Count;
    using Angle = Encoder::Angle;

    Angle Encoder::count_to_angle(const Count count) noexcept
    {
        return std::clamp(Angle{count * (MAX_ANGLE_DEG - MIN_ANGLE_DEG) / COUNTS_PER_REVOLUTION + MIN_ANGLE_DEG},
                          MIN_ANGLE_DEG,
                          MAX_ANGLE_DEG);
    }

    Encoder::Encoder(TimerHandle timer) noexcept : timer_{timer}
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

    Angle Encoder::get_angle() noexcept
    {
        if (!this->initialized_) {
            assert(true);
        }
        // this->count_ +=
        //     static_cast<Count>(__HAL_TIM_GetCounter(this->timer_) - std::exchange(this->last_count_, this->count_));
        // this->count_ %= COUNTER_PERIOD;
        // return count_to_angle(this->count_);
        return count_to_angle(static_cast<Count>(__HAL_TIM_GetCounter(this->timer_)));
    }

}; // namespace InvertedSway
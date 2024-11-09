#include "encoder.hpp"

namespace InvertedSway {

    using uCount = Encoder::uCount;
    using Angle = Encoder::Angle;

    Encoder::Encoder(TimerHandle timer) noexcept : timer_{timer}
    {
        this->initialize();
    }

    Encoder::~Encoder() noexcept
    {
        this->deinitialize();
    }

    Angle Encoder::pulses_to_angle(const uCount pulses) noexcept
    {
        return Angle{static_cast<Count>(pulses) * (MAX_ANGLE_DEG - MIN_ANGLE_DEG) / COUNTS_PER_REVOLUTION +
                     MIN_ANGLE_DEG};
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
        return pulses_to_angle(static_cast<uCount>(__HAL_TIM_GetCounter(this->timer_)));
    }

}; // namespace InvertedSway
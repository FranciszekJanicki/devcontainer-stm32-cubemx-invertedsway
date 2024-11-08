#include "encoder.hpp"

namespace InvertedSway {

    using Count = Encoder::Count;
    using Angle = Encoder::Angle;

    Encoder::Encoder(TimerHandle timer, const Angle starting_angle) noexcept : timer_{timer}, angle_{starting_angle}
    {
        this->initialize();
    }

    Encoder::~Encoder() noexcept
    {
        this->deinitialize();
    }

    Angle Encoder::pulses_to_angle(const Count pulses) noexcept
    {
        assert(pulses <= PULSES_PER_REVOLUTION && pulses >= 0);
        return std::clamp(Angle{pulses * (MAX_ANGLE_DEG - MIN_ANGLE_DEG) / PULSES_PER_REVOLUTION + MIN_ANGLE_DEG},
                          MIN_ANGLE_DEG,
                          MAX_ANGLE_DEG);
    }

    Count Encoder::count_to_pulses(const Count count) noexcept
    {
        return (count % COUNTER_PERIOD) / COUNTS_PER_PULSE;
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

    Count Encoder::get_previous_count() noexcept
    {
        if (!this->initialized_) {
            assert(true);
        }
        return std::exchange(this->last_count_, static_cast<Count>(__HAL_TIM_GetCounter(this->timer_)));
    }

    Angle Encoder::get_angle() noexcept
    {
        if (!this->initialized_) {
            assert(true);
        }
        return pulses_to_angle(
            count_to_pulses(static_cast<Count>(__HAL_TIM_GetCounter(this->timer_)) - this->get_previous_count()));
    }

}; // namespace InvertedSway
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

    Angle Encoder::encoder_count_to_angle(const Count encoder_count) noexcept
    {
        assert(encoder_count <= ENCODER_COUNT_PER_REVOLUTION && encoder_count >= 0);
        return std::clamp(Angle{(encoder_count - 0) * (MAX_ANGLE_DEG - MIN_ANGLE_DEG) / (ENCODER_COUNT_PER_REVOLUTION) +
                                MIN_ANGLE_DEG},
                          MIN_ANGLE_DEG,
                          MAX_ANGLE_DEG);
    }

    Count Encoder::count_to_encoder_count(const Count count) noexcept
    {
        assert(count <= MAX_COUNT && count >= MIN_COUNT);
        return (count / COUNT_PER_ENCODER_COUNT) % ENCODER_COUNT_PER_REVOLUTION;
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
        const auto count_difference{static_cast<Count>(__HAL_TIM_GetCounter(this->timer_)) -
                                    this->get_previous_count()};
        if (count_difference >= COUNT_PER_ENCODER_COUNT || count_difference <= -COUNT_PER_ENCODER_COUNT) {
            this->angle_ += encoder_count_to_angle(count_to_encoder_count(count_difference));
        }
        return this->angle_;
    }

}; // namespace InvertedSway
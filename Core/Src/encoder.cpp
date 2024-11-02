#include "encoder.hpp"

namespace InvertedSway {

    using Count = Encoder::Count;
    using Angle = Encoder::Angle;
    using Error = Encoder::Error;

    const char* Encoder::error_to_string(const Error error) noexcept
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

    Encoder::Encoder(TimerHandle timer) noexcept : timer_{timer}
    {
        this->initialize();
    }

    Encoder::~Encoder() noexcept
    {
        this->deinitialize();
    }

    [[nodiscard]] Error Encoder::initialize() noexcept
    {
        if (this->initialized_) {
            return Error::INIT;
        }
        if (HAL_TIM_Encoder_Start(this->timer_, TIM_CHANNEL_ALL) != HAL_OK) {
            return Error::INIT;
        }
        return Error::OK;
    }

    [[nodiscard]] Error Encoder::deinitialize() noexcept
    {
        if (!this->initialized_) {
            return Error::DEINIT;
        }
        if (HAL_TIM_Encoder_Stop(this->timer_, TIM_CHANNEL_ALL) != HAL_OK) {
            return Error::DEINIT;
        }
        return Error::OK;
    }

    [[nodiscard]] Count Encoder::get_count() noexcept
    {
        return this->last_count_ = static_cast<Count>(__HAL_TIM_GetCounter(this->timer_));
    }

    [[nodiscard]] Angle Encoder::get_angle() noexcept
    {
        const auto count_difference{
            static_cast<Count>(__HAL_TIM_GetCounter(this->timer_)) -
            std::exchange(this->last_count_, static_cast<Count>(__HAL_TIM_GetCounter(this->timer_)))};
        if (count_difference >= COUNT_PER_ENCODER_COUNT || count_difference <= -COUNT_PER_ENCODER_COUNT) {
            this->angle_ = std::clamp(
                Angle{this->angle_ + static_cast<Angle>((count_difference / COUNT_PER_ENCODER_COUNT) % MAX_COUNT)},
                MIN_ANGLE_DEG,
                MAX_ANGLE_DEG);
        }
        return this->angle_;
    }

}; // namespace InvertedSway
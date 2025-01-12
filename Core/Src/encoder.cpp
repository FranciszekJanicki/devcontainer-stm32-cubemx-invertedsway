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

    OptionalAngle Encoder::get_angle() noexcept
    {
        return this->cnt_device.get_count().transform(
            [this](Count const count) { return degrees_to_radians(this->count_to_angle(count)); });
        // return OptionalAngle{degrees_to_radians(static_cast<Angle>(__HAL_TIM_GetCounter(this->timer_)))};
    }

    OptionalSpeed Encoder::get_angular_speed(float const dt) noexcept
    {
        return this->cnt_device.get_count_difference().transform([this, dt](Count const count_diff) {
            return degrees_to_radians(this->count_to_angle_diff(count_diff)) / dt;
        });
    }

    Angle Encoder::count_to_angle(Count const count) const noexcept
    {
        return count * 360.0F / (this->pulses_per_360 * this->counts_per_pulse);
    }

    Angle Encoder::count_to_angle_diff(Count const count_diff) const noexcept
    {
        return std::fmod(this->count_to_angle(count_diff) + 360.0F, 360.0F);
    }

}; // namespace InvertedSway
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

    OptionalAngle Encoder::get_angle_degrees() const noexcept
    {
        return this->cnt_device.get_count().transform(
            [this](Count const count) { return this->count_to_degrees(count); });
    }

    OptionalAngle Encoder::get_angle_radians() const noexcept
    {
        return this->get_angle_degrees().transform(&degrees_to_radians<Angle>);
    }

    OptionalSpeed Encoder::get_speed_degrees(float const dt) const noexcept
    {
        return this->cnt_device.get_count_difference().transform(
            [this, dt](Count const count_diff) { return this->count_to_degree_diff(count_diff) / dt; });
    }

    OptionalSpeed Encoder::get_speed_radians(float const dt) const noexcept
    {
        return this->get_speed_degrees(dt).transform(&degrees_to_radians<Speed>);
    }

    Angle Encoder::count_to_degrees(Count const count) const noexcept
    {
        return count * 360.0F / (this->pulses_per_360 * this->counts_per_pulse);
    }

    Angle Encoder::count_to_degree_diff(Count const count_diff) const noexcept
    {
        return std::fmod(this->count_to_degrees(count_diff) + 360.0F, 360.0F);
    }

}; // namespace InvertedSway
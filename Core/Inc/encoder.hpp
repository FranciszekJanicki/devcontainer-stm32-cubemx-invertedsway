#ifndef ENCODER_HPP
#define ENCODER_HPP

#include "cnt_device.hpp"
#include "common.hpp"
#include <cmath>
#include <cstdint>
#include <optional>
#include <utility>

namespace InvertedSway {

    struct Encoder {
    public:
        using Count = CNTDevice::Count;
        using Angle = std::float_t;
        using Speed = std::float_t;
        using OptionalAngle = std::optional<Angle>;
        using OptionalSpeed = std::optional<Speed>;

        [[nodiscard]] OptionalAngle get_angle() noexcept;
        [[nodiscard]] OptionalSpeed get_angular_speed(float const dt) noexcept;

        CNTDevice cnt_device{};

        Count counts_per_pulse{};
        Count pulses_per_360{};

    private:
        Angle count_to_angle(Count const count) const noexcept;
        Angle count_to_angle_diff(Count const count_diff) const noexcept;
    };

}; // namespace InvertedSway

#endif // ENCODER_HPP
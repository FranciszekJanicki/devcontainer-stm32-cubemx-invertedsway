#ifndef FILTERS_HPP
#define FILTERS_HPP

#include "arithmetic.hpp"
#include <queue>
#include <utility>

namespace InvertedSway {

    template <typename Filtered, typename Samples = Filtered>
    [[nodiscard]] constexpr auto make_recursive_average(const Filtered start_condition = {}) noexcept
    {
        return [estimate = Filtered{}, prev_estimate = start_condition, samples = Samples{1}](
                   const Filtered measurement) mutable {
            estimate = (std::exchange(prev_estimate, estimate) * Samples{samples - Samples{1}} / samples) +
                       (measurement / samples);
            samples += Samples{1};
            return estimate;
        };
    }

    template <typename Filtered, typename Samples = Filtered>
    [[nodiscard]] constexpr auto make_moving_average(const Filtered start_condition = {},
                                                     const Samples last_samples = 10)
    {
        assert(last_samples > 0);
        return [estimate = Filtered{},
                prev_estimate = start_condition,
                measurements = std::queue<Filtered>{start_condition},
                last_samples](const Filtered measurement) mutable {
            estimate = std::exchange(prev_estimate, estimate) + ((measurement - measurements.front()) / last_samples);
            measurements.pop();
            measurements.push(measurement);
            return estimate;
        };
    }

    template <typename Filtered, typename Samples = Filtered, typename Alpha = Filtered>
    [[nodiscard]] constexpr auto make_low_pass(const Filtered start_condition = {}, const Alpha alpha = 1) noexcept
    {
        assert(alpha >= 0 && alpha <= 1);
        return [estimate = Filtered{}, prev_estimate = start_condition, alpha](const Filtered measurement) {
            estimate = (std::exchange(prev_estimate, estimate) * alpha) + (measurement * (Alpha{1} - alpha));
            return estimate;
        };
    }

}; // namespace InvertedSway

#endif // FILTERS_HPP
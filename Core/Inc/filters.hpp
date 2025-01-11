#ifndef FILTERS_HPP
#define FILTERS_HPP

#include "arithmetic.hpp"
#include "kalman.hpp"
#include <functional>
#include <queue>
#include <utility>

namespace InvertedSway::Filters {

    template <Linalg::Arithmetic Value>
    using Filter = std::function<Value(Value)>;

    template <Linalg::Arithmetic Value>
    [[nodiscard]] constexpr auto make_recursive_average(Value const start_condition = 0) noexcept
    {
        return [estimate = start_condition, samples = Value{1}](Value const measurement) mutable {
            estimate = (estimate * (samples - 1) + measurement) / samples;
            samples += Value{1};
            return estimate;
        };
    }

    template <Linalg::Arithmetic Value>
    [[nodiscard]] constexpr auto make_moving_average(Value const start_condition = 0, Value const last_samples = 10)
    {
        assert(last_samples > 0);
        std::queue<Value> measurements{};
        for (Value i{}; i < last_samples; ++i) {
            measurements.push(start_condition);
        }
        return [estimate = start_condition, last_samples, measurements = std::move(measurements)](
                   Value const measurement) mutable {
            estimate = estimate + (measurement - measurements.front()) / last_samples;
            if (!measurements.empty()) {
                measurements.pop();
            }
            measurements.push(measurement);
            return estimate;
        };
    }

    template <Linalg::Arithmetic Value>
    [[nodiscard]] constexpr auto make_low_pass(Value const start_condition = 0, Value const alpha = 0.5) noexcept
    {
        assert(alpha >= 0 && alpha <= 1);
        return [estimate = start_condition, alpha](Value const measurement) mutable {
            estimate = (estimate * alpha) + (measurement * (Value{1} - alpha));
            return estimate;
        };
    }

}; // namespace InvertedSway::Filters

#endif // FILTERS_HPP
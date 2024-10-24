#ifndef REGULATOR_HPP
#define REGULATOR_HPP

#include <utility>
#include <variant>

enum struct RegulatorAlgorithm {
    PID,
    LQR,
};

template <typename Value>
struct RegulatorPID {
    Value operator()(const Value value) noexcept
    {
        return value;
    }
};

template <typename Value>
struct RegulatorLQR {
    Value operator()(const Value value) noexcept
    {
        return value;
    }
};

using RegulatorVariant = std::variant<RegulatorLQR<Value>, RegulatorPID<Value>>;

template <typename Value, typename... RegulatorArgs>
[[nodiscard]] RegulatorVariant make_regulator(const RegulatorAlgorithm algorithm, RegulatorArgs&&... args)
{
    switch (algorithm) {
        case RegulatorAlgorithm::PID:
            return RegulatorVariant{std::in_place_type<RegulatorPID>, std::forward<RegulatorArgs>(args)...};

        case RegulatorAlgorithm::LQR:
            return RegulatorVariant{std::in_place_type<RegulatorLQR>, std::forward<RegulatorArgs>(args)...};
        default:
            return RegulatorVariant{};
    }
}

#endif // REGULATOR_HPP
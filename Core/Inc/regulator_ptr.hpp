#ifndef REGULATOR_HPP
#define REGULATOR_HPP

#include <memory>
#include <utility>

template <typename Value>
struct Regulator {
    virtual ~Regulator() noexcept = 0;
    virtual Value operator()(const Value) noexcept = 0;
};

enum struct RegulatorAlgorithm {
    PID,
    LQR,
};

template <typename Value>
struct RegulatorPID : public Regulator {
    Value operator()(const Value value) noexcept override
    {
        return value;
    }
};

template <typename Value>
struct RegulatorLQR : public Regulator {
    Value operator()(const Value value) noexcept override
    {
        return value;
    }
};

using RegulatorPtr = std::unique_ptr<Regulator<Value>>;

template <typename Value, typename... RegulatorArgs>
[[nodiscard]] RegulatorPtr make_regulator(const RegulatorAlgorithm algorithm, RegulatorArgs&&... args)
{
    switch (algorithm) {
        case RegulatorAlgorithm::PID:
            return std::make_unique<RegulatorPID<Value>>(std::forward<RegulatorArgs>(args)...);
        case RegulatorAlgorithm::LQR:
            return std::make_unique<RegulatorLQR<Value>>(std::forward<RegulatorArgs>(args)...);
        default:
            return RegulatorPtr{nullptr};
    }
}

#endif // REGULATOR_HPP
#ifndef REGULATOR_HPP
#define REGULATOR_HPP

#include <memory>
#include <utility>
#include <variant>

template <typename Unit>
struct Regulator {
    virtual ~Regulator() noexcept = 0;
    virtual Unit operator()(const Unit) noexcept = 0;

    Unit sampling_time_{1};
    Unit previous_input_{};
};

enum struct RegulatorAlgo {
    PID,
    LQR,
    ADRC,
    BINARY,
};

template <typename Unit>
struct RegulatorPID : public Regulator<Unit> {
    Unit operator()(const Unit input) noexcept override
    {
        return P * input + (input - this->previous_input) / this->sampling_time * D +
               I * (integral += input * this->sampling_time);
    }

    Unit integral{};
    Unit P{};
    Unit I{};
    Unit D{};
};

template <typename Unit>
struct RegulatorLQR : public Regulator<Unit> {
    Unit operator()(const Unit input) noexcept override
    {
        // implement lqr algorithm here
        return input;
    }

    // cache results/ system matrixes as members
};

template <typename Unit>
struct RegulatorADRC : public Regulator<Unit> {
    Unit operator()(const Unit input) noexcept override
    {
        // implement adrc algorithm here
        return input;
    }

    // cache results/ system matrixes as members
};

template <typename Unit>
struct RegulatorBinary : public Regulator<Unit> {
    bool operator()(const Unit input) noexcept override
    {
        if (state) {
            if (input < hysteresis_down) {
                state = false;
            } else {
                state = true;
            }
        } else {
            if (input > hysteresis_up) {
                state = true;
            } else {
                state = false;
            }
        }
        return state;
    }

    Unit hysteresis_up{};
    Unit hysteresis_down{};
    bool state{false};
};

/* UNIQUE_PTR SOLUTION */
template <typename Unit>
using RegulatorPtr = std::unique_ptr<Regulator<Unit>>;

template <typename Unit, typename... RegulatorArgs>
[[nodiscard]] RegulatorPtr<Unit> make_regulator_ptr(const RegulatorAlgo regulator_algo,
                                                    RegulatorArgs&&... regulator_args)
{
    switch (regulator_algo) {
        case RegulatorAlgo::PID:
            return std::make_unique<RegulatorPID>(std::forward<RegulatorArgs>(regulator_args)...);
        case RegulatorAlgo::LQR:
            return std::make_unique<RegulatorLQR>(std::forward<RegulatorArgs>(regulator_args)...);
        case RegulatorAlgo::ADRC:
            return std::make_unique<RegulatorADRC>(std::forward<RegulatorArgs>(regulator_args)...);
        case RegulatorAlgo::BINARY:
            return std::make_unique<RegulatorBinary>(std::forward<RegulatorArgs>(regulator_args)...);
        default:
            return RegulatorPtr<Unit>{nullptr};
    }
}

/* VARIANT SOLUTION */
template <typename Unit>
using RegulatorVariant =
    std::variant<RegulatorLQR<Unit>, RegulatorPID<Unit>, RegulatorADRC<Unit>, RegulatorBinary<Unit>>;

template <typename Unit, typename... RegulatorArgs>
[[nodiscard]] RegulatorVariant<Unit> make_regulator_variant(const RegulatorAlgo regulator_algo,
                                                            RegulatorArgs&&... regulator_args) noexcept
{
    switch (regulator_algo) {
        case RegulatorAlgo::PID:
            return RegulatorVariant<Unit>{std::in_place_type<RegulatorPID<Unit>>,
                                          std::forward<RegulatorArgs>(regulator_args)...};
        case RegulatorAlgo::LQR:
            return RegulatorVariant<Unit>{std::in_place_type<RegulatorLQR<Unit>>,
                                          std::forward<RegulatorArgs>(regulator_args)...};
        case RegulatorAlgo::ADRC:
            return RegulatorVariant<Unit>{std::in_place_type<RegulatorADRC<Unit>>,
                                          std::forward<RegulatorArgs>(regulator_args)...};
        case RegulatorAlgo::BINARY:
            return RegulatorVariant<Unit>{std::in_place_type<RegulatorBinary<Unit>>,
                                          std::forward<RegulatorArgs>(regulator_args)...};
        default:
            return RegulatorVariant<Unit>{};
    }
}

/* LAMBDA SOLUTION */
template <typename Unit>
using RegulatorLambda = decltype([](Unit) { return Unit{}; });

template <typename Unit, typename... RegulatorArgs>
[[nodiscard]] RegulatorLambda<Unit> make_regulator_lambda(const RegulatorAlgo regulator_algo,
                                                          RegulatorArgs&&... regulator_args) noexcept
{
    switch (regulator_algo) {
        case RegulatorAlgo::PID:
            return [pid = RegulatorPID{std::forward<RegulatorArgs>(regulator_args)...}](const Unit input) mutable {
                return pid(input);
            };
        case RegulatorAlgo::LQR:
            return [lqr = RegulatorLQR{std::forward<RegulatorArgs>(regulator_args)...}](const Unit input) mutable {
                return lqr(input);
            };
        case RegulatorAlgo::ADRC:
            return [adrc = RegulatorADRC{std::forward<RegulatorArgs>(regulator_args)...}](const Unit input) mutable {
                return adrc(input);
            };
        case RegulatorAlgo::BINARY:
            return [binary = RegulatorLQR{std::forward<RegulatorArgs>(regulator_args)...}](const Unit input) mutable {
                return binary(input);
            };
        default:
            return [](const Unit) { return Unit{}; };
    }
}

#endif // REGULATOR_HPP
#ifndef REGULATOR_HPP
#define REGULATOR_HPP

#include <memory>
#include <utility>
#include <variant>

/* REGULATOR BASE */
template <typename Unit, typename Time = Unit>
struct Regulator {
    virtual ~Regulator() noexcept = 0;
    virtual Unit operator()(const Unit, const Time) noexcept = 0;
};

enum struct RegulatorAlgo {
    PID,
    LQR,
    ADRC,
    BINARY,
    TERNARY,
};

/* PID */
template <typename Unit, typename Time = Unit>
struct RegulatorPID : public Regulator<Unit, Time> {
    Unit operator()(const Unit input, const Time dt) noexcept override
    {
        return P * input + D * ((input - previous_input) / dt) + I * (integral += (input * dt));
    }

    Unit P{};
    Unit I{};
    Unit D{};
    Unit integral{};
    Unit previous_input{};
};

/* LQR */
template <typename Unit, typename Time = Unit>
struct RegulatorLQR : public Regulator<Unit, Time> {
    Unit operator()(const Unit input, const Time dt) noexcept override
    {
        // implement lqr algorithm here
        return input;
    }
};

template <typename Unit, typename Time = Unit>
struct RegulatorADRC : public Regulator<Unit, Time> {
    Unit operator()(const Unit input, const Time dt) noexcept override
    {
        // implement adrc algorithm here
        return input;
    }
};

/* BINARY */
template <typename Unit, typename Time = Unit>
struct RegulatorBinary : public Regulator<Unit, Time> {
    enum struct State {
        POSITIVE,
        ZERO,
    };

    State operator()(const Unit input, const Time) noexcept override
    {
        if (state == State::POSITIVE) {
            if (input < hysteresis_down) {
                state = State::ZERO;
            } else {
                state = State::POSITIVE;
            }
        } else {
            if (input > hysteresis_up) {
                state = State::POSITIVE;
            } else {
                state = State::ZERO;
            }
        }
        return state;
    }

    Unit hysteresis_up{};
    Unit hysteresis_down{};
    State state{State::ZERO};
};

/* TERNARY */
template <typename Unit, typename Time = Unit>
struct RegulatorTernary : public Regulator<Unit, Time> {
    enum struct State {
        POSITIVE,
        NEGATIVE,
        ZERO,
    };

    State operator()(const Unit input, const Time) noexcept override
    {
        switch (state) {
            case State::POSITIVE:
                if (input < hysteresis_down_right) {
                    state = State::ZERO;
                }
                break;
            case State::NEGATIVE:
                if (input > hysteresis_up_left) {
                    state = State::ZERO;
                }
                break;
            case State::ZERO:
                if (input > hysteresis_up_right) {
                    state = State::POSITIVE;
                } else if (input < hysteresis_down_left) {
                    state = State::NEGATIVE;
                }
                break;
        }
        return state;
    }

    Unit hysteresis_up_left{};
    Unit hysteresis_down_left{};
    Unit hysteresis_up_right{};
    Unit hysteresis_down_right{};
    State state{State::ZERO};
};

/* UNIQUE_PTR SOLUTION (RUNTIME POLYMORPHISM)*/
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
        case RegulatorAlgo::TERNARY:
            return std::make_unique<RegulatorTernary>(std::forward<RegulatorArgs>(regulator_args)...);
        default:
            return RegulatorPtr<Unit>{nullptr};
    }
}

/* VARIANT SOLUTION (COMPILE TIME POLYMORPHISM) */
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
        case RegulatorAlgo::TERNARY:
            return RegulatorVariant<Unit>{std::in_place_type < RegulatorTernary<Unit>,
                                          std::forward<RegulatorArgs>(regulator_args)...};
        default:
            return RegulatorVariant<Unit>{};
    }
}

/* LAMBDA SOLUTION (TYPE ERASURE, although you will need std::function to containerize state-full lambda) */
template <typename Unit>
using RegulatorLambda = decltype([](Unit) { return Unit{}; });

template <typename Unit, typename... RegulatorArgs>
[[nodiscard]] RegulatorLambda<Unit> make_regulator_lambda(const RegulatorAlgo regulator_algo,
                                                          RegulatorArgs&&... regulator_args) noexcept
{
    switch (regulator_algo) {
        case RegulatorAlgo::PID:
            return [pid = RegulatorPID{std::forward<RegulatorArgs>(regulator_args)...}](const Unit input,
                                                                                        const Time dt) mutable -> Unit {
                return pid(input, dt);
            };
        case RegulatorAlgo::LQR:
            return [lqr = RegulatorLQR{std::forward<RegulatorArgs>(regulator_args)...}](const Unit input,
                                                                                        const Time dt) mutable -> Unit {
                return lqr(input, dt);
            };
        case RegulatorAlgo::ADRC:
            return [adrc = RegulatorADRC{std::forward<RegulatorArgs>(regulator_args)...}](
                       const Unit input,
                       const Time dt) mutable -> Unit { return adrc(input, dt); };
        case RegulatorAlgo::BINARY:
            return [binary = RegulatorBinary{std::forward<RegulatorArgs>(regulator_args)...}](
                       const Unit input,
                       const Time dt) mutable -> RegulatorBinary<Unit>::State { return binary(input, dt); };
        case RegulatorAlgo::TERNARY:
            return [ternary = RegulatorTernary{std::forward<RegulatorArgs>(
                        regulator_args)...}](const Unit input, const Time dt) mutable -> RegulatorTernary<Unit>::State {
                return ternary(input, dt);
                default:
                    return [](const Unit) { return Unit{}; };
            }
    }

#endif // REGULATOR_HPP
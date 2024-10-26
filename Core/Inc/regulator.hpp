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
template <typename Unit>
struct RegulatorBinary : public Regulator<Unit> {
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
template <typename Unit>
struct RegulatorTernary : public Regulator<Unit> {
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
template <typename Unit, typename Time = Unit>
using RegulatorPtr = std::unique_ptr<Regulator<Unit, Time>>;

template <typename Unit, typename Time = Unit, typename... RegulatorArgs>
[[nodiscard]] RegulatorPtr<Unit, Time> make_regulator_ptr(const RegulatorAlgo regulator_algo,
                                                          RegulatorArgs&&... regulator_args)
{
    switch (regulator_algo) {
        case RegulatorAlgo::PID:
            return std::make_unique<RegulatorPID<Unit, Time>>(std::forward<RegulatorArgs>(regulator_args)...);
        case RegulatorAlgo::LQR:
            return std::make_unique<RegulatorLQR<Unit, Time>>(std::forward<RegulatorArgs>(regulator_args)...);
        case RegulatorAlgo::ADRC:
            return std::make_unique<RegulatorADRC<Unit, Time>>(std::forward<RegulatorArgs>(regulator_args)...);
        case RegulatorAlgo::BINARY:
            return std::make_unique<RegulatorBinary<Unit>>(std::forward<RegulatorArgs>(regulator_args)...);
        case RegulatorAlgo::TERNARY:
            return std::make_unique<RegulatorTernary<Unit>>(std::forward<RegulatorArgs>(regulator_args)...);
        default:
            return RegulatorPtr<Unit>{nullptr};
    }
}

/* VARIANT SOLUTION (COMPILE TIME POLYMORPHISM) */
template <typename Unit, typename Time = Unit>
using RegulatorVariant = std::variant<RegulatorLQR<Unit, Time>,
                                      RegulatorPID<Unit, Time>,
                                      RegulatorADRC<Unit, Time>,
                                      RegulatorBinary<Unit>,
                                      RegulatorTernary<Unit>>;

template <typename Unit, typename Time = Unit, typename... RegulatorArgs>
[[nodiscard]] RegulatorVariant<Unit, Time> make_regulator_variant(const RegulatorAlgo regulator_algo,
                                                                  RegulatorArgs&&... regulator_args) noexcept
{
    switch (regulator_algo) {
        case RegulatorAlgo::PID:
            return RegulatorVariant<Unit, Time>{std::in_place_type<RegulatorPID<Unit, Time>>,
                                                std::forward<RegulatorArgs>(regulator_args)...};
        case RegulatorAlgo::LQR:
            return RegulatorVariant<Unit, Time>{std::in_place_type<RegulatorLQR<Unit, Time>>,
                                                std::forward<RegulatorArgs>(regulator_args)...};
        case RegulatorAlgo::ADRC:
            return RegulatorVariant<Unit, Time>{std::in_place_type<RegulatorADRC<Unit, Time>>,
                                                std::forward<RegulatorArgs>(regulator_args)...};
        case RegulatorAlgo::BINARY:
            return RegulatorVariant<Unit>{std::in_place_type<RegulatorBinary<Unit, Time>>,
                                          std::forward<RegulatorArgs>(regulator_args)...};
        case RegulatorAlgo::TERNARY:
            return RegulatorVariant<Unit>{std::in_place_type < RegulatorTernary<Unit, Time>,
                                          std::forward<RegulatorArgs>(regulator_args)...};
        default:
            return RegulatorVariant<Unit>{};
    }
}

/* LAMBDA SOLUTION (TYPE ERASURE, although you will need std::function to containerize state-full lambda) */
template <typename Unit, typename Time = Unit>
using RegulatorLambda = decltype([](Unit, Time) { return Unit{}; });

template <typename Unit, typename Time = Unit, typename... RegulatorArgs>
[[nodiscard]] RegulatorLambda<Unit, Time> make_regulator_lambda(const RegulatorAlgo regulator_algo,
                                                                RegulatorArgs&&... regulator_args) noexcept
{
    switch (regulator_algo) {
        case RegulatorAlgo::PID:
            return [pid = RegulatorPID<Unit, Time>{std::forward<RegulatorArgs>(
                        regulator_args)...}](const Unit input, const Time dt) mutable { return pid(input, dt); };
        case RegulatorAlgo::LQR:
            return [lqr = RegulatorLQR<Unit, Time>{std::forward<RegulatorArgs>(
                        regulator_args)...}](const Unit input, const Time dt) mutable { return lqr(input, dt); };
        case RegulatorAlgo::ADRC:
            return [adrc = RegulatorADRC<Unit, Time>{std::forward<RegulatorArgs>(
                        regulator_args)...}](const Unit input, const Time dt) mutable { return adrc(input, dt); };
        case RegulatorAlgo::BINARY:
            return [binary = RegulatorBinary<Unit>{std::forward<RegulatorArgs>(
                        regulator_args)...}](const Unit input, const Time dt) mutable { return binary(input, dt); };
        case RegulatorAlgo::TERNARY:
            return [ternary = RegulatorTernary<Unit>{
                        std::forward<RegulatorArgs>(regulator_args)...}](const Unit input, const Time dt) mutable {
                return ternary(input, dt);
                default:
                    return [](const Unit, const Time) { return Unit{}; };
            }
    }

#endif // REGULATOR_HPP
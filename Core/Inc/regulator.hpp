#ifndef REGULATOR_HPP
#define REGULATOR_HPP

#include "arithmetic.hpp"
#include <algorithm>
#include <memory>
#include <utility>
#include <variant>

/* REGULATOR BASE */
template <Linalg::Arithmetic Value>
struct Regulator {
    virtual ~Regulator() noexcept = 0;
    virtual Value operator()(const Value, const Value) noexcept = 0;
};

enum struct RegulatorAlgo {
    PID,
    LQR,
    ADRC,
    BINARY,
    TERNARY,
};

/* PID */
template <Linalg::Arithmetic Value>
struct RegulatorPID : public Regulator<Value> {
    Value operator()(const Value error, const Value dt) noexcept override
    {
        sum += (error + previous_error) / 2 * dt;
        sum = std::clamp(sum, -windup, windup);
        return P * error + D * (error - std::exchange(previous_error, error)) / dt + I * sum;
    }

    Value P{};
    Value I{};
    Value D{};

    Value sum{};
    Value previous_error{};
    Value windup{};
};

/* LQR */
template <Linalg::Arithmetic Value>
struct RegulatorLQR : public Regulator<Value> {
    Value operator()(const Value error, const Value) noexcept override
    {
        // implement lqr algorithm here
        return error;
    }
};

template <Linalg::Arithmetic Value>
struct RegulatorADRC : public Regulator<Value> {
    Value operator()(const Value error, const Value) noexcept override
    {
        // implement adrc algorithm here
        return error;
    }
};

/* BINARY */
template <Linalg::Arithmetic Value>
struct RegulatorBinary : public Regulator<Value> {
    enum struct State {
        POSITIVE,
        ZERO,
    };

    State operator()(const Value error) noexcept override
    {
        if (state == State::POSITIVE) {
            if (error < hysteresis_down) {
                state = State::ZERO;
            } else {
                state = State::POSITIVE;
            }
        } else {
            if (error > hysteresis_up) {
                state = State::POSITIVE;
            } else {
                state = State::ZERO;
            }
        }
        return state;
    }

    Value hysteresis_up{};
    Value hysteresis_down{};
    State state{State::ZERO};
};

/* TERNARY */
template <Linalg::Arithmetic Value>
struct RegulatorTernary : public Regulator<Value> {
    enum struct State {
        POSITIVE,
        NEGATIVE,
        ZERO,
    };

    State operator()(const Value error) noexcept override
    {
        switch (state) {
            case State::POSITIVE:
                if (error < hysteresis_down_right) {
                    state = State::ZERO;
                }
                break;
            case State::NEGATIVE:
                if (error > hysteresis_up_left) {
                    state = State::ZERO;
                }
                break;
            case State::ZERO:
                if (error > hysteresis_up_right) {
                    state = State::POSITIVE;
                } else if (error < hysteresis_down_left) {
                    state = State::NEGATIVE;
                }
                break;
        }
        return state;
    }

    Value hysteresis_up_left{};
    Value hysteresis_down_left{};
    Value hysteresis_up_right{};
    Value hysteresis_down_right{};
    State state{State::ZERO};
};

/* UNIQUE_PTR SOLUTION (RUNTIME POLYMORPHISM)*/
template <Linalg::Arithmetic Value>
using RegulatorPtr = std::unique_ptr<Regulator<Value>>;

template <Linalg::Arithmetic Value, typename... RegulatorArgs>
[[nodiscard]] RegulatorPtr<Value> make_regulator_ptr(const RegulatorAlgo regulator_algo,
                                                     RegulatorArgs&&... regulator_args)
{
    switch (regulator_algo) {
        case RegulatorAlgo::PID:
            return std::make_unique<RegulatorPID<Value>>(std::forward<RegulatorArgs>(regulator_args)...);
        case RegulatorAlgo::LQR:
            return std::make_unique<RegulatorLQR<Value>>(std::forward<RegulatorArgs>(regulator_args)...);
        case RegulatorAlgo::ADRC:
            return std::make_unique<RegulatorADRC<Value>>(std::forward<RegulatorArgs>(regulator_args)...);
        case RegulatorAlgo::BINARY:
            return std::make_unique<RegulatorBinary<Value>>(std::forward<RegulatorArgs>(regulator_args)...);
        case RegulatorAlgo::TERNARY:
            return std::make_unique<RegulatorTernary<Value>>(std::forward<RegulatorArgs>(regulator_args)...);
        default:
            return RegulatorPtr<Value>{nullptr};
    }
}

/* VARIANT SOLUTION (COMPILE TIME POLYMORPHISM) */
template <Linalg::Arithmetic Value>
using RegulatorVariant = std::variant<RegulatorLQR<Value>,
                                      RegulatorPID<Value>,
                                      RegulatorADRC<Value>,
                                      RegulatorBinary<Value>,
                                      RegulatorTernary<Value>>;

template <Linalg::Arithmetic Value, typename... RegulatorArgs>
[[nodiscard]] RegulatorVariant<Value> make_regulator_variant(const RegulatorAlgo regulator_algo,
                                                             RegulatorArgs&&... regulator_args) noexcept
{
    switch (regulator_algo) {
        case RegulatorAlgo::PID:
            return RegulatorVariant<Value>{std::in_place_type<RegulatorPID<Value>>,
                                           std::forward<RegulatorArgs>(regulator_args)...};
        case RegulatorAlgo::LQR:
            return RegulatorVariant<Value>{std::in_place_type<RegulatorLQR<Value>>,
                                           std::forward<RegulatorArgs>(regulator_args)...};
        case RegulatorAlgo::ADRC:
            return RegulatorVariant<Value>{std::in_place_type<RegulatorADRC<Value>>,
                                           std::forward<RegulatorArgs>(regulator_args)...};
        case RegulatorAlgo::BINARY:
            return RegulatorVariant<Value>{std::in_place_type<RegulatorBinary<Value>>,
                                           std::forward<RegulatorArgs>(regulator_args)...};
        case RegulatorAlgo::TERNARY:
            return RegulatorVariant<Value>{std::in_place_type<RegulatorTernary<Value>>,
                                           std::forward<RegulatorArgs>(regulator_args)...};
        default:
            return RegulatorVariant<Value>{};
    }
}

/* LAMBDA SOLUTION (TYPE ERASURE, although you will need std::function to containerize state-full lambda) */
template <Linalg::Arithmetic Value>
using RegulatorFunction = std::function<Value(Value, Value)>;

template <Linalg::Arithmetic Value, typename... RegulatorArgs>
[[nodiscard]] auto make_regulator_lambda(const RegulatorAlgo regulator_algo, RegulatorArgs&&... regulator_args) noexcept
{
    switch (regulator_algo) {
        case RegulatorAlgo::PID:
            return [pid = RegulatorPID<Value>{std::forward<RegulatorArgs>(regulator_args)...}](const Value error,
                                                                                               const Value dt) mutable {
                return pid(error, dt);
            };
        case RegulatorAlgo::LQR:
            return [lqr = RegulatorLQR<Value>{std::forward<RegulatorArgs>(regulator_args)...}](const Value error,
                                                                                               const Value dt) mutable {
                return lqr(error, dt);
            };
        case RegulatorAlgo::ADRC:
            return [adrc = RegulatorADRC<Value>{std::forward<RegulatorArgs>(
                        regulator_args)...}](const Value error, const Value dt) mutable { return adrc(error, dt); };
        case RegulatorAlgo::BINARY:
            return [binary = RegulatorBinary<Value>{std::forward<RegulatorArgs>(regulator_args)...}](
                       const Value error) mutable { return binary(error); };
        case RegulatorAlgo::TERNARY:
            return [ternary = RegulatorTernary<Value>{std::forward<RegulatorArgs>(regulator_args)...}](
                       const Value error) mutable { return ternary(error); };
        default:
            return [](const Value error) mutable { return error; };
    }
}

#endif // REGULATOR_HPP
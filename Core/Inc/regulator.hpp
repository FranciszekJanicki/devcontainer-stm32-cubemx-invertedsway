#ifndef REGULATOR_HPP
#define REGULATOR_HPP

#include "arithmetic.hpp"
#include <algorithm>
#include <concepts>
#include <functional>
#include <memory>
#include <utility>
#include <variant>

// i have chosen std::variant for polymorphic regulator, as its sigma container
#define REGULATOR_VARIANT

namespace InvertedSway {

    namespace Regulator {

        enum struct Algorithm {
            PID,
            LQR,
            ADRC,
            BINARY,
            TERNARY,
        };

        template <typename First, typename... Rest>
        struct FirstType {
            using Type = First;
        };

        template <Linalg::Arithmetic Value>
        struct PID
#ifdef REGULATOR_PTR
            : public Base<Value>
#endif
        {
            Value operator()(const Value error, const Value dt) noexcept
            {
                this->sum += (error + this->previous_error) / 2 * dt;
                this->sum = std::clamp(this->sum, -this->windup, this->windup);
                return this->kp * error + this->kd * (error - std::exchange(this->previous_error, error)) / dt +
                       this->ki * this->sum;
            }

            Value kp{};
            Value ki{};
            Value kd{};
            Value windup{};

            Value sum{0};
            Value previous_error{0};
        };

        template <Linalg::Arithmetic Value>
        struct LQR
#ifdef REGULATOR_PTR
            : public Base<Value>
#endif
        {
            Value operator()(const Value error, const Value) noexcept
            {
                // implement lqr algorithm here
                return error;
            }
        };

        template <Linalg::Arithmetic Value>
        struct ADRC
#ifdef REGULATOR_PTR
            : public Base<Value>
#endif
        {
            Value operator()(const Value error, const Value) noexcept
            {
                // implement adrc algorithm here
                return error;
            }
        };

        template <Linalg::Arithmetic Value>
        struct Binary
#ifdef REGULATOR_PTR
            : public Base<Value>
#endif
        {
            enum struct State {
                POSITIVE,
                ZERO,
            };

            Value operator()(const Value error, const Value dt) noexcept
            {
                return error;
            }

            State operator()(const Value error) noexcept
            {
                switch (this->state) {
                    case State::POSITIVE:
                        if (error < this->hysteresis_down) {
                            this->state = State::ZERO;
                        } else {
                            this->state = State::POSITIVE;
                        }
                        break;
                    case State::ZERO:
                        if (error > this->hysteresis_up) {
                            this->state = State::POSITIVE;
                        } else {
                            this->state = State::ZERO;
                        }
                        break;
                    default:
                        break;
                }
                return this->state;
            }

            Value hysteresis_up{};
            Value hysteresis_down{};

            State state{State::ZERO};
        };

        template <Linalg::Arithmetic Value>
        struct Ternary
#ifdef REGULATOR_PTR
            : public Base<Value>
#endif
        {
            enum struct State {
                POSITIVE,
                NEGATIVE,
                ZERO,
            };

            Value operator()(const Value error, const Value dt) noexcept
            {
                return error;
            }

            State operator()(const Value error) noexcept
            {
                switch (this->state) {
                    case State::POSITIVE:
                        if (error < this->hysteresis_down) {
                            this->state = State::ZERO;
                        }
                        break;
                    case State::NEGATIVE:
                        if (error > this->hysteresis_up) {
                            this->state = State::ZERO;
                        }
                        break;
                    case State::ZERO:
                        if (error > this->hysteresis_up) {
                            this->state = State::POSITIVE;
                        } else if (error < this->hysteresis_down) {
                            this->state = State::NEGATIVE;
                        }
                        break;
                    default:
                        break;
                }
                return this->state;
            }

            Value hysteresis_up{};
            Value hysteresis_down{};

            State state{State::ZERO};
        };

/* VARIANT SOLUTION (COMPILE TIME POLYMORPHISM) */
#ifdef REGULATOR_VARIANT

        template <Linalg::Arithmetic Value>
        using Regulator = std::variant<LQR<Value>, PID<Value>, ADRC<Value>, Binary<Value>, Ternary<Value>>;

        template <Algorithm algorithm, typename... Args>
        [[nodiscard]] auto make_regulator(Args... args) noexcept
        {
            using Value = typename FirstType<Args...>::Type;

            if constexpr (algorithm == Algorithm::PID) {
                return Regulator<Value>{std::in_place_type<PID<Value>>, args...};
            }
            if constexpr (algorithm == Algorithm::LQR) {
                return Regulator<Value>{std::in_place_type<LQR<Value>>, args...};
            }
            if constexpr (algorithm == Algorithm::ADRC) {
                return Regulator<Value>{std::in_place_type<ADRC<Value>>, args...};
            }
            if constexpr (algorithm == Algorithm::BINARY) {
                return Regulator<Value>{std::in_place_type<Binary<Value>>, args...};
            }
            if constexpr (algorithm == Algorithm::TERNARY) {
                return Regulator<Value>{std::in_place_type<Ternary<Value>>, args...};
            }
        }

#endif // REGULATOR_VARIANT

/* LAMBDA SOLUTION (TYPE ERASURE, although you will need std::function to containerize state-full lambda) */
#ifdef REGULATOR_LAMBDA

        template <Linalg::Arithmetic Value>
        using Regulator = std::function<Value(Value, Value)>;

        template <Algorithm algorithm, Linalg::Arithmetic... Args>
        [[nodiscard]] auto make_regulator(Args... args) noexcept
        {
            using Value = typename FirstType<Args...>::Type;

            if constexpr (algorithm == Algorithm::PID) {
                return
                    [pid = PID<Value>{args...}](const Value error, const Value dt) mutable { return pid(error, dt); };
            }
            if constexpr (algorithm == Algorithm::LQR) {
                return
                    [lqr = LQR<Value>{args...}](const Value error, const Value dt) mutable { return lqr(error, dt); };
            }
            if constexpr (algorithm == Algorithm::ADRC) {
                return [adrc = ADRC<Value>{args...}](const Value error, const Value dt) mutable {
                    return adrc(error, dt);
                };
            }
            if constexpr (algorithm == Algorithm::BINARY) {
                return [binary = Binary<Value>{args...}](const Value error) mutable { return binary(error); };
            }
            if constexpr (algorithm == Algorithm::TERNARY) {
                return [ternary = Ternary<Value>{args...}](const Value error) mutable { return ternary(error); };
            }
        }
    }

#endif // REGULATOR_LAMBDA

/* UNIQUE_PTR SOLUTION (RUNTIME POLYMORPHISM)*/
#ifdef REGULATOR_PTR

    /* REGULATOR BASE */
    template <Linalg::Arithmetic Value>
    struct Base {
        virtual ~Base() noexcept = 0;
    };

    template <Linalg::Arithmetic Value>
    using Regulator = std::unique_ptr<Regulator<Value>>;

    template <Algorithm algorithm, Linalg::Arithmetic... Args>
    [[nodiscard]] Regulator<Value> make_regulator(Args... args)
    {
        using Value = typename FirstType<Args...>::Type;

        if constexpr (algorithm == Algorithm::PID) {
            return std::make_unique<PID<Value>>(args...);
        }
        if constexpr (algorithm == Algorithm::LQR) {
            return std::make_unique<LQR<Value>>(args...);
        }
        if constexpr (algorithm == Algorithm::ADRC) {
            return std::make_unique<ADRC<Value>>(args...);
        }
        if constexpr (algorithm == Algorithm::BINARY) {
            return std::make_unique<Binary<Value>>(args...);
        }
        if constexpr (algorithm == Algorithm::TERNARY) {
            return std::make_unique<Ternary<Value>>(args...);
        }
    }
}

#endif // REGULATOR_PTR
}
; // namespace Regulator
}
; // namespace InvertedSway

#endif // REGULATOR_HPP
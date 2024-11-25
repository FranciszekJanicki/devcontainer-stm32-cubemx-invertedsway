#ifndef REGULATOR_HPP
#define REGULATOR_HPP

#include "arithmetic.hpp"
#include <algorithm>
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
                return this->P * error + this->D * (error - std::exchange(this->previous_error, error)) / dt +
                       this->I * this->sum;
            }

            Value P{};
            Value I{};
            Value D{};
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

            Value placeholder1;
            Value placeholder2;
            Value placeholder3;
            Value placeholder4;
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

            Value placeholder1;
            Value placeholder2;
            Value placeholder3;
            Value placeholder4;
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

            Value operator()(const Value error, [[maybe_unused]] const Value dt) noexcept
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
                // return this->state;
                return Value{}; // for compatibility with other variants
            }

            Value placeholder1;
            Value placeholder2;

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

            Value operator()(const Value error, [[maybe_unused]] const Value dt) noexcept
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
                // return this->state;
                return Value{}; // for compatibility with other variants
            }

            Value placeholder1;
            Value placeholder2;

            Value hysteresis_up{};
            Value hysteresis_down{};

            State state{State::ZERO};
        };

/* VARIANT SOLUTION (COMPILE TIME POLYMORPHISM) */
#ifdef REGULATOR_VARIANT

        template <Linalg::Arithmetic Value>
        using Regulator = std::variant<LQR<Value>, PID<Value>, ADRC<Value>, Binary<Value>, Ternary<Value>>;

        template <Linalg::Arithmetic Value, typename... Args>
        [[nodiscard]] auto make_regulator(const Algorithm algorithm, Args&&... args) noexcept
        {
            switch (algorithm) {
                case Algorithm::PID:
                    return Regulator<Value>{std::in_place_type_t<PID<Value>>{}, std::forward<Args>(args)...};
                case Algorithm::LQR:
                    return Regulator<Value>{std::in_place_type_t<LQR<Value>>{}, std::forward<Args>(args)...};
                case Algorithm::ADRC:
                    return Regulator<Value>{std::in_place_type_t<ADRC<Value>>{}, std::forward<Args>(args)...};
                case Algorithm::BINARY:
                    return Regulator<Value>{std::in_place_type_t<Binary<Value>>{}, std::forward<Args>(args)...};
                case Algorithm::TERNARY:
                    return Regulator<Value>{std::in_place_type_t<Ternary<Value>>{}, std::forward<Args>(args)...};
                default:
                    return Regulator<Value>{};
            }
        }

#endif // REGULATOR_VARIANT

/* LAMBDA SOLUTION (TYPE ERASURE, although you will need std::function to containerize state-full lambda) */
#ifdef REGULATOR_LAMBDA

        template <Linalg::Arithmetic Value>
        using Regulator = std::function<Value(Value, Value)>;

        template <Linalg::Arithmetic Value, typename... Args>
        [[nodiscard]] auto make_regulator(const Algorithm algorithm, Args&&... args) noexcept
        {
            switch (algorithm) {
                case Algorithm::PID:
                    return [pid = PID<Value>{std::forward<Args>(args)...}](const Value error, const Value dt) mutable {
                        return pid(error, dt);
                    };
                case Algorithm::LQR:
                    return [lqr = LQR<Value>{std::forward<Args>(args)...}](const Value error, const Value dt) mutable {
                        return lqr(error, dt);
                    };
                case Algorithm::ADRC:
                    return
                        [adrc = ADRC<Value>{std::forward<Args>(args)...}](const Value error, const Value dt) mutable {
                            return adrc(error, dt);
                        };
                case Algorithm::BINARY:
                    return [binary = Binary<Value>{std::forward<Args>(args)...}](const Value error) mutable {
                        return binary(error);
                    };
                case Algorithm::TERNARY:
                    return [ternary = Ternary<Value>{std::forward<Args>(args)...}](const Value error) mutable {
                        return ternary(error);
                    };
                default:
                    return [](const Value error) mutable { return error; };
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

        template <Linalg::Arithmetic Value, typename... Args>
        [[nodiscard]] auto make_regulator(const Algorithm algorithm, Args&&... args)
        {
            switch (algorithm) {
                case Algorithm::PID:
                    return std::make_unique<PID<Value>>(std::forward<Args>(args)...);
                case Algorithm::LQR:
                    return std::make_unique<LQR<Value>>(std::forward<Args>(args)...);
                case Algorithm::ADRC:
                    return std::make_unique<ADRC<Value>>(std::forward<Args>(args)...);
                case Algorithm::BINARY:
                    return std::make_unique<Binary<Value>>(std::forward<Args>(args)...);
                case Algorithm::TERNARY:
                    return std::make_unique<Ternary<Value>>(std::forward<Args>(args)...);
                default:
                    return Regulator<Value>{nullptr};
            }
        }

#endif // REGULATOR_PTR

    }; // namespace Regulator

}; // namespace InvertedSway

#endif // REGULATOR_HPP
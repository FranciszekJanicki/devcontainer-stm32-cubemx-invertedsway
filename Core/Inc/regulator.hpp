#ifndef REGULATOR_HPP
#define REGULATOR_HPP

#include "arithmetic.hpp"
#include <algorithm>
#include <functional>
#include <memory>
#include <utility>
#include <variant>

namespace InvertedSway {

    enum struct RegulatorAlgo {
        PID,
        LQR,
        ADRC,
        BINARY,
        TERNARY,
    };

// i have chosen std::variant for polymorphic regulator, as its sigma container
#define REGULATOR_VARIANT

/* UNIQUE_PTR SOLUTION (RUNTIME POLYMORPHISM)*/
#ifdef REGULATOR_PTR

    /* REGULATOR BASE */
    template <Linalg::Arithmetic Value>
    struct Regulator {
        virtual ~Regulator() noexcept = 0;
    };

    /* PID */
    template <Linalg::Arithmetic Value>
    struct RegulatorPID : public Regulator<Value> {
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

    /* LQR */
    template <Linalg::Arithmetic Value>
    struct RegulatorLQR : public Regulator<Value> {
        Value operator()(const Value error, const Value) noexcept
        {
            // implement lqr algorithm here
            return error;
        }

        Value placeholder1{};
        Value placeholder2{};
        Value placeholder3{};
        Value placeholder4{};
    };

    template <Linalg::Arithmetic Value>
    struct RegulatorADRC : public Regulator<Value> {
        Value operator()(const Value error, const Value) noexcept
        {
            // implement adrc algorithm here
            return error;
        }

        Value placeholder1{};
        Value placeholder2{};
        Value placeholder3{};
        Value placeholder4{};
    };

    /* BINARY */
    template <Linalg::Arithmetic Value>
    struct RegulatorBinary : public Regulator<Value> {
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

        Value placeholder1{};
        Value placeholder2{};

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

        Value placeholder1{};
        Value placeholder2{};

        Value hysteresis_up{};
        Value hysteresis_down{};

        State state{State::ZERO};
    };

    template <Linalg::Arithmetic Value>
    using Regulator = std::unique_ptr<Regulator<Value>>;

    template <Linalg::Arithmetic Value, typename... RegulatorArgs>
    [[nodiscard]] Regulator<Value> make_regulator(const RegulatorAlgo regulator_algo, RegulatorArgs&&... regulator_args)
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
                return Regulator<Value>{nullptr};
        }
    }

#endif

/* VARIANT SOLUTION (COMPILE TIME POLYMORPHISM) */
#ifdef REGULATOR_VARIANT

    /* PID */
    template <Linalg::Arithmetic Value>
    struct RegulatorPID {
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

    /* LQR */
    template <Linalg::Arithmetic Value>
    struct RegulatorLQR {
        Value operator()(const Value error, const Value) noexcept
        {
            // implement lqr algorithm here
            return error;
        }

        Value placeholder1{};
        Value placeholder2{};
        Value placeholder3{};
        Value placeholder4{};
    };

    template <Linalg::Arithmetic Value>
    struct RegulatorADRC {
        Value operator()(const Value error, const Value) noexcept
        {
            // implement adrc algorithm here
            return error;
        }

        Value placeholder1{};
        Value placeholder2{};
        Value placeholder3{};
        Value placeholder4{};
    };

    /* BINARY */
    template <Linalg::Arithmetic Value>
    struct RegulatorBinary {
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

        Value placeholder1{};
        Value placeholder2{};

        Value hysteresis_up{};
        Value hysteresis_down{};

        State state{State::ZERO};
    };

    /* TERNARY */
    template <Linalg::Arithmetic Value>
    struct RegulatorTernary {
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

        Value placeholder1{};
        Value placeholder2{};

        Value hysteresis_up{};
        Value hysteresis_down{};

        State state{State::ZERO};
    };

    template <Linalg::Arithmetic Value>
    using Regulator = std::variant<RegulatorLQR<Value>,
                                   RegulatorPID<Value>,
                                   RegulatorADRC<Value>,
                                   RegulatorBinary<Value>,
                                   RegulatorTernary<Value>>;

    template <Linalg::Arithmetic Value, typename... RegulatorArgs>
    [[nodiscard]] Regulator<Value> make_regulator(const RegulatorAlgo regulator_algo,
                                                  RegulatorArgs&&... regulator_args) noexcept
    {
        switch (regulator_algo) {
            case RegulatorAlgo::PID:
                return Regulator<Value>{std::in_place_type<RegulatorPID<Value>>,
                                        std::forward<RegulatorArgs>(regulator_args)...};
            case RegulatorAlgo::LQR:
                return Regulator<Value>{std::in_place_type<RegulatorLQR<Value>>,
                                        std::forward<RegulatorArgs>(regulator_args)...};
            case RegulatorAlgo::ADRC:
                return Regulator<Value>{std::in_place_type<RegulatorADRC<Value>>,
                                        std::forward<RegulatorArgs>(regulator_args)...};
            case RegulatorAlgo::BINARY:
                return Regulator<Value>{std::in_place_type<RegulatorBinary<Value>>,
                                        std::forward<RegulatorArgs>(regulator_args)...};
            case RegulatorAlgo::TERNARY:
                return Regulator<Value>{std::in_place_type<RegulatorTernary<Value>>,
                                        std::forward<RegulatorArgs>(regulator_args)...};
            default:
                return Regulator<Value>{};
        }
    }

#endif

/* LAMBDA SOLUTION (TYPE ERASURE, although you will need std::function to containerize state-full lambda) */
#ifdef REGULATOR_LAMBDA

    template <Linalg::Arithmetic Value>
    struct RegulatorPID {
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

    /* LQR */
    template <Linalg::Arithmetic Value>
    struct RegulatorLQR {
        Value operator()(const Value error, const Value) noexcept
        {
            // implement lqr algorithm here
            return error;
        }

        Value placeholder1{};
        Value placeholder2{};
        Value placeholder3{};
        Value placeholder4{};
    };

    template <Linalg::Arithmetic Value>
    struct RegulatorADRC {
        Value operator()(const Value error, const Value) noexcept
        {
            // implement adrc algorithm here
            return error;
        }

        Value placeholder1{};
        Value placeholder2{};
        Value placeholder3{};
        Value placeholder4{};
    };

    /* BINARY */
    template <Linalg::Arithmetic Value>
    struct RegulatorBinary {
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

        Value placeholder1{};
        Value placeholder2{};

        Value hysteresis_up{};
        Value hysteresis_down{};

        State state{State::ZERO};
    };

    /* TERNARY */
    template <Linalg::Arithmetic Value>
    struct RegulatorTernary {
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

        Value placeholder1{};
        Value placeholder2{};

        Value hysteresis_up{};
        Value hysteresis_down{};

        State state{State::ZERO};
    };

    template <Linalg::Arithmetic Value>
    using Regulator = std::function<Value(Value, Value)>;

    template <Linalg::Arithmetic Value, typename... RegulatorArgs>
    [[nodiscard]] auto make_regulator(const RegulatorAlgo regulator_algo, RegulatorArgs&&... regulator_args) noexcept
    {
        switch (regulator_algo) {
            case RegulatorAlgo::PID:
                return [pid = RegulatorPID<Value>{std::forward<RegulatorArgs>(
                            regulator_args)...}](const Value error, const Value dt) mutable { return pid(error, dt); };
            case RegulatorAlgo::LQR:
                return [lqr = RegulatorLQR<Value>{std::forward<RegulatorArgs>(
                            regulator_args)...}](const Value error, const Value dt) mutable { return lqr(error, dt); };
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

#endif

}; // namespace InvertedSway

#endif // REGULATOR_HPP
#ifndef REGULATOR_HPP
#define REGULATOR_HPP

#include "arithmetic.hpp"
#include <algorithm>
#include <concepts>
#include <utility>

namespace InvertedSway::Regulators {

    template <Linalg::Arithmetic Value>
    struct PID {
        [[nodiscard]] inline Value operator()(Value const error, Value const dt) noexcept
        {
            this->sum += (error + this->previous_error) / 2 * dt;
            this->sum = std::clamp(this->sum, -this->windup / this->ki, this->windup / this->ki);
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
    struct LQR {
        [[nodiscard]] inline Value operator()(Value const error, Value const dt) noexcept
        {
            // implement lqr algorithm here
            return error;
        }
    };

    template <Linalg::Arithmetic Value>
    struct ADRC {
        [[nodiscard]] inline Value operator()(Value const error, Value const dt) noexcept
        {
            // implement adrc algorithm here
            return error;
        }
    };

    template <Linalg::Arithmetic Value>
    struct Binary

    {
        enum struct State {
            POSITIVE,
            ZERO,
        };

        [[nodiscard]] inline State operator()(Value const error) noexcept
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
    struct Ternary {
        enum struct State {
            POSITIVE,
            NEGATIVE,
            ZERO,
        };

        [[nodiscard]] inline State operator()(Value const error) noexcept
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

}; // namespace InvertedSway::Regulators

#endif // REGULATOR_HPP
#ifndef KALMAN_HPP
#define KALMAN_HPP

#include "arithmetic.hpp"
#include <array>
#include <utility>

namespace InvertedSway {

    template <Linalg::Arithmetic Value>
    struct Kalman {
        constexpr Value operator()(const Value gyro, const Value accel, const Value dt) noexcept
        {
            predict(gyro, dt);
            update(accel);
            return k_angle;
        }

        constexpr void predict(const Value gyro, const Value dt) noexcept
        {
            k_angle += dt * (gyro - k_bias);

            P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
            P[0][1] -= dt * P[1][1];
            P[1][0] -= dt * P[1][1];
            P[1][1] += dt * Q_angle;

            K[0] = P[0][0] / (P[0][0] + R);
            K[1] = P[1][0] / (P[0][0] + R);
        }

        constexpr void update(const Value accel) noexcept
        {
            k_angle += K[0] * (accel - k_angle);
            k_bias += K[1] * (accel - k_angle);

            P[0][0] *= (Value{1} - K[0]);
            P[0][1] *= (Value{1} - K[0]);
            P[1][0] -= K[1] * P[0][0];
            P[1][1] -= K[1] * P[0][1];
        }

        Value k_angle{};
        Value k_bias{};
        Value Q_angle{};
        Value Q_bias{};
        Value R{};

        std::array<Value, 2> K{};
        std::array<std::array<Value, 2>, 2> P{};
    };

    template <Linalg::Arithmetic Value, typename... KalmanArgs>
    [[nodiscard]] constexpr auto make_kalman(KalmanArgs&&... kalman_args) noexcept
    {
        return Kalman<Value>{std::forward<KalmanArgs>(kalman_args)...};
    }

}; // namespace InvertedSway

#endif // KALMAN_HPP
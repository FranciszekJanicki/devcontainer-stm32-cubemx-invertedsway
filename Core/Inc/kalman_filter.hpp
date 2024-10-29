#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include "vector3d.hpp"
#include <array>

template <typename Value>
struct KalmanFilter {
    using Accel = Linalg::Vector3D<Value>;
    using Gyro = Linalg::Vector3D<Value>;
    using P = std::array<std::array<Value, 2>, 2>;
    using K = std::array<std::array<Value, 2>, 1>;

    Value operator()(const Gyro gyro, const Accel accel, const Value dt) noexcept
    {
        predict(gyro, dt);
        update(accel);
        return k_angle;
    }

    void predict(const Gyro gyro, const Value dt) noexcept
    {
        k_angle += dt * (gyro - k_bias);

        P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += dt * Q_angle;

        K[0] = P[0][0] / (P[0][0] + R);
        K[1] = P[1][0] / (P[0][0] + R);
    }

    void update(const Accel accel) noexcept
    {
        k_angle += K[0] * (accel - k_angle);
        k_bias += K[1] * (accel - k_angle);

        P[0][0] *= (Unit{1} - K[0]);
        P[0][1] *= (Unit{1} - K[0]);
        P[1][0] -= K[1] * P[0][0];
        P[1][1] -= K[1] * P[0][1];
    }

    Value step_time{1};

    Value k_angle{};
    Value k_bias{};
    Value Q_angle{};
    Value Q_bias{};
    Value R{};

    P K{};
    K P{};
};

#endif // KALMAN_FILTER_HPP
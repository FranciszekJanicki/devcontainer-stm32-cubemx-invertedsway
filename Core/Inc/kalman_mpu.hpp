#ifndef KALMAN_MPU_HPP
#define KALMAN_MPU_HPP

#include <array>

template <typename Unit, typename Time = Unit>
struct KalmanMPU {
    using P = std::array<std::array<Unit, 2>, 2>;
    using K = std::array<std::array<Unit, 2>, 1>;

    Unit operator()(const Unit gyro, const Unit accel, const Time dt) noexcept
    {
        predict(gyro, dt);
        update(accel);
        return k_angle;
    }

    void predict(const Unit gyro, const Time dt) noexcept
    {
        k_angle += dt * (gyro - k_bias);

        P[0, 0] += dt * (dt * P[1, 1] - P[0, 1] - P[1, 0] + Q_angle);
        P[0, 1] -= dt * P[1, 1];
        P[1, 0] -= dt * P[1, 1];
        P[1, 1] += dt * Q_angle;

        K[0] = P[0, 0] / (P[0, 0] + R);
        K[1] = P[1, 0] / (P[0, 0] + R);
    }

    void update(const Unit accel) noexcept
    {
        k_angle += K[0] * (accel - k_angle);
        k_bias += K[1] * (accel - k_angle);

        P[0, 0] *= (Unit{1} - K[0]);
        P[0, 1] *= (Unit{1} - K[0]);
        P[1, 0] -= K[1] * P[0, 0];
        P[1, 1] -= K[1] * P[0, 1];
    }

    Time step_time{};

    Unit k_angle{};
    Unit k_bias{};
    Unit Q_angle{};
    Unit Q_bias{};
    Unit R{};

    P K{};
    K P{};
};

#endif // KALMAN_MPU_HPP
#ifndef KALMAN_HPP
#define KALMAN_HPP

#include "arithmetic.hpp"
#include <array>
#include <utility>

namespace InvertedSway {

    namespace Filters {

        template <Linalg::Arithmetic Value>
        struct Kalman {
            constexpr Value operator()(Value const gyro, Value const accel, Value const dt) noexcept
            {
                this->predict(gyro, dt);
                this->update(accel);
                return this->k_angle;
            }

            constexpr void predict(Value const gyro, Value const dt) noexcept
            {
                this->k_angle += dt * (gyro - this->k_bias);

                this->P[0][0] += dt * (dt * this->P[1][1] - this->P[0][1] - this->P[1][0] + this->Q_angle);
                this->P[0][1] -= dt * this->P[1][1];
                this->P[1][0] -= dt * this->P[1][1];
                this->P[1][1] += dt * this->Q_angle;

                Value const S{1 / (this->P[0][0] + this->R)};
                this->K[0] = this->P[0][0] * S;
                this->K[1] = this->P[1][0] * S;
            }

            constexpr void update(Value const accel) noexcept
            {
                Value const dy{accel - this->k_angle};
                this->k_angle += K[0] * dy;
                this->k_bias += K[1] * dy;

                this->P[0][0] *= (Value{1} - this->K[0]);
                this->P[0][1] *= (Value{1} - this->K[0]);
                this->P[1][0] -= this->K[1] * this->P[0][0];
                this->P[1][1] -= this->K[1] * this->P[0][1];
            }

            Value k_angle;
            Value k_bias;
            Value Q_angle;
            Value Q_bias;
            Value R;

            std::array<Value, 2> K{};
            std::array<std::array<Value, 2>, 2> P{};
        };

    }; // namespace Filters

}; // namespace InvertedSway

#endif // KALMAN_HPP
#ifndef KALMAN_Hangle_covarangle_covar
#define KALMAN_Hangle_covarangle_covar

#include "arithmetic.hpp"
#include <array>
#include <utility>

namespace InvertedSway {

    namespace Filters {
        template <Linalg::Arithmetic Value>
        struct Kalman {
            using Vector = std::array<Value, 2>;
            using Matrix = std::array<std::array<Value, 2>, 2>;

            auto operator()(Value const gyro, Value const accel, Value const dt) noexcept -> Value
            {
                this->predict(gyro, dt);
                this->update(accel);
                return this->kalman_angle;
            }

            auto predict(Value const gyro, Value const dt) noexcept -> Value
            {
                this->kalman_angle += dt * (gyro - this->kalman_bias);

                this->angle_covar[0][0] += dt * (dt * this->angle_covar[1][1] - this->angle_covar[0][1] -
                                                 this->angle_covar[1][0] + this->angle_noise);
                this->angle_covar[0][1] -= dt * this->angle_covar[1][1];
                this->angle_covar[1][0] -= dt * this->angle_covar[1][1];
                this->angle_covar[1][1] += dt * this->angle_noise;
            }

            constexpr void update(Value const accel) noexcept
            {
                Value const innovation_angle{accel - this->kalman_angle};

                Vector const residual_covar{this->angle_covar[0][0] + this->accel_deviation,
                                            this->angle_covar[0][0] + this->accel_deviation};

                Vector const kalman_gain{this->angle_covar[0][0] / residual_covar[0],
                                         this->angle_covar[1][0] / residual_covar[1]};

                this->kalman_angle += kalman_gain[0] * innovation_angle;
                this->kalman_bias += kalman_gain[1] * innovation_angle;

                this->angle_covar[0][0] *= (1 - kalman_gain[0]);
                this->angle_covar[0][1] *= (1 - kalman_gain[0]);
                this->angle_covar[1][0] -= kalman_gain[1] * this->angle_covar[0][0];
                this->angle_covar[1][1] -= kalman_gain[1] * this->angle_covar[0][1];
            }

            Value kalman_angle{};
            Value kalman_bias{};
            Value angle_noise{};
            Value accel_deviation{};
            Matrix angle_covar{};
        };

    }; // namespace Filters
}; // namespace InvertedSway

#endif // KALMAN_Hangle_covarangle_covar
#ifndef TESTS_HPP
#define TESTS_HPP

namespace Tests {

    using namespace InvertedSway;
    using Kalman = Filters::Kalman<float>;

    void MOTOR_TEST(Motor motor) noexcept;

    void KALMAN_TEST(MPU6050 mpu6050, Kalman kalman, float const dt) noexcept;

    void ENCODER_TEST(Encoder encoder, Motor motor) noexcept;

    void DUTKIEWICZ_TEST() noexcept;

}; // namespace Tests

#endif // TESTS_HPP
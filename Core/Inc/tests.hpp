#ifndef TESTS_HPP
#define TESTS_HPP

#include "encoder.hpp"
#include "kalman.hpp"
#include "motor.hpp"
#include "mpu6050.hpp"

namespace Tests {

    using namespace InvertedSway;
    using Kalman = Filters::Kalman<float>;

    void MOTOR_TEST(Motor motor) noexcept;

    void MOTOR_BOOST_TEST(Motor motor, float const voltage_start_threshold) noexcept;

    void KALMAN_TEST(MPU6050 mpu6050, Kalman kalman, std::uint32_t const sampling_rate) noexcept;

    void ENCODER_TEST(Encoder encoder, Motor motor) noexcept;

    void DUTKIEWICZ_TEST() noexcept;

}; // namespace Tests

#endif // TESTS_HPP
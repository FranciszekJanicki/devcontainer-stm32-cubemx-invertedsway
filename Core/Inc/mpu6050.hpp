#ifndef MPU6050_HPP
#define MPU6050_HPP

#include "stm32l4xx_hal.h"
#include "vector3d.hpp"
#include <concepts>
#include <cstdint>
#include <type_traits>

namespace MPU6050 {

    using gyro_result = linalg::vector3D<std::double_t>;
    using accel_result = linalg::vector3D<std::double_t>;
    using roll_pitch_yaw = linalg::vector3D<std::double_t>;
    using gyro_raw_result = linalg::vector3D<std::int32_t>;
    using accel_raw_result = linalg::vector3D<std::int32_t>;

    //***********************************//
    //			FUNCTIONS				 //
    //***********************************//
    void Init(I2C_HandleTypeDef& i2c, const std::uint8_t GyroRange, const std::uint8_t AccelRange) noexcept;
    std::uint8_t GetDeviceID(I2C_HandleTypeDef& i2c) noexcept;

    //
    //	CONFIG
    //
    void SetDlpf(I2C_HandleTypeDef& i2c, const std::uint8_t Value) noexcept;

    //
    //	PWR_MGMT_12
    //
    void DeviceReset(I2C_HandleTypeDef& i2c, const std::uint8_t Reset) noexcept;
    void SetClockSource(I2C_HandleTypeDef& i2c, const std::uint8_t Source) noexcept;
    void SetSleepEnabled(I2C_HandleTypeDef& i2c, const std::uint8_t Enable) noexcept;
    void SetCycleEnabled(I2C_HandleTypeDef& i2c, const std::uint8_t Enable) noexcept;
    void SetTemperatureSensorDisbled(I2C_HandleTypeDef& i2c, const std::uint8_t Disable) noexcept;

    //
    //	PWR_MGMT_2
    //
    void SetLowPowerWakeUpFrequency(I2C_HandleTypeDef& i2c, const std::uint8_t Frequency) noexcept;
    void AccelerometerAxisStandby(I2C_HandleTypeDef& i2c,
                                  const std::uint8_t XA_Stby,
                                  const std::uint8_t YA_Stby,
                                  const std::uint8_t ZA_Stby) noexcept;
    void GyroscopeAxisStandby(I2C_HandleTypeDef& i2c,
                              const std::uint8_t XG_Stby,
                              const std::uint8_t YG_Stby,
                              const std::uint8_t ZG_Stby) noexcept;

    //
    //	Measurement scale configuration
    //
    void SetFullScaleGyroRange(I2C_HandleTypeDef& i2c, const std::uint8_t Range) noexcept;
    void SetFullScaleAccelRange(I2C_HandleTypeDef& i2c, const std::uint8_t Range) noexcept;

    //
    // Reading data
    //
    std::int32_t GetTemperatureRAW(I2C_HandleTypeDef& i2c) noexcept;
    std::double_t GetTemperatureCelsius(I2C_HandleTypeDef& i2c) noexcept;

    std::int32_t GetAccelerationXRAW(I2C_HandleTypeDef& i2c) noexcept;
    std::int32_t GetAccelerationYRAW(I2C_HandleTypeDef& i2c) noexcept;
    std::int32_t GetAccelerationZRAW(I2C_HandleTypeDef& i2c) noexcept;
    accel_raw_result GetAccelerometerRAW(I2C_HandleTypeDef& i2c) noexcept;
    accel_result GetAccelerometerScaled(I2C_HandleTypeDef& i2c, const std::uint8_t AccelRange) noexcept;

    std::int32_t GetRotationXRAW(I2C_HandleTypeDef& i2c) noexcept;
    std::int32_t GetRotationYRAW(I2C_HandleTypeDef& i2c) noexcept;
    std::int32_t GetRotationZRAW(I2C_HandleTypeDef& i2c) noexcept;
    gyro_raw_result GetGyroscopeRAW(I2C_HandleTypeDef& i2c) noexcept;
    gyro_result GetGyroscopeScaled(I2C_HandleTypeDef& i2c, const std::uint8_t GyroRange) noexcept;

    roll_pitch_yaw GetRollPitchYaw(I2C_HandleTypeDef& i2c, const std::uint8_t AccelRange) noexcept;

    //
    //	Setting INT pin
    //
    // INT_PIN_CFG register
    void SetInterruptMode(I2C_HandleTypeDef& i2c, const std::uint8_t Mode) noexcept;
    void SetInterruptDrive(I2C_HandleTypeDef& i2c, const std::uint8_t Drive) noexcept;
    void SetInterruptLatch(I2C_HandleTypeDef& i2c, const std::uint8_t Latch) noexcept;
    void SetInterruptLatchClear(I2C_HandleTypeDef& i2c, const std::uint8_t Clear) noexcept;
    // INT_ENABLE register
    void SetIntEnableRegister(I2C_HandleTypeDef& i2c, std::uint8_t Value) noexcept;
    void SetIntDataReadyEnabled(I2C_HandleTypeDef& i2c, const std::uint8_t Enable) noexcept;
    // INT_STATUS register
    std::uint8_t GetIntStatusRegister(I2C_HandleTypeDef& i2c) noexcept;

    //
    //	Motion functions - not included in documentation/register map
    //
    std::uint8_t GetMotionStatusRegister(I2C_HandleTypeDef& i2c) noexcept;

    void SetDHPFMode(I2C_HandleTypeDef& i2c, const std::uint8_t Dhpf) noexcept;
    // INT_ENABLE register
    void SetIntZeroMotionEnabled(I2C_HandleTypeDef& i2c, const std::uint8_t Enable) noexcept;
    void SetIntMotionEnabled(I2C_HandleTypeDef& i2c, const std::uint8_t Enable) noexcept;
    void SetIntFreeFallEnabled(I2C_HandleTypeDef& i2c, const std::uint8_t Enable) noexcept;

    void SetMotionDetectionThreshold(I2C_HandleTypeDef& i2c, std::uint8_t Threshold) noexcept;
    void SetMotionDetectionDuration(I2C_HandleTypeDef& i2c, std::uint8_t Duration) noexcept;

    void SetZeroMotionDetectionThreshold(I2C_HandleTypeDef& i2c, std::uint8_t Threshold) noexcept;
    void SetZeroMotionDetectionDuration(I2C_HandleTypeDef& i2c, std::uint8_t Duration) noexcept;

    void SetFreeFallDetectionThreshold(I2C_HandleTypeDef& i2c, std::uint8_t Threshold) noexcept;
    void SetFreeFallDetectionDuration(I2C_HandleTypeDef& i2c, std::uint8_t Duration) noexcept;

}; // namespace MPU6050

#endif // MPU6050_HPP
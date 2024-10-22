#include "mpu6050.hpp"
#include "common.hpp"
#include "stm32l4xx_hal.h"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <expected>

using Error = MPU6050::Error;
using Scaled = MPU6050::Scaled;
using GyroScaled = MPU6050::GyroScaled;
using AccelScaled = MPU6050::AccelScaled;
using RollPitchYaw = MPU6050::RollPitchYaw;
using TempScaled = MPU6050::TempScaled;
using Raw = MPU6050::Raw;
using GyroRaw = MPU6050::GyroRaw;
using AccelRaw = MPU6050::AccelRaw;
using TempRaw = MPU6050::TempRaw;
using ExpectedRaw = MPU6050::ExpectedRaw;
using ExpectedTempRaw = MPU6050::ExpectedTempRaw;
using ExpectedTempScaled = MPU6050::ExpectedTempScaled;
using ExpectedGyroRaw = MPU6050::ExpectedGyroRaw;
using ExpectedGyroScaled = MPU6050::ExpectedGyroScaled;
using ExpectedAccelRaw = MPU6050::ExpectedAccelRaw;
using ExpectedAccelScaled = MPU6050::ExpectedAccelScaled;
using ExpectedAddres = MPU6050::ExpectedAddres;
using ExpectedRPY = MPU6050::ExpectedRPY;
using Unexpected = MPU6050::Unexpected;

const char* MPU6050::error_to_string(const Error error) noexcept
{
    switch (error) {
        case Error::OK:
            return "OK ERROR";
        case Error::INIT:
            return "INIT ERROR";
        case Error::DEINIT:
            return "DEINIT ERROR";
        case Error::ACCEL:
            return "ACCEL ERROR";
        case Error::GYRO:
            return "GYRO ERROR";
        case Error::TEMP:
            return "TEMP ERROR";
        default:
            return "NONE ERROR";
    }
}

Scaled MPU6050::gyro_range_to_scale(const std::uint8_t gyro_range) noexcept
{
    switch (gyro_range) {
        case GYRO_FS_250:
            return 0.007633;
        case GYRO_FS_500:
            return 0.015267;
        case GYRO_FS_1000:
            return 0.030487;
        case GYRO_FS_2000:
            return 0.060975;
        default:
            return 0.0;
    }
}

Scaled MPU6050::accel_range_to_scale(const std::uint8_t accel_range) noexcept
{
    switch (accel_range) {
        case ACCEL_FS_2:
            return 0.000061;
        case ACCEL_FS_4:
            return 0.000122;
        case ACCEL_FS_8:
            return 0.000244;
        case ACCEL_FS_16:
            return 0.0004882;
        default:
            return 0.0;
    }
}

MPU6050::MPU6050(UartHandle uart,
                 I2cHandle i2c,
                 const std::uint8_t addres,
                 const std::uint8_t gyro_range,
                 const std::uint8_t accel_range) noexcept :
    uart_{uart}, i2c_{i2c}, addres_{addres}, gyro_range_{gyro_range}, accel_range_{accel_range}
{
    print_and_return(initialize());
}

MPU6050::~MPU6050() noexcept
{
    print_and_return(initialize());
}

Error MPU6050::initialize() noexcept
{
    if (initialized_) {
        return Error::INIT;
    }
    if (auto err{device_reset(1)}; err != Error::OK) {
        return print_and_return(err);
    }
    if (auto err{set_sleep_enabled(0)}; err != Error::OK) {
        return print_and_return(err);
    }
    if (auto err{set_clock_source(CLOCK_INTERNAL)}; err != Error::OK) {
        return print_and_return(err);
    }
    if (auto err{set_dlpf(DLPF_BW_20)}; err != Error::OK) {
        return print_and_return(err);
    }
    if (auto err{set_full_scale_gyro_range(gyro_range_)}; err != Error::OK) {
        return print_and_return(err);
    }
    if (auto err{set_full_scale_accel_range(accel_range_)}; err != Error::OK) {
        return print_and_return(err);
    }
    if (auto err{set_interrupt()}; err != Error::OK) {
        return print_and_return(err);
    }
    initialized_ = true;
    return Error::OK;
}

Error MPU6050::deinitialize() noexcept
{
    if (!initialized_) {
        return Error::DEINIT;
    }
    device_reset(PWR1_DEVICE_RESET_BIT);
    initialized_ = false;
    return Error::OK;
}

Error MPU6050::set_dlpf(const std::uint8_t value) const noexcept
{
    std::uint8_t buffer;
    if (auto err{HAL_I2C_Mem_Read(i2c_, addres_, RA_CONFIG, sizeof(RA_CONFIG), &buffer, sizeof(buffer), i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::INIT;
    }
    buffer &= std::uint8_t{0xF8};
    buffer |= std::uint8_t(value & 0x7);
    if (auto err{HAL_I2C_Mem_Write(i2c_, addres_, RA_CONFIG, sizeof(RA_CONFIG), &buffer, sizeof(buffer), i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::INIT;
    }
    return Error::OK;
}

//
// PWR_MGMT_1
//
Error MPU6050::device_reset(const std::uint8_t reset) const noexcept
{
    std::uint8_t buffer;
    if (auto err{HAL_I2C_Mem_Read(i2c_,
                                  addres_,
                                  RA_PWR_MGMT_1,
                                  sizeof(RA_PWR_MGMT_1),
                                  &buffer,
                                  sizeof(buffer),
                                  i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::DEINIT;
    }
    buffer &= ~std::uint8_t(0x01 << PWR1_DEVICE_RESET_BIT);
    buffer |= std::uint8_t((reset & 0x1) << PWR1_DEVICE_RESET_BIT);
    if (auto err{HAL_I2C_Mem_Write(i2c_,
                                   addres_,
                                   RA_PWR_MGMT_1,
                                   sizeof(RA_PWR_MGMT_1),
                                   &buffer,
                                   sizeof(buffer),
                                   i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::DEINIT;
    }
    return Error::OK;
}

Error MPU6050::set_sleep_enabled(const std::uint8_t enable) const noexcept
{
    std::uint8_t buffer;
    if (auto err{HAL_I2C_Mem_Read(i2c_,
                                  addres_,
                                  RA_PWR_MGMT_1,
                                  sizeof(RA_PWR_MGMT_1),
                                  &buffer,
                                  sizeof(buffer),
                                  i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::INIT;
    }
    buffer &= ~std::uint8_t(0x01 << PWR1_SLEEP_BIT);
    buffer |= std::uint8_t((enable & 0x1) << PWR1_SLEEP_BIT);
    if (auto err{HAL_I2C_Mem_Write(i2c_,
                                   addres_,
                                   RA_PWR_MGMT_1,
                                   sizeof(RA_PWR_MGMT_1),
                                   &buffer,
                                   sizeof(buffer),
                                   i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::INIT;
    }
    return Error::OK;
}

Error MPU6050::set_cycle_enabled(const std::uint8_t enable) const noexcept
{
    std::uint8_t buffer;
    if (auto err{HAL_I2C_Mem_Read(i2c_,
                                  addres_,
                                  RA_PWR_MGMT_1,
                                  sizeof(RA_PWR_MGMT_1),
                                  &buffer,
                                  sizeof(buffer),
                                  i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::INIT;
    }
    buffer &= ~std::uint8_t(0x01 << PWR1_CYCLE_BIT);
    buffer |= std::uint8_t((enable & 0x1) << PWR1_CYCLE_BIT);
    if (auto err{HAL_I2C_Mem_Write(i2c_,
                                   addres_,
                                   RA_PWR_MGMT_1,
                                   sizeof(RA_PWR_MGMT_1),
                                   &buffer,
                                   sizeof(buffer),
                                   i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::INIT;
    }
    return Error::OK;
}

Error MPU6050::set_temperature_sensor_disabled(const std::uint8_t disable) const noexcept
{
    std::uint8_t buffer;
    if (auto err{HAL_I2C_Mem_Read(i2c_,
                                  addres_,
                                  RA_PWR_MGMT_1,
                                  sizeof(RA_PWR_MGMT_1),
                                  &buffer,
                                  sizeof(buffer),
                                  i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::DEINIT;
    }
    buffer &= ~std::uint8_t(0x01 << PWR1_TEMP_DIS_BIT);
    buffer |= std::uint8_t((disable & 0x1) << PWR1_TEMP_DIS_BIT);
    if (auto err{HAL_I2C_Mem_Write(i2c_,
                                   addres_,
                                   RA_PWR_MGMT_1,
                                   sizeof(RA_PWR_MGMT_1),
                                   &buffer,
                                   sizeof(buffer),
                                   i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::DEINIT;
    }
    return Error::OK;
}

Error MPU6050::set_clock_source(const std::uint8_t source) const noexcept
{
    std::uint8_t buffer;
    if (auto err{HAL_I2C_Mem_Read(i2c_,
                                  addres_,
                                  RA_PWR_MGMT_1,
                                  sizeof(RA_PWR_MGMT_1),
                                  &buffer,
                                  sizeof(buffer),
                                  i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::INIT;
    }
    buffer &= 0xF8;
    buffer |= std::uint8_t(source & 0x7);
    if (auto err{HAL_I2C_Mem_Write(i2c_,
                                   addres_,
                                   RA_PWR_MGMT_1,
                                   sizeof(RA_PWR_MGMT_1),
                                   &buffer,
                                   sizeof(buffer),
                                   i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::INIT;
    }
    return Error::OK;
}

//
//	PWR_MGMT_2
//
Error MPU6050::set_low_power_wake_up_frequency(const std::uint8_t frequency) const noexcept
{
    std::uint8_t buffer;
    if (auto err{HAL_I2C_Mem_Read(i2c_,
                                  addres_,
                                  RA_PWR_MGMT_2,
                                  sizeof(RA_PWR_MGMT_2),
                                  &buffer,
                                  sizeof(buffer),
                                  i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::INIT;
    }
    buffer &= 0x3F;
    buffer |= std::uint8_t(frequency & 0x3) << PWR2_LP_WAKE_CTRL_BIT;
    if (auto err{HAL_I2C_Mem_Write(i2c_,
                                   addres_,
                                   RA_PWR_MGMT_2,
                                   sizeof(RA_PWR_MGMT_2),
                                   &buffer,
                                   sizeof(buffer),
                                   i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::INIT;
    }
    return Error::OK;
}

Error MPU6050::accelerometer_axis_standby(const std::uint8_t x_accel_standby,
                                          const std::uint8_t y_accel_standby,
                                          const std::uint8_t z_accel_standby) const noexcept
{
    std::uint8_t buffer;
    if (auto err{HAL_I2C_Mem_Read(i2c_,
                                  addres_,
                                  RA_PWR_MGMT_2,
                                  sizeof(RA_PWR_MGMT_2),
                                  &buffer,
                                  sizeof(buffer),
                                  i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::ACCEL;
    }
    buffer &= 0xC7;
    buffer |= std::uint8_t((x_accel_standby & 0x1) << PWR2_STBY_XA_BIT) |
              ((y_accel_standby & 0x1) << PWR2_STBY_YA_BIT) | ((z_accel_standby & 0x1) << PWR2_STBY_ZA_BIT);
    if (auto err{HAL_I2C_Mem_Write(i2c_,
                                   addres_,
                                   RA_PWR_MGMT_2,
                                   sizeof(RA_PWR_MGMT_2),
                                   &buffer,
                                   sizeof(buffer),
                                   i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::ACCEL;
    }
    return Error::OK;
}

Error MPU6050::gyroscope_axis_standby(const std::uint8_t x_gyro_standby,
                                      const std::uint8_t y_gyro_standby,
                                      const std::uint8_t z_gyro_standby) const noexcept
{
    std::uint8_t buffer;
    if (auto err{HAL_I2C_Mem_Read(i2c_,
                                  addres_,
                                  RA_PWR_MGMT_2,
                                  sizeof(RA_PWR_MGMT_2),
                                  &buffer,
                                  sizeof(buffer),
                                  i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::GYRO;
    }
    buffer &= 0xF8;
    buffer |= std::uint8_t((x_gyro_standby & 0x1) << PWR2_STBY_XG_BIT) | ((y_gyro_standby & 0x1) << PWR2_STBY_YG_BIT) |
              ((z_gyro_standby & 0x1) << PWR2_STBY_ZG_BIT);
    if (auto err{HAL_I2C_Mem_Write(i2c_,
                                   addres_,
                                   RA_PWR_MGMT_2,
                                   sizeof(RA_PWR_MGMT_2),
                                   &buffer,
                                   sizeof(buffer),
                                   i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::GYRO;
    }
    return Error::OK;
}

//
//	Measurement scale configuration
//
Error MPU6050::set_full_scale_gyro_range(const std::uint8_t range) const noexcept
{
    std::uint8_t buffer;
    if (auto err{HAL_I2C_Mem_Read(i2c_,
                                  addres_,
                                  RA_GYRO_CONFIG,
                                  sizeof(RA_GYRO_CONFIG),
                                  &buffer,
                                  sizeof(buffer),
                                  i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::GYRO;
    }
    buffer &= 0xE7;
    buffer |= std::uint8_t((range & 0x7) << 3);
    if (auto err{HAL_I2C_Mem_Write(i2c_,
                                   addres_,
                                   RA_GYRO_CONFIG,
                                   sizeof(RA_GYRO_CONFIG),
                                   &buffer,
                                   sizeof(buffer),
                                   i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::GYRO;
    }
    return Error::OK;
}

Error MPU6050::set_full_scale_accel_range(const std::uint8_t range) const noexcept
{
    std::uint8_t buffer;
    if (auto err{HAL_I2C_Mem_Read(i2c_,
                                  addres_,
                                  RA_ACCEL_CONFIG,
                                  sizeof(RA_ACCEL_CONFIG),
                                  &buffer,
                                  sizeof(buffer),
                                  i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::ACCEL;
    }
    buffer &= 0xE7;
    buffer |= std::uint8_t((range & 0x7) << 3);
    if (auto err{HAL_I2C_Mem_Write(i2c_,
                                   addres_,
                                   RA_ACCEL_CONFIG,
                                   sizeof(RA_ACCEL_CONFIG),
                                   &buffer,
                                   sizeof(buffer),
                                   i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::ACCEL;
    }
    return Error::OK;
}

//
// Reading data
//
ExpectedTempRaw MPU6050::get_temperature_raw() const noexcept
{
    std::uint8_t buffer[2];
    if (auto err{
            HAL_I2C_Mem_Read(i2c_, addres_, RA_TEMP_OUT_H, sizeof(RA_TEMP_OUT_H), buffer, sizeof(buffer), i2c_TIMEOUT)};
        err != HAL_OK) {
        return Unexpected{Error::TEMP};
    }
    return ExpectedTempRaw{((static_cast<TempRaw>(buffer[0])) << 8) | buffer[1]};
}

ExpectedTempScaled MPU6050::get_temperature_celsius() const noexcept
{
    if (auto temp_raw{get_temperature_raw()}; !temp_raw.has_value()) {
        return Unexpected{Error::TEMP};
    } else {
        return ExpectedTempScaled{static_cast<TempScaled>((temp_raw.value())) / 340 + 36.53};
    }
}

ExpectedRaw MPU6050::get_acceleration_x_raw() const noexcept
{
    std::uint8_t buffer[2];
    if (auto err{HAL_I2C_Mem_Read(i2c_,
                                  addres_,
                                  RA_ACCEL_XOUT_H,
                                  sizeof(RA_ACCEL_XOUT_H),
                                  buffer,
                                  sizeof(buffer),
                                  i2c_TIMEOUT)};
        err != HAL_OK) {
        return Unexpected{Error::ACCEL};
    }
    return ExpectedRaw{((static_cast<Raw>(buffer[0])) << 8) | buffer[1]};
}

ExpectedRaw MPU6050::get_acceleration_y_raw() const noexcept
{
    std::uint8_t buffer[2];
    if (auto err{HAL_I2C_Mem_Read(i2c_,
                                  addres_,
                                  RA_ACCEL_YOUT_H,
                                  sizeof(RA_ACCEL_YOUT_H),
                                  buffer,
                                  sizeof(buffer),
                                  i2c_TIMEOUT)};
        err != HAL_OK) {
        return Unexpected{Error::ACCEL};
    }
    return ExpectedRaw{((static_cast<Raw>(buffer[0])) << 8) | buffer[1]};
}

ExpectedRaw MPU6050::get_acceleration_z_raw() const noexcept
{
    std::uint8_t buffer[2];
    if (auto err{HAL_I2C_Mem_Read(i2c_,
                                  addres_,
                                  RA_ACCEL_ZOUT_H,
                                  sizeof(RA_ACCEL_ZOUT_H),
                                  buffer,
                                  sizeof(buffer),
                                  i2c_TIMEOUT)};
        err != HAL_OK) {
        return Unexpected{Error::INIT};
    }
    return ExpectedRaw{((static_cast<Raw>(buffer[0])) << 8) | buffer[1]};
}

ExpectedAccelRaw MPU6050::get_accelerometer_raw() const noexcept
{
    std::uint8_t buffer[6];
    if (auto err{HAL_I2C_Mem_Read(i2c_,
                                  addres_,
                                  RA_ACCEL_XOUT_H,
                                  sizeof(RA_ACCEL_XOUT_H),
                                  buffer,
                                  sizeof(buffer),
                                  i2c_TIMEOUT)};
        err != HAL_OK) {
        return Unexpected{Error::ACCEL};
    }
    return ExpectedAccelRaw{std::in_place,
                            ((static_cast<Raw>(buffer[0])) << 8) | buffer[1],
                            ((static_cast<Raw>(buffer[2])) << 8) | buffer[3],
                            ((static_cast<Raw>(buffer[4])) << 8) | buffer[5]};
}

ExpectedAccelScaled MPU6050::get_accelerometer_scaled() const noexcept
{
    const auto accel_scale{accel_range_to_scale(accel_range_)};
    if (auto accel_raw{get_accelerometer_raw()}; !accel_raw.has_value()) {
        return Unexpected{accel_raw.error()};
    } else {
        return ExpectedAccelScaled{std::in_place,
                                   static_cast<Scaled>(accel_raw.value().x) * accel_scale,
                                   static_cast<Scaled>(accel_raw.value().y) * accel_scale,
                                   static_cast<Scaled>(accel_raw.value().z) * accel_scale};
    }
}

ExpectedRaw MPU6050::get_rotation_x_raw() const noexcept
{
    std::uint8_t buffer[2];
    if (auto err{HAL_I2C_Mem_Read(i2c_,
                                  addres_,
                                  RA_GYRO_XOUT_H,
                                  sizeof(RA_GYRO_XOUT_H),
                                  buffer,
                                  sizeof(buffer),
                                  i2c_TIMEOUT)};
        err != HAL_OK) {
        return Unexpected{Error::GYRO};
    }
    return ExpectedRaw{((static_cast<Raw>(buffer[0])) << 8) | buffer[1]};
}

ExpectedRaw MPU6050::get_rotation_y_raw() const noexcept
{
    std::uint8_t buffer[2];
    if (auto err{HAL_I2C_Mem_Read(i2c_,
                                  addres_,
                                  RA_GYRO_YOUT_H,
                                  sizeof(RA_GYRO_YOUT_H),
                                  buffer,
                                  sizeof(buffer),
                                  i2c_TIMEOUT)};
        err != HAL_OK) {
        return Unexpected{Error::GYRO};
    }
    return ExpectedRaw{((static_cast<Raw>(buffer[0])) << 8) | buffer[1]};
}

ExpectedRaw MPU6050::get_rotation_z_raw() const noexcept
{
    std::uint8_t buffer[2];
    if (auto err{HAL_I2C_Mem_Read(i2c_,
                                  addres_,
                                  RA_GYRO_ZOUT_H,
                                  sizeof(RA_GYRO_ZOUT_H),
                                  buffer,
                                  sizeof(buffer),
                                  i2c_TIMEOUT)};
        err != HAL_OK) {
        return Unexpected{Error::GYRO};
    }
    return ExpectedRaw{((static_cast<Raw>(buffer[0])) << 8) | buffer[1]};
}

ExpectedGyroRaw MPU6050::get_gyroscope_raw() const noexcept
{
    std::uint8_t buffer[6];
    if (auto err{HAL_I2C_Mem_Read(i2c_,
                                  addres_,
                                  RA_GYRO_XOUT_H,
                                  sizeof(RA_GYRO_XOUT_H),
                                  buffer,
                                  sizeof(buffer),
                                  i2c_TIMEOUT)};
        err != HAL_OK) {
        return Unexpected{Error::GYRO};
    }
    return ExpectedGyroRaw{std::in_place,
                           ((static_cast<Raw>(buffer[0])) << 8) | buffer[1],
                           ((static_cast<Raw>(buffer[2])) << 8) | buffer[3],
                           ((static_cast<Raw>(buffer[4])) << 8) | buffer[5]};
}

ExpectedGyroScaled MPU6050::get_gyroscope_scaled() const noexcept
{
    const auto gyro_scale{gyro_range_to_scale(gyro_range_)};
    if (auto gyro_raw{get_gyroscope_raw()}; !gyro_raw.has_value()) {
        return Unexpected{gyro_raw.error()};
    } else {
        return ExpectedGyroScaled{std::in_place,
                                  static_cast<Scaled>(gyro_raw.value().x) * gyro_scale,
                                  static_cast<Scaled>(gyro_raw.value().y) * gyro_scale,
                                  static_cast<Scaled>(gyro_raw.value().z) * gyro_scale};
    }
}

ExpectedRPY MPU6050::get_roll_pitch_yaw() const noexcept
{
    if (auto accel_scaled{get_accelerometer_scaled()}; !accel_scaled.has_value()) {
        return Unexpected{accel_scaled.error()};
    } else {
        return ExpectedRPY{std::in_place,
                           std::atan2(accel_scaled.value().y, accel_scaled.value().z) * 180.0 / M_PI,
                           -(std::atan2(accel_scaled.value().x,
                                        std::sqrt(accel_scaled.value().y * accel_scaled.value().y +
                                                  accel_scaled.value().z * accel_scaled.value().z)) *
                             180.0) /
                               M_PI};
    }
}

Error MPU6050::set_interrupt() const noexcept
{
    if (auto err{set_interrupt_mode(INTMODE_ACTIVEHIGH)}; err != Error::OK) {
        return print_and_return(err);
    }
    if (auto err{set_interrupt_drive(INTDRV_PUSHPULL)}; err != Error::OK) {
        return print_and_return(err);
    }
    if (auto err{set_interrupt_latch(INTLATCH_WAITCLEAR)}; err != Error::OK) {
        return print_and_return(err);
    }
    if (auto err{set_interrupt_latch_clear(INTCLEAR_STATUSREAD)}; err != Error::OK) {
        return print_and_return(err);
    }

    // Disable all interrupts
    if (auto err{set_int_enable_register(0)}; err != Error::OK) {
        return print_and_return(err);
    }

    // Enable Motion interrputs
    if (auto err{set_dhpf_mode(DHPF_5)}; err != Error::OK) {
        return print_and_return(err);
    }

    if (auto err{set_int_motion_enabled(1)}; err != Error::OK) {
        return print_and_return(err);
    }
    if (auto err{set_int_zero_motion_enabled(1)}; err != Error::OK) {
        return print_and_return(err);
    }
    if (auto err{set_int_free_fall_enabled(1)}; err != Error::OK) {
        return print_and_return(err);
    }

    if (auto err{set_free_fall_detection_duration(2)}; err != Error::OK) {
        return print_and_return(err);
    }
    if (auto err{set_free_fall_detection_threshold(5)}; err != Error::OK) {
        return print_and_return(err);
    }

    if (auto err{set_motion_detection_duration(5)}; err != Error::OK) {
        return print_and_return(err);
    }
    if (auto err{set_motion_detection_threshold(2)}; err != Error::OK) {
        return print_and_return(err);
    }

    if (auto err{set_zero_motion_detection_duration(2)}; err != Error::OK) {
        return print_and_return(err);
    }
    if (auto err{set_zero_motion_detection_threshold(4)}; err != Error::OK) {
        return print_and_return(err);
    }
    return Error::OK;
}

//
//	set_ting Raw pin
//
Error MPU6050::set_interrupt_mode(const std::uint8_t mode) const noexcept
{
    std::uint8_t buffer;
    if (auto err{HAL_I2C_Mem_Read(i2c_,
                                  addres_,
                                  RA_INT_PIN_CFG,
                                  sizeof(RA_INT_PIN_CFG),
                                  &buffer,
                                  sizeof(buffer),
                                  i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::INIT;
    }
    buffer &= ~std::uint8_t(0x01 << INTCFG_INT_LEVEL_BIT);
    buffer |= std::uint8_t((mode & 0x1) << INTCFG_INT_LEVEL_BIT);
    if (auto err{HAL_I2C_Mem_Write(i2c_,
                                   addres_,
                                   RA_INT_PIN_CFG,
                                   sizeof(RA_INT_PIN_CFG),
                                   &buffer,
                                   sizeof(buffer),
                                   i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::INIT;
    }
    return Error::OK;
}

Error MPU6050::set_interrupt_drive(const std::uint8_t drive) const noexcept
{
    std::uint8_t buffer;
    if (auto err{HAL_I2C_Mem_Read(i2c_,
                                  addres_,
                                  RA_INT_PIN_CFG,
                                  sizeof(RA_INT_PIN_CFG),
                                  &buffer,
                                  sizeof(buffer),
                                  i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::INIT;
    }
    buffer &= ~std::uint8_t(0x01 << INTCFG_INT_OPEN_BIT);
    buffer |= std::uint8_t((drive & 0x1) << INTCFG_INT_OPEN_BIT);
    if (auto err{HAL_I2C_Mem_Write(i2c_,
                                   addres_,
                                   RA_INT_PIN_CFG,
                                   sizeof(RA_INT_PIN_CFG),
                                   &buffer,
                                   sizeof(buffer),
                                   i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::INIT;
    }
    return Error::OK;
}

Error MPU6050::set_interrupt_latch(const std::uint8_t latch) const noexcept
{
    std::uint8_t buffer;
    if (auto err{HAL_I2C_Mem_Read(i2c_,
                                  addres_,
                                  RA_INT_PIN_CFG,
                                  sizeof(RA_INT_PIN_CFG),
                                  &buffer,
                                  sizeof(buffer),
                                  i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::INIT;
    }
    buffer &= ~std::uint8_t(0x01 << INTCFG_INT_RD_CLEAR_BIT);
    buffer |= std::uint8_t((latch & 0x1) << INTCFG_INT_RD_CLEAR_BIT);
    if (auto err{HAL_I2C_Mem_Write(i2c_,
                                   addres_,
                                   RA_INT_PIN_CFG,
                                   sizeof(RA_INT_PIN_CFG),
                                   &buffer,
                                   sizeof(buffer),
                                   i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::INIT;
    }
    return Error::OK;
}

Error MPU6050::set_interrupt_latch_clear(const std::uint8_t clear) const noexcept
{
    std::uint8_t buffer;
    if (auto err{HAL_I2C_Mem_Read(i2c_,
                                  addres_,
                                  RA_INT_PIN_CFG,
                                  sizeof(RA_INT_PIN_CFG),
                                  &buffer,
                                  sizeof(buffer),
                                  i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::INIT;
    }
    buffer &= ~std::uint8_t(0x01 << INTCFG_LATCH_INT_EN_BIT);
    buffer |= std::uint8_t((clear & 0x1) << INTCFG_LATCH_INT_EN_BIT);
    if (auto err{HAL_I2C_Mem_Write(i2c_,
                                   addres_,
                                   RA_INT_PIN_CFG,
                                   sizeof(RA_INT_PIN_CFG),
                                   &buffer,
                                   sizeof(buffer),
                                   i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::INIT;
    }
    return Error::OK;
}

Error MPU6050::set_int_enable_register(std::uint8_t value) const noexcept
{
    if (auto err{HAL_I2C_Mem_Write(i2c_, addres_, RA_INT_ENABLE, sizeof(RA_INT_ENABLE), &value, 1, i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::INIT;
    }
    return Error::OK;
}

Error MPU6050::set_int_data_ready_enabled(const std::uint8_t enable) const noexcept
{
    std::uint8_t buffer;
    if (auto err{HAL_I2C_Mem_Read(i2c_,
                                  addres_,
                                  RA_INT_ENABLE,
                                  sizeof(RA_INT_ENABLE),
                                  &buffer,
                                  sizeof(buffer),
                                  i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::INIT;
    }
    buffer &= ~std::uint8_t(0x01 << INTERRUPT_DATA_RDY_BIT);
    buffer |= std::uint8_t((enable & 0x1) << INTERRUPT_DATA_RDY_BIT);
    if (auto err{HAL_I2C_Mem_Write(i2c_,
                                   addres_,
                                   RA_INT_ENABLE,
                                   sizeof(RA_INT_ENABLE),
                                   &buffer,
                                   sizeof(buffer),
                                   i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::INIT;
    }
    return Error::OK;
}

ExpectedAddres MPU6050::get_int_status_register() const noexcept
{
    std::uint8_t buffer;
    if (auto err{HAL_I2C_Mem_Read(i2c_,
                                  addres_,
                                  RA_INT_STATUS,
                                  sizeof(RA_INT_STATUS),
                                  &buffer,
                                  sizeof(buffer),
                                  i2c_TIMEOUT)};
        err != HAL_OK) {
        return Unexpected{Error::INIT};
    }
    return ExpectedAddres{buffer};
}

ExpectedAddres MPU6050::get_device_id() const noexcept
{
    std::uint8_t buffer;
    if (auto err{
            HAL_I2C_Mem_Read(i2c_, addres_, RA_WHO_AM_I, sizeof(RA_WHO_AM_I), &buffer, sizeof(buffer), i2c_TIMEOUT)};
        err != HAL_OK) {
        return Unexpected{Error::INIT};
    }
    return ExpectedAddres{buffer << std::uint8_t{1}};
}

//
//	motion_ functions - not included in documentation/register map
//
Error MPU6050::set_dhpf_mode(const std::uint8_t dhpf) const noexcept
{
    std::uint8_t buffer;
    if (auto err{HAL_I2C_Mem_Read(i2c_,
                                  addres_,
                                  RA_ACCEL_CONFIG,
                                  sizeof(RA_ACCEL_CONFIG),
                                  &buffer,
                                  sizeof(buffer),
                                  i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::INIT;
    }
    buffer &= ~std::uint8_t(0x07);
    buffer |= std::uint8_t(dhpf & 0x7);
    if (auto err{HAL_I2C_Mem_Write(i2c_,
                                   addres_,
                                   RA_ACCEL_CONFIG,
                                   sizeof(RA_ACCEL_CONFIG),
                                   &buffer,
                                   sizeof(buffer),
                                   i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::INIT;
    }
    return Error::OK;
}

ExpectedAddres MPU6050::get_motion_status_register() const noexcept
{
    std::uint8_t buffer;
    if (auto err{HAL_I2C_Mem_Read(i2c_,
                                  addres_,
                                  RA_MOT_DETECT_STATUS,
                                  sizeof(RA_MOT_DETECT_STATUS),
                                  &buffer,
                                  sizeof(buffer),
                                  i2c_TIMEOUT)};
        err != HAL_OK) {
        return Unexpected{Error::INIT};
    }
    return ExpectedAddres{buffer};
}

Error MPU6050::set_int_zero_motion_enabled(const std::uint8_t enable) const noexcept
{
    std::uint8_t buffer;
    if (auto err{HAL_I2C_Mem_Read(i2c_,
                                  addres_,
                                  RA_INT_ENABLE,
                                  sizeof(RA_INT_ENABLE),
                                  &buffer,
                                  sizeof(buffer),
                                  i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::INIT;
    }
    buffer &= ~std::uint8_t(0x01 << INTERRUPT_ZMOT_BIT);
    buffer |= std::uint8_t((enable & 0x1) << INTERRUPT_ZMOT_BIT);
    if (auto err{HAL_I2C_Mem_Write(i2c_,
                                   addres_,
                                   RA_INT_ENABLE,
                                   sizeof(RA_INT_ENABLE),
                                   &buffer,
                                   sizeof(buffer),
                                   i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::INIT;
    }
    return Error::OK;
}

Error MPU6050::set_int_motion_enabled(const std::uint8_t enable) const noexcept
{
    std::uint8_t buffer;
    if (auto err{HAL_I2C_Mem_Read(i2c_,
                                  addres_,
                                  RA_INT_ENABLE,
                                  sizeof(RA_INT_ENABLE),
                                  &buffer,
                                  sizeof(buffer),
                                  i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::INIT;
    }
    buffer &= ~std::uint8_t(0x01 << INTERRUPT_MOT_BIT);
    buffer |= std::uint8_t((enable & 0x1) << INTERRUPT_MOT_BIT);
    if (auto err{HAL_I2C_Mem_Write(i2c_,
                                   addres_,
                                   RA_INT_ENABLE,
                                   sizeof(RA_INT_ENABLE),
                                   &buffer,
                                   sizeof(buffer),
                                   i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::INIT;
    }
    return Error::OK;
}

Error MPU6050::set_int_free_fall_enabled(const std::uint8_t enable) const noexcept
{
    std::uint8_t buffer;
    if (auto err{HAL_I2C_Mem_Read(i2c_,
                                  addres_,
                                  RA_INT_ENABLE,
                                  sizeof(RA_INT_ENABLE),
                                  &buffer,
                                  sizeof(buffer),
                                  i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::INIT;
    }
    buffer &= ~std::uint8_t(0x01 << INTERRUPT_FF_BIT);
    buffer |= std::uint8_t((enable & 0x1) << INTERRUPT_FF_BIT);
    if (auto err{HAL_I2C_Mem_Write(i2c_,
                                   addres_,
                                   RA_INT_ENABLE,
                                   sizeof(RA_INT_ENABLE),
                                   &buffer,
                                   sizeof(buffer),
                                   i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::INIT;
    }
    return Error::OK;
}

Error MPU6050::set_motion_detection_threshold(std::uint8_t threshold) const noexcept
{
    if (auto err{HAL_I2C_Mem_Write(i2c_,
                                   addres_,
                                   RA_MOT_THR,
                                   sizeof(RA_MOT_THR),
                                   &threshold,
                                   sizeof(threshold),
                                   i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::INIT;
    }
    return Error::OK;
}

Error MPU6050::set_motion_detection_duration(std::uint8_t duration) const noexcept
{
    if (auto err{
            HAL_I2C_Mem_Write(i2c_, addres_, RA_MOT_DUR, sizeof(RA_MOT_DUR), &duration, sizeof(duration), i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::INIT;
    }
    return Error::OK;
}

Error MPU6050::set_zero_motion_detection_threshold(std::uint8_t threshold) const noexcept
{
    if (auto err{HAL_I2C_Mem_Write(i2c_,
                                   addres_,
                                   RA_ZRMOT_THR,
                                   sizeof(RA_ZRMOT_THR),
                                   &threshold,
                                   sizeof(threshold),
                                   i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::INIT;
    }
    return Error::OK;
}

Error MPU6050::set_zero_motion_detection_duration(std::uint8_t duration) const noexcept
{
    if (auto err{HAL_I2C_Mem_Write(i2c_,
                                   addres_,
                                   RA_ZRMOT_DUR,
                                   sizeof(RA_ZRMOT_DUR),
                                   &duration,
                                   sizeof(duration),
                                   i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::INIT;
    }
    return Error::OK;
}

Error MPU6050::set_free_fall_detection_threshold(std::uint8_t threshold) const noexcept
{
    if (auto err{
            HAL_I2C_Mem_Write(i2c_, addres_, RA_FF_THR, sizeof(RA_FF_THR), &threshold, sizeof(threshold), i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::INIT;
    }
    return Error::OK;
}

Error MPU6050::set_free_fall_detection_duration(std::uint8_t duration) const noexcept
{
    if (auto err{
            HAL_I2C_Mem_Write(i2c_, addres_, RA_FF_DUR, sizeof(RA_FF_DUR), &duration, sizeof(duration), i2c_TIMEOUT)};
        err != HAL_OK) {
        return Error::INIT;
    }
    return Error::OK;
}

Error MPU6050::print_and_return(const Error error) const noexcept
{
    sprintf(uart_buffer_, error_to_string(error));
    uart_send_string(uart_, uart_buffer_);
    return error;
}
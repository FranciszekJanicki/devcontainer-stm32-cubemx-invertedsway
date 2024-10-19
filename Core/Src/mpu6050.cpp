#include "mpu6050.hpp"
#include "mpu6050_config.hpp"
#include "stm32l4xx_hal.h"
#include <array>
#include <cmath>
#include <cstdint>

namespace MPU6050 {

static inline std::uint32_t i2c_TIMEOUT{10};

static constexpr auto GyroRangeToScale(const std::uint8_t GyroRange) noexcept {
  switch (GyroRange) {
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

static constexpr auto
AccelRangeToScale(const std::uint8_t AccelRange) noexcept {
  switch (AccelRange) {
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

//
// CONFIG
//
void SetDlpf(I2C_HandleTypeDef &i2c, const std::uint8_t Value) noexcept {
  std::uint8_t buffer;
  HAL_I2C_Mem_Read(&i2c, ADDRESS, RA_CONFIG, sizeof(RA_CONFIG), &buffer,
                   sizeof(buffer), i2c_TIMEOUT);
  buffer &= 0xF8;
  buffer |= (Value & 0x7);
  HAL_I2C_Mem_Write(&i2c, ADDRESS, RA_CONFIG, sizeof(RA_CONFIG), &buffer,
                    sizeof(buffer), i2c_TIMEOUT);
}

//
// PWR_MGMT_1
//
void DeviceReset(I2C_HandleTypeDef &i2c, const std::uint8_t Reset) noexcept {
  std::uint8_t buffer;
  HAL_I2C_Mem_Read(&i2c, ADDRESS, RA_PWR_MGMT_1, sizeof(RA_PWR_MGMT_1), &buffer,
                   sizeof(buffer), i2c_TIMEOUT);
  buffer &= ~(0x01 << PWR1_DEVICE_RESET_BIT);
  buffer |= ((Reset & 0x1) << PWR1_DEVICE_RESET_BIT);
  HAL_I2C_Mem_Write(&i2c, ADDRESS, RA_PWR_MGMT_1, sizeof(RA_PWR_MGMT_1),
                    &buffer, sizeof(buffer), i2c_TIMEOUT);
}

void SetSleepEnabled(I2C_HandleTypeDef &i2c,
                     const std::uint8_t Enable) noexcept {
  std::uint8_t buffer;
  HAL_I2C_Mem_Read(&i2c, ADDRESS, RA_PWR_MGMT_1, sizeof(RA_PWR_MGMT_1), &buffer,
                   sizeof(buffer), i2c_TIMEOUT);
  buffer &= ~(0x01 << PWR1_SLEEP_BIT);
  buffer |= ((Enable & 0x1) << PWR1_SLEEP_BIT);
  HAL_I2C_Mem_Write(&i2c, ADDRESS, RA_PWR_MGMT_1, sizeof(RA_PWR_MGMT_1),
                    &buffer, sizeof(buffer), i2c_TIMEOUT);
}

void SetCycleEnabled(I2C_HandleTypeDef &i2c,
                     const std::uint8_t Enable) noexcept {
  std::uint8_t buffer;
  HAL_I2C_Mem_Read(&i2c, ADDRESS, RA_PWR_MGMT_1, sizeof(RA_PWR_MGMT_1), &buffer,
                   sizeof(buffer), i2c_TIMEOUT);
  buffer &= ~(0x01 << PWR1_CYCLE_BIT);
  buffer |= ((Enable & 0x1) << PWR1_CYCLE_BIT);
  HAL_I2C_Mem_Write(&i2c, ADDRESS, RA_PWR_MGMT_1, sizeof(RA_PWR_MGMT_1),
                    &buffer, sizeof(buffer), i2c_TIMEOUT);
}

void SetTemperatureSensorDisbled(I2C_HandleTypeDef &i2c,
                                 const std::uint8_t Disable) noexcept {
  std::uint8_t buffer;
  HAL_I2C_Mem_Read(&i2c, ADDRESS, RA_PWR_MGMT_1, sizeof(RA_PWR_MGMT_1), &buffer,
                   sizeof(buffer), i2c_TIMEOUT);
  buffer &= ~(0x01 << PWR1_TEMP_DIS_BIT);
  buffer |= ((Disable & 0x1) << PWR1_TEMP_DIS_BIT);
  HAL_I2C_Mem_Write(&i2c, ADDRESS, RA_PWR_MGMT_1, sizeof(RA_PWR_MGMT_1),
                    &buffer, sizeof(buffer), i2c_TIMEOUT);
}

void SetClockSource(I2C_HandleTypeDef &i2c,
                    const std::uint8_t Source) noexcept {
  std::uint8_t buffer;
  HAL_I2C_Mem_Read(&i2c, ADDRESS, RA_PWR_MGMT_1, sizeof(RA_PWR_MGMT_1), &buffer,
                   sizeof(buffer), i2c_TIMEOUT);
  buffer &= 0xF8;
  buffer |= (Source & 0x7);
  HAL_I2C_Mem_Write(&i2c, ADDRESS, RA_PWR_MGMT_1, sizeof(RA_PWR_MGMT_1),
                    &buffer, sizeof(buffer), i2c_TIMEOUT);
}

//
//	PWR_MGMT_2
//
void SetLowPowerWakeUpFrequency(I2C_HandleTypeDef &i2c,
                                const std::uint8_t Frequency) noexcept {
  std::uint8_t buffer;
  HAL_I2C_Mem_Read(&i2c, ADDRESS, RA_PWR_MGMT_2, sizeof(RA_PWR_MGMT_2), &buffer,
                   sizeof(buffer), i2c_TIMEOUT);
  buffer &= 0x3F;
  buffer |= (Frequency & 0x3) << PWR2_LP_WAKE_CTRL_BIT;
  HAL_I2C_Mem_Write(&i2c, ADDRESS, RA_PWR_MGMT_2, sizeof(RA_PWR_MGMT_2),
                    &buffer, sizeof(buffer), i2c_TIMEOUT);
}

void AccelerometerAxisStandby(I2C_HandleTypeDef &i2c,
                              const std::uint8_t XA_Stby,
                              const std::uint8_t YA_Stby,
                              const std::uint8_t ZA_Stby) noexcept {
  std::uint8_t buffer;
  HAL_I2C_Mem_Read(&i2c, ADDRESS, RA_PWR_MGMT_2, sizeof(RA_PWR_MGMT_2), &buffer,
                   sizeof(buffer), i2c_TIMEOUT);
  buffer &= 0xC7;
  buffer |= ((XA_Stby & 0x1) << PWR2_STBY_XA_BIT) |
            ((YA_Stby & 0x1) << PWR2_STBY_YA_BIT) |
            ((ZA_Stby & 0x1) << PWR2_STBY_ZA_BIT);
  HAL_I2C_Mem_Write(&i2c, ADDRESS, RA_PWR_MGMT_2, sizeof(RA_PWR_MGMT_2),
                    &buffer, sizeof(buffer), i2c_TIMEOUT);
}

void GyroscopeAxisStandby(I2C_HandleTypeDef &i2c, const std::uint8_t XG_Stby,
                          const std::uint8_t YG_Stby,
                          const std::uint8_t ZG_Stby) noexcept {
  std::uint8_t buffer;
  HAL_I2C_Mem_Read(&i2c, ADDRESS, RA_PWR_MGMT_2, sizeof(RA_PWR_MGMT_2), &buffer,
                   sizeof(buffer), i2c_TIMEOUT);
  buffer &= 0xF8;
  buffer |= ((XG_Stby & 0x1) << PWR2_STBY_XG_BIT) |
            ((YG_Stby & 0x1) << PWR2_STBY_YG_BIT) |
            ((ZG_Stby & 0x1) << PWR2_STBY_ZG_BIT);
  HAL_I2C_Mem_Write(&i2c, ADDRESS, RA_PWR_MGMT_2, sizeof(RA_PWR_MGMT_2),
                    &buffer, sizeof(buffer), i2c_TIMEOUT);
}

//
//	Measurement scale configuration
//
void SetFullScaleGyroRange(I2C_HandleTypeDef &i2c,
                           const std::uint8_t Range) noexcept {
  std::uint8_t buffer;
  HAL_I2C_Mem_Read(&i2c, ADDRESS, RA_GYRO_CONFIG, sizeof(RA_GYRO_CONFIG),
                   &buffer, sizeof(buffer), i2c_TIMEOUT);
  buffer &= 0xE7;
  buffer |= ((Range & 0x7) << 3);
  HAL_I2C_Mem_Write(&i2c, ADDRESS, RA_GYRO_CONFIG, sizeof(RA_GYRO_CONFIG),
                    &buffer, sizeof(buffer), i2c_TIMEOUT);
}

void SetFullScaleAccelRange(I2C_HandleTypeDef &i2c,
                            const std::uint8_t Range) noexcept {
  std::uint8_t buffer;
  HAL_I2C_Mem_Read(&i2c, ADDRESS, RA_ACCEL_CONFIG, sizeof(RA_ACCEL_CONFIG),
                   &buffer, sizeof(buffer), i2c_TIMEOUT);
  buffer &= 0xE7;
  buffer |= ((Range & 0x7) << 3);
  HAL_I2C_Mem_Write(&i2c, ADDRESS, RA_ACCEL_CONFIG, sizeof(RA_ACCEL_CONFIG),
                    &buffer, sizeof(buffer), i2c_TIMEOUT);
}

//
// Reading data
//
std::int32_t GetTemperatureRAW(I2C_HandleTypeDef &i2c) noexcept {
  std::uint8_t buffer[2];
  HAL_I2C_Mem_Read(&i2c, ADDRESS, RA_TEMP_OUT_H, sizeof(RA_TEMP_OUT_H), buffer,
                   sizeof(buffer), i2c_TIMEOUT);
  return ((static_cast<std::int32_t>(buffer[0])) << 8) | buffer[1];
}

std::double_t GetTemperatureCelsius(I2C_HandleTypeDef &i2c) noexcept {
  return static_cast<std::double_t>(GetTemperatureRAW(i2c)) / 340 + 36.53;
}

std::int32_t GetAccelerationXRAW(I2C_HandleTypeDef &i2c) noexcept {
  std::uint8_t buffer[2];
  HAL_I2C_Mem_Read(&i2c, ADDRESS, RA_ACCEL_XOUT_H, sizeof(RA_ACCEL_XOUT_H),
                   buffer, sizeof(buffer), i2c_TIMEOUT);
  return ((static_cast<std::int32_t>(buffer[0])) << 8) | buffer[1];
}

std::int32_t GetAccelerationYRAW(I2C_HandleTypeDef &i2c) noexcept {
  std::uint8_t buffer[2];
  HAL_I2C_Mem_Read(&i2c, ADDRESS, RA_ACCEL_YOUT_H, sizeof(RA_ACCEL_YOUT_H),
                   buffer, sizeof(buffer), i2c_TIMEOUT);
  return ((static_cast<std::int32_t>(buffer[0])) << 8) | buffer[1];
}

std::int32_t GetAccelerationZRAW(I2C_HandleTypeDef &i2c) noexcept {
  std::uint8_t buffer[2];
  HAL_I2C_Mem_Read(&i2c, ADDRESS, RA_ACCEL_ZOUT_H, sizeof(RA_ACCEL_ZOUT_H),
                   buffer, sizeof(buffer), i2c_TIMEOUT);
  return ((static_cast<std::int32_t>(buffer[0])) << 8) | buffer[1];
}

accel_raw_result GetAccelerometerRAW(I2C_HandleTypeDef &i2c) noexcept {
  std::uint8_t buffer[6];
  HAL_I2C_Mem_Read(&i2c, ADDRESS, RA_ACCEL_XOUT_H, sizeof(RA_ACCEL_XOUT_H),
                   buffer, sizeof(buffer), i2c_TIMEOUT);

  return accel_raw_result{
      ((static_cast<std::int32_t>(buffer[0])) << 8) | buffer[1],
      ((static_cast<std::int32_t>(buffer[2])) << 8) | buffer[3],
      ((static_cast<std::int32_t>(buffer[4])) << 8) | buffer[5]};
}

accel_result GetAccelerometerScaled(I2C_HandleTypeDef &i2c,
                                    const std::uint8_t AccelRange) noexcept {
  const auto accel_scale{AccelRangeToScale(AccelRange)};
  const auto accel_raw_result{GetAccelerometerRAW(i2c)};
  return accel_result{static_cast<double>(accel_raw_result.x) * accel_scale,
                      static_cast<double>(accel_raw_result.y) * accel_scale,
                      static_cast<double>(accel_raw_result.z) * accel_scale};
}

std::int32_t GetRotationXRAW(I2C_HandleTypeDef &i2c) noexcept {
  std::uint8_t buffer[2];
  HAL_I2C_Mem_Read(&i2c, ADDRESS, RA_GYRO_XOUT_H, sizeof(RA_GYRO_XOUT_H),
                   buffer, sizeof(buffer), i2c_TIMEOUT);
  return ((static_cast<std::int32_t>(buffer[0])) << 8) | buffer[1];
}

std::int32_t GetRotationYRAW(I2C_HandleTypeDef &i2c) noexcept {
  std::uint8_t buffer[2];
  HAL_I2C_Mem_Read(&i2c, ADDRESS, RA_GYRO_YOUT_H, sizeof(RA_GYRO_YOUT_H),
                   buffer, sizeof(buffer), i2c_TIMEOUT);
  return ((static_cast<std::int32_t>(buffer[0])) << 8) | buffer[1];
}

std::int32_t GetRotationZRAW(I2C_HandleTypeDef &i2c) noexcept {
  std::uint8_t buffer[2];
  HAL_I2C_Mem_Read(&i2c, ADDRESS, RA_GYRO_ZOUT_H, sizeof(RA_GYRO_ZOUT_H),
                   buffer, sizeof(buffer), i2c_TIMEOUT);
  return ((static_cast<std::int32_t>(buffer[0])) << 8) | buffer[1];
}

gyro_raw_result GetGyroscopeRAW(I2C_HandleTypeDef &i2c) noexcept {
  std::uint8_t buffer[6];
  HAL_I2C_Mem_Read(&i2c, ADDRESS, RA_GYRO_XOUT_H, sizeof(RA_GYRO_XOUT_H),
                   buffer, sizeof(buffer), i2c_TIMEOUT);

  return gyro_raw_result{
      ((static_cast<std::int32_t>(buffer[0])) << 8) | buffer[1],
      ((static_cast<std::int32_t>(buffer[2])) << 8) | buffer[3],
      ((static_cast<std::int32_t>(buffer[4])) << 8) | buffer[5]};
}

gyro_result GetGyroscopeScaled(I2C_HandleTypeDef &i2c,
                               const std::uint8_t GyroRange) noexcept {
  const auto gyro_scale{GyroRangeToScale(GyroRange)};
  const auto gyro_raw_result{GetGyroscopeRAW(i2c)};
  return gyro_result{static_cast<double>(gyro_raw_result.x) * gyro_scale,
                     static_cast<double>(gyro_raw_result.y) * gyro_scale,
                     static_cast<double>(gyro_raw_result.z) * gyro_scale};
}

roll_pitch_yaw GetRollPitchYaw(I2C_HandleTypeDef &i2c,
                               const std::uint8_t AccelRange) noexcept {
  const auto accel_result{GetAccelerometerScaled(i2c, AccelRange)};

  return roll_pitch_yaw{
      std::atan2(accel_result.y, accel_result.z) * 180.0 / M_PI,
      -(std::atan2(accel_result.x, std::sqrt(accel_result.y * accel_result.y +
                                             accel_result.z * accel_result.z)) *
        180.0) /
          M_PI,
      {}};
}

//
//	Setting std::int32_t pin
//
void SetInterruptMode(I2C_HandleTypeDef &i2c,
                      const std::uint8_t Mode) noexcept {
  std::uint8_t buffer;
  HAL_I2C_Mem_Read(&i2c, ADDRESS, RA_INT_PIN_CFG, sizeof(RA_INT_PIN_CFG),
                   &buffer, sizeof(buffer), i2c_TIMEOUT);
  buffer &= ~(0x01 << INTCFG_INT_LEVEL_BIT);
  buffer |= ((Mode & 0x1) << INTCFG_INT_LEVEL_BIT);
  HAL_I2C_Mem_Write(&i2c, ADDRESS, RA_INT_PIN_CFG, sizeof(RA_INT_PIN_CFG),
                    &buffer, sizeof(buffer), i2c_TIMEOUT);
}

void SetInterruptDrive(I2C_HandleTypeDef &i2c,
                       const std::uint8_t Drive) noexcept {
  std::uint8_t buffer;
  HAL_I2C_Mem_Read(&i2c, ADDRESS, RA_INT_PIN_CFG, sizeof(RA_INT_PIN_CFG),
                   &buffer, sizeof(buffer), i2c_TIMEOUT);
  buffer &= ~(0x01 << INTCFG_INT_OPEN_BIT);
  buffer |= ((Drive & 0x1) << INTCFG_INT_OPEN_BIT);
  HAL_I2C_Mem_Write(&i2c, ADDRESS, RA_INT_PIN_CFG, sizeof(RA_INT_PIN_CFG),
                    &buffer, sizeof(buffer), i2c_TIMEOUT);
}

void SetInterruptLatch(I2C_HandleTypeDef &i2c,
                       const std::uint8_t Latch) noexcept {
  std::uint8_t buffer;
  HAL_I2C_Mem_Read(&i2c, ADDRESS, RA_INT_PIN_CFG, sizeof(RA_INT_PIN_CFG),
                   &buffer, sizeof(buffer), i2c_TIMEOUT);
  buffer &= ~(0x01 << INTCFG_INT_RD_CLEAR_BIT);
  buffer |= ((Latch & 0x1) << INTCFG_INT_RD_CLEAR_BIT);
  HAL_I2C_Mem_Write(&i2c, ADDRESS, RA_INT_PIN_CFG, sizeof(RA_INT_PIN_CFG),
                    &buffer, sizeof(buffer), i2c_TIMEOUT);
}

void SetInterruptLatchClear(I2C_HandleTypeDef &i2c,
                            const std::uint8_t Clear) noexcept {
  std::uint8_t buffer;
  HAL_I2C_Mem_Read(&i2c, ADDRESS, RA_INT_PIN_CFG, sizeof(RA_INT_PIN_CFG),
                   &buffer, sizeof(buffer), i2c_TIMEOUT);
  buffer &= ~(0x01 << INTCFG_LATCH_INT_EN_BIT);
  buffer |= ((Clear & 0x1) << INTCFG_LATCH_INT_EN_BIT);
  HAL_I2C_Mem_Write(&i2c, ADDRESS, RA_INT_PIN_CFG, sizeof(RA_INT_PIN_CFG),
                    &buffer, sizeof(buffer), i2c_TIMEOUT);
}

void SetIntEnableRegister(I2C_HandleTypeDef &i2c, std::uint8_t Value) noexcept {
  HAL_I2C_Mem_Write(&i2c, ADDRESS, RA_INT_ENABLE, sizeof(RA_INT_ENABLE), &Value,
                    1, i2c_TIMEOUT);
}

void SetIntDataReadyEnabled(I2C_HandleTypeDef &i2c,
                            const std::uint8_t Enable) noexcept {
  std::uint8_t buffer;
  HAL_I2C_Mem_Read(&i2c, ADDRESS, RA_INT_ENABLE, sizeof(RA_INT_ENABLE), &buffer,
                   sizeof(buffer), i2c_TIMEOUT);
  buffer &= ~(0x01 << INTERRUPT_DATA_RDY_BIT);
  buffer |= ((Enable & 0x1) << INTERRUPT_DATA_RDY_BIT);
  HAL_I2C_Mem_Write(&i2c, ADDRESS, RA_INT_ENABLE, sizeof(RA_INT_ENABLE),
                    &buffer, sizeof(buffer), i2c_TIMEOUT);
}

std::uint8_t GetIntStatusRegister(I2C_HandleTypeDef &i2c) noexcept {
  std::uint8_t buffer;
  HAL_I2C_Mem_Read(&i2c, ADDRESS, RA_INT_STATUS, sizeof(RA_INT_STATUS), &buffer,
                   sizeof(buffer), i2c_TIMEOUT);
  return buffer;
}

std::uint8_t GetDeviceID(I2C_HandleTypeDef &i2c) noexcept {
  std::uint8_t buffer;
  HAL_I2C_Mem_Read(&i2c, ADDRESS, RA_WHO_AM_I, sizeof(RA_WHO_AM_I), &buffer,
                   sizeof(buffer), i2c_TIMEOUT);
  return buffer << 1;
}

//
//	Motion functions - not included in documentation/register map
//
void SetDHPFMode(I2C_HandleTypeDef &i2c, const std::uint8_t Dhpf) noexcept {
  std::uint8_t buffer;
  HAL_I2C_Mem_Read(&i2c, ADDRESS, RA_ACCEL_CONFIG, sizeof(RA_ACCEL_CONFIG),
                   &buffer, sizeof(buffer), i2c_TIMEOUT);
  buffer &= ~(0x07);
  buffer |= Dhpf & 0x7;
  HAL_I2C_Mem_Write(&i2c, ADDRESS, RA_ACCEL_CONFIG, sizeof(RA_ACCEL_CONFIG),
                    &buffer, sizeof(buffer), i2c_TIMEOUT);
}

std::uint8_t GetMotionStatusRegister(I2C_HandleTypeDef &i2c) noexcept {
  std::uint8_t buffer;
  HAL_I2C_Mem_Read(&i2c, ADDRESS, RA_MOT_DETECT_STATUS,
                   sizeof(RA_MOT_DETECT_STATUS), &buffer, sizeof(buffer),
                   i2c_TIMEOUT);
  return buffer;
}

void SetIntZeroMotionEnabled(I2C_HandleTypeDef &i2c,
                             const std::uint8_t Enable) noexcept {
  std::uint8_t buffer;
  HAL_I2C_Mem_Read(&i2c, ADDRESS, RA_INT_ENABLE, sizeof(RA_INT_ENABLE), &buffer,
                   sizeof(buffer), i2c_TIMEOUT);
  buffer &= ~(0x01 << INTERRUPT_ZMOT_BIT);
  buffer |= ((Enable & 0x1) << INTERRUPT_ZMOT_BIT);
  HAL_I2C_Mem_Write(&i2c, ADDRESS, RA_INT_ENABLE, sizeof(RA_INT_ENABLE),
                    &buffer, sizeof(buffer), i2c_TIMEOUT);
}

void SetIntMotionEnabled(I2C_HandleTypeDef &i2c,
                         const std::uint8_t Enable) noexcept {
  std::uint8_t buffer;
  HAL_I2C_Mem_Read(&i2c, ADDRESS, RA_INT_ENABLE, sizeof(RA_INT_ENABLE), &buffer,
                   sizeof(buffer), i2c_TIMEOUT);
  buffer &= ~(0x01 << INTERRUPT_MOT_BIT);
  buffer |= ((Enable & 0x1) << INTERRUPT_MOT_BIT);
  HAL_I2C_Mem_Write(&i2c, ADDRESS, RA_INT_ENABLE, sizeof(RA_INT_ENABLE),
                    &buffer, sizeof(buffer), i2c_TIMEOUT);
}

void SetIntFreeFallEnabled(I2C_HandleTypeDef &i2c,
                           const std::uint8_t Enable) noexcept {
  std::uint8_t buffer;
  HAL_I2C_Mem_Read(&i2c, ADDRESS, RA_INT_ENABLE, sizeof(RA_INT_ENABLE), &buffer,
                   sizeof(buffer), i2c_TIMEOUT);
  buffer &= ~(0x01 << INTERRUPT_FF_BIT);
  buffer |= ((Enable & 0x1) << INTERRUPT_FF_BIT);
  HAL_I2C_Mem_Write(&i2c, ADDRESS, RA_INT_ENABLE, sizeof(RA_INT_ENABLE),
                    &buffer, sizeof(buffer), i2c_TIMEOUT);
}

void SetMotionDetectionThreshold(I2C_HandleTypeDef &i2c,
                                 std::uint8_t Threshold) noexcept {
  HAL_I2C_Mem_Write(&i2c, ADDRESS, RA_MOT_THR, sizeof(RA_MOT_THR), &Threshold,
                    sizeof(Threshold), i2c_TIMEOUT);
}

void SetMotionDetectionDuration(I2C_HandleTypeDef &i2c,
                                std::uint8_t Duration) noexcept {
  HAL_I2C_Mem_Write(&i2c, ADDRESS, RA_MOT_DUR, sizeof(RA_MOT_DUR), &Duration,
                    sizeof(Duration), i2c_TIMEOUT);
}

void SetZeroMotionDetectionThreshold(I2C_HandleTypeDef &i2c,
                                     std::uint8_t Threshold) noexcept {
  HAL_I2C_Mem_Write(&i2c, ADDRESS, RA_ZRMOT_THR, sizeof(RA_ZRMOT_THR),
                    &Threshold, sizeof(Threshold), i2c_TIMEOUT);
}

void SetZeroMotionDetectionDuration(I2C_HandleTypeDef &i2c,
                                    std::uint8_t Duration) noexcept {
  HAL_I2C_Mem_Write(&i2c, ADDRESS, RA_ZRMOT_DUR, sizeof(RA_ZRMOT_DUR),
                    &Duration, sizeof(Duration), i2c_TIMEOUT);
}

void SetFreeFallDetectionThreshold(I2C_HandleTypeDef &i2c,
                                   std::uint8_t Threshold) noexcept {
  HAL_I2C_Mem_Write(&i2c, ADDRESS, RA_FF_THR, sizeof(RA_FF_THR), &Threshold,
                    sizeof(Threshold), i2c_TIMEOUT);
}

void SetFreeFallDetectionDuration(I2C_HandleTypeDef &i2c,
                                  std::uint8_t Duration) noexcept {
  HAL_I2C_Mem_Write(&i2c, ADDRESS, RA_FF_DUR, sizeof(RA_FF_DUR), &Duration,
                    sizeof(Duration), i2c_TIMEOUT);
}

//
//	Initialization
//

void Init(I2C_HandleTypeDef &i2c, const std::uint8_t GyroRange,
          const std::uint8_t AccelRange) noexcept {
  DeviceReset(i2c, 1);
  SetSleepEnabled(i2c, 0);
  SetClockSource(i2c, CLOCK_INTERNAL);
  SetDlpf(i2c, DLPF_BW_20);
  SetFullScaleGyroRange(i2c, GyroRange);
  SetFullScaleAccelRange(i2c, AccelRange);
}

}; // namespace MPU6050
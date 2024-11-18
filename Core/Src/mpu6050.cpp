#include "mpu6050.hpp"
#include "common.hpp"
#include "main.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <expected>

namespace InvertedSway {

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
    using GyroFilter = MPU6050::GyroFilter;
    using AccelFilter = MPU6050::AccelFilter;

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

    GyroFilter MPU6050::make_gyro_filter() noexcept
    {
        return make_recursive_average<GyroRaw, Raw>();
    }

    AccelFilter MPU6050::make_accel_filter() noexcept
    {
        return make_recursive_average<AccelRaw, Raw>();
    }

    Scaled MPU6050::gyro_range_to_scale(const std::uint8_t gyro_range) noexcept
    {
        switch (gyro_range) {
            case GYRO_FS_250:
                return 131.0f;
            case GYRO_FS_500:
                return 65.5f;
            case GYRO_FS_1000:
                return 32.8f;
            case GYRO_FS_2000:
                return 16.4f;
            default:
                return 0.0f;
        }
    }

    Scaled MPU6050::accel_range_to_scale(const std::uint8_t accel_range) noexcept
    {
        switch (accel_range) {
            case ACCEL_FS_2:
                return 16384.0f;
            case ACCEL_FS_4:
                return 8192.0f;
            case ACCEL_FS_8:
                return 4096.0f;
            case ACCEL_FS_16:
                return 2048.0f;
            default:
                return 0.0f;
        }
    }

    MPU6050::MPU6050(I2cHandle i2c,
                     const std::uint8_t addres,
                     const std::uint8_t gyro_range,
                     const std::uint8_t accel_range) noexcept :
        i2c_{i2c}, address_{addres}, gyro_range_{gyro_range}, accel_range_{accel_range}

    {
        this->initialize();
    }

    MPU6050::MPU6050(I2cHandle i2c,
                     const std::uint8_t addres,
                     const std::uint8_t gyro_range,
                     const std::uint8_t accel_range,
                     GyroFilter&& gyro_filter,
                     AccelFilter&& accel_filter) noexcept :
        i2c_{i2c},
        address_{addres},
        gyro_range_{gyro_range},
        accel_range_{accel_range},
        gyro_filter_{std::forward<GyroFilter>(gyro_filter)},
        accel_filter_{std::forward<AccelFilter>(accel_filter)}

    {
        this->initialize();
    }

    MPU6050::~MPU6050() noexcept
    {
        this->deinitialize();
    }

    void MPU6050::initialize() noexcept
    {
        if (!this->initialized_) {
            if (HAL_I2C_IsDeviceReady(this->i2c_, this->address_, 10, I2C_TIMEOUT) != HAL_OK) {
                printf("device is not ready\r\n");
                return;
            }
            this->set_address_pin(AD0_GPIO_Port, AD0_Pin);
            if (this->get_device_id() == this->address_) {
                this->device_reset(1);
                HAL_Delay(50);
                this->set_sleep_enabled(0);
                HAL_Delay(50);
                this->set_clock_source(CLOCK_INTERNAL);
                HAL_Delay(50);
                this->set_sampling_rate_and_dlpf(SAMPLING_RATE_HZ, DLPF_BW_256);
                HAL_Delay(50);
                this->set_full_scale_gyro_range(this->gyro_range_);
                HAL_Delay(50);
                this->set_full_scale_accel_range(this->accel_range_);
                HAL_Delay(50);
                this->set_interrupt();
                this->initialized_ = true;
            }
        }
    }

    void MPU6050::deinitialize() noexcept
    {
        if (this->initialized_) {
            if (this->get_device_id() == this->address_) {
                this->device_reset(1);
                this->initialized_ = false;
            }
        }
    }

    void MPU6050::set_dlpf(const std::uint8_t value) const noexcept
    {
        std::uint8_t buffer = value & 0x7;
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          RA_CONFIG,
                          sizeof(RA_CONFIG),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_address_pin(GpioHandle gpio, const std::uint16_t address_pin) const noexcept
    {
        if (this->address_ == ADDRESS) {
            HAL_GPIO_WritePin(gpio, address_pin, GPIO_PinState::GPIO_PIN_RESET);
        } else if (this->address_ == ADDRESS2) {
            HAL_GPIO_WritePin(gpio, address_pin, GPIO_PinState::GPIO_PIN_SET);
        }
    }

    void MPU6050::device_reset(const std::uint8_t reset) const noexcept
    {
        std::uint8_t buffer = (reset & 1U) << PWR1_DEVICE_RESET_BIT;
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          RA_PWR_MGMT_1,
                          sizeof(RA_PWR_MGMT_1),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_sampling_rate_and_dlpf(const std::uint32_t rate, const std::uint8_t dlpf) const noexcept
    {
        this->set_dlpf(dlpf);
        std::uint8_t buffer = ([](const std::uint8_t dlpf) {
                                  if (dlpf == DLPF_BW_256) {
                                      return GYRO_OUTPUT_RATE_DLPF_DIS_HZ;
                                  } else {
                                      return GYRO_OUTPUT_RATE_DLPF_EN_HZ;
                                  }
                              }(dlpf) /
                               rate) -
                              1;
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          RA_SMPLRT_DIV,
                          sizeof(RA_SMPLRT_DIV),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_sleep_enabled(const std::uint8_t enable) const noexcept
    {
        std::uint8_t buffer = (enable & 1U) << PWR1_SLEEP_BIT;
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          RA_PWR_MGMT_1,
                          sizeof(RA_PWR_MGMT_1),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_cycle_enabled(const std::uint8_t enable) const noexcept
    {
        std::uint8_t buffer = (enable & 1U) << PWR1_CYCLE_BIT;
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          RA_PWR_MGMT_1,
                          sizeof(RA_PWR_MGMT_1),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_temperature_sensor_disabled(const std::uint8_t disable) const noexcept
    {
        std::uint8_t buffer = (disable & 1U) << PWR1_TEMP_DIS_BIT;
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          RA_PWR_MGMT_1,
                          sizeof(RA_PWR_MGMT_1),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_clock_source(const std::uint8_t source) const noexcept
    {
        std::uint8_t buffer = (source & 0x7);
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          RA_PWR_MGMT_1,
                          sizeof(RA_PWR_MGMT_1),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_low_power_wake_up_frequency(const std::uint8_t frequency) const noexcept
    {
        std::uint8_t buffer = (frequency & 0x3) << PWR2_LP_WAKE_CTRL_BIT;
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          RA_PWR_MGMT_2,
                          sizeof(RA_PWR_MGMT_2),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::accelerometer_axis_standby(const std::uint8_t x_accel_standby,
                                             const std::uint8_t y_accel_standby,
                                             const std::uint8_t z_accel_standby) const noexcept
    {
        std::uint8_t buffer = ((x_accel_standby & 1U) << PWR2_STBY_XA_BIT) |
                              ((y_accel_standby & 1U) << PWR2_STBY_YA_BIT) |
                              ((z_accel_standby & 1U) << PWR2_STBY_ZA_BIT);
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          RA_PWR_MGMT_2,
                          sizeof(RA_PWR_MGMT_2),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::gyroscope_axis_standby(const std::uint8_t x_gyro_standby,
                                         const std::uint8_t y_gyro_standby,
                                         const std::uint8_t z_gyro_standby) const noexcept
    {
        std::uint8_t buffer = ((x_gyro_standby & 1U) << PWR2_STBY_XG_BIT) |
                              ((y_gyro_standby & 1U) << PWR2_STBY_YG_BIT) | ((z_gyro_standby & 1U) << PWR2_STBY_ZG_BIT);
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          RA_PWR_MGMT_2,
                          sizeof(RA_PWR_MGMT_2),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_full_scale_gyro_range(const std::uint8_t range) const noexcept
    {
        std::uint8_t buffer = (range & 0x7) << 3;
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          RA_GYRO_CONFIG,
                          sizeof(RA_GYRO_CONFIG),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_full_scale_accel_range(const std::uint8_t range) const noexcept
    {
        std::uint8_t buffer = (range & 0x7) << 3;
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          RA_ACCEL_CONFIG,
                          sizeof(RA_ACCEL_CONFIG),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    TempRaw MPU6050::get_temperature_raw() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }
        std::uint8_t buffer[2];
        HAL_I2C_Mem_Read(this->i2c_,
                         this->address_,
                         RA_TEMP_OUT_H,
                         sizeof(RA_TEMP_OUT_H),
                         buffer,
                         sizeof(buffer),
                         I2C_TIMEOUT);
        return ((static_cast<TempRaw>(buffer[0])) << 8) | buffer[1];
    }

    TempScaled MPU6050::get_temperature_celsius() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }
        return static_cast<TempScaled>(this->get_temperature_raw()) / 340 + 36.53;
    }

    Raw MPU6050::get_acceleration_x_raw() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }
        std::uint8_t buffer[2];
        HAL_I2C_Mem_Read(this->i2c_,
                         this->address_,
                         RA_ACCEL_XOUT_H,
                         sizeof(RA_ACCEL_XOUT_H),
                         buffer,
                         sizeof(buffer),
                         I2C_TIMEOUT);
        return ((static_cast<Raw>(buffer[0])) << 8) | buffer[1];
    }

    Raw MPU6050::get_acceleration_y_raw() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }
        std::uint8_t buffer[2];
        HAL_I2C_Mem_Read(this->i2c_,
                         this->address_,
                         RA_ACCEL_YOUT_H,
                         sizeof(RA_ACCEL_YOUT_H),
                         buffer,
                         sizeof(buffer),
                         I2C_TIMEOUT);
        return ((static_cast<Raw>(buffer[0])) << 8) | buffer[1];
    }

    Raw MPU6050::get_acceleration_z_raw() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }
        std::uint8_t buffer[2];
        HAL_I2C_Mem_Read(this->i2c_,
                         this->address_,
                         RA_ACCEL_ZOUT_H,
                         sizeof(RA_ACCEL_ZOUT_H),
                         buffer,
                         sizeof(buffer),
                         I2C_TIMEOUT);
        return ((static_cast<Raw>(buffer[0])) << 8) | buffer[1];
    }

    AccelRaw MPU6050::get_accelerometer_raw() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }
        std::uint8_t buffer[6];
        HAL_I2C_Mem_Read(this->i2c_,
                         this->address_,
                         RA_ACCEL_XOUT_H,
                         sizeof(RA_ACCEL_XOUT_H),
                         buffer,
                         sizeof(buffer),
                         I2C_TIMEOUT);

        return this->accel_filter_(AccelRaw{((static_cast<Raw>(buffer[0])) << 8) | buffer[1],
                                            ((static_cast<Raw>(buffer[2])) << 8) | buffer[3],
                                            ((static_cast<Raw>(buffer[4])) << 8) | buffer[5]});
    }

    AccelScaled MPU6050::get_accelerometer_scaled() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }
        const auto accel_scale{accel_range_to_scale(this->accel_range_)};
        const auto accel_raw_result{this->get_accelerometer_raw()};
        return AccelScaled{static_cast<Scaled>(accel_raw_result.x) / accel_scale,
                           static_cast<Scaled>(accel_raw_result.y) / accel_scale,
                           static_cast<Scaled>(accel_raw_result.z) / accel_scale};
    }

    Raw MPU6050::get_rotation_x_raw() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }
        std::uint8_t buffer[2];
        HAL_I2C_Mem_Read(this->i2c_,
                         this->address_,
                         RA_GYRO_XOUT_H,
                         sizeof(RA_GYRO_XOUT_H),
                         buffer,
                         sizeof(buffer),
                         I2C_TIMEOUT);
        return ((static_cast<Raw>(buffer[0])) << 8) | buffer[1];
    }

    Raw MPU6050::get_rotation_y_raw() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }
        std::uint8_t buffer[2];
        HAL_I2C_Mem_Read(this->i2c_,
                         this->address_,
                         RA_GYRO_YOUT_H,
                         sizeof(RA_GYRO_YOUT_H),
                         buffer,
                         sizeof(buffer),
                         I2C_TIMEOUT);
        return ((static_cast<Raw>(buffer[0])) << 8) | buffer[1];
    }

    Raw MPU6050::get_rotation_z_raw() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }
        std::uint8_t buffer[2];
        HAL_I2C_Mem_Read(this->i2c_,
                         this->address_,
                         RA_GYRO_ZOUT_H,
                         sizeof(RA_GYRO_ZOUT_H),
                         buffer,
                         sizeof(buffer),
                         I2C_TIMEOUT);
        return ((static_cast<Raw>(buffer[0])) << 8) | buffer[1];
    }

    GyroRaw MPU6050::get_gyroscope_raw() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }
        std::uint8_t buffer[6];
        HAL_I2C_Mem_Read(this->i2c_,
                         this->address_,
                         RA_GYRO_XOUT_H,
                         sizeof(RA_GYRO_XOUT_H),
                         buffer,
                         sizeof(buffer),
                         I2C_TIMEOUT);
        return this->gyro_filter_(GyroRaw{((static_cast<Raw>(buffer[0])) << 8) | buffer[1],
                                          ((static_cast<Raw>(buffer[2])) << 8) | buffer[3],
                                          ((static_cast<Raw>(buffer[4])) << 8) | buffer[5]});
    }

    GyroScaled MPU6050::get_gyroscope_scaled() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }
        const auto gyro_scale{gyro_range_to_scale(this->gyro_range_)};
        const auto gyro_raw{this->get_gyroscope_raw()};
        return GyroScaled{static_cast<Scaled>(gyro_raw.x) / gyro_scale,
                          static_cast<Scaled>(gyro_raw.y) / gyro_scale,
                          static_cast<Scaled>(gyro_raw.z) / gyro_scale};
    }

    RollPitchYaw MPU6050::get_roll_pitch_yaw() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }
        const auto accel_scaled{get_accelerometer_scaled()};
        return RollPitchYaw{
            std::atan2(accel_scaled.y, accel_scaled.z) * 180.0 / M_PI,
            -(std::atan2(accel_scaled.x, std::sqrt(accel_scaled.y * accel_scaled.y + accel_scaled.z * accel_scaled.z)) *
              180.0) /
                M_PI,
            {}};
    }

    void MPU6050::set_interrupt() const noexcept
    {
        this->set_interrupt_mode(INTMODE_ACTIVEHIGH);
        // this->set_interrupt_drive(INTDRV_PUSHPULL);
        this->set_interrupt_latch(INTLATCH_50USPULSE);
        this->set_interrupt_latch_clear(INTCLEAR_STATUSREAD);
        this->set_int_enable_register(1);

        // Enable Motion interrputs
        // this->set_dhpf_mode(DHPF_5);
        // this->set_int_motion_enabled(1);
        // this->set_int_zero_motion_enabled(1);
        // this->set_int_free_fall_enabled(1);
        // this->set_free_fall_detection_duration(2);
        // this->set_free_fall_detection_threshold(5);
        // this->set_motion_detection_duration(5);
        // this->set_motion_detection_threshold(2);
        // this->set_zero_motion_detection_duration(2);
        // this->set_zero_motion_detection_threshold(4);
    }

    void MPU6050::set_interrupt_mode(const std::uint8_t mode) const noexcept
    {
        std::uint8_t buffer = (mode & 1U) << INTCFG_INT_LEVEL_BIT;
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          RA_INT_PIN_CFG,
                          sizeof(RA_INT_PIN_CFG),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_interrupt_drive(const std::uint8_t drive) const noexcept
    {
        std::uint8_t buffer = (drive & 1U) << INTCFG_INT_OPEN_BIT;
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          RA_INT_PIN_CFG,
                          sizeof(RA_INT_PIN_CFG),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_interrupt_latch(const std::uint8_t latch) const noexcept
    {
        std::uint8_t buffer = (latch & 1U) << INTCFG_INT_RD_CLEAR_BIT;
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          RA_INT_PIN_CFG,
                          sizeof(RA_INT_PIN_CFG),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_interrupt_latch_clear(const std::uint8_t clear) const noexcept
    {
        std::uint8_t buffer = (clear & 1U) << INTCFG_LATCH_INT_EN_BIT;
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          RA_INT_PIN_CFG,
                          sizeof(RA_INT_PIN_CFG),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_int_enable_register(const std::uint8_t enable) const noexcept
    {
        std::uint8_t buffer = enable;
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          RA_INT_ENABLE,
                          sizeof(RA_INT_ENABLE),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_int_data_ready_enabled(const std::uint8_t enable) const noexcept
    {
        std::uint8_t buffer = (enable & 1U) << INTERRUPT_DATA_RDY_BIT;
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          RA_INT_ENABLE,
                          sizeof(RA_INT_ENABLE),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    std::uint8_t MPU6050::get_int_status_register() const noexcept
    {
        std::uint8_t buffer;
        HAL_I2C_Mem_Read(this->i2c_,
                         this->address_,
                         RA_INT_STATUS,
                         sizeof(RA_INT_STATUS),
                         &buffer,
                         sizeof(buffer),
                         I2C_TIMEOUT);
        return buffer;
    }

    std::uint8_t MPU6050::get_device_id() const noexcept
    {
        std::uint8_t buffer;
        HAL_I2C_Mem_Read(this->i2c_,
                         this->address_,
                         RA_WHO_AM_I,
                         sizeof(RA_WHO_AM_I),
                         &buffer,
                         sizeof(buffer),
                         I2C_TIMEOUT);
        return buffer << 1;
    }

    void MPU6050::set_dhpf_mode(const std::uint8_t dhpf) const noexcept
    {
        std::uint8_t buffer = dhpf & 0x7;
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          RA_ACCEL_CONFIG,
                          sizeof(RA_ACCEL_CONFIG),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    std::uint8_t MPU6050::get_motion_status_register() const noexcept
    {
        std::uint8_t buffer;
        HAL_I2C_Mem_Read(this->i2c_,
                         this->address_,
                         RA_MOT_DETECT_STATUS,
                         sizeof(RA_MOT_DETECT_STATUS),
                         &buffer,
                         sizeof(buffer),
                         I2C_TIMEOUT);
        return buffer;
    }

    void MPU6050::set_int_zero_motion_enabled(const std::uint8_t enable) const noexcept
    {
        std::uint8_t buffer = (enable & 1U) << INTERRUPT_ZMOT_BIT;
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          RA_INT_ENABLE,
                          sizeof(RA_INT_ENABLE),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_int_motion_enabled(const std::uint8_t enable) const noexcept
    {
        std::uint8_t buffer = (enable & 1U) << INTERRUPT_MOT_BIT;
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          RA_INT_ENABLE,
                          sizeof(RA_INT_ENABLE),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_int_free_fall_enabled(const std::uint8_t enable) const noexcept
    {
        std::uint8_t buffer = (enable & 1U) << INTERRUPT_FF_BIT;
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          RA_INT_ENABLE,
                          sizeof(RA_INT_ENABLE),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_motion_detection_threshold(std::uint8_t threshold) const noexcept
    {
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          RA_MOT_THR,
                          sizeof(RA_MOT_THR),
                          &threshold,
                          sizeof(threshold),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_motion_detection_duration(std::uint8_t duration) const noexcept
    {
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          RA_MOT_DUR,
                          sizeof(RA_MOT_DUR),
                          &duration,
                          sizeof(duration),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_zero_motion_detection_threshold(std::uint8_t threshold) const noexcept
    {
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          RA_ZRMOT_THR,
                          sizeof(RA_ZRMOT_THR),
                          &threshold,
                          sizeof(threshold),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_zero_motion_detection_duration(std::uint8_t duration) const noexcept
    {
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          RA_ZRMOT_DUR,
                          sizeof(RA_ZRMOT_DUR),
                          &duration,
                          sizeof(duration),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_free_fall_detection_threshold(std::uint8_t threshold) const noexcept
    {
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          RA_FF_THR,
                          sizeof(RA_FF_THR),
                          &threshold,
                          sizeof(threshold),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_free_fall_detection_duration(std::uint8_t duration) const noexcept
    {
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          RA_FF_DUR,
                          sizeof(RA_FF_DUR),
                          &duration,
                          sizeof(duration),
                          I2C_TIMEOUT);
    }

}; // namespace InvertedSway
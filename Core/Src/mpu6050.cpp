#include "mpu6050.hpp"
#include "common.hpp"
#include "main.h"
#include "stm32l4xx_hal_i2c.h"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <expected>

// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
// {
//     if (GPIO_Pin == INTR_Pin) {
//         const auto interrupts{mpu6050_handle->get_int_status_register()};
//         mpu6050_handle->get_motion_status_register();
//         printf( "int_ status triggered: %X\n\n\r", interrupts);

//         if (interrupts & (0x01 << MPU6050::INTERRUPT_FIFO_OFLOW_BIT)) // Bit 4 (0x10)
//         {
//             printf( "FIFO Overflow detected\n\r");
//         }

//         if (interrupts & (0x01 << MPU6050::INTERRUPT_MOT_BIT)) // Bit 6 (0x40)
//         {
//             printf( "Motion detected\n\r");
//         }

//         if (interrupts & (0x01 << MPU6050::INTERRUPT_ZMOT_BIT)) // Bit 5 (0x20)
//         {
//             printf( "Zero Motion detected\n\r");
//         }

//         if (interrupts & (0x01 << MPU6050::INTERRUPT_FF_BIT)) // Bit 7 (0x80)
//         {
//             printf( "Freefall detected\n\r");
//         }
//     }
// }

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

    MPU6050::MPU6050(I2cHandle i2c,
                     const std::uint8_t addres,
                     const std::uint8_t gyro_range,
                     const std::uint8_t accel_range,
                     GyroFilter&& gyro_filter,
                     AccelFilter&& accel_filter) noexcept :
        i2c_{i2c},
        addres_{addres},
        gyro_range_{gyro_range},
        accel_range_{accel_range},
        gyro_filter_{gyro_filter},
        accel_filter_{accel_filter}
    {
        initialize();
    }

    MPU6050::MPU6050(I2cHandle i2c,
                     const std::uint8_t addres,
                     const std::uint8_t gyro_range,
                     const std::uint8_t accel_range) noexcept :
        i2c_{i2c}, addres_{addres}, gyro_range_{gyro_range}, accel_range_{accel_range}

    {
        initialize();
    }

    MPU6050::~MPU6050() noexcept
    {
        deinitialize();
    }

    void MPU6050::initialize() noexcept
    {
        if (HAL_I2C_IsDeviceReady(i2c_, addres_, 10, i2c_TIMEOUT) != HAL_OK) {
            printf("device is not ready\r\n");
            return;
        }
        printf("device is ready\r\n");

        if (get_device_id() != addres_) {
            printf("device id isnt correct\r\n");
            return;
        }
        printf("device id is correct\r\n");

        device_reset(1);
        set_sleep_enabled(0);
        set_clock_source(CLOCK_INTERNAL);
        set_dlpf(DLPF_BW_20);
        set_full_scale_gyro_range(gyro_range_);
        set_full_scale_accel_range(accel_range_);
        // set_interrupt();
    }

    void MPU6050::deinitialize() noexcept
    {
        if (get_device_id()) {
            device_reset(PWR1_DEVICE_RESET_BIT);
        }
    }

    void MPU6050::set_dlpf(const std::uint8_t value) const noexcept
    {
        std::uint8_t buffer;
        HAL_I2C_Mem_Read(i2c_, addres_, RA_CONFIG, sizeof(RA_CONFIG), &buffer, sizeof(buffer), i2c_TIMEOUT);
        buffer &= 0xF8;
        buffer |= value & 0x7;
        HAL_I2C_Mem_Write(i2c_, addres_, RA_CONFIG, sizeof(RA_CONFIG), &buffer, sizeof(buffer), i2c_TIMEOUT);
    }

    //
    // PWR_MGMT_1
    //
    void MPU6050::device_reset(const std::uint8_t reset) const noexcept
    {
        std::uint8_t buffer;
        HAL_I2C_Mem_Read(i2c_, addres_, RA_PWR_MGMT_1, sizeof(RA_PWR_MGMT_1), &buffer, sizeof(buffer), i2c_TIMEOUT);
        buffer &= ~1U << PWR1_DEVICE_RESET_BIT;
        buffer |= (reset & 1U) << PWR1_DEVICE_RESET_BIT;
        HAL_I2C_Mem_Write(i2c_, addres_, RA_PWR_MGMT_1, sizeof(RA_PWR_MGMT_1), &buffer, sizeof(buffer), i2c_TIMEOUT);
    }

    void MPU6050::set_sleep_enabled(const std::uint8_t enable) const noexcept
    {
        std::uint8_t buffer;
        HAL_I2C_Mem_Read(i2c_, addres_, RA_PWR_MGMT_1, sizeof(RA_PWR_MGMT_1), &buffer, sizeof(buffer), i2c_TIMEOUT);
        buffer &= ~1U << PWR1_SLEEP_BIT;
        buffer |= (enable & 1U) << PWR1_SLEEP_BIT;
        HAL_I2C_Mem_Write(i2c_, addres_, RA_PWR_MGMT_1, sizeof(RA_PWR_MGMT_1), &buffer, sizeof(buffer), i2c_TIMEOUT);
    }

    void MPU6050::set_cycle_enabled(const std::uint8_t enable) const noexcept
    {
        std::uint8_t buffer;
        HAL_I2C_Mem_Read(i2c_, addres_, RA_PWR_MGMT_1, sizeof(RA_PWR_MGMT_1), &buffer, sizeof(buffer), i2c_TIMEOUT);
        buffer &= ~1U << PWR1_CYCLE_BIT;
        buffer |= (enable & 1U) << PWR1_CYCLE_BIT;
        HAL_I2C_Mem_Write(i2c_, addres_, RA_PWR_MGMT_1, sizeof(RA_PWR_MGMT_1), &buffer, sizeof(buffer), i2c_TIMEOUT);
    }

    void MPU6050::set_temperature_sensor_disabled(const std::uint8_t disable) const noexcept
    {
        std::uint8_t buffer;
        HAL_I2C_Mem_Read(i2c_, addres_, RA_PWR_MGMT_1, sizeof(RA_PWR_MGMT_1), &buffer, sizeof(buffer), i2c_TIMEOUT);
        buffer &= ~1U << PWR1_TEMP_DIS_BIT;
        buffer |= (disable & 1U) << PWR1_TEMP_DIS_BIT;
        HAL_I2C_Mem_Write(i2c_, addres_, RA_PWR_MGMT_1, sizeof(RA_PWR_MGMT_1), &buffer, sizeof(buffer), i2c_TIMEOUT);
    }

    void MPU6050::set_clock_source(const std::uint8_t source) const noexcept
    {
        std::uint8_t buffer;
        HAL_I2C_Mem_Read(i2c_, addres_, RA_PWR_MGMT_1, sizeof(RA_PWR_MGMT_1), &buffer, sizeof(buffer), i2c_TIMEOUT);
        buffer &= 0xF8;
        buffer |= source & 0x7;
        HAL_I2C_Mem_Write(i2c_, addres_, RA_PWR_MGMT_1, sizeof(RA_PWR_MGMT_1), &buffer, sizeof(buffer), i2c_TIMEOUT);
    }

    //
    //	PWR_MGMT_2
    //
    void MPU6050::set_low_power_wake_up_frequency(const std::uint8_t frequency) const noexcept
    {
        std::uint8_t buffer;
        HAL_I2C_Mem_Read(i2c_, addres_, RA_PWR_MGMT_2, sizeof(RA_PWR_MGMT_2), &buffer, sizeof(buffer), i2c_TIMEOUT);
        buffer &= 0x3F;
        buffer |= frequency & 0x3 << PWR2_LP_WAKE_CTRL_BIT;
        HAL_I2C_Mem_Write(i2c_, addres_, RA_PWR_MGMT_2, sizeof(RA_PWR_MGMT_2), &buffer, sizeof(buffer), i2c_TIMEOUT);
    }

    void MPU6050::accelerometer_axis_standby(const std::uint8_t x_accel_standby,
                                             const std::uint8_t y_accel_standby,
                                             const std::uint8_t z_accel_standby) const noexcept
    {
        std::uint8_t buffer;
        HAL_I2C_Mem_Read(i2c_, addres_, RA_PWR_MGMT_2, sizeof(RA_PWR_MGMT_2), &buffer, sizeof(buffer), i2c_TIMEOUT);
        buffer &= 0xC7;
        buffer |= ((x_accel_standby & 1U) << PWR2_STBY_XA_BIT) | ((y_accel_standby & 1U) << PWR2_STBY_YA_BIT) |
                  ((z_accel_standby & 1U) << PWR2_STBY_ZA_BIT);
        HAL_I2C_Mem_Write(i2c_, addres_, RA_PWR_MGMT_2, sizeof(RA_PWR_MGMT_2), &buffer, sizeof(buffer), i2c_TIMEOUT);
    }

    void MPU6050::gyroscope_axis_standby(const std::uint8_t x_gyro_standby,
                                         const std::uint8_t y_gyro_standby,
                                         const std::uint8_t z_gyro_standby) const noexcept
    {
        std::uint8_t buffer;
        HAL_I2C_Mem_Read(i2c_, addres_, RA_PWR_MGMT_2, sizeof(RA_PWR_MGMT_2), &buffer, sizeof(buffer), i2c_TIMEOUT);
        buffer &= 0xF8;
        buffer |= ((x_gyro_standby & 1U) << PWR2_STBY_XG_BIT) | ((y_gyro_standby & 1U) << PWR2_STBY_YG_BIT) |
                  ((z_gyro_standby & 1U) << PWR2_STBY_ZG_BIT);
        HAL_I2C_Mem_Write(i2c_, addres_, RA_PWR_MGMT_2, sizeof(RA_PWR_MGMT_2), &buffer, sizeof(buffer), i2c_TIMEOUT);
    }

    //
    //	Measurement scale configuration
    //
    void MPU6050::set_full_scale_gyro_range(const std::uint8_t range) const noexcept
    {
        std::uint8_t buffer;
        HAL_I2C_Mem_Read(i2c_, addres_, RA_GYRO_CONFIG, sizeof(RA_GYRO_CONFIG), &buffer, sizeof(buffer), i2c_TIMEOUT);
        buffer &= 0xE7;
        buffer |= (range & 0x7) << 3;
        HAL_I2C_Mem_Write(i2c_, addres_, RA_GYRO_CONFIG, sizeof(RA_GYRO_CONFIG), &buffer, sizeof(buffer), i2c_TIMEOUT);
    }

    void MPU6050::set_full_scale_accel_range(const std::uint8_t range) const noexcept
    {
        std::uint8_t buffer;
        HAL_I2C_Mem_Read(i2c_, addres_, RA_ACCEL_CONFIG, sizeof(RA_ACCEL_CONFIG), &buffer, sizeof(buffer), i2c_TIMEOUT);
        buffer &= 0xE7;
        buffer |= (range & 0x7) << 3;
        HAL_I2C_Mem_Write(i2c_,
                          addres_,
                          RA_ACCEL_CONFIG,
                          sizeof(RA_ACCEL_CONFIG),
                          &buffer,
                          sizeof(buffer),
                          i2c_TIMEOUT);
    }

    //
    // Reading data
    //
    TempRaw MPU6050::get_temperature_raw() const noexcept
    {
        std::uint8_t buffer[2];
        HAL_I2C_Mem_Read(i2c_, addres_, RA_TEMP_OUT_H, sizeof(RA_TEMP_OUT_H), buffer, sizeof(buffer), i2c_TIMEOUT);
        return ((static_cast<TempRaw>(buffer[0])) << 8) | buffer[1];
    }

    TempScaled MPU6050::get_temperature_celsius() const noexcept
    {
        return static_cast<TempScaled>(get_temperature_raw()) / 340 + 36.53;
    }

    Raw MPU6050::get_acceleration_x_raw() const noexcept
    {
        std::uint8_t buffer[2];
        HAL_I2C_Mem_Read(i2c_, addres_, RA_ACCEL_XOUT_H, sizeof(RA_ACCEL_XOUT_H), buffer, sizeof(buffer), i2c_TIMEOUT);
        return ((static_cast<Raw>(buffer[0])) << 8) | buffer[1];
    }

    Raw MPU6050::get_acceleration_y_raw() const noexcept
    {
        std::uint8_t buffer[2];
        HAL_I2C_Mem_Read(i2c_, addres_, RA_ACCEL_YOUT_H, sizeof(RA_ACCEL_YOUT_H), buffer, sizeof(buffer), i2c_TIMEOUT);
        return ((static_cast<Raw>(buffer[0])) << 8) | buffer[1];
    }

    Raw MPU6050::get_acceleration_z_raw() const noexcept
    {
        std::uint8_t buffer[2];
        HAL_I2C_Mem_Read(i2c_, addres_, RA_ACCEL_ZOUT_H, sizeof(RA_ACCEL_ZOUT_H), buffer, sizeof(buffer), i2c_TIMEOUT);
        return ((static_cast<Raw>(buffer[0])) << 8) | buffer[1];
    }

    AccelRaw MPU6050::get_accelerometer_raw() const noexcept
    {
        std::uint8_t buffer[6];
        HAL_I2C_Mem_Read(i2c_, addres_, RA_ACCEL_XOUT_H, sizeof(RA_ACCEL_XOUT_H), buffer, sizeof(buffer), i2c_TIMEOUT);

        return accel_filter_(AccelRaw{((static_cast<Raw>(buffer[0])) << 8) | buffer[1],
                                      ((static_cast<Raw>(buffer[2])) << 8) | buffer[3],
                                      ((static_cast<Raw>(buffer[4])) << 8) | buffer[5]});
    }

    AccelScaled MPU6050::get_accelerometer_scaled() const noexcept
    {
        const auto accel_scale{accel_range_to_scale(accel_range_)};
        const auto accel_raw_result{get_accelerometer_raw()};
        return AccelScaled{static_cast<Scaled>(accel_raw_result.x) * accel_scale,
                           static_cast<Scaled>(accel_raw_result.y) * accel_scale,
                           static_cast<Scaled>(accel_raw_result.z) * accel_scale};
    }

    Raw MPU6050::get_rotation_x_raw() const noexcept
    {
        std::uint8_t buffer[2];
        HAL_I2C_Mem_Read(i2c_, addres_, RA_GYRO_XOUT_H, sizeof(RA_GYRO_XOUT_H), buffer, sizeof(buffer), i2c_TIMEOUT);
        return ((static_cast<Raw>(buffer[0])) << 8) | buffer[1];
    }

    Raw MPU6050::get_rotation_y_raw() const noexcept
    {
        std::uint8_t buffer[2];
        HAL_I2C_Mem_Read(i2c_, addres_, RA_GYRO_YOUT_H, sizeof(RA_GYRO_YOUT_H), buffer, sizeof(buffer), i2c_TIMEOUT);
        return ((static_cast<Raw>(buffer[0])) << 8) | buffer[1];
    }

    Raw MPU6050::get_rotation_z_raw() const noexcept
    {
        std::uint8_t buffer[2];
        HAL_I2C_Mem_Read(i2c_, addres_, RA_GYRO_ZOUT_H, sizeof(RA_GYRO_ZOUT_H), buffer, sizeof(buffer), i2c_TIMEOUT);
        return ((static_cast<Raw>(buffer[0])) << 8) | buffer[1];
    }

    GyroRaw MPU6050::get_gyroscope_raw() const noexcept
    {
        std::uint8_t buffer[6];
        HAL_I2C_Mem_Read(i2c_, addres_, RA_GYRO_XOUT_H, sizeof(RA_GYRO_XOUT_H), buffer, sizeof(buffer), i2c_TIMEOUT);
        return gyro_filter_(GyroRaw{((static_cast<Raw>(buffer[0])) << 8) | buffer[1],
                                    ((static_cast<Raw>(buffer[2])) << 8) | buffer[3],
                                    ((static_cast<Raw>(buffer[4])) << 8) | buffer[5]});
    }

    GyroScaled MPU6050::get_gyroscope_scaled() const noexcept
    {
        const auto gyro_scale{gyro_range_to_scale(gyro_range_)};
        const auto gyro_raw{get_gyroscope_raw()};
        return GyroScaled{static_cast<Scaled>(gyro_raw.x) * gyro_scale,
                          static_cast<Scaled>(gyro_raw.y) * gyro_scale,
                          static_cast<Scaled>(gyro_raw.z) * gyro_scale};
    }

    RollPitchYaw MPU6050::get_roll_pitch_yaw() const noexcept
    {
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
        set_interrupt_mode(INTMODE_ACTIVEHIGH);
        set_interrupt_drive(INTDRV_PUSHPULL);
        set_interrupt_latch(INTLATCH_WAITCLEAR);
        set_interrupt_latch_clear(INTCLEAR_STATUSREAD);

        // Disable all interrupts
        set_int_enable_register(0);

        // Enable Motion interrputs
        set_dhpf_mode(DHPF_5);

        set_int_motion_enabled(1);
        set_int_zero_motion_enabled(1);
        set_int_free_fall_enabled(1);

        set_free_fall_detection_duration(2);
        set_free_fall_detection_threshold(5);

        set_motion_detection_duration(5);
        set_motion_detection_threshold(2);

        set_zero_motion_detection_duration(2);
        set_zero_motion_detection_threshold(4);
    }

    //
    //	set_ting Raw pin
    //
    void MPU6050::set_interrupt_mode(const std::uint8_t mode) const noexcept
    {
        std::uint8_t buffer;
        HAL_I2C_Mem_Read(i2c_, addres_, RA_INT_PIN_CFG, sizeof(RA_INT_PIN_CFG), &buffer, sizeof(buffer), i2c_TIMEOUT);
        buffer &= ~1U << INTCFG_INT_LEVEL_BIT;
        buffer |= (mode & 1U) << INTCFG_INT_LEVEL_BIT;
        HAL_I2C_Mem_Write(i2c_, addres_, RA_INT_PIN_CFG, sizeof(RA_INT_PIN_CFG), &buffer, sizeof(buffer), i2c_TIMEOUT);
    }

    void MPU6050::set_interrupt_drive(const std::uint8_t drive) const noexcept
    {
        std::uint8_t buffer;
        HAL_I2C_Mem_Read(i2c_, addres_, RA_INT_PIN_CFG, sizeof(RA_INT_PIN_CFG), &buffer, sizeof(buffer), i2c_TIMEOUT);
        buffer &= ~1U << INTCFG_INT_OPEN_BIT;
        buffer |= (drive & 1U) << INTCFG_INT_OPEN_BIT;
        HAL_I2C_Mem_Write(i2c_, addres_, RA_INT_PIN_CFG, sizeof(RA_INT_PIN_CFG), &buffer, sizeof(buffer), i2c_TIMEOUT);
    }

    void MPU6050::set_interrupt_latch(const std::uint8_t latch) const noexcept
    {
        std::uint8_t buffer;
        HAL_I2C_Mem_Read(i2c_, addres_, RA_INT_PIN_CFG, sizeof(RA_INT_PIN_CFG), &buffer, sizeof(buffer), i2c_TIMEOUT);
        buffer &= ~1U << INTCFG_INT_RD_CLEAR_BIT;
        buffer |= (latch & 1U) << INTCFG_INT_RD_CLEAR_BIT;
        HAL_I2C_Mem_Write(i2c_, addres_, RA_INT_PIN_CFG, sizeof(RA_INT_PIN_CFG), &buffer, sizeof(buffer), i2c_TIMEOUT);
    }

    void MPU6050::set_interrupt_latch_clear(const std::uint8_t clear) const noexcept
    {
        std::uint8_t buffer;
        HAL_I2C_Mem_Read(i2c_, addres_, RA_INT_PIN_CFG, sizeof(RA_INT_PIN_CFG), &buffer, sizeof(buffer), i2c_TIMEOUT);
        buffer &= ~1U << INTCFG_LATCH_INT_EN_BIT;
        buffer |= (clear & 1U) << INTCFG_LATCH_INT_EN_BIT;
        HAL_I2C_Mem_Write(i2c_, addres_, RA_INT_PIN_CFG, sizeof(RA_INT_PIN_CFG), &buffer, sizeof(buffer), i2c_TIMEOUT);
    }

    void MPU6050::set_int_enable_register(std::uint8_t value) const noexcept
    {
        HAL_I2C_Mem_Write(i2c_, addres_, RA_INT_ENABLE, sizeof(RA_INT_ENABLE), &value, 1, i2c_TIMEOUT);
    }

    void MPU6050::set_int_data_ready_enabled(const std::uint8_t enable) const noexcept
    {
        std::uint8_t buffer;
        HAL_I2C_Mem_Read(i2c_, addres_, RA_INT_ENABLE, sizeof(RA_INT_ENABLE), &buffer, sizeof(buffer), i2c_TIMEOUT);
        buffer &= ~1U << INTERRUPT_DATA_RDY_BIT;
        buffer |= (enable & 1U) << INTERRUPT_DATA_RDY_BIT;
        HAL_I2C_Mem_Write(i2c_, addres_, RA_INT_ENABLE, sizeof(RA_INT_ENABLE), &buffer, sizeof(buffer), i2c_TIMEOUT);
    }

    std::uint8_t MPU6050::get_int_status_register() const noexcept
    {
        std::uint8_t buffer;
        HAL_I2C_Mem_Read(i2c_, addres_, RA_INT_STATUS, sizeof(RA_INT_STATUS), &buffer, sizeof(buffer), i2c_TIMEOUT);
        return buffer;
    }

    std::uint8_t MPU6050::get_device_id() const noexcept
    {
        std::uint8_t buffer;
        HAL_I2C_Mem_Read(i2c_, addres_, RA_WHO_AM_I, sizeof(RA_WHO_AM_I), &buffer, sizeof(buffer), i2c_TIMEOUT);
        return buffer << 1;
    }

    //
    //	motion_ functions - not included in documentation/register map
    //
    void MPU6050::set_dhpf_mode(const std::uint8_t dhpf) const noexcept
    {
        std::uint8_t buffer;
        HAL_I2C_Mem_Read(i2c_, addres_, RA_ACCEL_CONFIG, sizeof(RA_ACCEL_CONFIG), &buffer, sizeof(buffer), i2c_TIMEOUT);
        buffer &= ~0x07;
        buffer |= dhpf & 0x7;
        HAL_I2C_Mem_Write(i2c_,
                          addres_,
                          RA_ACCEL_CONFIG,
                          sizeof(RA_ACCEL_CONFIG),
                          &buffer,
                          sizeof(buffer),
                          i2c_TIMEOUT);
    }

    std::uint8_t MPU6050::get_motion_status_register() const noexcept
    {
        std::uint8_t buffer;
        HAL_I2C_Mem_Read(i2c_,
                         addres_,
                         RA_MOT_DETECT_STATUS,
                         sizeof(RA_MOT_DETECT_STATUS),
                         &buffer,
                         sizeof(buffer),
                         i2c_TIMEOUT);
        return buffer;
    }

    void MPU6050::set_int_zero_motion_enabled(const std::uint8_t enable) const noexcept
    {
        std::uint8_t buffer;
        HAL_I2C_Mem_Read(i2c_, addres_, RA_INT_ENABLE, sizeof(RA_INT_ENABLE), &buffer, sizeof(buffer), i2c_TIMEOUT);
        buffer &= ~1U << INTERRUPT_ZMOT_BIT;
        buffer |= (enable & 1U) << INTERRUPT_ZMOT_BIT;
        HAL_I2C_Mem_Write(i2c_, addres_, RA_INT_ENABLE, sizeof(RA_INT_ENABLE), &buffer, sizeof(buffer), i2c_TIMEOUT);
    }

    void MPU6050::set_int_motion_enabled(const std::uint8_t enable) const noexcept
    {
        std::uint8_t buffer;
        HAL_I2C_Mem_Read(i2c_, addres_, RA_INT_ENABLE, sizeof(RA_INT_ENABLE), &buffer, sizeof(buffer), i2c_TIMEOUT);
        buffer &= ~1U << INTERRUPT_MOT_BIT;
        buffer |= (enable & 1U) << INTERRUPT_MOT_BIT;
        HAL_I2C_Mem_Write(i2c_, addres_, RA_INT_ENABLE, sizeof(RA_INT_ENABLE), &buffer, sizeof(buffer), i2c_TIMEOUT);
    }

    void MPU6050::set_int_free_fall_enabled(const std::uint8_t enable) const noexcept
    {
        std::uint8_t buffer;
        HAL_I2C_Mem_Read(i2c_, addres_, RA_INT_ENABLE, sizeof(RA_INT_ENABLE), &buffer, sizeof(buffer), i2c_TIMEOUT);
        buffer &= ~1U << INTERRUPT_FF_BIT;
        buffer |= (enable & 1U) << INTERRUPT_FF_BIT;
        HAL_I2C_Mem_Write(i2c_, addres_, RA_INT_ENABLE, sizeof(RA_INT_ENABLE), &buffer, sizeof(buffer), i2c_TIMEOUT);
    }

    void MPU6050::set_motion_detection_threshold(std::uint8_t threshold) const noexcept
    {
        HAL_I2C_Mem_Write(i2c_, addres_, RA_MOT_THR, sizeof(RA_MOT_THR), &threshold, sizeof(threshold), i2c_TIMEOUT);
    }

    void MPU6050::set_motion_detection_duration(std::uint8_t duration) const noexcept
    {
        HAL_I2C_Mem_Write(i2c_, addres_, RA_MOT_DUR, sizeof(RA_MOT_DUR), &duration, sizeof(duration), i2c_TIMEOUT);
    }

    void MPU6050::set_zero_motion_detection_threshold(std::uint8_t threshold) const noexcept
    {
        HAL_I2C_Mem_Write(i2c_,
                          addres_,
                          RA_ZRMOT_THR,
                          sizeof(RA_ZRMOT_THR),
                          &threshold,
                          sizeof(threshold),
                          i2c_TIMEOUT);
    }

    void MPU6050::set_zero_motion_detection_duration(std::uint8_t duration) const noexcept
    {
        HAL_I2C_Mem_Write(i2c_, addres_, RA_ZRMOT_DUR, sizeof(RA_ZRMOT_DUR), &duration, sizeof(duration), i2c_TIMEOUT);
    }

    void MPU6050::set_free_fall_detection_threshold(std::uint8_t threshold) const noexcept
    {
        HAL_I2C_Mem_Write(i2c_, addres_, RA_FF_THR, sizeof(RA_FF_THR), &threshold, sizeof(threshold), i2c_TIMEOUT);
    }

    void MPU6050::set_free_fall_detection_duration(std::uint8_t duration) const noexcept
    {
        HAL_I2C_Mem_Write(i2c_, addres_, RA_FF_DUR, sizeof(RA_FF_DUR), &duration, sizeof(duration), i2c_TIMEOUT);
    }

}; // namespace InvertedSway
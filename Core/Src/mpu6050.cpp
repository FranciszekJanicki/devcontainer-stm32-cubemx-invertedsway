#include "mpu6050.hpp"
#include "common.hpp"
#include "main.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <utility>

using namespace InvertedSway;
using Scaled = MPU6050::Scaled;
using GyroScaled = MPU6050::GyroScaled;
using AccelScaled = MPU6050::AccelScaled;
using RollPitchYaw = MPU6050::RollPitchYaw;
using TempScaled = MPU6050::TempScaled;
using Raw = MPU6050::Raw;
using GyroRaw = MPU6050::GyroRaw;
using AccelRaw = MPU6050::AccelRaw;
using TempRaw = MPU6050::TempRaw;
using Address = MPU6050::Address;
using RA = MPU6050::RA;
using PWR1 = MPU6050::PWR1;
using PWR2 = MPU6050::PWR2;
using Clock = MPU6050::Clock;
using Intr = MPU6050::Intr;
using IntrLatch = MPU6050::IntrLatch;
using IntrDrive = MPU6050::IntrDrive;
using IntrMode = MPU6050::IntrMode;
using IntrCfg = MPU6050::IntrCfg;
using IntrClear = MPU6050::IntrClear;

namespace InvertedSway {

    Scaled MPU6050::gyro_range_to_scale(GyroRange const gyro_range) noexcept
    {
        switch (gyro_range) {
            case GyroRange::GYRO_FS_250:
                return 131.0f;
            case GyroRange::GYRO_FS_500:
                return 65.5f;
            case GyroRange::GYRO_FS_1000:
                return 32.8f;
            case GyroRange::GYRO_FS_2000:
                return 16.4f;
            default:
                return 0.0f;
        }
    }

    Scaled MPU6050::accel_range_to_scale(AccelRange const accel_range) noexcept
    {
        switch (accel_range) {
            case AccelRange::ACCEL_FS_2:
                return 16384.0f;
            case AccelRange::ACCEL_FS_4:
                return 8192.0f;
            case AccelRange::ACCEL_FS_8:
                return 4096.0f;
            case AccelRange::ACCEL_FS_16:
                return 2048.0f;
            default:
                return 0.0f;
        }
    }

    MPU6050::MPU6050(I2cHandle i2c,
                     Address const address,
                     GyroRange const gyro_range,
                     AccelRange const accel_range,
                     std::uint32_t const sampling_rate) noexcept :
        i2c_{i2c}, address_{std::to_underlying(address)}, gyro_range_{gyro_range}, accel_range_{accel_range}
    {
        this->initialize(sampling_rate);
    }

    MPU6050::~MPU6050() noexcept
    {
        this->deinitialize();
    }

    void MPU6050::initialize(std::uint32_t const sampling_rate) noexcept
    {
        if (!this->initialized_) {
            if (HAL_I2C_IsDeviceReady(this->i2c_, this->address_, 10, I2C_TIMEOUT) != HAL_OK) {
                printf("device is not ready\r\n");
                return;
            }

            this->set_address_pin(AD0_GPIO_Port, AD0_Pin);
            if (this->get_device_id() == this->address_) {
                this->device_reset();
                HAL_Delay(50);
                this->set_sleep_enabled(Enable::OFF);
                HAL_Delay(50);
                this->set_clock_source(Clock::INTERNAL);
                HAL_Delay(50);
                this->set_dlpf(DLPF::BW_256);
                HAL_Delay(50);
                this->set_sampling_divider(get_sampling_divider(sampling_rate, DLPF::BW_256));
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
                this->device_reset();
                this->initialized_ = false;
            }
        }
    }

    void MPU6050::set_dlpf(DLPF const value) const noexcept
    {
        std::uint8_t buffer{std::to_underlying(value) & 0x7};
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          std::to_underlying(RA::CONFIG),
                          sizeof(RA::CONFIG),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_address_pin(GpioHandle gpio, std::uint16_t const address_pin) const noexcept
    {
        if (this->address_ == std::to_underlying(Address::ADDRESS2)) {
            HAL_GPIO_WritePin(gpio, address_pin, GPIO_PinState::GPIO_PIN_RESET);
        } else if (this->address_ == std::to_underlying(Address::ADDRESS2)) {
            HAL_GPIO_WritePin(gpio, address_pin, GPIO_PinState::GPIO_PIN_SET);
        }
    }

    void MPU6050::device_reset() const noexcept
    {
        std::uint8_t buffer{1U << std::to_underlying(PWR1::DEVICE_RESET_BIT)};
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          std::to_underlying(RA::PWR_MGMT_1),
                          sizeof(RA::PWR_MGMT_1),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    std::uint32_t MPU6050::get_sampling_divider(std::uint32_t const rate, DLPF const dlpf) noexcept
    {
        if (dlpf == DLPF::BW_256) {
            return (GYRO_OUTPUT_RATE_DLPF_DIS_HZ / rate) - 1U;
        } else {
            return (GYRO_OUTPUT_RATE_DLPF_EN_HZ / rate) - 1U;
        }
    }

    void MPU6050::set_sampling_divider(std::uint8_t const divider) const noexcept
    {
        std::uint8_t buffer{divider};
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          std::to_underlying(RA::SMPLRT_DIV),
                          sizeof(RA::SMPLRT_DIV),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_sleep_enabled(Enable const enable) const noexcept
    {
        std::uint8_t buffer{std::to_underlying(enable) << std::to_underlying(PWR1::SLEEP_BIT)};
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          std::to_underlying(RA::PWR_MGMT_1),
                          sizeof(RA::PWR_MGMT_1),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_cycle_enabled(Enable const enable) const noexcept
    {
        std::uint8_t buffer{std::to_underlying(enable) << std::to_underlying(PWR1::CYCLE_BIT)};
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          std::to_underlying(RA::PWR_MGMT_1),
                          sizeof(RA::PWR_MGMT_1),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_temperature_sensor_enabled(Enable const enable) const noexcept
    {
        std::uint8_t buffer{(!std::to_underlying(enable)) << std::to_underlying(PWR1::TEMP_DIS_BIT)};
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          std::to_underlying(RA::PWR_MGMT_1),
                          sizeof(RA::PWR_MGMT_1),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_clock_source(Clock const source) const noexcept
    {
        std::uint8_t buffer{(std::to_underlying(source) & 0x7)};
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          std::to_underlying(RA::PWR_MGMT_1),
                          sizeof(RA::PWR_MGMT_1),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_low_power_wake_up_frequency(WakeFreq const frequency) const noexcept
    {
        std::uint8_t buffer{(std::to_underlying(frequency) & 0x3) << std::to_underlying(PWR2::LP_WAKE_CTRL_BIT)};
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          std::to_underlying(RA::PWR_MGMT_2),
                          sizeof(RA::PWR_MGMT_2),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::accelerometer_axis_standby(std::uint8_t const x_accel_standby,
                                             std::uint8_t const y_accel_standby,
                                             std::uint8_t const z_accel_standby) const noexcept
    {
        std::uint8_t buffer{((x_accel_standby & 1U) << std::to_underlying(PWR2::STBY_XA_BIT)) |
                            ((y_accel_standby & 1U) << std::to_underlying(PWR2::STBY_YA_BIT)) |
                            ((z_accel_standby & 1U) << std::to_underlying(PWR2::STBY_ZA_BIT))};
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          std::to_underlying(RA::PWR_MGMT_2),
                          sizeof(RA::PWR_MGMT_2),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::gyroscope_axis_standby(std::uint8_t const x_gyro_standby,
                                         std::uint8_t const y_gyro_standby,
                                         std::uint8_t const z_gyro_standby) const noexcept
    {
        std::uint8_t buffer{((x_gyro_standby & 1U) << std::to_underlying(PWR2::STBY_XG_BIT)) |
                            ((y_gyro_standby & 1U) << std::to_underlying(PWR2::STBY_YG_BIT)) |
                            ((z_gyro_standby & 1U) << std::to_underlying(PWR2::STBY_ZG_BIT))};
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          std::to_underlying(RA::PWR_MGMT_2),
                          sizeof(RA::PWR_MGMT_2),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_full_scale_gyro_range(GyroRange const range) const noexcept
    {
        std::uint8_t buffer{(std::to_underlying(range) & 0x7) << 3};
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          std::to_underlying(RA::GYRO_CONFIG),
                          sizeof(RA::GYRO_CONFIG),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_full_scale_accel_range(AccelRange const range) const noexcept
    {
        std::uint8_t buffer{(std::to_underlying(range) & 0x7) << 3};
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          std::to_underlying(RA::ACCEL_CONFIG),
                          sizeof(RA::ACCEL_CONFIG),
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
                         std::to_underlying(RA::TEMP_OUT_H),
                         sizeof(RA::TEMP_OUT_H),
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
                         std::to_underlying(RA::ACCEL_XOUT_H),
                         sizeof(RA::ACCEL_XOUT_H),
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
                         std::to_underlying(RA::ACCEL_YOUT_H),
                         sizeof(RA::ACCEL_YOUT_H),
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
                         std::to_underlying(RA::ACCEL_ZOUT_H),
                         sizeof(RA::ACCEL_ZOUT_H),
                         buffer,
                         sizeof(buffer),
                         I2C_TIMEOUT);
        return ((static_cast<Raw>(buffer[0])) << 8) | buffer[1];
    }

    Scaled MPU6050::get_acceleration_x_scaled() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        return static_cast<Scaled>(this->get_acceleration_x_raw()) / accel_range_to_scale(this->accel_range_);
    }

    Scaled MPU6050::get_acceleration_y_scaled() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        return static_cast<Scaled>(this->get_acceleration_y_raw()) / accel_range_to_scale(this->accel_range_);
    }

    Scaled MPU6050::get_acceleration_z_scaled() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        return static_cast<Scaled>(this->get_acceleration_z_raw()) * accel_range_to_scale(this->accel_range_);
    }

    AccelRaw MPU6050::get_accelerometer_raw() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        std::uint8_t buffer[6];
        HAL_I2C_Mem_Read(this->i2c_,
                         this->address_,
                         std::to_underlying(RA::ACCEL_XOUT_H),
                         sizeof(RA::ACCEL_XOUT_H),
                         buffer,
                         sizeof(buffer),
                         I2C_TIMEOUT);

        return AccelRaw{((static_cast<Raw>(buffer[0])) << 8) | buffer[1],
                        ((static_cast<Raw>(buffer[2])) << 8) | buffer[3],
                        ((static_cast<Raw>(buffer[4])) << 8) | buffer[5]};
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
                         std::to_underlying(RA::GYRO_XOUT_H),
                         sizeof(RA::GYRO_XOUT_H),
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
                         std::to_underlying(RA::GYRO_YOUT_H),
                         sizeof(RA::GYRO_YOUT_H),
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
                         std::to_underlying(RA::GYRO_ZOUT_H),
                         sizeof(RA::GYRO_ZOUT_H),
                         buffer,
                         sizeof(buffer),
                         I2C_TIMEOUT);

        return ((static_cast<Raw>(buffer[0])) << 8) | buffer[1];
    }

    Scaled MPU6050::get_rotation_x_scaled() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        return static_cast<Scaled>(this->get_rotation_x_raw()) / gyro_range_to_scale(this->gyro_range_);
    }

    Scaled MPU6050::get_rotation_y_scaled() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        return static_cast<Scaled>(this->get_rotation_y_raw()) / gyro_range_to_scale(this->gyro_range_);
    }

    Scaled MPU6050::get_rotation_z_scaled() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        return static_cast<Scaled>(this->get_rotation_x_raw()) * gyro_range_to_scale(this->gyro_range_);
    }

    GyroRaw MPU6050::get_gyroscope_raw() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        std::uint8_t buffer[6];
        HAL_I2C_Mem_Read(this->i2c_,
                         this->address_,
                         std::to_underlying(RA::GYRO_XOUT_H),
                         sizeof(RA::GYRO_XOUT_H),
                         buffer,
                         sizeof(buffer),
                         I2C_TIMEOUT);

        return GyroRaw{((static_cast<Raw>(buffer[0])) << 8) | buffer[1],
                       ((static_cast<Raw>(buffer[2])) << 8) | buffer[3],
                       ((static_cast<Raw>(buffer[4])) << 8) | buffer[5]};
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

    Scaled MPU6050::get_roll() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        const auto accel_scaled{get_accelerometer_scaled()};

        return std::atan2(accel_scaled.y, accel_scaled.z) * 180.0 / M_PI;
    }

    Scaled MPU6050::get_pitch() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        const auto accel_scaled{get_accelerometer_scaled()};

        return -(std::atan2(accel_scaled.x,
                            std::sqrt(accel_scaled.y * accel_scaled.y + accel_scaled.z * accel_scaled.z)) *
                 180.0) /
               M_PI;
    }

    Scaled MPU6050::get_yaw() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        const auto accel_scaled{get_accelerometer_scaled()};

        return {};
    }

    void MPU6050::set_interrupt() const noexcept
    {
        this->set_interrupt_mode(IntrMode::ACTIVEHIGH);
        // this->set_interrupt_drive(IntrDrive::PUSHPULL);
        this->set_interrupt_latch(IntrLatch::PULSE50US);
        this->set_interrupt_latch_clear(IntrClear::STATUSREAD);
        this->set_int_enable_register(Enable::ON);

        // Enable Motion interrputs
        // this->set_dhpf_mode(DHPF_5);
        // this->set_int_motion_enabled(Enable::ON);
        // this->set_int_zero_motion_enabled(1);
        // this->set_int_free_fall_enabled(1);
        // this->set_free_fall_detection_duration(2);
        // this->set_free_fall_detection_threshold(5);
        // this->set_motion_detection_duration(5);
        // this->set_motion_detection_threshold(2);
        // this->set_zero_motion_detection_duration(2);
        // this->set_zero_motion_detection_threshold(4);
    }

    void MPU6050::set_interrupt_mode(IntrMode const mode) const noexcept
    {
        std::uint8_t buffer{(std::to_underlying(mode) & 1U) << std::to_underlying(IntrCfg::INT_LEVEL_BIT)};
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          std::to_underlying(RA::INT_PIN_CFG),
                          sizeof(RA::INT_PIN_CFG),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_interrupt_drive(IntrDrive const drive) const noexcept
    {
        std::uint8_t buffer{(std::to_underlying(drive) & 1U) << std::to_underlying(IntrCfg::INT_OPEN_BIT)};
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          std::to_underlying(RA::INT_PIN_CFG),
                          sizeof(RA::INT_PIN_CFG),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_interrupt_latch(IntrLatch const latch) const noexcept
    {
        std::uint8_t buffer{(std::to_underlying(latch) & 1U) << std::to_underlying(IntrCfg::INT_RD_CLEAR_BIT)};
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          std::to_underlying(RA::INT_PIN_CFG),
                          sizeof(RA::INT_PIN_CFG),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_interrupt_latch_clear(IntrClear const clear) const noexcept
    {
        std::uint8_t buffer{(std::to_underlying(clear) & 1U) << std::to_underlying(IntrCfg::LATCH_INT_EN_BIT)};
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          std::to_underlying(RA::INT_PIN_CFG),
                          sizeof(RA::INT_PIN_CFG),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_int_enable_register(Enable const enable) const noexcept
    {
        std::uint8_t buffer{std::to_underlying(enable)};
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          std::to_underlying(RA::INT_ENABLE),
                          sizeof(RA::INT_ENABLE),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_int_data_ready_enabled(Enable const enable) const noexcept
    {
        std::uint8_t buffer{std::to_underlying(enable) << std::to_underlying(Intr::DATA_RDY_BIT)};
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          std::to_underlying(RA::INT_ENABLE),
                          sizeof(RA::INT_ENABLE),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    std::uint8_t MPU6050::get_int_status_register() const noexcept
    {
        std::uint8_t buffer;
        HAL_I2C_Mem_Read(this->i2c_,
                         this->address_,
                         std::to_underlying(RA::INT_STATUS),
                         sizeof(RA::INT_STATUS),
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
                         std::to_underlying(RA::WHO_AM_I),
                         sizeof(RA::WHO_AM_I),
                         &buffer,
                         sizeof(buffer),
                         I2C_TIMEOUT);
        return buffer << 1;
    }

    void MPU6050::set_dhpf_mode(DHPF const dhpf) const noexcept
    {
        std::uint8_t buffer{std::to_underlying(dhpf) & 0x7};
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          std::to_underlying(RA::ACCEL_CONFIG),
                          sizeof(RA::ACCEL_CONFIG),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    std::uint8_t MPU6050::get_motion_status_register() const noexcept
    {
        std::uint8_t buffer;
        HAL_I2C_Mem_Read(this->i2c_,
                         this->address_,
                         std::to_underlying(RA::MOT_DETECT_STATUS),
                         sizeof(RA::MOT_DETECT_STATUS),
                         &buffer,
                         sizeof(buffer),
                         I2C_TIMEOUT);
        return buffer;
    }

    void MPU6050::set_int_zero_motion_enabled(Enable const enable) const noexcept
    {
        std::uint8_t buffer{std::to_underlying(enable) << std::to_underlying(Intr::ZMOT_BIT)};
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          std::to_underlying(RA::INT_ENABLE),
                          sizeof(RA::INT_ENABLE),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_int_motion_enabled(Enable const enable) const noexcept
    {
        std::uint8_t buffer{std::to_underlying(enable) << std::to_underlying(Intr::MOT_BIT)};
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          std::to_underlying(RA::INT_ENABLE),
                          sizeof(RA::INT_ENABLE),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_int_free_fall_enabled(Enable const enable) const noexcept
    {
        std::uint8_t buffer{std::to_underlying(enable) << std::to_underlying(Intr::FF_BIT)};
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          std::to_underlying(RA::INT_ENABLE),
                          sizeof(RA::INT_ENABLE),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_motion_detection_threshold(std::uint8_t const threshold) const noexcept
    {
        std::uint8_t buffer{threshold};
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          std::to_underlying(RA::MOT_THR),
                          sizeof(RA::MOT_THR),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_motion_detection_duration(std::uint8_t const duration) const noexcept
    {
        std::uint8_t buffer{duration};
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          std::to_underlying(RA::MOT_DUR),
                          sizeof(RA::MOT_DUR),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_zero_motion_detection_threshold(std::uint8_t const threshold) const noexcept
    {
        std::uint8_t buffer{threshold};
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          std::to_underlying(RA::ZRMOT_THR),
                          sizeof(RA::ZRMOT_THR),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_zero_motion_detection_duration(std::uint8_t const duration) const noexcept
    {
        std::uint8_t buffer{duration};
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          std::to_underlying(RA::ZRMOT_DUR),
                          sizeof(RA::ZRMOT_DUR),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_free_fall_detection_threshold(std::uint8_t const threshold) const noexcept
    {
        std::uint8_t buffer{threshold};
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          std::to_underlying(RA::FF_THR),
                          sizeof(RA::FF_THR),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

    void MPU6050::set_free_fall_detection_duration(std::uint8_t const duration) const noexcept
    {
        std::uint8_t buffer{duration};
        HAL_I2C_Mem_Write(this->i2c_,
                          this->address_,
                          std::to_underlying(RA::FF_DUR),
                          sizeof(RA::FF_DUR),
                          &buffer,
                          sizeof(buffer),
                          I2C_TIMEOUT);
    }

}; // namespace InvertedSway
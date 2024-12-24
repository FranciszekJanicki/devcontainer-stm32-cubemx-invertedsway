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
using DevAddress = MPU6050::DevAddress;
using RegAddress = MPU6050::RegAddress;
using Power1 = MPU6050::Power1;
using Power2 = MPU6050::Power2;
using Clock = MPU6050::Clock;
using Interrupt = MPU6050::Interrupt;
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

    std::uint8_t MPU6050::get_sampling_divider(std::uint32_t const rate, DLPF const dlpf) noexcept
    {
        if (dlpf == DLPF::BW_256) {
            return static_cast<std::uint8_t>((GYRO_OUTPUT_RATE_DLPF_DIS_HZ / rate) - 1U);
        } else {
            return static_cast<std::uint8_t>((GYRO_OUTPUT_RATE_DLPF_EN_HZ / rate) - 1U);
        }
    }

    MPU6050::MPU6050(I2CHandle const i2c_bus,
                     DevAddress const device_address,
                     GyroRange const gyro_range,
                     AccelRange const accel_range,
                     std::uint32_t const sampling_rate) noexcept :
        i2c_bus_{i2c_bus}, device_address_{device_address}, gyro_range_{gyro_range}, accel_range_{accel_range}
    {
        this->initialize(sampling_rate);
    }

    MPU6050::~MPU6050() noexcept
    {
        this->deinitialize();
    }

    TempScaled MPU6050::get_temperature_celsius() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        return static_cast<TempScaled>(this->get_temperature_raw()) / 340 + 36.53;
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

    AccelScaled MPU6050::get_acceleration_scaled() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        const auto accel_scale{accel_range_to_scale(this->accel_range_)};
        const auto accel_raw_result{this->get_acceleration_raw()};

        return AccelScaled{static_cast<Scaled>(accel_raw_result.x) / accel_scale,
                           static_cast<Scaled>(accel_raw_result.y) / accel_scale,
                           static_cast<Scaled>(accel_raw_result.z) / accel_scale};
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

    GyroScaled MPU6050::get_rotation_scaled() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        const auto gyro_scale{gyro_range_to_scale(this->gyro_range_)};
        const auto gyro_raw{this->get_rotation_raw()};

        return GyroScaled{static_cast<Scaled>(gyro_raw.x) / gyro_scale,
                          static_cast<Scaled>(gyro_raw.y) / gyro_scale,
                          static_cast<Scaled>(gyro_raw.z) / gyro_scale};
    }

    RollPitchYaw MPU6050::get_roll_pitch_yaw() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        const auto accel_scaled{get_acceleration_scaled()};

        return RollPitchYaw{
            std::atan2(accel_scaled.y, accel_scaled.z) * 180.0f / PI,
            -(std::atan2(accel_scaled.x, std::sqrt(accel_scaled.y * accel_scaled.y + accel_scaled.z * accel_scaled.z)) *
              180.0) /
                PI,
            {}};
    }

    Scaled MPU6050::get_roll() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }
        const auto accel_scaled{get_acceleration_scaled()};

        return std::atan2(accel_scaled.y, accel_scaled.z) * 180.0f / PI;
    }

    Scaled MPU6050::get_pitch() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        const auto accel_scaled{get_acceleration_scaled()};

        return -(std::atan2(accel_scaled.x,
                            std::sqrt(accel_scaled.y * accel_scaled.y + accel_scaled.z * accel_scaled.z)) *
                 180.0f) /
               PI;
    }

    Scaled MPU6050::get_yaw() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        const auto accel_scaled{get_acceleration_scaled()};

        return {};
    }

    void
    MPU6050::i2c_write_bytes(RegAddress const reg_address, std::uint8_t* data, std::size_t const bytes) const noexcept
    {
        HAL_I2C_Mem_Write(this->i2c_bus_,
                          std::to_underlying(this->device_address_) << 1,
                          std::to_underlying(reg_address),
                          sizeof(reg_address),
                          data,
                          bytes,
                          I2C_TIMEOUT);
    }

    void MPU6050::i2c_write_byte(RegAddress const reg_address, std::uint8_t data) const noexcept
    {
        HAL_I2C_Mem_Write(this->i2c_bus_,
                          std::to_underlying(this->device_address_) << 1,
                          std::to_underlying(reg_address),
                          sizeof(reg_address),
                          &data,
                          sizeof(data),
                          I2C_TIMEOUT);
    }

    void MPU6050::i2c_write_bit(RegAddress const reg_address, std::uint8_t const bit) const noexcept
    {
        std::uint8_t data{1U << bit};
        HAL_I2C_Mem_Read(this->i2c_bus_,
                         std::to_underlying(this->device_address_) << 1,
                         std::to_underlying(reg_address),
                         sizeof(reg_address),
                         &data,
                         sizeof(data),
                         I2C_TIMEOUT);
    }

    void
    MPU6050::i2c_read_bytes(RegAddress const reg_address, std::uint8_t* data, std::size_t const bytes) const noexcept
    {
        HAL_I2C_Mem_Read(this->i2c_bus_,
                         std::to_underlying(this->device_address_) << 1,
                         std::to_underlying(reg_address),
                         sizeof(reg_address),
                         data,
                         bytes,
                         I2C_TIMEOUT);
    }

    std::uint8_t MPU6050::i2c_read_byte(RegAddress const reg_address) const noexcept
    {
        std::uint8_t data;
        HAL_I2C_Mem_Read(this->i2c_bus_,
                         std::to_underlying(this->device_address_) << 1,
                         std::to_underlying(reg_address),
                         sizeof(reg_address),
                         &data,
                         sizeof(data),
                         I2C_TIMEOUT);
        return data;
    }

    bool MPU6050::i2c_read_bit(RegAddress const reg_address, std::uint8_t const bit) const noexcept
    {
        std::uint8_t data;
        HAL_I2C_Mem_Read(this->i2c_bus_,
                         std::to_underlying(this->device_address_) << 1,
                         std::to_underlying(reg_address),
                         sizeof(reg_address),
                         &data,
                         sizeof(data),
                         I2C_TIMEOUT);
        return data & (1U << bit);
    }

    void MPU6050::initialize(std::uint32_t const sampling_rate) noexcept
    {
        if (this->get_device_id() == std::to_underlying(this->device_address_)) {
            this->device_reset();
            HAL_Delay(50);
            this->set_sleep_enabled(false);
            HAL_Delay(50);
            this->set_clock_source(Clock::PLL_XGYRO);
            HAL_Delay(50);
            this->set_sampling_divider(get_sampling_divider(sampling_rate, DLPF::BW_256));
            HAL_Delay(50);
            this->set_dlpf_mode(DLPF::BW_256);
            HAL_Delay(50);
            this->set_full_scale_gyro_range(this->gyro_range_);
            HAL_Delay(50);
            this->set_full_scale_accel_range(this->accel_range_);
            HAL_Delay(50);
            this->set_interrupt();
            this->initialized_ = true;
        }
    }

    void MPU6050::deinitialize() noexcept
    {
        if (this->get_device_id() == std::to_underlying(this->device_address_)) {
            this->device_reset();
            this->initialized_ = false;
        }
    }

    /* SMPLRD_DIV REGISTER */
    void MPU6050::set_sampling_divider(std::uint8_t const divider) const noexcept
    {
        this->i2c_write_byte(RegAddress::SMPLRT_DIV, divider);
    }

    /* CONFIG REGISTER */
    void MPU6050::set_external_frame_sync(std::uint8_t const frame_sync) const noexcept
    {}

    void MPU6050::set_dlpf_mode(DLPF const dlpf) const noexcept
    {
        this->i2c_write_byte(RegAddress::CONFIG, std::to_underlying(dlpf) & 0x7);
    }

    /* GYRO_CONFIG REGISTER */
    void MPU6050::set_full_scale_gyro_range(GyroRange const range) const noexcept
    {
        this->i2c_write_byte(RegAddress::GYRO_CONFIG, (std::to_underlying(range) & 0x7) << 3);
    }

    /* ACCEL_CONFIG REGISTER */
    void MPU6050::set_full_scale_accel_range(AccelRange const range) const noexcept
    {
        this->i2c_write_byte(RegAddress::ACCEL_CONFIG, (std::to_underlying(range) & 0x7) << 3);
    }

    void MPU6050::set_dhpf_mode(DHPF const dhpf) const noexcept
    {
        this->i2c_write_byte(RegAddress::ACCEL_CONFIG, std::to_underlying(dhpf) & 0x7);
    }

    /* FF_THR REGISTER */
    void MPU6050::set_free_fall_detection_threshold(std::uint8_t const threshold) const noexcept
    {
        this->i2c_write_byte(RegAddress::FF_THR, threshold);
    }

    /* FF_DUR REGISTER */
    void MPU6050::set_free_fall_detection_duration(std::uint8_t const duration) const noexcept
    {
        this->i2c_write_byte(RegAddress::FF_DUR, duration);
    }

    /* MOT_THR REGISTER */
    void MPU6050::set_motion_detection_threshold(std::uint8_t const threshold) const noexcept
    {
        this->i2c_write_byte(RegAddress::MOT_THR, threshold);
    }

    /* MOT_DUR REGISTER */
    void MPU6050::set_motion_detection_duration(std::uint8_t const duration) const noexcept
    {
        this->i2c_write_byte(RegAddress::MOT_DUR, duration);
    }

    /* ZRMOT_THR REGISTER */
    void MPU6050::set_zero_motion_detection_threshold(std::uint8_t const threshold) const noexcept
    {
        this->i2c_write_byte(RegAddress::ZRMOT_THR, threshold);
    }

    /* ZRMOT_DUR REGISTER */
    void MPU6050::set_zero_motion_detection_duration(std::uint8_t const duration) const noexcept
    {
        this->i2c_write_byte(RegAddress::ZRMOT_DUR, duration);
    }

    /* FIFO_EN REGISTER */
    void MPU6050::set_temp_fifo_enabled(bool const temp_fifo) const noexcept
    {}

    void MPU6050::set_gyro_fifo_enabled(bool const x_fifo, bool const y_fifo, bool const z_fifo) const noexcept
    {}

    void MPU6050::set_accel_fifo_enabled(bool const x_fifo, bool const y_fifo, bool const z_fifo) const noexcept
    {}

    /* INT_PIN_CFG REGISTER */
    void MPU6050::set_interrupt() const noexcept
    {
        this->set_interrupt_mode(IntrMode::ACTIVEHIGH);
        // this->set_interrupt_drive(IntrDrive::PUSHPULL);
        this->set_interrupt_latch(IntrLatch::PULSE50US);
        this->set_interrupt_latch_clear(IntrClear::ANYREAD);
        this->set_int_enabled(true);
    }

    void MPU6050::set_interrupt_mode(IntrMode const mode) const noexcept
    {
        this->i2c_write_byte(RegAddress::INT_PIN_CFG,
                             std::to_underlying(mode) << std::to_underlying(IntrCfg::INT_LEVEL_BIT));
    }

    void MPU6050::set_interrupt_drive(IntrDrive const drive) const noexcept
    {
        this->i2c_write_byte(RegAddress::INT_PIN_CFG,
                             std::to_underlying(drive) << std::to_underlying(IntrCfg::INT_OPEN_BIT));
    }

    void MPU6050::set_interrupt_latch(IntrLatch const latch) const noexcept
    {
        this->i2c_write_byte(RegAddress::INT_PIN_CFG,
                             std::to_underlying(latch) << std::to_underlying(IntrCfg::INT_RD_CLEAR_BIT));
    }

    void MPU6050::set_interrupt_latch_clear(IntrClear const clear) const noexcept
    {
        this->i2c_write_byte(RegAddress::INT_PIN_CFG,
                             std::to_underlying(clear) << std::to_underlying(IntrCfg::LATCH_INT_EN_BIT));
    }

    /* INT_ENABLE REGISTER */
    void MPU6050::set_motion_interrupt() const noexcept
    {
        // Motion interrputs
        // this->set_dhpf_mode(DHPF_5);
        // this->set_int_motion_enabled(true);
        // this->set_int_zero_motion_enabled(true);
        // this->set_int_free_fall_enabled(true);
        // this->set_free_fall_detection_duration(2);
        // this->set_free_fall_detection_threshold(5);
        // this->set_motion_detection_duration(5);
        // this->set_motion_detection_threshold(2);
        // this->set_zero_motion_detection_duration(2);
        // this->set_zero_motion_detection_threshold(4);
    }

    void MPU6050::set_int_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_byte(RegAddress::INT_ENABLE, enabled);
    }

    void MPU6050::set_int_data_ready_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_byte(RegAddress::INT_ENABLE, enabled << std::to_underlying(Interrupt::DATA_RDY_BIT));
    }

    void MPU6050::set_int_zero_motion_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_byte(RegAddress::INT_ENABLE, enabled << std::to_underlying(Interrupt::ZMOT_BIT));
    }

    void MPU6050::set_int_motion_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_byte(RegAddress::INT_ENABLE, enabled << std::to_underlying(Interrupt::MOT_BIT));
    }

    void MPU6050::set_int_free_fall_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_byte(RegAddress::INT_ENABLE, enabled << std::to_underlying(Interrupt::FF_BIT));
    }

    void MPU6050::set_int_fifo_overflow_enabled(bool const enabled) const noexcept
    {}

    void MPU6050::set_int_i2c_master_enabled(bool const enabled) const noexcept
    {}

    /* INT_STATUS REGISTER */
    std::uint8_t MPU6050::get_int_status() const noexcept
    {
        return this->i2c_read_bit(RegAddress::INT_STATUS, std::to_underlying(Interrupt::FF_BIT));
    }

    std::uint8_t MPU6050::get_int_free_fall_status() const noexcept
    {
        return this->i2c_read_byte(RegAddress::INT_STATUS);
    }

    std::uint8_t MPU6050::get_int_motion_status() const noexcept
    {
        return this->i2c_read_byte(RegAddress::MOT_DETECT_STATUS);
    }

    std::uint8_t MPU6050::get_int_zero_motion_status() const noexcept
    {}

    std::uint8_t MPU6050::get_int_fifo_overflow_status() const noexcept
    {}

    std::uint8_t MPU6050::get_int_i2c_master_status() const noexcept
    {}

    std::uint8_t MPU6050::get_int_data_ready_status() const noexcept
    {}

    /* ACCEL_OUT REGISTERS */
    AccelRaw MPU6050::get_acceleration_raw() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        std::uint8_t buffer[6];
        this->i2c_read_bytes(RegAddress::ACCEL_XOUT_H, buffer, sizeof(buffer));

        return AccelRaw{static_cast<Raw>((static_cast<Raw>(buffer[0]) << 8) | static_cast<Raw>(buffer[1])),
                        static_cast<Raw>((static_cast<Raw>(buffer[2]) << 8) | static_cast<Raw>(buffer[3])),
                        static_cast<Raw>((static_cast<Raw>(buffer[4]) << 8) | static_cast<Raw>(buffer[5]))};
    }

    Raw MPU6050::get_acceleration_x_raw() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        std::uint8_t buffer[2];
        this->i2c_read_bytes(RegAddress::ACCEL_XOUT_H, buffer, sizeof(buffer));

        return ((static_cast<Raw>(buffer[0])) << 8) | buffer[1];
    }

    Raw MPU6050::get_acceleration_y_raw() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        std::uint8_t buffer[2];
        this->i2c_read_bytes(RegAddress::ACCEL_YOUT_H, buffer, sizeof(buffer));

        return ((static_cast<Raw>(buffer[0])) << 8) | buffer[1];
    }

    Raw MPU6050::get_acceleration_z_raw() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        std::uint8_t buffer[2];
        this->i2c_read_bytes(RegAddress::ACCEL_ZOUT_H, buffer, sizeof(buffer));

        return ((static_cast<Raw>(buffer[0])) << 8) | buffer[1];
    }

    /* TEMP_OUT REGISTERS */
    TempRaw MPU6050::get_temperature_raw() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        std::uint8_t buffer[2];
        this->i2c_read_bytes(RegAddress::TEMP_OUT_H, buffer, sizeof(buffer));

        return ((static_cast<TempRaw>(buffer[0])) << 8) | buffer[1];
    }

    /* GYRO_OUT REGISTERS */
    GyroRaw MPU6050::get_rotation_raw() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        std::uint8_t buffer[6];
        this->i2c_read_bytes(RegAddress::GYRO_XOUT_H, buffer, sizeof(buffer));

        return GyroRaw{static_cast<Raw>((static_cast<Raw>(buffer[0]) << 8) | static_cast<Raw>(buffer[1])),
                       static_cast<Raw>((static_cast<Raw>(buffer[2]) << 8) | static_cast<Raw>(buffer[3])),
                       static_cast<Raw>((static_cast<Raw>(buffer[4]) << 8) | static_cast<Raw>(buffer[5]))};
    }

    Raw MPU6050::get_rotation_x_raw() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        std::uint8_t buffer[2];
        this->i2c_read_bytes(RegAddress::GYRO_XOUT_H, buffer, sizeof(buffer));

        return ((static_cast<Raw>(buffer[0])) << 8) | buffer[1];
    }

    Raw MPU6050::get_rotation_y_raw() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        std::uint8_t buffer[2];
        this->i2c_read_bytes(RegAddress::GYRO_YOUT_H, buffer, sizeof(buffer));

        return ((static_cast<Raw>(buffer[0])) << 8) | buffer[1];
    }

    Raw MPU6050::get_rotation_z_raw() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        std::uint8_t buffer[2];
        this->i2c_read_bytes(RegAddress::GYRO_ZOUT_H, buffer, sizeof(buffer));

        return ((static_cast<Raw>(buffer[0])) << 8) | buffer[1];
    }

    /* MOT_DETECT_STATUS REGISTER */
    std::uint8_t MPU6050::get_motion_status() const noexcept
    {}

    bool MPU6050::get_x_neg_motion_detected() const noexcept
    {}

    bool MPU6050::get_x_pos_motion_detected() const noexcept
    {}

    bool MPU6050::get_y_neg_motion_detected() const noexcept
    {}

    bool MPU6050::get_y_pos_motion_detected() const noexcept
    {}

    bool MPU6050::get_z_neg_motion_detected() const noexcept
    {}

    bool MPU6050::get_z_pos_motion_detected() const noexcept
    {}

    /* SIGNAL_PATH_RESET REGISTER */
    void MPU6050::reset_gyro_path() const noexcept
    {}

    void MPU6050::reset_accel_path() const noexcept
    {}

    void MPU6050::reset_temperature_path() const noexcept
    {}

    /* MOT_DETECT_CTRL REGISTER */
    void MPU6050::set_accel_power_on_delay(Delay const delay) const noexcept
    {}

    void MPU6050::set_free_fall_detection_counter_decrement(DetectDecrement const decrement) const noexcept
    {}

    void MPU6050::set_motion_detection_counter_decrement(DetectDecrement const decrement) const noexcept
    {}

    /* USER_CTRL REGISTER */
    void MPU6050::set_i2c_master_mode_enabled(bool const enabled) const noexcept
    {}

    void MPU6050::set_fifo_enabled(bool const enabled) const noexcept
    {}

    void MPU6050::set_spi_enabled(bool const enabled) const noexcept
    {}

    void MPU6050::reset_fifo() const noexcept
    {}

    void MPU6050::reset_i2c_master() const noexcept
    {}

    void MPU6050::reset_sensors() const noexcept
    {}

    /* PWR_MGMT_1 REGISTER */
    void MPU6050::device_reset() const noexcept
    {
        this->i2c_write_byte(RegAddress::PWR_MGMT_1, 1U << std::to_underlying(Power1::DEVICE_RESET_BIT));
    }

    void MPU6050::set_sleep_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_byte(RegAddress::PWR_MGMT_1, enabled << std::to_underlying(Power1::SLEEP_BIT));
    }

    void MPU6050::set_wake_cycle_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_byte(RegAddress::PWR_MGMT_1, enabled << std::to_underlying(Power1::CYCLE_BIT));
    }

    void MPU6050::set_temperature_sensor_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_byte(RegAddress::PWR_MGMT_1, (!enabled) << std::to_underlying(Power1::TEMP_DIS_BIT));
    }

    void MPU6050::set_clock_source(Clock const source) const noexcept
    {
        this->i2c_write_byte(RegAddress::PWR_MGMT_1, (std::to_underlying(source) & 0x7));
    }

    /* PWR_MGMT_2 REGISTER */
    void MPU6050::set_wake_up_frequency(WakeFreq const frequency) const noexcept
    {
        this->i2c_write_byte(RegAddress::PWR_MGMT_2,
                             (std::to_underlying(frequency) & 0x3) << std::to_underlying(Power2::LP_WAKE_CTRL_BIT));
    }

    void
    MPU6050::set_accel_axis_standby(bool const x_standby, bool const y_standby, bool const z_standby) const noexcept
    {
        this->i2c_write_byte(RegAddress::PWR_MGMT_2,
                             (x_standby << std::to_underlying(Power2::STBY_XA_BIT)) |
                                 (y_standby << std::to_underlying(Power2::STBY_YA_BIT)) |
                                 (z_standby << std::to_underlying(Power2::STBY_ZA_BIT)));
    }

    void MPU6050::set_gyro_axis_standby(bool const x_standby, bool const y_standby, bool const z_standby) const noexcept
    {
        this->i2c_write_byte(RegAddress::PWR_MGMT_2,
                             (x_standby << std::to_underlying(Power2::STBY_XG_BIT)) |
                                 (y_standby << std::to_underlying(Power2::STBY_YG_BIT)) |
                                 (z_standby << std::to_underlying(Power2::STBY_ZG_BIT)));
    }

    /* FIFO_COUNT REGISTERS */
    std::uint16_t MPU6050::get_fifo_count() const noexcept
    {
        std::uint8_t buffer[2];
        this->i2c_read_bytes(RegAddress::FIFO_COUNTH, buffer, sizeof(buffer));

        return ((static_cast<std::uint16_t>(buffer[0]) << 8) | buffer[1]);
    }

    /* FIFO_R_W REGISTER */
    std::uint8_t MPU6050::get_fifo_byte() const noexcept
    {
        return this->i2c_read_byte(RegAddress::FIFO_R_W);
    }

    void MPU6050::get_fifo_bytes(std::uint8_t* data, std::size_t const bytes) const noexcept
    {
        return this->i2c_read_bytes(RegAddress::FIFO_R_W, data, bytes);
    }

    void MPU6050::set_fifo_byte(std::uint8_t const data) const noexcept
    {
        this->i2c_write_byte(RegAddress::FIFO_R_W, data);
    }

    void MPU6050::set_fifo_bytes(std::uint8_t* data, std::size_t const bytes) const noexcept
    {}

    /* WHO_AM_I REGISTER */
    std::uint8_t MPU6050::get_device_id() const noexcept
    {
        return this->i2c_read_byte(RegAddress::WHO_AM_I);
    }

    /* XG_OFFS_TC REGISTER */
    std::uint8_t MPU6050::get_otp_bank_valid() const noexcept
    {}

    void MPU6050::set_otp_bank_valid(bool const enabled) const noexcept
    {}

    void MPU6050::set_gyro_x_offset_tc(std::int8_t const offset) const noexcept
    {}

    /* YG_OFFS_TC REGISTER */
    void MPU6050::set_gyro_y_offset_tc(std::int8_t const offset) const noexcept
    {}

    /* ZG_OFFS_TC REGISTER */
    void MPU6050::set_gyro_z_offset_tc(std::int8_t const offset) const noexcept
    {}

    /* X_FINE_GAIN REGISTER */
    void MPU6050::set_x_fine_gain(std::int8_t const gain) const noexcept
    {}

    /* Y_FINE_GAIN REGISTER */
    void MPU6050::set_y_fine_gain(std::int8_t const gain) const noexcept
    {}

    /* Z_FINE_GAIN REGISTER */
    void MPU6050::set_z_fine_gain(std::int8_t const gain) const noexcept
    {}

    /* XA_OFFS REGISTER */
    void MPU6050::set_accel_x_offset(std::int16_t const offset) const noexcept
    {}

    /* YA_OFFS_* REGISTER */
    void MPU6050::set_accel_y_offset(std::int16_t const offset) const noexcept
    {}

    /* ZA_OFFS_* REGISTER */
    void MPU6050::set_accel_z_offset(std::int16_t const offset) const noexcept
    {}

    /* XG_OFFS_USR* REGISTER */
    void MPU6050::set_gyro_x_offset(std::int16_t const offset) const noexcept
    {}

    /* YG_OFFS_USR* REGISTER */
    void MPU6050::set_gyro_y_offset(std::int16_t const offset) const noexcept
    {}

    /* ZG_OFFS_USR* REGISTER */
    void MPU6050::set_gyro_z_offset(std::int16_t const offset) const noexcept
    {}

    /* INT_ENABLE REGISTER (DMP functions) */
    void MPU6050::set_int_pll_ready_enabled(bool const enabled) const noexcept
    {}

    void MPU6050::set_int_dmp_enabled(bool const enabled) const noexcept
    {}

    /* DMP_INT_STATUS REGISTER */
    bool MPU6050::get_dmp_int_5_status() const noexcept
    {}

    bool MPU6050::get_dmp_int_4_status() const noexcept
    {}

    bool MPU6050::get_dmp_int_3_status() const noexcept
    {}

    bool MPU6050::get_dmp_int_2_status() const noexcept
    {}

    bool MPU6050::get_dmp_int_1_status() const noexcept
    {}

    bool MPU6050::get_dmp_int_0_status() const noexcept
    {}

    /* INT_STATUS REGISTER (DMP functions) */
    bool MPU6050::get_int_pll_ready_status() const noexcept
    {}

    bool MPU6050::get_int_dmp_status() const noexcept
    {}

    /* USER_CTRL REGISTER (DMP functions) */
    void MPU6050::set_dmp_enabled(bool const enabled) const noexcept
    {}

    void MPU6050::reset_dmp() const noexcept
    {}

    /* BANK_SEL REGISTER */
    void
    MPU6050::set_memory_bank(std::uint8_t const bank, bool const prefetch_enabled, bool const user_bank) const noexcept
    {}

    /* MEM_START_ADDR REGISTER */
    void MPU6050::set_memory_start_address(std::uint8_t const address) const noexcept
    {}

    /* MEM_R_W REGISTER */
    std::uint8_t MPU6050::read_memory_byte() const noexcept
    {}

    void MPU6050::write_memory_byte(std::uint8_t data) const noexcept
    {}

    void MPU6050::read_memory_block(std::uint8_t* data,
                                    std::size_t const bytes,
                                    std::uint8_t const bank,
                                    std::uint8_t const address) const noexcept
    {}
    void MPU6050::write_memory_block(std::uint8_t* data,
                                     std::size_t const bytes,
                                     std::uint8_t const bank,
                                     std::uint8_t const address,
                                     bool const verify,
                                     bool const use_prog_mem) const noexcept
    {}

    void MPU6050::write_dmp_configuration_set(std::uint8_t* data,
                                              std::size_t const bytes,
                                              bool const use_prog_mem) const noexcept
    {}

    /* DMP_CFG_1 REGISTER */
    void MPU6050::set_dmp_config1(std::uint8_t const config) const noexcept
    {}

    /* DMP_CFG_2 REGISTER */
    void MPU6050::set_dmp_config2(std::uint8_t const config) const noexcept
    {}

}; // namespace InvertedSway
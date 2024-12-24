#include "mpu6050.hpp"
#include "common.hpp"
#include "i2c_driver.hpp"
#include "main.h"
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
using IntrDMP = MPU6050::IntrDMP;
using TC = MPU6050::TC;

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

        return static_cast<Scaled>(this->get_rotation_x_raw()) / gyro_range_to_scale(this->gyro_range_);
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

    void MPU6050::i2c_write_words(RegAddress const reg_address,
                                  std::uint16_t* write_data,
                                  std::uint8_t write_size) const noexcept
    {
        I2CDriver::write_words(this->i2c_bus_,
                               std::to_underlying(this->device_address_),
                               std::to_underlying(reg_address),
                               write_data,
                               write_size);
    }

    void MPU6050::i2c_write_word(RegAddress const reg_address, std::uint16_t write_data) const noexcept
    {
        I2CDriver::write_word(this->i2c_bus_,
                              std::to_underlying(this->device_address_),
                              std::to_underlying(reg_address),
                              write_data);
    }

    void MPU6050::i2c_write_bytes(RegAddress const reg_address,
                                  std::uint8_t* write_data,
                                  std::size_t const write_size) const noexcept
    {
        I2CDriver::write_bytes(this->i2c_bus_,
                               std::to_underlying(this->device_address_),
                               std::to_underlying(reg_address),
                               write_data,
                               write_size);
    }

    void MPU6050::i2c_write_byte(RegAddress const reg_address, std::uint8_t write_data) const noexcept
    {
        I2CDriver::write_byte(this->i2c_bus_,
                              std::to_underlying(this->device_address_),
                              std::to_underlying(reg_address),
                              write_data);
    }

    void MPU6050::i2c_write_bit(RegAddress const reg_address,
                                bool const write_data,
                                std::uint8_t const write_position) const noexcept
    {
        I2CDriver::write_bit(this->i2c_bus_,
                             std::to_underlying(this->device_address_),
                             std::to_underlying(reg_address),
                             write_data,
                             write_position);
    }

    void MPU6050::i2c_write_bits(RegAddress const reg_address,
                                 std::uint8_t const write_data,
                                 std::uint8_t const write_position,
                                 std::size_t const write_size) const noexcept
    {
        I2CDriver::write_bits(this->i2c_bus_,
                              std::to_underlying(this->device_address_),
                              std::to_underlying(reg_address),
                              write_data,
                              write_position,
                              write_size);
    }

    void MPU6050::i2c_read_words(RegAddress const reg_address,
                                 std::uint16_t* read_data,
                                 std::size_t const read_size) const noexcept
    {
        I2CDriver::read_words(this->i2c_bus_,
                              std::to_underlying(this->device_address_),
                              std::to_underlying(reg_address),
                              read_data,
                              read_size);
    }

    std::uint16_t MPU6050::i2c_read_word(RegAddress const reg_address) const noexcept
    {
        return I2CDriver::read_word(this->i2c_bus_,
                                    std::to_underlying(this->device_address_),
                                    std::to_underlying(reg_address));
    }

    void MPU6050::i2c_read_bytes(RegAddress const reg_address,
                                 std::uint8_t* read_data,
                                 std::size_t const read_size) const noexcept
    {
        I2CDriver::read_bytes(this->i2c_bus_,
                              std::to_underlying(this->device_address_),
                              std::to_underlying(reg_address),
                              read_data,
                              read_size);
    }

    std::uint8_t MPU6050::i2c_read_byte(RegAddress const reg_address) const noexcept
    {
        return I2CDriver::read_byte(this->i2c_bus_,
                                    std::to_underlying(this->device_address_),
                                    std::to_underlying(reg_address));
    }

    bool MPU6050::i2c_read_bit(RegAddress const reg_address, std::uint8_t const read_position) const noexcept
    {
        return I2CDriver::read_bit(this->i2c_bus_,
                                   std::to_underlying(this->device_address_),
                                   std::to_underlying(reg_address),
                                   read_position);
    }

    std::uint8_t MPU6050::i2c_read_bits(RegAddress const reg_address,
                                        std::uint8_t const read_position,
                                        std::size_t const read_size) const noexcept
    {
        return I2CDriver::read_bits(this->i2c_bus_,
                                    std::to_underlying(this->device_address_),
                                    std::to_underlying(reg_address),
                                    read_position,
                                    read_size);
    }

    void MPU6050::initialize(std::uint32_t const sampling_rate) noexcept
    {
        if (this->get_device_id() == 0x68) {
            this->device_reset();
            HAL_Delay(50);
            this->set_sleep_enabled(false);
            HAL_Delay(50);
            this->set_clock_source(Clock::INTERNAL);
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

    void MPU6050::set_sampling_divider(std::uint8_t const divider) const noexcept
    {
        this->i2c_write_byte(RegAddress::SMPLRT_DIV, divider);
    }

    void MPU6050::set_external_frame_sync(std::uint8_t const frame_sync) const noexcept
    {
        this->i2c_write_bits(RegAddress::CONFIG,
                             frame_sync,
                             std::to_underlying(Config::EXT_SYNC_SET_BIT),
                             std::to_underlying(Config::EXT_SYNC_SET_LENGTH));
    }

    void MPU6050::set_dlpf_mode(DLPF const dlpf) const noexcept
    {
        this->i2c_write_bits(RegAddress::CONFIG,
                             std::to_underlying(dlpf),
                             std::to_underlying(Config::DLPF_CFG_BIT),
                             std::to_underlying(Config::DLPF_CFG_LENGTH));
    }

    void MPU6050::set_full_scale_gyro_range(GyroRange const range) const noexcept
    {
        this->i2c_write_bits(RegAddress::GYRO_CONFIG,
                             std::to_underlying(range),
                             std::to_underlying(GyroConfig::FS_SEL_BIT),
                             std::to_underlying(GyroConfig::FS_SEL_LENGTH));
    }

    void MPU6050::set_full_scale_accel_range(AccelRange const range) const noexcept
    {
        this->i2c_write_bits(RegAddress::ACCEL_CONFIG,
                             std::to_underlying(range),
                             std::to_underlying(AccelConfig::AFS_SEL_BIT),
                             std::to_underlying(AccelConfig::AFS_SEL_LENGTH));
    }

    void MPU6050::set_dhpf_mode(DHPF const dhpf) const noexcept
    {
        this->i2c_write_bits(RegAddress::ACCEL_CONFIG,
                             std::to_underlying(dhpf),
                             std::to_underlying(AccelConfig::ACCEL_HPF_BIT),
                             std::to_underlying(AccelConfig::ACCEL_HPF_LENGTH));
    }

    void MPU6050::set_free_fall_detection_threshold(std::uint8_t const threshold) const noexcept
    {
        this->i2c_write_byte(RegAddress::FF_THR, threshold);
    }

    void MPU6050::set_free_fall_detection_duration(std::uint8_t const duration) const noexcept
    {
        this->i2c_write_byte(RegAddress::FF_DUR, duration);
    }

    void MPU6050::set_motion_detection_threshold(std::uint8_t const threshold) const noexcept
    {
        this->i2c_write_byte(RegAddress::FF_THR, threshold);
    }

    void MPU6050::set_motion_detection_duration(std::uint8_t const duration) const noexcept
    {
        this->i2c_write_byte(RegAddress::MOT_DUR, duration);
    }

    void MPU6050::set_zero_motion_detection_threshold(std::uint8_t const threshold) const noexcept
    {
        this->i2c_write_byte(RegAddress::ZRMOT_THR, threshold);
    }

    void MPU6050::set_zero_motion_detection_duration(std::uint8_t const duration) const noexcept
    {
        this->i2c_write_byte(RegAddress::ZRMOT_DUR, duration);
    }

    void MPU6050::set_temp_fifo_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_bit(RegAddress::FIFO_EN, enabled, std::to_underlying(FIFO::TEMP_EN_BIT));
    }

    void MPU6050::set_gyro_x_fifo_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_bit(RegAddress::FIFO_EN, enabled, std::to_underlying(FIFO::XG_EN_BIT));
    }

    void MPU6050::set_gyro_y_fifo_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_bit(RegAddress::FIFO_EN, enabled, std::to_underlying(FIFO::YG_EN_BIT));
    }

    void MPU6050::set_gyro_z_fifo_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_bit(RegAddress::FIFO_EN, enabled, std::to_underlying(FIFO::ZG_EN_BIT));
    }

    void MPU6050::set_accel_fifo_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_bit(RegAddress::FIFO_EN, enabled, std::to_underlying(FIFO::ACCEL_EN_BIT));
    }

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
        this->i2c_write_bit(RegAddress::INT_PIN_CFG,
                            std::to_underlying(mode),
                            std::to_underlying(IntrCfg::INT_LEVEL_BIT));
    }

    void MPU6050::set_interrupt_drive(IntrDrive const drive) const noexcept
    {
        this->i2c_write_bit(RegAddress::INT_PIN_CFG,
                            std::to_underlying(drive),
                            std::to_underlying(IntrCfg::INT_OPEN_BIT));
    }

    void MPU6050::set_interrupt_latch(IntrLatch const latch) const noexcept
    {
        this->i2c_write_bit(RegAddress::INT_PIN_CFG,
                            std::to_underlying(latch),
                            std::to_underlying(IntrCfg::INT_RD_CLEAR_BIT));
    }

    void MPU6050::set_interrupt_latch_clear(IntrClear const clear) const noexcept
    {
        this->i2c_write_bit(RegAddress::INT_PIN_CFG,
                            std::to_underlying(clear),
                            std::to_underlying(IntrCfg::LATCH_INT_EN_BIT));
    }

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

    void MPU6050::set_int_enabled(std::uint8_t const enabled) const noexcept
    {
        this->i2c_write_byte(RegAddress::INT_ENABLE, enabled);
    }

    void MPU6050::set_int_data_ready_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_bit(RegAddress::INT_ENABLE, enabled, std::to_underlying(Interrupt::DATA_RDY_BIT));
    }

    void MPU6050::set_int_zero_motion_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_bit(RegAddress::INT_ENABLE, enabled, std::to_underlying(Interrupt::ZMOT_BIT));
    }

    void MPU6050::set_int_motion_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_bit(RegAddress::INT_ENABLE, enabled, std::to_underlying(Interrupt::MOT_BIT));
    }

    void MPU6050::set_int_free_fall_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_bit(RegAddress::INT_ENABLE, enabled, std::to_underlying(Interrupt::FF_BIT));
    }

    void MPU6050::set_int_fifo_overflow_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_bit(RegAddress::INT_ENABLE, enabled, std::to_underlying(Interrupt::FIFO_OFLOW_BIT));
    }

    void MPU6050::set_int_i2c_master_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_bit(RegAddress::INT_ENABLE, enabled, std::to_underlying(Interrupt::I2C_MST_INT_BIT));
    }

    std::uint8_t MPU6050::get_int_status() const noexcept
    {
        return this->i2c_read_byte(RegAddress::INT_STATUS);
    }

    bool MPU6050::get_int_free_fall_status() const noexcept
    {
        return this->i2c_read_bit(RegAddress::INT_STATUS, std::to_underlying(Interrupt::FF_BIT));
    }

    bool MPU6050::get_int_motion_status() const noexcept
    {
        return this->i2c_read_bit(RegAddress::INT_STATUS, std::to_underlying(Interrupt::MOT_BIT));
    }

    bool MPU6050::get_int_zero_motion_status() const noexcept
    {
        return this->i2c_read_bit(RegAddress::INT_STATUS, std::to_underlying(Interrupt::ZMOT_BIT));
    }

    bool MPU6050::get_int_fifo_overflow_status() const noexcept
    {
        return this->i2c_read_bit(RegAddress::INT_STATUS, std::to_underlying(Interrupt::FIFO_OFLOW_BIT));
    }

    bool MPU6050::get_int_i2c_master_status() const noexcept
    {
        return this->i2c_read_bit(RegAddress::INT_STATUS, std::to_underlying(Interrupt::I2C_MST_INT_BIT));
    }

    bool MPU6050::get_int_data_ready_status() const noexcept
    {
        return this->i2c_read_bit(RegAddress::INT_STATUS, std::to_underlying(Interrupt::DATA_RDY_BIT));
    }

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

    TempRaw MPU6050::get_temperature_raw() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        std::uint8_t buffer[2];
        this->i2c_read_bytes(RegAddress::TEMP_OUT_H, buffer, sizeof(buffer));

        return ((static_cast<TempRaw>(buffer[0])) << 8) | buffer[1];
    }

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

    std::uint8_t MPU6050::get_motion_status() const noexcept
    {
        return this->i2c_read_byte(RegAddress::MOT_DETECT_STATUS);
    }

    bool MPU6050::get_x_neg_motion_detected() const noexcept
    {
        return this->i2c_read_bit(RegAddress::MOT_DETECT_STATUS, std::to_underlying(Motion::MOT_XNEG_BIT));
    }

    bool MPU6050::get_x_pos_motion_detected() const noexcept
    {
        return this->i2c_read_bit(RegAddress::MOT_DETECT_STATUS, std::to_underlying(Motion::MOT_XPOS_BIT));
    }

    bool MPU6050::get_y_neg_motion_detected() const noexcept
    {
        return this->i2c_read_bit(RegAddress::MOT_DETECT_STATUS, std::to_underlying(Motion::MOT_YNEG_BIT));
    }

    bool MPU6050::get_y_pos_motion_detected() const noexcept
    {
        return this->i2c_read_bit(RegAddress::MOT_DETECT_STATUS, std::to_underlying(Motion::MOT_YPOS_BIT));
    }

    bool MPU6050::get_z_neg_motion_detected() const noexcept
    {
        return this->i2c_read_bit(RegAddress::MOT_DETECT_STATUS, std::to_underlying(Motion::MOT_ZNEG_BIT));
    }

    bool MPU6050::get_z_pos_motion_detected() const noexcept
    {
        return this->i2c_read_bit(RegAddress::MOT_DETECT_STATUS, std::to_underlying(Motion::MOT_ZPOS_BIT));
    }

    void MPU6050::reset_gyro_path() const noexcept
    {
        this->i2c_write_bit(RegAddress::SIGNAL_PATH_RESET, true, std::to_underlying(PathReset::GYRO_RESET_BIT));
    }

    void MPU6050::reset_accel_path() const noexcept
    {
        this->i2c_write_bit(RegAddress::SIGNAL_PATH_RESET, true, std::to_underlying(PathReset::ACCEL_RESET_BIT));
    }

    void MPU6050::reset_temperature_path() const noexcept
    {
        this->i2c_write_bit(RegAddress::SIGNAL_PATH_RESET, true, std::to_underlying(PathReset::TEMP_RESET_BIT));
    }

    void MPU6050::set_accel_power_on_delay(Delay const delay) const noexcept
    {
        this->i2c_write_bits(RegAddress::MOT_DETECT_CTRL,
                             std::to_underlying(delay),
                             std::to_underlying(Detect::ACCEL_ON_DELAY_BIT),
                             std::to_underlying(Detect::ACCEL_ON_DELAY_LENGTH));
    }

    void MPU6050::set_free_fall_detection_counter_decrement(DetectDecrement const decrement) const noexcept
    {
        this->i2c_write_bits(RegAddress::MOT_DETECT_CTRL,
                             std::to_underlying(decrement),
                             std::to_underlying(Detect::FF_COUNT_BIT),
                             std::to_underlying(Detect::FF_COUNT_LENGTH));
    }

    void MPU6050::set_motion_detection_counter_decrement(DetectDecrement const decrement) const noexcept
    {
        this->i2c_write_bits(RegAddress::MOT_DETECT_CTRL,
                             std::to_underlying(decrement),
                             std::to_underlying(Detect::MOT_COUNT_BIT),
                             std::to_underlying(Detect::MOT_COUNT_LENGTH));
    }

    void MPU6050::set_i2c_master_mode_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_bit(RegAddress::USER_CTRL, enabled, std::to_underlying(UserCtrl::I2C_MST_EN_BIT));
    }

    void MPU6050::set_fifo_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_bit(RegAddress::USER_CTRL, enabled, std::to_underlying(UserCtrl::FIFO_EN_BIT));
    }

    void MPU6050::reset_fifo() const noexcept
    {
        this->i2c_write_bit(RegAddress::USER_CTRL, true, std::to_underlying(UserCtrl::FIFO_RESET_BIT));
    }

    void MPU6050::reset_i2c_master() const noexcept
    {
        this->i2c_write_bit(RegAddress::USER_CTRL, true, std::to_underlying(UserCtrl::I2C_MST_RESET_BIT));
    }

    void MPU6050::reset_sensors() const noexcept
    {
        this->i2c_write_bit(RegAddress::USER_CTRL, true, std::to_underlying(UserCtrl::SIG_COND_RESET_BIT));
    }

    void MPU6050::device_reset() const noexcept
    {
        this->i2c_write_bit(RegAddress::PWR_MGMT_1, true, std::to_underlying(Power1::DEVICE_RESET_BIT));
    }

    void MPU6050::set_sleep_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_bit(RegAddress::PWR_MGMT_1, enabled, std::to_underlying(Power1::SLEEP_BIT));
    }

    void MPU6050::set_wake_cycle_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_bit(RegAddress::PWR_MGMT_1, enabled, std::to_underlying(Power1::CYCLE_BIT));
    }

    void MPU6050::set_temperature_sensor_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_bit(RegAddress::PWR_MGMT_1, !enabled, std::to_underlying(Power1::TEMP_DIS_BIT));
    }

    void MPU6050::set_clock_source(Clock const source) const noexcept
    {
        this->i2c_write_bits(RegAddress::PWR_MGMT_1,
                             std::to_underlying(source),
                             std::to_underlying(Power1::CLKSEL_BIT),
                             std::to_underlying(Power1::CLKSEL_LENGTH));
    }

    void MPU6050::set_wake_up_frequency(WakeFreq const frequency) const noexcept
    {
        this->i2c_write_bits(RegAddress::PWR_MGMT_2,
                             std::to_underlying(frequency),
                             std::to_underlying(Power2::LP_WAKE_CTRL_BIT),
                             std::to_underlying(Power2::LP_WAKE_CTRL_LENGTH));
    }

    void MPU6050::set_accel_x_axis_standby(bool const standby) const noexcept
    {
        this->i2c_write_bit(RegAddress::PWR_MGMT_2, standby, std::to_underlying(Power2::STBY_XA_BIT));
    }

    void MPU6050::set_accel_y_axis_standby(bool const standby) const noexcept
    {
        this->i2c_write_bit(RegAddress::PWR_MGMT_2, standby, std::to_underlying(Power2::STBY_YA_BIT));
    }

    void MPU6050::set_accel_z_axis_standby(bool const standby) const noexcept
    {
        this->i2c_write_bit(RegAddress::PWR_MGMT_2, standby, std::to_underlying(Power2::STBY_ZA_BIT));
    }

    void MPU6050::set_gyro_x_axis_standby(bool const standby) const noexcept
    {
        this->i2c_write_bit(RegAddress::PWR_MGMT_2, standby, std::to_underlying(Power2::STBY_XG_BIT));
    }

    void MPU6050::set_gyro_y_axis_standby(bool const standby) const noexcept
    {
        this->i2c_write_bit(RegAddress::PWR_MGMT_2, standby, std::to_underlying(Power2::STBY_YG_BIT));
    }

    void MPU6050::set_gyro_z_axis_standby(bool const standby) const noexcept
    {
        this->i2c_write_bit(RegAddress::PWR_MGMT_2, standby, std::to_underlying(Power2::STBY_ZG_BIT));
    }

    std::uint16_t MPU6050::get_fifo_count() const noexcept
    {
        std::uint8_t buffer[2];
        this->i2c_read_bytes(RegAddress::FIFO_COUNTH, buffer, sizeof(buffer));

        return ((static_cast<std::uint16_t>(buffer[0]) << 8) | buffer[1]);
    }

    std::uint8_t MPU6050::get_fifo_byte() const noexcept
    {
        return this->i2c_read_byte(RegAddress::FIFO_R_W);
    }

    void MPU6050::get_fifo_bytes(std::uint8_t* read_data, std::size_t const read_size) const noexcept
    {
        this->i2c_read_bytes(RegAddress::FIFO_R_W, read_data, read_size);
    }

    void MPU6050::set_fifo_byte(std::uint8_t const write_data) const noexcept
    {
        this->i2c_write_byte(RegAddress::FIFO_R_W, write_data);
    }

    void MPU6050::set_fifo_bytes(std::uint8_t* write_data, std::size_t const write_size) const noexcept
    {
        this->i2c_write_bytes(RegAddress::FIFO_R_W, write_data, write_size);
    }

    std::uint8_t MPU6050::get_device_id() const noexcept
    {
        return this->i2c_read_byte(RegAddress::WHO_AM_I);
    }

    bool MPU6050::get_otp_bank_valid() const noexcept
    {
        return this->i2c_read_bit(RegAddress::XG_OFFS_TC, std::to_underlying(TC::OTP_BNK_VLD_BIT));
    }

    void MPU6050::set_otp_bank_valid(bool const valid) const noexcept
    {
        this->i2c_write_bit(RegAddress::XG_OFFS_TC, valid, std::to_underlying(TC::OTP_BNK_VLD_BIT));
    }

    void MPU6050::set_gyro_x_offset_tc(std::uint8_t const offset) const noexcept
    {
        this->i2c_write_bits(RegAddress::XG_OFFS_TC,
                             offset,
                             std::to_underlying(TC::OFFSET_BIT),
                             std::to_underlying(TC::OFFSET_LENGTH));
    }

    void MPU6050::set_gyro_y_offset_tc(std::uint8_t const offset) const noexcept
    {
        this->i2c_write_bits(RegAddress::YG_OFFS_TC,
                             offset,
                             std::to_underlying(TC::OFFSET_BIT),
                             std::to_underlying(TC::OFFSET_LENGTH));
    }

    void MPU6050::set_gyro_z_offset_tc(std::uint8_t const offset) const noexcept
    {
        this->i2c_write_bits(RegAddress::ZG_OFFS_TC,
                             offset,
                             std::to_underlying(TC::OFFSET_BIT),
                             std::to_underlying(TC::OFFSET_LENGTH));
    }

    void MPU6050::set_x_fine_gain(std::uint8_t const gain) const noexcept
    {
        this->i2c_write_byte(RegAddress::X_FINE_GAIN, gain);
    }

    void MPU6050::set_y_fine_gain(std::uint8_t const gain) const noexcept
    {
        this->i2c_write_byte(RegAddress::Y_FINE_GAIN, gain);
    }

    void MPU6050::set_z_fine_gain(std::uint8_t const gain) const noexcept
    {
        this->i2c_write_byte(RegAddress::Z_FINE_GAIN, gain);
    }

    void MPU6050::set_accel_x_offset(std::uint16_t const offset) const noexcept
    {
        this->i2c_write_word(RegAddress::XA_OFFS_H, offset);
    }

    void MPU6050::set_accel_y_offset(std::uint16_t const offset) const noexcept
    {
        this->i2c_write_word(RegAddress::YA_OFFS_H, offset);
    }

    void MPU6050::set_accel_z_offset(std::uint16_t const offset) const noexcept
    {
        this->i2c_write_word(RegAddress::ZA_OFFS_H, offset);
    }

    void MPU6050::set_gyro_x_offset(std::uint16_t const offset) const noexcept
    {
        this->i2c_write_word(RegAddress::XG_OFFS_USRH, offset);
    }

    void MPU6050::set_gyro_y_offset(std::uint16_t const offset) const noexcept
    {
        this->i2c_write_word(RegAddress::YG_OFFS_USRH, offset);
    }

    void MPU6050::set_gyro_z_offset(std::uint16_t const offset) const noexcept
    {
        this->i2c_write_word(RegAddress::ZG_OFFS_USRH, offset);
    }

    void MPU6050::set_int_pll_ready_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_bit(RegAddress::INT_ENABLE, enabled, std::to_underlying(Interrupt::PLL_RDY_INT_BIT));
    }

    void MPU6050::set_int_dmp_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_bit(RegAddress::INT_ENABLE, enabled, std::to_underlying(Interrupt::DMP_INT_BIT));
    }

    bool MPU6050::get_dmp_int_5_status() const noexcept
    {
        return this->i2c_read_bit(RegAddress::DMP_INT_STATUS, std::to_underlying(IntrDMP::DMPINT_5_BIT));
    }

    bool MPU6050::get_dmp_int_4_status() const noexcept
    {
        return this->i2c_read_bit(RegAddress::DMP_INT_STATUS, std::to_underlying(IntrDMP::DMPINT_4_BIT));
    }

    bool MPU6050::get_dmp_int_3_status() const noexcept
    {
        return this->i2c_read_bit(RegAddress::DMP_INT_STATUS, std::to_underlying(IntrDMP::DMPINT_3_BIT));
    }

    bool MPU6050::get_dmp_int_2_status() const noexcept
    {
        return this->i2c_read_bit(RegAddress::DMP_INT_STATUS, std::to_underlying(IntrDMP::DMPINT_2_BIT));
    }

    bool MPU6050::get_dmp_int_1_status() const noexcept
    {
        return this->i2c_read_bit(RegAddress::DMP_INT_STATUS, std::to_underlying(IntrDMP::DMPINT_1_BIT));
    }

    bool MPU6050::get_dmp_int_0_status() const noexcept
    {
        return this->i2c_read_bit(RegAddress::DMP_INT_STATUS, std::to_underlying(IntrDMP::DMPINT_0_BIT));
    }

    bool MPU6050::get_int_pll_ready_status() const noexcept
    {
        return this->i2c_read_bit(RegAddress::INT_STATUS, std::to_underlying(Interrupt::PLL_RDY_INT_BIT));
    }

    bool MPU6050::get_int_dmp_status() const noexcept
    {
        return this->i2c_read_bit(RegAddress::INT_STATUS, std::to_underlying(Interrupt::DMP_INT_BIT));
    }

    void MPU6050::set_dmp_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_bit(RegAddress::USER_CTRL, enabled, std::to_underlying(UserCtrl::DMP_EN_BIT));
    }

    void MPU6050::reset_dmp() const noexcept
    {
        this->i2c_write_bit(RegAddress::USER_CTRL, true, std::to_underlying(UserCtrl::DMP_RESET_BIT));
    }

    void
    MPU6050::set_memory_bank(std::uint8_t const bank, bool const prefetch_enabled, bool const user_bank) const noexcept
    {
        std::uint8_t data = bank & 0x1F;
        if (user_bank)
            data |= 0x20;
        if (prefetch_enabled)
            data |= 0x40;
        this->i2c_write_byte(RegAddress::BANK_SEL, data);
    }

    void MPU6050::set_memory_start_address(std::uint8_t const address) const noexcept
    {
        this->i2c_write_byte(RegAddress::MEM_START_ADDR, address);
    }

    std::uint8_t MPU6050::read_memory_byte() const noexcept
    {
        return this->i2c_read_byte(RegAddress::MEM_R_W);
    }

    void MPU6050::write_memory_byte(std::uint8_t const data) const noexcept
    {
        this->i2c_write_byte(RegAddress::MEM_R_W, data);
    }

    void MPU6050::read_memory_block(std::uint8_t* read_data,
                                    std::size_t const read_size,
                                    std::uint8_t bank,
                                    std::uint8_t address) const noexcept
    {
        this->set_memory_bank(bank);
        this->set_memory_start_address(address);

        for (std::uint16_t i = 0; i < read_size;) {
            std::uint8_t chunk_size = DMP_MEMORY_CHUNK_SIZE;

            if (i + chunk_size > read_size) {
                chunk_size = read_size - i;
            }

            if (chunk_size > 256 - address) {
                chunk_size = 256 - address;
            }

            this->i2c_read_bytes(RegAddress::MEM_R_W, read_data + i, chunk_size);

            i += chunk_size;
            address += chunk_size;

            if (i < read_size) {
                if (address == 0) {
                    bank++;
                }

                this->set_memory_bank(bank);
                this->set_memory_start_address(address);
            }
        }
    }

    void MPU6050::write_memory_block(std::uint8_t* write_data,
                                     std::size_t const write_size,
                                     std::uint8_t bank,
                                     std::uint8_t address) const noexcept
    {
        this->set_memory_bank(bank);
        this->set_memory_start_address(address);

        for (std::uint16_t i = 0; i < write_size;) {
            std::uint8_t chunk_size = DMP_MEMORY_CHUNK_SIZE;

            if (i + chunk_size > write_size) {
                chunk_size = write_size - i;
            }

            if (chunk_size > 256 - address) {
                chunk_size = 256 - address;
            }

            std::uint8_t* prog_buffer = (uint8_t*)write_data + i;
            this->i2c_write_bytes(RegAddress::MEM_R_W, prog_buffer, chunk_size);

            i += chunk_size;
            address += chunk_size;

            if (i < write_size) {
                if (address == 0) {
                    bank++;
                }

                this->set_memory_bank(bank);
                this->set_memory_start_address(address);
            }
        }
    }

    void MPU6050::write_dmp_configuration_set(std::uint8_t* write_data, std::size_t const write_size) const noexcept
    {
        std::uint16_t i, j;
        uint8_t bank, offset, length;

        for (i = 0; i < write_size;) {
            bank = write_data[i++];
            offset = write_data[i++];
            length = write_data[i++];

            if (length > 0) {
                std::uint8_t* prog_buffer = (uint8_t*)write_data + i;

                this->write_memory_block(prog_buffer, length, bank, offset);
                i += length;
            } else {
                if (write_data[i++] == 0x01) {
                    this->i2c_write_byte(RegAddress::INT_ENABLE, 0x32);
                }
            }
        }
    }

    void MPU6050::set_dmp_config1(std::uint8_t const config) const noexcept
    {
        this->i2c_write_byte(RegAddress::DMP_CFG_1, config);
    }

    void MPU6050::set_dmp_config2(std::uint8_t const config) const noexcept
    {
        this->i2c_write_byte(RegAddress::DMP_CFG_2, config);
    }

}; // namespace InvertedSway
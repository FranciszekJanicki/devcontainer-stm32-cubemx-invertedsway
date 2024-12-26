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
using Scaled = MPU6050::Scaled;
using Raw = MPU6050::Raw;
using GyroRaw = MPU6050::GyroRaw;
using AccelRaw = MPU6050::AccelRaw;
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

    std::uint8_t MPU6050::get_sampling_divider(std::uint32_t const sampling_rate, DLPF const dlpf) noexcept
    {
        return static_cast<std::uint8_t>(
            ((dlpf == DLPF::BW_256 ? GYRO_OUTPUT_RATE_DLPF_DIS_HZ : GYRO_OUTPUT_RATE_DLPF_EN_HZ) / sampling_rate) - 1U);
    }

    Scaled MPU6050::temp_raw_to_scaled(Raw const temp_raw) noexcept
    {
        return static_cast<Scaled>(temp_raw) / 340.0f + 36.53f;
    }

    AccelScaled MPU6050::accel_raw_to_scaled(AccelRaw const accel_raw, AccelRange const accel_range) noexcept
    {
        Scaled const scale{accel_range_to_scale(accel_range)};
        return AccelScaled{static_cast<Scaled>(accel_raw.x) / scale,
                           static_cast<Scaled>(accel_raw.y) / scale,
                           static_cast<Scaled>(accel_raw.z) / scale};
    }

    Scaled MPU6050::accel_raw_to_scaled(Raw const accel_raw, AccelRange const accel_range) noexcept
    {
        return static_cast<Scaled>(accel_raw) / accel_range_to_scale(accel_range);
    }

    GyroScaled MPU6050::gyro_raw_to_scaled(GyroRaw const gyro_raw, GyroRange const gyro_range) noexcept
    {
        Scaled const scale{gyro_range_to_scale(gyro_range)};
        return GyroScaled{static_cast<Scaled>(gyro_raw.x) / scale,
                          static_cast<Scaled>(gyro_raw.y) / scale,
                          static_cast<Scaled>(gyro_raw.z) / scale};
    }

    Scaled MPU6050::gyro_raw_to_scaled(Raw const gyro_raw, GyroRange const gyro_range) noexcept
    {
        return static_cast<Scaled>(gyro_raw) / gyro_range_to_scale(gyro_range);
    }

    RollPitchYaw MPU6050::accel_to_rpy(AccelScaled const accel_scaled) noexcept
    {
        return RollPitchYaw{
            std::atan2(accel_scaled.y, accel_scaled.z) * 180.0f / PI,
            -(std::atan2(accel_scaled.x, std::sqrt(accel_scaled.y * accel_scaled.y + accel_scaled.z * accel_scaled.z)) *
              180.0f) /
                PI,
            {}};
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

    Scaled MPU6050::get_temperature_celsius() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        return temp_raw_to_scaled(this->get_temperature_raw());
    }

    Scaled MPU6050::get_acceleration_x_scaled() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        return accel_raw_to_scaled(this->get_acceleration_x_raw(), this->accel_range_);
    }

    Scaled MPU6050::get_acceleration_y_scaled() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        return accel_raw_to_scaled(this->get_acceleration_y_raw(), this->accel_range_);
    }

    Scaled MPU6050::get_acceleration_z_scaled() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        return accel_raw_to_scaled(this->get_acceleration_z_raw(), this->accel_range_);
    }

    AccelScaled MPU6050::get_acceleration_scaled() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        return accel_raw_to_scaled(this->get_acceleration_raw(), this->accel_range_);
    }

    Scaled MPU6050::get_rotation_x_scaled() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        return gyro_raw_to_scaled(this->get_rotation_x_raw(), this->gyro_range_);
    }

    Scaled MPU6050::get_rotation_y_scaled() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        return gyro_raw_to_scaled(this->get_rotation_y_raw(), this->gyro_range_);
    }

    Scaled MPU6050::get_rotation_z_scaled() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        return gyro_raw_to_scaled(this->get_rotation_z_raw(), this->gyro_range_);
    }

    GyroScaled MPU6050::get_rotation_scaled() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        return gyro_raw_to_scaled(this->get_rotation_raw(), this->gyro_range_);
    }

    RollPitchYaw MPU6050::get_roll_pitch_yaw() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        return accel_to_rpy(get_acceleration_scaled());
    }

    Scaled MPU6050::get_roll() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        return accel_to_rpy(get_acceleration_scaled()).x;
    }

    Scaled MPU6050::get_pitch() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        return accel_to_rpy(get_acceleration_scaled()).y;
    }

    Scaled MPU6050::get_yaw() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        return accel_to_rpy(get_acceleration_scaled()).z;
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

    bool MPU6050::is_valid_device_id() const noexcept
    {
        return this->get_device_id() == std::to_underlying(this->device_address_);
    }

    void MPU6050::initialize(std::uint32_t const sampling_rate) noexcept
    {
        if (this->is_valid_device_id()) {
            this->device_reset();
            this->initialize_main(sampling_rate);
            this->initialize_interrupt();
            this->initialize_motion_interrupt();
            this->initialized_ = true;
        }
    }

    void MPU6050::initialize_main(std::uint32_t const sampling_rate) const noexcept
    {
        this->set_sleep_enabled(false);
        // this->set_clock_source(Clock::INTERNAL);
        this->set_sampling_rate(sampling_rate, DLPF::BW_256);
        this->set_dlpf_mode(DLPF::BW_256);
        this->set_full_scale_gyro_range(this->gyro_range_);
        this->set_full_scale_accel_range(this->accel_range_);
    }

    void MPU6050::initialize_interrupt() const noexcept
    {
        this->set_interrupt_mode(IntrMode::ACTIVEHIGH);
        // this->set_interrupt_drive(IntrDrive::PUSHPULL);
        this->set_interrupt_latch(IntrLatch::PULSE50US);
        this->set_interrupt_latch_clear(IntrClear::ANYREAD);
        this->set_int_data_ready_enabled(true);
    }

    void MPU6050::initialize_motion_interrupt() const noexcept
    {
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

    void MPU6050::deinitialize() noexcept
    {
        if (this->is_valid_device_id()) {
            this->device_reset();
            this->initialized_ = false;
        }
    }

    void MPU6050::set_sampling_rate(std::uint32_t const sampling_rate, DLPF const dlpf) const noexcept
    {
        this->i2c_write_byte(RegAddress::SMPLRT_DIV, get_sampling_divider(sampling_rate, dlpf));
    }

    void MPU6050::set_external_frame_sync(ExtSync const frame_sync) const noexcept
    {
        this->i2c_write_bits(RegAddress::CONFIG,
                             std::to_underlying(frame_sync),
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

    void MPU6050::set_fifo_enabled(std::uint8_t const fifo_enabled) const noexcept
    {
        this->i2c_write_byte(RegAddress::FIFO_EN, fifo_enabled);
    }

    void MPU6050::set_temp_fifo_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_bit(RegAddress::FIFO_EN, enabled, std::to_underlying(FIFO::TEMP_EN_BIT));
    }

    void MPU6050::set_x_gyro_fifo_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_bit(RegAddress::FIFO_EN, enabled, std::to_underlying(FIFO::XG_EN_BIT));
    }

    void MPU6050::set_y_gyro_fifo_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_bit(RegAddress::FIFO_EN, enabled, std::to_underlying(FIFO::YG_EN_BIT));
    }

    void MPU6050::set_z_gyro_fifo_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_bit(RegAddress::FIFO_EN, enabled, std::to_underlying(FIFO::ZG_EN_BIT));
    }

    void MPU6050::set_accel_fifo_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_bit(RegAddress::FIFO_EN, enabled, std::to_underlying(FIFO::ACCEL_EN_BIT));
    }

    void MPU6050::set_slave2_fifo_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_bit(RegAddress::FIFO_EN, enabled, std::to_underlying(FIFO::SLV2_EN_BIT));
    }

    void MPU6050::set_slave1_fifo_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_bit(RegAddress::FIFO_EN, enabled, std::to_underlying(FIFO::SLV1_EN_BIT));
    }

    void MPU6050::set_slave0_fifo_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_bit(RegAddress::FIFO_EN, enabled, std::to_underlying(FIFO::SLV0_EN_BIT));
    }

    void MPU6050::set_multi_master_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_bit(RegAddress::I2C_MST_CTRL, enabled, std::to_underlying(I2C::MULT_MST_EN_BIT));
    }

    void MPU6050::set_wait_for_external_sensor_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_bit(RegAddress::I2C_MST_CTRL, enabled, std::to_underlying(I2C::WAIT_FOR_ES_BIT));
    }

    void MPU6050::set_slave3_fifo_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_bit(RegAddress::I2C_MST_CTRL, enabled, std::to_underlying(I2CSlave::SLV_3_FIFO_EN_BIT));
    }

    void MPU6050::set_slave_read_write_transition_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_bit(RegAddress::I2C_MST_CTRL, enabled, std::to_underlying(I2C::MST_P_NSR_BIT));
    }

    void MPU6050::set_master_clock_speed(std::uint8_t const speed) const noexcept
    {
        this->i2c_write_bits(RegAddress::I2C_MST_CTRL,
                             speed,
                             std::to_underlying(I2C::MST_CLK_BIT),
                             std::to_underlying(I2C::MST_CLK_LENGTH));
    }

    static RegAddress slave_num_to_address(std::uint8_t const num) noexcept
    {
        switch (num) {
            case 0:
                return RegAddress::I2C_SLV0_ADDR;
            case 1:
                return RegAddress::I2C_SLV1_ADDR;
            case 2:
                return RegAddress::I2C_SLV2_ADDR;
            case 3:
                return RegAddress::I2C_SLV3_ADDR;
            default:
                std::unreachable();
        }
    }

    static RegAddress slave_num_to_register(std::uint8_t const num) noexcept
    {
        switch (num) {
            case 0:
                return RegAddress::I2C_SLV0_REG;
            case 1:
                return RegAddress::I2C_SLV1_REG;
            case 2:
                return RegAddress::I2C_SLV2_REG;
            case 3:
                return RegAddress::I2C_SLV3_REG;
            default:
                std::unreachable();
        }
    }

    static RegAddress slave_num_to_control(std::uint8_t const num) noexcept
    {
        switch (num) {
            case 0:
                return RegAddress::I2C_SLV0_CTRL;
            case 1:
                return RegAddress::I2C_SLV1_CTRL;
            case 2:
                return RegAddress::I2C_SLV2_CTRL;
            case 3:
                return RegAddress::I2C_SLV3_CTRL;
            default:
                std::unreachable();
        }
    }

    static RegAddress slave_num_to_output_byte(std::uint8_t const num) noexcept
    {
        switch (num) {
            case 0:
                return RegAddress::I2C_SLV0_DO;
            case 1:
                return RegAddress::I2C_SLV1_DO;
            case 2:
                return RegAddress::I2C_SLV2_DO;
            case 3:
                return RegAddress::I2C_SLV3_DO;
            default:
                std::unreachable();
        }
    }

    void MPU6050::set_slave_address(std::uint8_t const num, std::uint8_t const address) const noexcept
    {
        this->i2c_write_byte(slave_num_to_address(num), address);
    }

    void MPU6050::set_slave_register(std::uint8_t const num, std::uint8_t const reg) const noexcept
    {
        this->i2c_write_byte(slave_num_to_register(num), reg);
    }

    void MPU6050::set_slave_enabled(std::uint8_t const num, bool const enabled) const noexcept
    {
        this->i2c_write_bit(slave_num_to_control(num), enabled, std::to_underlying(I2CSlave::SLV_EN_BIT));
    }

    void MPU6050::set_slave_word_byte_swap(std::uint8_t const num, bool const enabled) const noexcept
    {
        this->i2c_write_bit(slave_num_to_control(num), enabled, std::to_underlying(I2CSlave::SLV_SW_BIT));
    }

    void MPU6050::set_slave_write_mode(std::uint8_t const num, bool const mode) const noexcept
    {
        this->i2c_write_bit(slave_num_to_control(num), mode, std::to_underlying(I2CSlave::SLV_REG_DIS_BIT));
    }

    void MPU6050::set_slave_word_group_offset(std::uint8_t const num, bool const enabled) const noexcept
    {
        this->i2c_write_bit(slave_num_to_control(num), enabled, std::to_underlying(I2CSlave::SLV_GRP_BIT));
    }

    void MPU6050::set_slave_data_length(std::uint8_t const num, std::uint8_t const length) const noexcept
    {
        this->i2c_write_bits(slave_num_to_control(num),
                             length,
                             std::to_underlying(I2CSlave::SLV_LEN_BIT),
                             std::to_underlying(I2CSlave::SLV_LEN_LENGTH));
    }

    void MPU6050::set_slave4_address(std::uint8_t const address) const noexcept
    {
        this->i2c_write_byte(RegAddress::I2C_SLV4_ADDR, address);
    }

    void MPU6050::set_slave4_register(std::uint8_t const reg) const noexcept
    {
        this->i2c_write_byte(RegAddress::I2C_SLV4_REG, reg);
    }

    void MPU6050::set_slave4_output_byte(std::uint8_t const data) const noexcept
    {
        this->i2c_write_byte(RegAddress::I2C_SLV4_DO, data);
    }

    void MPU6050::set_slave4_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_bit(RegAddress::I2C_SLV4_CTRL, enabled, std::to_underlying(I2CSlave4::SLV4_EN_BIT));
    }

    void MPU6050::set_slave4_interrupt_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_bit(RegAddress::I2C_SLV4_CTRL, enabled, std::to_underlying(I2CSlave4::SLV4_INT_EN_BIT));
    }

    void MPU6050::set_slave4_write_mode(bool const mode) const noexcept
    {
        this->i2c_write_bit(RegAddress::I2C_SLV4_ADDR, mode, std::to_underlying(I2CSlave4::SLV4_REG_DIS_BIT));
    }

    void MPU6050::set_slave4_master_delay(std::uint8_t const delay) const noexcept
    {
        this->i2c_write_bits(RegAddress::I2C_SLV4_ADDR,
                             delay,
                             std::to_underlying(I2CSlave4::SLV4_MST_DLY_BIT),
                             std::to_underlying(I2CSlave4::SLV4_MST_DLY_LENGTH));
    }

    std::uint8_t MPU6050::get_slave4_input_byte() const noexcept
    {
        return this->i2c_read_byte(RegAddress::I2C_SLV4_DI);
    }

    bool MPU6050::get_passthrough_status() const noexcept
    {
        return this->i2c_read_bit(RegAddress::I2C_MST_STATUS, std::to_underlying(I2CMaster::PASS_THROUGH_BIT));
    }

    bool MPU6050::get_slave4_is_done() const noexcept
    {
        return this->i2c_read_bit(RegAddress::I2C_MST_STATUS, std::to_underlying(I2CMaster::SLV4_DONE_BIT));
    }

    bool MPU6050::get_lost_arbitration() const noexcept
    {
        return this->i2c_read_bit(RegAddress::I2C_MST_STATUS, std::to_underlying(I2CMaster::LOST_ARB_BIT));
    }

    bool MPU6050::get_slave4_nack() const noexcept
    {
        return this->i2c_read_bit(RegAddress::I2C_MST_STATUS, std::to_underlying(I2CMaster::SLV4_NACK_BIT));
    }

    bool MPU6050::get_slave3_nack() const noexcept
    {
        return this->i2c_read_bit(RegAddress::I2C_MST_STATUS, std::to_underlying(I2CMaster::SLV3_NACK_BIT));
    }

    bool MPU6050::get_slave2_nack() const noexcept
    {
        return this->i2c_read_bit(RegAddress::I2C_MST_STATUS, std::to_underlying(I2CMaster::SLV2_NACK_BIT));
    }

    bool MPU6050::get_slave1_nack() const noexcept
    {
        return this->i2c_read_bit(RegAddress::I2C_MST_STATUS, std::to_underlying(I2CMaster::SLV1_NACK_BIT));
    }

    bool MPU6050::get_slave0_nack() const noexcept
    {
        return this->i2c_read_bit(RegAddress::I2C_MST_STATUS, std::to_underlying(I2CMaster::SLV0_NACK_BIT));
    }

    void MPU6050::set_interrupt(std::uint8_t const interrupt) const noexcept
    {
        this->i2c_write_byte(RegAddress::INT_PIN_CFG, interrupt);
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

    void MPU6050::set_f_sync_interrupt_level(bool const level) const noexcept
    {
        this->i2c_write_bit(RegAddress::INT_PIN_CFG, level, std::to_underlying(IntrCfg::FSYNC_INT_LEVEL_BIT));
    }

    void MPU6050::set_f_sync_interrupt_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_bit(RegAddress::INT_PIN_CFG, enabled, std::to_underlying(IntrCfg::FSYNC_INT_EN_BIT));
    }

    void MPU6050::set_i2c_bypass_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_bit(RegAddress::INT_PIN_CFG, enabled, std::to_underlying(IntrCfg::I2C_BYPASS_EN_BIT));
    }

    void MPU6050::set_clock_output_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_bit(RegAddress::INT_PIN_CFG, enabled, std::to_underlying(IntrCfg::CLK_OUT_BIT));
    }

    void MPU6050::set_int_enabled(std::uint8_t const int_enabled) const noexcept
    {
        this->i2c_write_byte(RegAddress::INT_ENABLE, int_enabled);
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

        return AccelRaw{(static_cast<Raw>(buffer[0]) << 8) | static_cast<Raw>(buffer[1]),
                        (static_cast<Raw>(buffer[2]) << 8) | static_cast<Raw>(buffer[3]),
                        (static_cast<Raw>(buffer[4]) << 8) | static_cast<Raw>(buffer[5])};
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

    Raw MPU6050::get_temperature_raw() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }

        std::uint8_t buffer[2];
        this->i2c_read_bytes(RegAddress::TEMP_OUT_H, buffer, sizeof(buffer));

        return ((static_cast<Raw>(buffer[0])) << 8) | buffer[1];
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

    std::uint8_t MPU6050::get_external_sensor_byte(std::uint8_t const position) const noexcept
    {
        return this->i2c_read_byte(
            static_cast<RegAddress>(std::to_underlying(RegAddress::EXT_SENS_DATA_00) + position));
    }

    std::uint16_t MPU6050::get_external_sensor_word(std::uint8_t const position) const noexcept
    {
        return this->i2c_read_word(
            static_cast<RegAddress>(std::to_underlying(RegAddress::EXT_SENS_DATA_00) + position));
    }

    std::uint32_t MPU6050::get_external_sensor_dword(std::uint8_t const position) const noexcept
    {
        std::uint16_t read[2];
        this->i2c_read_words(static_cast<RegAddress>(std::to_underlying(RegAddress::EXT_SENS_DATA_00) + position),
                             read,
                             sizeof(read));

        return *reinterpret_cast<uint32_t*>(read);
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

    bool MPU6050::get_zero_motion_detected() const noexcept
    {
        return this->i2c_read_bit(RegAddress::MOT_DETECT_STATUS, std::to_underlying(Motion::MOT_ZRMOT_BIT));
    }

    void MPU6050::set_slave_output_byte(std::uint8_t const num, std::uint8_t const data) const noexcept
    {
        this->i2c_write_byte(slave_num_to_output_byte(num), data);
    }

    void MPU6050::set_external_shadow_delay_enabled(bool const enabled) const noexcept
    {
        this->i2c_write_bit(RegAddress::I2C_MST_CTRL, enabled, std::to_underlying(DelayCtrl::DELAY_ES_SHADOW_BIT));
    }

    void MPU6050::set_slave_delay_enabled(std::uint8_t const num, bool const enabled) const noexcept
    {
        this->i2c_write_bit(RegAddress::I2C_MST_DELAY_CTRL, enabled, num);
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

    void MPU6050::set_x_accel_standby(bool const standby) const noexcept
    {
        this->i2c_write_bit(RegAddress::PWR_MGMT_2, standby, std::to_underlying(Power2::STBY_XA_BIT));
    }

    void MPU6050::set_y_accel_standby(bool const standby) const noexcept
    {
        this->i2c_write_bit(RegAddress::PWR_MGMT_2, standby, std::to_underlying(Power2::STBY_YA_BIT));
    }

    void MPU6050::set_z_accel_standby(bool const standby) const noexcept
    {
        this->i2c_write_bit(RegAddress::PWR_MGMT_2, standby, std::to_underlying(Power2::STBY_ZA_BIT));
    }

    void MPU6050::set_x_gyro_standby(bool const standby) const noexcept
    {
        this->i2c_write_bit(RegAddress::PWR_MGMT_2, standby, std::to_underlying(Power2::STBY_XG_BIT));
    }

    void MPU6050::set_y_gyro_standby(bool const standby) const noexcept
    {
        this->i2c_write_bit(RegAddress::PWR_MGMT_2, standby, std::to_underlying(Power2::STBY_YG_BIT));
    }

    void MPU6050::set_z_gyro_standby(bool const standby) const noexcept
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

    void MPU6050::get_current_fifo_packet(std::uint8_t* packet_data, std::size_t const packet_size) const noexcept
    {}

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

}; // namespace InvertedSway
#include "mpu6050.hpp"
#include "common.hpp"
#include "i2c.h"
#include "i2c_device.hpp"
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
        if (dlpf == DLPF::BW_256) {
            return static_cast<std::uint8_t>((GYRO_OUTPUT_RATE_DLPF_DIS_HZ / sampling_rate) - 1U);
        } else {
            return static_cast<std::uint8_t>((GYRO_OUTPUT_RATE_DLPF_EN_HZ / sampling_rate) - 1U);
        }
    }

    RollPitchYaw MPU6050::accel_to_roll_pitch_yaw(AccelScaled const accel_scaled) noexcept
    {
        return RollPitchYaw{
            std::atan2(accel_scaled.y, accel_scaled.z) * 180.0f / PI,
            -(std::atan2(accel_scaled.x, std::sqrt(accel_scaled.y * accel_scaled.y + accel_scaled.z * accel_scaled.z)) *
              180.0f) /
                PI,
            {}};
    }

    Scaled MPU6050::accel_to_roll(AccelScaled const accel_scaled) noexcept
    {
        return std::atan2(accel_scaled.y, accel_scaled.z) * 180.0f / PI;
    }

    Scaled MPU6050::accel_to_pitch(AccelScaled const accel_scaled) noexcept
    {
        return -(std::atan2(accel_scaled.x,
                            std::sqrt(accel_scaled.y * accel_scaled.y + accel_scaled.z * accel_scaled.z)) *
                 180.0f) /
               PI;
    }

    Scaled MPU6050::accel_to_yaw(AccelScaled const accel_scaled) noexcept
    {
        return {};
    }

    MPU6050::MPU6050(I2CDevice const i2c_device,
                     std::uint32_t const sampling_rate,
                     GyroRange const gyro_range,
                     AccelRange const accel_range,
                     DLPF const dlpf,
                     DHPF const dhpf) noexcept :
        i2c_device_{i2c_device},
        gyro_scale_{gyro_range_to_scale(gyro_range)},
        accel_scale_{accel_range_to_scale(accel_range)}
    {
        this->initialize(sampling_rate, gyro_range, accel_range, dlpf, dhpf);
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
        return static_cast<Scaled>(this->get_temperature_raw()) / 340.0f + 36.53f;
    }

    Scaled MPU6050::get_acceleration_x_scaled() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }
        return static_cast<Scaled>(this->get_acceleration_x_raw()) / this->accel_scale_;
    }

    Scaled MPU6050::get_acceleration_y_scaled() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }
        return static_cast<Scaled>(this->get_acceleration_y_raw()) / this->accel_scale_;
    }

    Scaled MPU6050::get_acceleration_z_scaled() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }
        return static_cast<Scaled>(this->get_acceleration_z_raw()) / this->accel_scale_;
    }

    AccelScaled MPU6050::get_acceleration_scaled() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }
        return static_cast<AccelScaled>(this->get_acceleration_raw()) / this->accel_scale_;
    }

    Scaled MPU6050::get_rotation_x_scaled() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }
        return static_cast<Scaled>(this->get_rotation_x_raw()) / this->gyro_scale_;
    }

    Scaled MPU6050::get_rotation_y_scaled() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }
        return static_cast<Scaled>(this->get_rotation_y_raw()) / this->gyro_scale_;
    }

    Scaled MPU6050::get_rotation_z_scaled() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }
        return static_cast<Scaled>(this->get_rotation_z_raw()) / this->gyro_scale_;
    }

    GyroScaled MPU6050::get_rotation_scaled() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }
        return static_cast<GyroScaled>(this->get_rotation_raw()) / this->gyro_scale_;
    }

    RollPitchYaw MPU6050::get_roll_pitch_yaw() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }
        return accel_to_roll_pitch_yaw(this->get_acceleration_scaled());
    }

    Scaled MPU6050::get_roll() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }
        return accel_to_roll(this->get_acceleration_scaled());
    }

    Scaled MPU6050::get_pitch() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }
        return accel_to_pitch(this->get_acceleration_scaled());
    }

    Scaled MPU6050::get_yaw() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }
        return accel_to_yaw(this->get_acceleration_scaled());
    }

    bool MPU6050::is_valid_device_id() const noexcept
    {
        return this->get_device_id() == this->i2c_device_.device_address;
    }

    void MPU6050::initialize(std::uint32_t const sampling_rate,
                             GyroRange const gyro_range,
                             AccelRange const accel_range,
                             DLPF const dlpf,
                             DHPF const dhpf) noexcept
    {
        if (this->is_valid_device_id()) {
            // this->i2c_device_.write_byte(std::to_underlying(RegAddress::PWR_MGMT_1),
            //                             1U << std::to_underlying(Power1::DEVICE_RESET_BIT));
            // this->i2c_device_.write_byte(std::to_underlying(RegAddress::PWR_MGMT_1), 0U);
            // this->i2c_device_.write_byte(std::to_underlying(RegAddress::SMPLRT_DIV),
            //                              get_sampling_divider(8000U, DLPF::BW_256));
            // this->i2c_device_.write_byte(std::to_underlying(RegAddress::CONFIG),
            //                              std::to_underlying(DLPF::BW_256) & 0x7);
            // this->i2c_device_.write_byte(std::to_underlying(RegAddress::GYRO_CONFIG),
            //                              (std::to_underlying(GyroRange::GYRO_FS_250) & 0x7) << 3);
            // this->i2c_device_.write_byte(std::to_underlying(RegAddress::ACCEL_CONFIG),
            //                              (std::to_underlying(AccelRange::ACCEL_FS_2) & 0x7) << 3);
            // this->i2c_device_.write_byte(std::to_underlying(RegAddress::INT_PIN_CFG),
            //                              std::to_underlying(IntrMode::ACTIVELOW)
            //                                  << std::to_underlying(IntrCfg::INT_LEVEL_BIT));
            // this->i2c_device_.write_byte(std::to_underlying(RegAddress::INT_PIN_CFG),
            //                              std::to_underlying(IntrLatch::PULSE50US)
            //                                  << std::to_underlying(IntrCfg::INT_RD_CLEAR_BIT));
            // this->i2c_device_.write_byte(std::to_underlying(RegAddress::INT_PIN_CFG),
            //                              std::to_underlying(IntrClear::ANYREAD)
            //                                  << std::to_underlying(IntrCfg::INT_RD_CLEAR_BIT));
            // this->i2c_device_.write_byte(std::to_underlying(RegAddress::INT_ENABLE),
            //                              1 << std::to_underlying(Interrupt::DATA_RDY_BIT));

            // this->device_reset();
            this->initialize_base(gyro_range, accel_range);
            this->initialize_rest(sampling_rate, dlpf, dhpf);
            this->initialize_interrupt();
            // this->initialize_motion_interrupt();
            this->initialized_ = true;
        }
    }

    void MPU6050::initialize_base(GyroRange const gyro_range, AccelRange const accel_range) const noexcept
    {
        this->device_wake_up();
        this->set_full_scale_accel_range(accel_range);
        this->set_full_scale_gyro_range(gyro_range);
        // this->set_sleep_enabled(false);
        // this->set_clock_source(Clock::PLL_XGYRO);
    }

    void MPU6050::initialize_rest(std::uint32_t const sampling_rate, DLPF const dlpf, DHPF const dhpf) const noexcept
    {
        this->set_sampling_rate(sampling_rate, dlpf);
        this->set_dlpf_mode(dlpf);
        // this->set_dhpf_mode(dhpf);
        // this->set_external_frame_sync(ExtSync::DISABLED);
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
        this->set_int_motion_enabled(true);
        this->set_int_zero_motion_enabled(true);
        this->set_int_free_fall_enabled(true);
        this->set_free_fall_detection_duration(2);
        this->set_free_fall_detection_threshold(5);
        this->set_motion_detection_duration(5);
        this->set_motion_detection_threshold(2);
        this->set_zero_motion_detection_duration(2);
        this->set_zero_motion_detection_threshold(4);
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
        this->i2c_device_.write_byte(std::to_underlying(RegAddress::SMPLRT_DIV),
                                     get_sampling_divider(sampling_rate, dlpf));
    }

    void MPU6050::set_external_frame_sync(ExtSync const frame_sync) const noexcept
    {
        this->i2c_device_.write_bits(std::to_underlying(RegAddress::CONFIG),
                                     std::to_underlying(frame_sync),
                                     std::to_underlying(Config::EXT_SYNC_SET_BIT),
                                     std::to_underlying(Config::EXT_SYNC_SET_LENGTH));
    }

    void MPU6050::set_dlpf_mode(DLPF const dlpf) const noexcept
    {
        this->i2c_device_.write_bits(std::to_underlying(RegAddress::CONFIG),
                                     std::to_underlying(dlpf),
                                     std::to_underlying(Config::DLPF_CFG_BIT),
                                     std::to_underlying(Config::DLPF_CFG_LENGTH));
    }

    void MPU6050::set_full_scale_gyro_range(GyroRange const range) const noexcept
    {
        this->i2c_device_.write_bits(std::to_underlying(RegAddress::GYRO_CONFIG),
                                     std::to_underlying(range),
                                     std::to_underlying(GyroConfig::FS_SEL_BIT),
                                     std::to_underlying(GyroConfig::FS_SEL_LENGTH));
    }

    void MPU6050::set_full_scale_accel_range(AccelRange const range) const noexcept
    {
        this->i2c_device_.write_bits(std::to_underlying(RegAddress::ACCEL_CONFIG),
                                     std::to_underlying(range),
                                     std::to_underlying(AccelConfig::AFS_SEL_BIT),
                                     std::to_underlying(AccelConfig::AFS_SEL_LENGTH));
    }

    void MPU6050::set_dhpf_mode(DHPF const dhpf) const noexcept
    {
        this->i2c_device_.write_bits(std::to_underlying(RegAddress::ACCEL_CONFIG),
                                     std::to_underlying(dhpf),
                                     std::to_underlying(AccelConfig::ACCEL_HPF_BIT),
                                     std::to_underlying(AccelConfig::ACCEL_HPF_LENGTH));
    }

    void MPU6050::set_free_fall_detection_threshold(std::uint8_t const threshold) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RegAddress::FF_THR), threshold);
    }

    void MPU6050::set_free_fall_detection_duration(std::uint8_t const duration) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RegAddress::FF_DUR), duration);
    }

    void MPU6050::set_motion_detection_threshold(std::uint8_t const threshold) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RegAddress::MOT_THR), threshold);
    }

    void MPU6050::set_motion_detection_duration(std::uint8_t const duration) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RegAddress::MOT_DUR), duration);
    }

    void MPU6050::set_zero_motion_detection_threshold(std::uint8_t const threshold) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RegAddress::ZRMOT_THR), threshold);
    }

    void MPU6050::set_zero_motion_detection_duration(std::uint8_t const duration) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RegAddress::ZRMOT_DUR), duration);
    }

    void MPU6050::set_fifo_enabled(std::uint8_t const fifo_enabled) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RegAddress::FIFO_EN), fifo_enabled);
    }

    void MPU6050::set_temp_fifo_enabled(bool const enabled) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::FIFO_EN),
                                    enabled,
                                    std::to_underlying(FIFO::TEMP_EN_BIT));
    }

    void MPU6050::set_x_gyro_fifo_enabled(bool const enabled) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::FIFO_EN),
                                    enabled,
                                    std::to_underlying(FIFO::XG_EN_BIT));
    }

    void MPU6050::set_y_gyro_fifo_enabled(bool const enabled) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::FIFO_EN),
                                    enabled,
                                    std::to_underlying(FIFO::YG_EN_BIT));
    }

    void MPU6050::set_z_gyro_fifo_enabled(bool const enabled) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::FIFO_EN),
                                    enabled,
                                    std::to_underlying(FIFO::ZG_EN_BIT));
    }

    void MPU6050::set_accel_fifo_enabled(bool const enabled) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::FIFO_EN),
                                    enabled,
                                    std::to_underlying(FIFO::ACCEL_EN_BIT));
    }

    void MPU6050::set_slave2_fifo_enabled(bool const enabled) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::FIFO_EN),
                                    enabled,
                                    std::to_underlying(FIFO::SLV2_EN_BIT));
    }

    void MPU6050::set_slave1_fifo_enabled(bool const enabled) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::FIFO_EN),
                                    enabled,
                                    std::to_underlying(FIFO::SLV1_EN_BIT));
    }

    void MPU6050::set_slave0_fifo_enabled(bool const enabled) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::FIFO_EN),
                                    enabled,
                                    std::to_underlying(FIFO::SLV0_EN_BIT));
    }

    void MPU6050::set_multi_master_enabled(bool const enabled) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::I2C_MST_CTRL),
                                    enabled,
                                    std::to_underlying(I2C::MULT_MST_EN_BIT));
    }

    void MPU6050::set_wait_for_external_sensor_enabled(bool const enabled) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::I2C_MST_CTRL),
                                    enabled,
                                    std::to_underlying(I2C::WAIT_FOR_ES_BIT));
    }

    void MPU6050::set_slave3_fifo_enabled(bool const enabled) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::I2C_MST_CTRL),
                                    enabled,
                                    std::to_underlying(I2CSlave::SLV_3_FIFO_EN_BIT));
    }

    void MPU6050::set_slave_read_write_transition_enabled(bool const enabled) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::I2C_MST_CTRL),
                                    enabled,
                                    std::to_underlying(I2C::MST_P_NSR_BIT));
    }

    void MPU6050::set_master_clock_speed(std::uint8_t const speed) const noexcept
    {
        this->i2c_device_.write_bits(std::to_underlying(RegAddress::I2C_MST_CTRL),
                                     speed,
                                     std::to_underlying(I2C::MST_CLK_BIT),
                                     std::to_underlying(I2C::MST_CLK_LENGTH));
    }

    std::uint8_t MPU6050::slave_num_to_address(std::uint8_t const num) noexcept
    {
        switch (num) {
            case 0:
                return std::to_underlying(RegAddress::I2C_SLV0_ADDR);
            case 1:
                return std::to_underlying(RegAddress::I2C_SLV1_ADDR);
            case 2:
                return std::to_underlying(RegAddress::I2C_SLV2_ADDR);
            case 3:
                return std::to_underlying(RegAddress::I2C_SLV3_ADDR);
            default:
                std::unreachable();
        }
    }

    std::uint8_t MPU6050::slave_num_to_register(std::uint8_t const num) noexcept
    {
        switch (num) {
            case 0:
                return std::to_underlying(RegAddress::I2C_SLV0_REG);
            case 1:
                return std::to_underlying(RegAddress::I2C_SLV1_REG);
            case 2:
                return std::to_underlying(RegAddress::I2C_SLV2_REG);
            case 3:
                return std::to_underlying(RegAddress::I2C_SLV3_REG);
            default:
                std::unreachable();
        }
    }

    std::uint8_t MPU6050::slave_num_to_control(std::uint8_t const num) noexcept
    {
        switch (num) {
            case 0:
                return std::to_underlying(RegAddress::I2C_SLV0_CTRL);
            case 1:
                return std::to_underlying(RegAddress::I2C_SLV1_CTRL);
            case 2:
                return std::to_underlying(RegAddress::I2C_SLV2_CTRL);
            case 3:
                return std::to_underlying(RegAddress::I2C_SLV3_CTRL);
            default:
                std::unreachable();
        }
    }

    std::uint8_t MPU6050::slave_num_to_output_byte(std::uint8_t const num) noexcept
    {
        switch (num) {
            case 0:
                return std::to_underlying(RegAddress::I2C_SLV0_DO);
            case 1:
                return std::to_underlying(RegAddress::I2C_SLV1_DO);
            case 2:
                return std::to_underlying(RegAddress::I2C_SLV2_DO);
            case 3:
                return std::to_underlying(RegAddress::I2C_SLV3_DO);
            default:
                std::unreachable();
        }
    }

    void MPU6050::set_slave_address(std::uint8_t const num, std::uint8_t const address) const noexcept
    {
        this->i2c_device_.write_byte(slave_num_to_address(num), address);
    }

    void MPU6050::set_slave_register(std::uint8_t const num, std::uint8_t const reg) const noexcept
    {
        this->i2c_device_.write_byte(slave_num_to_register(num), reg);
    }

    void MPU6050::set_slave_enabled(std::uint8_t const num, bool const enabled) const noexcept
    {
        this->i2c_device_.write_bit(slave_num_to_control(num), enabled, std::to_underlying(I2CSlave::SLV_EN_BIT));
    }

    void MPU6050::set_slave_word_byte_swap(std::uint8_t const num, bool const enabled) const noexcept
    {
        this->i2c_device_.write_bit(slave_num_to_control(num), enabled, std::to_underlying(I2CSlave::SLV_SW_BIT));
    }

    void MPU6050::set_slave_write_mode(std::uint8_t const num, bool const mode) const noexcept
    {
        this->i2c_device_.write_bit(slave_num_to_control(num), mode, std::to_underlying(I2CSlave::SLV_REG_DIS_BIT));
    }

    void MPU6050::set_slave_word_group_offset(std::uint8_t const num, bool const enabled) const noexcept
    {
        this->i2c_device_.write_bit(slave_num_to_control(num), enabled, std::to_underlying(I2CSlave::SLV_GRP_BIT));
    }

    void MPU6050::set_slave_data_length(std::uint8_t const num, std::uint8_t const length) const noexcept
    {
        this->i2c_device_.write_bits(slave_num_to_control(num),
                                     length,
                                     std::to_underlying(I2CSlave::SLV_LEN_BIT),
                                     std::to_underlying(I2CSlave::SLV_LEN_LENGTH));
    }

    void MPU6050::set_slave4_address(std::uint8_t const address) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RegAddress::I2C_SLV4_ADDR), address);
    }

    void MPU6050::set_slave4_register(std::uint8_t const reg) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RegAddress::I2C_SLV4_REG), reg);
    }

    void MPU6050::set_slave4_output_byte(std::uint8_t const data) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RegAddress::I2C_SLV4_DO), data);
    }

    void MPU6050::set_slave4_enabled(bool const enabled) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::I2C_SLV4_CTRL),
                                    enabled,
                                    std::to_underlying(I2CSlave4::SLV4_EN_BIT));
    }

    void MPU6050::set_slave4_interrupt_enabled(bool const enabled) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::I2C_SLV4_CTRL),
                                    enabled,
                                    std::to_underlying(I2CSlave4::SLV4_INT_EN_BIT));
    }

    void MPU6050::set_slave4_write_mode(bool const mode) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::I2C_SLV4_ADDR),
                                    mode,
                                    std::to_underlying(I2CSlave4::SLV4_REG_DIS_BIT));
    }

    void MPU6050::set_slave4_master_delay(std::uint8_t const delay) const noexcept
    {
        this->i2c_device_.write_bits(std::to_underlying(RegAddress::I2C_SLV4_ADDR),
                                     delay,
                                     std::to_underlying(I2CSlave4::SLV4_MST_DLY_BIT),
                                     std::to_underlying(I2CSlave4::SLV4_MST_DLY_LENGTH));
    }

    std::uint8_t MPU6050::get_slave4_input_byte() const noexcept
    {
        return this->i2c_device_.read_byte(std::to_underlying(RegAddress::I2C_SLV4_DI));
    }

    bool MPU6050::get_passthrough_status() const noexcept
    {
        return this->i2c_device_.read_bit(std::to_underlying(RegAddress::I2C_MST_STATUS),
                                          std::to_underlying(I2CMaster::PASS_THROUGH_BIT));
    }

    bool MPU6050::get_slave4_is_done() const noexcept
    {
        return this->i2c_device_.read_bit(std::to_underlying(RegAddress::I2C_MST_STATUS),
                                          std::to_underlying(I2CMaster::SLV4_DONE_BIT));
    }

    bool MPU6050::get_lost_arbitration() const noexcept
    {
        return this->i2c_device_.read_bit(std::to_underlying(RegAddress::I2C_MST_STATUS),
                                          std::to_underlying(I2CMaster::LOST_ARB_BIT));
    }

    bool MPU6050::get_slave4_nack() const noexcept
    {
        return this->i2c_device_.read_bit(std::to_underlying(RegAddress::I2C_MST_STATUS),
                                          std::to_underlying(I2CMaster::SLV4_NACK_BIT));
    }

    bool MPU6050::get_slave3_nack() const noexcept
    {
        return this->i2c_device_.read_bit(std::to_underlying(RegAddress::I2C_MST_STATUS),
                                          std::to_underlying(I2CMaster::SLV3_NACK_BIT));
    }

    bool MPU6050::get_slave2_nack() const noexcept
    {
        return this->i2c_device_.read_bit(std::to_underlying(RegAddress::I2C_MST_STATUS),
                                          std::to_underlying(I2CMaster::SLV2_NACK_BIT));
    }

    bool MPU6050::get_slave1_nack() const noexcept
    {
        return this->i2c_device_.read_bit(std::to_underlying(RegAddress::I2C_MST_STATUS),
                                          std::to_underlying(I2CMaster::SLV1_NACK_BIT));
    }

    bool MPU6050::get_slave0_nack() const noexcept
    {
        return this->i2c_device_.read_bit(std::to_underlying(RegAddress::I2C_MST_STATUS),
                                          std::to_underlying(I2CMaster::SLV0_NACK_BIT));
    }

    void MPU6050::set_interrupt(std::uint8_t const interrupt) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RegAddress::INT_PIN_CFG), interrupt);
    }

    void MPU6050::set_interrupt_mode(IntrMode const mode) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::INT_PIN_CFG),
                                    std::to_underlying(mode),
                                    std::to_underlying(IntrCfg::INT_LEVEL_BIT));
    }

    void MPU6050::set_interrupt_drive(IntrDrive const drive) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::INT_PIN_CFG),
                                    std::to_underlying(drive),
                                    std::to_underlying(IntrCfg::INT_OPEN_BIT));
    }

    void MPU6050::set_interrupt_latch(IntrLatch const latch) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::INT_PIN_CFG),
                                    std::to_underlying(latch),
                                    std::to_underlying(IntrCfg::INT_RD_CLEAR_BIT));
    }

    void MPU6050::set_interrupt_latch_clear(IntrClear const clear) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::INT_PIN_CFG),
                                    std::to_underlying(clear),
                                    std::to_underlying(IntrCfg::INT_RD_CLEAR_BIT));
    }

    void MPU6050::set_f_sync_interrupt_level(bool const level) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::INT_PIN_CFG),
                                    level,
                                    std::to_underlying(IntrCfg::FSYNC_INT_LEVEL_BIT));
    }

    void MPU6050::set_f_sync_interrupt_enabled(bool const enabled) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::INT_PIN_CFG),
                                    enabled,
                                    std::to_underlying(IntrCfg::FSYNC_INT_EN_BIT));
    }

    void MPU6050::set_i2c_bypass_enabled(bool const enabled) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::INT_PIN_CFG),
                                    enabled,
                                    std::to_underlying(IntrCfg::I2C_BYPASS_EN_BIT));
    }

    void MPU6050::set_clock_output_enabled(bool const enabled) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::INT_PIN_CFG),
                                    enabled,
                                    std::to_underlying(IntrCfg::CLK_OUT_BIT));
    }

    void MPU6050::set_int_enabled(std::uint8_t const int_enabled) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RegAddress::INT_ENABLE), int_enabled);
    }

    void MPU6050::set_int_data_ready_enabled(bool const enabled) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::INT_ENABLE),
                                    enabled,
                                    std::to_underlying(Interrupt::DATA_RDY_BIT));
    }

    void MPU6050::set_int_zero_motion_enabled(bool const enabled) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::INT_ENABLE),
                                    enabled,
                                    std::to_underlying(Interrupt::ZMOT_BIT));
    }

    void MPU6050::set_int_motion_enabled(bool const enabled) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::INT_ENABLE),
                                    enabled,
                                    std::to_underlying(Interrupt::MOT_BIT));
    }

    void MPU6050::set_int_free_fall_enabled(bool const enabled) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::INT_ENABLE),
                                    enabled,
                                    std::to_underlying(Interrupt::FF_BIT));
    }

    void MPU6050::set_int_fifo_overflow_enabled(bool const enabled) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::INT_ENABLE),
                                    enabled,
                                    std::to_underlying(Interrupt::FIFO_OFLOW_BIT));
    }

    void MPU6050::set_int_i2c_master_enabled(bool const enabled) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::INT_ENABLE),
                                    enabled,
                                    std::to_underlying(Interrupt::I2C_MST_INT_BIT));
    }

    std::uint8_t MPU6050::get_int_status() const noexcept
    {
        return this->i2c_device_.read_byte(std::to_underlying(RegAddress::INT_STATUS));
    }

    bool MPU6050::get_int_free_fall_status() const noexcept
    {
        return this->i2c_device_.read_bit(std::to_underlying(RegAddress::INT_STATUS),
                                          std::to_underlying(Interrupt::FF_BIT));
    }

    bool MPU6050::get_int_motion_status() const noexcept
    {
        return this->i2c_device_.read_bit(std::to_underlying(RegAddress::INT_STATUS),
                                          std::to_underlying(Interrupt::MOT_BIT));
    }

    bool MPU6050::get_int_zero_motion_status() const noexcept
    {
        return this->i2c_device_.read_bit(std::to_underlying(RegAddress::INT_STATUS),
                                          std::to_underlying(Interrupt::ZMOT_BIT));
    }

    bool MPU6050::get_int_fifo_overflow_status() const noexcept
    {
        return this->i2c_device_.read_bit(std::to_underlying(RegAddress::INT_STATUS),
                                          std::to_underlying(Interrupt::FIFO_OFLOW_BIT));
    }

    bool MPU6050::get_int_i2c_master_status() const noexcept
    {
        return this->i2c_device_.read_bit(std::to_underlying(RegAddress::INT_STATUS),
                                          std::to_underlying(Interrupt::I2C_MST_INT_BIT));
    }

    bool MPU6050::get_int_data_ready_status() const noexcept
    {
        return this->i2c_device_.read_bit(std::to_underlying(RegAddress::INT_STATUS),
                                          std::to_underlying(Interrupt::DATA_RDY_BIT));
    }

    AccelRaw MPU6050::get_acceleration_raw() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }
        auto buffer = this->i2c_device_.read_bytes<6>(std::to_underlying(RegAddress::ACCEL_XOUT_H));
        return AccelRaw{(static_cast<Raw>(buffer[0]) << 8) | static_cast<Raw>(buffer[1]),
                        (static_cast<Raw>(buffer[2]) << 8) | static_cast<Raw>(buffer[3]),
                        (static_cast<Raw>(buffer[4]) << 8) | static_cast<Raw>(buffer[5])};
    }

    Raw MPU6050::get_acceleration_x_raw() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }
        auto buffer = this->i2c_device_.read_bytes<2>(std::to_underlying(RegAddress::ACCEL_XOUT_H));
        return (static_cast<Raw>(buffer[0]) << 8) | static_cast<Raw>(buffer[1]);
    }

    Raw MPU6050::get_acceleration_y_raw() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }
        auto buffer = this->i2c_device_.read_bytes<2>(std::to_underlying(RegAddress::ACCEL_YOUT_H));
        return (static_cast<Raw>(buffer[0]) << 8) | static_cast<Raw>(buffer[1]);
    }

    Raw MPU6050::get_acceleration_z_raw() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }
        auto buffer = this->i2c_device_.read_bytes<2>(std::to_underlying(RegAddress::ACCEL_ZOUT_H));
        return (static_cast<Raw>(buffer[0]) << 8) | static_cast<Raw>(buffer[1]);
    }

    Raw MPU6050::get_temperature_raw() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }
        auto buffer = this->i2c_device_.read_bytes<2>(std::to_underlying(RegAddress::TEMP_OUT_H));
        return (static_cast<Raw>(buffer[0]) << 8) | static_cast<Raw>(buffer[1]);
    }

    GyroRaw MPU6050::get_rotation_raw() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }
        auto buffer = this->i2c_device_.read_bytes<6>(std::to_underlying(RegAddress::GYRO_XOUT_H));
        return GyroRaw{(static_cast<Raw>(buffer[0]) << 8) | static_cast<Raw>(buffer[1]),
                       (static_cast<Raw>(buffer[2]) << 8) | static_cast<Raw>(buffer[3]),
                       (static_cast<Raw>(buffer[4]) << 8) | static_cast<Raw>(buffer[5])};
    }

    Raw MPU6050::get_rotation_x_raw() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }
        auto buffer = this->i2c_device_.read_bytes<2>(std::to_underlying(RegAddress::GYRO_XOUT_H));
        return (static_cast<Raw>(buffer[0]) << 8) | static_cast<Raw>(buffer[1]);
    }

    Raw MPU6050::get_rotation_y_raw() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }
        auto buffer = this->i2c_device_.read_bytes<2>(std::to_underlying(RegAddress::GYRO_YOUT_H));
        return (static_cast<Raw>(buffer[0]) << 8) | static_cast<Raw>(buffer[1]);
    }

    Raw MPU6050::get_rotation_z_raw() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }
        auto buffer = this->i2c_device_.read_bytes<2>(std::to_underlying(RegAddress::GYRO_ZOUT_H));
        return (static_cast<Raw>(buffer[0]) << 8) | static_cast<Raw>(buffer[1]);
    }

    std::uint8_t MPU6050::get_external_sensor_byte(std::uint8_t const position) const noexcept
    {
        return this->i2c_device_.read_byte(std::to_underlying(RegAddress::EXT_SENS_DATA_00) + position);
    }

    std::uint16_t MPU6050::get_external_sensor_word(std::uint8_t const position) const noexcept
    {
        return this->i2c_device_.read_byte(std::to_underlying(RegAddress::EXT_SENS_DATA_00) + position);
    }

    std::uint32_t MPU6050::get_external_sensor_dword(std::uint8_t const position) const noexcept
    {
        return this->i2c_device_.read_byte(std::to_underlying(RegAddress::EXT_SENS_DATA_00) + position);
    }

    std::uint8_t MPU6050::get_motion_status() const noexcept
    {
        return this->i2c_device_.read_byte(std::to_underlying(RegAddress::MOT_DETECT_STATUS));
    }

    bool MPU6050::get_x_neg_motion_detected() const noexcept
    {
        return this->i2c_device_.read_bit(std::to_underlying(RegAddress::MOT_DETECT_STATUS),
                                          std::to_underlying(Motion::MOT_XNEG_BIT));
    }

    bool MPU6050::get_x_pos_motion_detected() const noexcept
    {
        return this->i2c_device_.read_bit(std::to_underlying(RegAddress::MOT_DETECT_STATUS),
                                          std::to_underlying(Motion::MOT_XPOS_BIT));
    }

    bool MPU6050::get_y_neg_motion_detected() const noexcept
    {
        return this->i2c_device_.read_bit(std::to_underlying(RegAddress::MOT_DETECT_STATUS),
                                          std::to_underlying(Motion::MOT_YNEG_BIT));
    }

    bool MPU6050::get_y_pos_motion_detected() const noexcept
    {
        return this->i2c_device_.read_bit(std::to_underlying(RegAddress::MOT_DETECT_STATUS),
                                          std::to_underlying(Motion::MOT_YPOS_BIT));
    }

    bool MPU6050::get_z_neg_motion_detected() const noexcept
    {
        return this->i2c_device_.read_bit(std::to_underlying(RegAddress::MOT_DETECT_STATUS),
                                          std::to_underlying(Motion::MOT_ZNEG_BIT));
    }

    bool MPU6050::get_z_pos_motion_detected() const noexcept
    {
        return this->i2c_device_.read_bit(std::to_underlying(RegAddress::MOT_DETECT_STATUS),
                                          std::to_underlying(Motion::MOT_ZPOS_BIT));
    }

    bool MPU6050::get_zero_motion_detected() const noexcept
    {
        return this->i2c_device_.read_bit(std::to_underlying(RegAddress::MOT_DETECT_STATUS),
                                          std::to_underlying(Motion::MOT_ZRMOT_BIT));
    }

    void MPU6050::set_slave_output_byte(std::uint8_t const num, std::uint8_t const data) const noexcept
    {
        this->i2c_device_.write_byte(slave_num_to_output_byte(num), data);
    }

    void MPU6050::set_external_shadow_delay_enabled(bool const enabled) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::I2C_MST_CTRL),
                                    enabled,
                                    std::to_underlying(DelayCtrl::DELAY_ES_SHADOW_BIT));
    }

    void MPU6050::set_slave_delay_enabled(std::uint8_t const num, bool const enabled) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::I2C_MST_DELAY_CTRL), enabled, num);
    }

    void MPU6050::reset_gyro_path() const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::SIGNAL_PATH_RESET),
                                    true,
                                    std::to_underlying(PathReset::GYRO_RESET_BIT));
    }

    void MPU6050::reset_accel_path() const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::SIGNAL_PATH_RESET),
                                    true,
                                    std::to_underlying(PathReset::ACCEL_RESET_BIT));
    }

    void MPU6050::reset_temperature_path() const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::SIGNAL_PATH_RESET),
                                    true,
                                    std::to_underlying(PathReset::TEMP_RESET_BIT));
    }

    void MPU6050::set_accel_power_on_delay(Delay const delay) const noexcept
    {
        this->i2c_device_.write_bits(std::to_underlying(RegAddress::MOT_DETECT_CTRL),
                                     std::to_underlying(delay),
                                     std::to_underlying(Detect::ACCEL_ON_DELAY_BIT),
                                     std::to_underlying(Detect::ACCEL_ON_DELAY_LENGTH));
    }

    void MPU6050::set_free_fall_detection_counter_decrement(DetectDecrement const decrement) const noexcept
    {
        this->i2c_device_.write_bits(std::to_underlying(RegAddress::MOT_DETECT_CTRL),
                                     std::to_underlying(decrement),
                                     std::to_underlying(Detect::FF_COUNT_BIT),
                                     std::to_underlying(Detect::FF_COUNT_LENGTH));
    }

    void MPU6050::set_motion_detection_counter_decrement(DetectDecrement const decrement) const noexcept
    {
        this->i2c_device_.write_bits(std::to_underlying(RegAddress::MOT_DETECT_CTRL),
                                     std::to_underlying(decrement),
                                     std::to_underlying(Detect::MOT_COUNT_BIT),
                                     std::to_underlying(Detect::MOT_COUNT_LENGTH));
    }

    void MPU6050::set_i2c_master_mode_enabled(bool const enabled) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::USER_CTRL),
                                    enabled,
                                    std::to_underlying(UserCtrl::I2C_MST_EN_BIT));
    }

    void MPU6050::set_fifo_enabled(bool const enabled) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::USER_CTRL),
                                    enabled,
                                    std::to_underlying(UserCtrl::FIFO_EN_BIT));
    }

    void MPU6050::reset_fifo() const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::USER_CTRL),
                                    true,
                                    std::to_underlying(UserCtrl::FIFO_RESET_BIT));
    }

    void MPU6050::reset_i2c_master() const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::USER_CTRL),
                                    true,
                                    std::to_underlying(UserCtrl::I2C_MST_RESET_BIT));
    }

    void MPU6050::reset_sensors() const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::USER_CTRL),
                                    true,
                                    std::to_underlying(UserCtrl::SIG_COND_RESET_BIT));
    }

    void MPU6050::device_reset() const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::PWR_MGMT_1),
                                    true,
                                    std::to_underlying(Power1::DEVICE_RESET_BIT));
    }

    void MPU6050::device_wake_up() const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RegAddress::PWR_MGMT_1), 0x00);
    }

    void MPU6050::set_sleep_enabled(bool const enabled) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::PWR_MGMT_1),
                                    enabled,
                                    std::to_underlying(Power1::SLEEP_BIT));
    }

    void MPU6050::set_wake_cycle_enabled(bool const enabled) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::PWR_MGMT_1),
                                    enabled,
                                    std::to_underlying(Power1::CYCLE_BIT));
    }

    void MPU6050::set_temperature_sensor_enabled(bool const enabled) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::PWR_MGMT_1),
                                    !enabled,
                                    std::to_underlying(Power1::TEMP_DIS_BIT));
    }

    void MPU6050::set_clock_source(Clock const source) const noexcept
    {
        this->i2c_device_.write_bits(std::to_underlying(RegAddress::PWR_MGMT_1),
                                     std::to_underlying(source),
                                     std::to_underlying(Power1::CLKSEL_BIT),
                                     std::to_underlying(Power1::CLKSEL_LENGTH));
    }

    void MPU6050::set_wake_up_frequency(WakeFreq const frequency) const noexcept
    {
        this->i2c_device_.write_bits(std::to_underlying(RegAddress::PWR_MGMT_2),
                                     std::to_underlying(frequency),
                                     std::to_underlying(Power2::LP_WAKE_CTRL_BIT),
                                     std::to_underlying(Power2::LP_WAKE_CTRL_LENGTH));
    }

    void MPU6050::set_x_accel_standby(bool const standby) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::PWR_MGMT_2),
                                    standby,
                                    std::to_underlying(Power2::STBY_XA_BIT));
    }

    void MPU6050::set_y_accel_standby(bool const standby) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::PWR_MGMT_2),
                                    standby,
                                    std::to_underlying(Power2::STBY_YA_BIT));
    }

    void MPU6050::set_z_accel_standby(bool const standby) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::PWR_MGMT_2),
                                    standby,
                                    std::to_underlying(Power2::STBY_ZA_BIT));
    }

    void MPU6050::set_x_gyro_standby(bool const standby) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::PWR_MGMT_2),
                                    standby,
                                    std::to_underlying(Power2::STBY_XG_BIT));
    }

    void MPU6050::set_y_gyro_standby(bool const standby) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::PWR_MGMT_2),
                                    standby,
                                    std::to_underlying(Power2::STBY_YG_BIT));
    }

    void MPU6050::set_z_gyro_standby(bool const standby) const noexcept
    {
        this->i2c_device_.write_bit(std::to_underlying(RegAddress::PWR_MGMT_2),
                                    standby,
                                    std::to_underlying(Power2::STBY_ZG_BIT));
    }

    std::uint16_t MPU6050::get_fifo_count() const noexcept
    {
        auto buffer = this->i2c_device_.read_bytes<2>(std::to_underlying(RegAddress::FIFO_COUNTH));

        return ((static_cast<std::uint16_t>(buffer[0]) << 8) | buffer[1]);
    }

    std::uint8_t MPU6050::get_fifo_byte() const noexcept
    {
        return this->i2c_device_.read_byte(std::to_underlying(RegAddress::FIFO_R_W));
    }

    void MPU6050::get_current_fifo_packet(std::uint8_t* packet_data, std::size_t const packet_size) const noexcept
    {}

    void MPU6050::get_fifo_bytes(std::uint8_t* read_data, std::size_t const read_size) const noexcept
    {
        this->i2c_device_.read_bytes(std::to_underlying(RegAddress::FIFO_R_W), read_data, read_size);
    }

    void MPU6050::set_fifo_byte(std::uint8_t const write_data) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RegAddress::FIFO_R_W), write_data);
    }

    void MPU6050::set_fifo_bytes(std::uint8_t* write_data, std::size_t const write_size) const noexcept
    {
        this->i2c_device_.write_bytes(std::to_underlying(RegAddress::FIFO_R_W), write_data, write_size);
    }

    std::uint8_t MPU6050::get_device_id() const noexcept
    {
        return this->i2c_device_.read_byte(std::to_underlying(RegAddress::WHO_AM_I));
    }

}; // namespace InvertedSway
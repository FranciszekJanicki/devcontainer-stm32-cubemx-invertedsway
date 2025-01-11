#include "mpu6050_bitfields.hpp"
#include "common.hpp"
#include "i2c.h"
#include "i2c_device.hpp"
#include "main.h"
#include "mpu_register_map.hpp"
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <utility>

using namespace InvertedSway::BitFields;
using Scaled = MPU6050::Scaled;
using GyroScaled = MPU6050::GyroScaled;
using AccelScaled = MPU6050::AccelScaled;
using RollPitchYaw = MPU6050::RollPitchYaw;
using Scaled = MPU6050::Scaled;
using Raw = MPU6050::Raw;
using GyroRaw = MPU6050::GyroRaw;
using AccelRaw = MPU6050::AccelRaw;
using DevAddress = MPU6050::DevAddress;
using Clock = MPU6050::Clock;
using IntLatch = MPU6050::IntLatch;
using IntDrive = MPU6050::IntDrive;
using IntMode = MPU6050::IntMode;
using IntClear = MPU6050::IntClear;

namespace InvertedSway::BitFields {

    MPU6050::MPU6050(I2CDevice const i2c_device,
                     std::uint32_t const sampling_rate,
                     GyroRange const gyro_range,
                     AccelRange const accel_range,
                     DLPF const dlpf,
                     DHPF const dhpf,
                     ExtSync const ext_sync) noexcept :
        i2c_device_{i2c_device},
        gyro_scale_{gyro_range_to_scale(gyro_range)},
        accel_scale_{accel_range_to_scale(accel_range)}
    {
        this->initialize(sampling_rate, gyro_range, accel_range, dlpf, dhpf, ext_sync);
    }

    MPU6050::~MPU6050() noexcept
    {
        this->deinitialize();
    }

    AccelScaled MPU6050::get_acceleration_scaled() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }
        return static_cast<AccelScaled>(this->get_acceleration_raw()) / this->accel_scale_;
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

    Scaled MPU6050::get_temperature_celsius() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }
        return static_cast<Scaled>(this->get_temperature_raw()) / 340.0f + 36.53F;
    }

    GyroScaled MPU6050::get_rotation_scaled() const noexcept
    {
        if (!this->initialized_) {
            std::unreachable();
        }
        return static_cast<GyroScaled>(this->get_rotation_raw()) / this->gyro_scale_;
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

    RollPitchYaw MPU6050::accel_to_roll_pitch_yaw(AccelScaled const& accel_scaled) noexcept
    {
        return RollPitchYaw{accel_to_roll(accel_scaled), accel_to_pitch(accel_scaled), accel_to_yaw(accel_scaled)};
    }

    Scaled MPU6050::accel_to_roll(AccelScaled const& accel_scaled) noexcept
    {
        return std::atan2(accel_scaled.y, accel_scaled.z) * 180.0F / PI;
    }

    Scaled MPU6050::accel_to_pitch(AccelScaled const& accel_scaled) noexcept
    {
        return -(std::atan2(accel_scaled.x,
                            std::sqrt(accel_scaled.y * accel_scaled.y + accel_scaled.z * accel_scaled.z)) *
                 180.0F) /
               PI;
    }

    Scaled MPU6050::accel_to_yaw(AccelScaled const& accel_scaled) noexcept
    {
        return {};
    }

    void MPU6050::device_reset() const noexcept
    {
        this->set_pwr_mgmt_1_register(
            PWR_MGMT_1{.device_reset = true, .sleep = false, .cycle = false, .temp_dis = false, .clksel = 0});
    }

    void MPU6050::device_wake_up() const noexcept
    {
        this->set_pwr_mgmt_1_register(
            PWR_MGMT_1{.device_reset = false, .sleep = false, .cycle = false, .temp_dis = false, .clksel = 0});
        HAL_Delay(200);
    }

    std::uint8_t MPU6050::get_device_id() const noexcept
    {
        return std::bit_cast<std::uint8_t>(this->get_who_am_i_register());
    }

    bool MPU6050::is_valid_device_id() const noexcept
    {
        return this->get_device_id() == this->i2c_device_.device_address;
    }

    void MPU6050::set_smplrt_div(std::uint8_t const sampling_rate, DLPF const dlpf) const noexcept
    {
        this->set_smplrt_div_register(SMPLRT_DIV{.smplrt_div = get_sampling_divider(sampling_rate, dlpf)});
    }

    void MPU6050::set_config(ExtSync const ext_sync, DLPF const dlpf) const noexcept
    {
        this->set_config_register(
            CONFIG{.ext_sync_set = std::to_underlying(ext_sync), .dlpf_cfg = std::to_underlying(dlpf)});
    }

    void MPU6050::set_gyro_config(bool const x_standby,
                                  bool const y_standby,
                                  bool const z_standby,
                                  GyroRange const gyro_range) const noexcept
    {
        this->set_gyro_config_register(GYRO_CONFIG{.xg_st = x_standby,
                                                   .yg_st = y_standby,
                                                   .zg_st = z_standby,
                                                   .fs_sel = std::to_underlying(gyro_range)});
    }

    void MPU6050::set_accel_config(bool const x_standby,
                                   bool const y_standby,
                                   bool const z_standby,
                                   AccelRange const accel_range,
                                   DHPF const dhpf) const noexcept
    {
        this->set_accel_config_register(ACCEL_CONFIG{.xa_st = x_standby,
                                                     .ya_st = y_standby,
                                                     .za_st = z_standby,
                                                     .afs_sel = std::to_underlying(accel_range),
                                                     .accel_hpf = std::to_underlying(dhpf)});
    }

    void MPU6050::initialize(std::uint32_t const sampling_rate,
                             GyroRange const gyro_range,
                             AccelRange const accel_range,
                             DLPF const dlpf,
                             DHPF const dhpf,
                             ExtSync const ext_sync) noexcept
    {
        if (this->is_valid_device_id()) {
            this->device_reset();
            this->device_wake_up();
            this->set_config(ext_sync, dlpf);
            this->set_accel_config(false, false, false, accel_range, dhpf);
            this->set_gyro_config(false, false, false, gyro_range);
            this->set_smplrt_div(sampling_rate, dlpf);
            this->initialized_ = true;
        }
    }

    void MPU6050::deinitialize() noexcept
    {
        if (this->is_valid_device_id()) {
            this->device_reset();
            this->initialized_ = false;
        }
    }

    AccelRaw MPU6050::get_acceleration_raw() const noexcept
    {
        return AccelRaw{this->get_acceleration_x_raw(), this->get_acceleration_y_raw(), this->get_acceleration_z_raw()};
    }

    Raw MPU6050::get_acceleration_x_raw() const noexcept
    {
        return std::bit_cast<Raw>(this->get_accel_xout_registers());
    }

    Raw MPU6050::get_acceleration_y_raw() const noexcept
    {
        return std::bit_cast<Raw>(this->get_accel_yout_registers());
    }

    Raw MPU6050::get_acceleration_z_raw() const noexcept
    {
        return std::bit_cast<Raw>(this->get_accel_zout_registers());
    }

    Raw MPU6050::get_temperature_raw() const noexcept
    {
        return std::bit_cast<Raw>(this->get_temp_out_registers());
    }

    GyroRaw MPU6050::get_rotation_raw() const noexcept
    {
        return GyroRaw{this->get_rotation_x_raw(), this->get_rotation_y_raw(), this->get_rotation_z_raw()};
    }

    Raw MPU6050::get_rotation_x_raw() const noexcept
    {
        return std::bit_cast<Raw>(this->get_gyro_xout_registers());
    }

    Raw MPU6050::get_rotation_y_raw() const noexcept
    {
        return std::bit_cast<Raw>(this->get_gyro_yout_registers());
    }

    Raw MPU6050::get_rotation_z_raw() const noexcept
    {
        return std::bit_cast<Raw>(this->get_gyro_zout_registers());
    }

    void MPU6050::set_xg_offs_register(XG_OFFS_TC const xg_offs_tc) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::XG_OFFS_TC), std::bit_cast<std::uint8_t>(xg_offs_tc));
    }

    void MPU6050::set_yg_offs_tc_register(YG_OFFS_TC const yg_offs_tc) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::YG_OFFS_TC), std::bit_cast<std::uint8_t>(yg_offs_tc));
    }

    void MPU6050::set_zg_offs_tc_register(ZG_OFFS_TC const zg_offs_tc) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::ZG_OFFS_TC), std::bit_cast<std::uint8_t>(zg_offs_tc));
    }

    void MPU6050::set_x_fine_gain_register(X_FINE_GAIN const x_fine_gain) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::X_FINE_GAIN), std::bit_cast<std::uint8_t>(x_fine_gain));
    }

    void MPU6050::set_y_fine_gain_register(Y_FINE_GAIN const y_fine_gain) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::Y_FINE_GAIN), std::bit_cast<std::uint8_t>(y_fine_gain));
    }

    void MPU6050::set_z_fine_gain_register(Z_FINE_GAIN const z_fine_gain) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::Z_FINE_GAIN), std::bit_cast<std::uint8_t>(z_fine_gain));
    }

    void MPU6050::set_xa_offs_registers(XA_OFFS const xa_offs) const noexcept
    {
        this->i2c_device_.write_word(std::to_underlying(RA::XA_OFFS_H), std::bit_cast<std::uint16_t>(xa_offs));
    }

    void MPU6050::set_ya_offs_registers(YA_OFFS const ya_offs) const noexcept
    {
        this->i2c_device_.write_word(std::to_underlying(RA::YA_OFFS_H), std::bit_cast<std::uint16_t>(ya_offs));
    }

    void MPU6050::set_za_offs_registers(ZA_OFFS const za_offs) const noexcept
    {
        this->i2c_device_.write_word(std::to_underlying(RA::ZA_OFFS_H), std::bit_cast<std::uint16_t>(za_offs));
    }

    void MPU6050::set_xg_offs_usr_registers(XG_OFFS_USR const xg_offs_usr) const noexcept
    {
        this->i2c_device_.write_word(std::to_underlying(RA::XG_OFFS_USRH), std::bit_cast<std::uint16_t>(xg_offs_usr));
    }

    void MPU6050::set_yg_offs_usr_registers(YG_OFFS_USR const yg_offs_usr) const noexcept
    {
        this->i2c_device_.write_word(std::to_underlying(RA::YG_OFFS_USRH), std::bit_cast<std::uint16_t>(yg_offs_usr));
    }

    void MPU6050::set_zg_offs_usr_registers(ZG_OFFS_USR const zg_offs_usr) const noexcept
    {
        this->i2c_device_.write_word(std::to_underlying(RA::ZG_OFFS_USRH), std::bit_cast<std::uint16_t>(zg_offs_usr));
    }

    void MPU6050::set_self_test_x_register(SELF_TEST_X const self_test_x) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::SELF_TEST_X), std::bit_cast<std::uint8_t>(self_test_x));
    }

    void MPU6050::set_self_test_y_register(SELF_TEST_Y const self_test_y) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::SELF_TEST_Y), std::bit_cast<std::uint8_t>(self_test_y));
    }

    void MPU6050::set_self_test_x_register(SELF_TEST_Z const self_test_z) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::SELF_TEST_Z), std::bit_cast<std::uint8_t>(self_test_z));
    }

    void MPU6050::set_self_test_a_register(SELF_TEST_A const self_test_a) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::SELF_TEST_A), std::bit_cast<std::uint8_t>(self_test_a));
    }

    void MPU6050::set_smplrt_div_register(SMPLRT_DIV const smplrt_div) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::SMPLRT_DIV), std::bit_cast<std::uint8_t>(smplrt_div));
    }

    void MPU6050::set_config_register(CONFIG const config) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::CONFIG), std::bit_cast<std::uint8_t>(config));
    }

    void MPU6050::set_gyro_config_register(GYRO_CONFIG const gyro_config) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::GYRO_CONFIG), std::bit_cast<std::uint8_t>(gyro_config));
    }

    void MPU6050::set_accel_config_register(ACCEL_CONFIG const accel_config) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::ACCEL_CONFIG), std::bit_cast<std::uint8_t>(accel_config));
    }

    void MPU6050::set_ff_thr_register(FF_THR const ff_thr) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::FF_THR), std::bit_cast<std::uint8_t>(ff_thr));
    }

    void MPU6050::set_ff_dur_register(FF_DUR const ff_dur) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::FF_DUR), std::bit_cast<std::uint8_t>(ff_dur));
    }

    void MPU6050::set_mot_thr_register(MOT_THR const mot_thr) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::MOT_THR), std::bit_cast<std::uint8_t>(mot_thr));
    }

    void MPU6050::set_mot_dur_register(MOT_DUR const mot_dur) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::MOT_DUR), std::bit_cast<std::uint8_t>(mot_dur));
    }

    void MPU6050::set_zrmot_thr_register(ZRMOT_THR const zrmot_thr) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::ZRMOT_THR), std::bit_cast<std::uint8_t>(zrmot_thr));
    }

    void MPU6050::set_zrmot_dur_register(ZRMOT_DUR const zrmot_dur) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::ZRMOT_DUR), std::bit_cast<std::uint8_t>(zrmot_dur));
    }

    void MPU6050::set_fifo_en_register(FIFO_EN const fifo_en) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::FIFO_EN), std::bit_cast<std::uint8_t>(fifo_en));
    }

    void MPU6050::set_i2c_mst_ctrl_register(I2C_MST_CTRL const i2c_mst_ctrl) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::I2C_MST_CTRL), std::bit_cast<std::uint8_t>(i2c_mst_ctrl));
    }

    void MPU6050::set_i2c_slv_addr_register(SlaveNum const slave_num, I2C_SLV_ADDR const i2c_slv_addr) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::I2C_SLV0_ADDR) + 3 * std::to_underlying(slave_num),
                                     std::bit_cast<std::uint8_t>(i2c_slv_addr));
    }

    void MPU6050::set_i2c_slv_reg_register(SlaveNum const slave_num, I2C_SLV_REG const i2c_slv_reg) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::I2C_SLV0_REG) + 3 * std::to_underlying(slave_num),
                                     std::bit_cast<std::uint8_t>(i2c_slv_reg));
    }

    void MPU6050::set_i2c_slv_ctrl_register(SlaveNum const slave_num, I2C_SLV_CTRL const i2c_slv_ctrl) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::I2C_SLV0_CTRL) + 3 * std::to_underlying(slave_num),
                                     std::bit_cast<std::uint8_t>(i2c_slv_ctrl));
    }

    void MPU6050::set_i2c_slv4_addr_register(I2C_SLV4_ADDR const i2c_slv4_addr) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::I2C_SLV4_ADDR), std::bit_cast<std::uint8_t>(i2c_slv4_addr));
    }

    void MPU6050::set_i2c_slv4_reg_register(I2C_SLV4_REG const i2c_slv4_reg) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::I2C_SLV4_REG), std::bit_cast<std::uint8_t>(i2c_slv4_reg));
    }

    void MPU6050::set_i2c_slv4_ctrl_register(I2C_SLV4_CTRL const i2c_slv4_ctrl) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::I2C_SLV4_CTRL), std::bit_cast<std::uint8_t>(i2c_slv4_ctrl));
    }

    void MPU6050::set_i2c_slv4_do_register(I2C_SLV4_DO const i2c_slv4_do) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::I2C_SLV4_DO), std::bit_cast<std::uint8_t>(i2c_slv4_do));
    }

    I2C_SLV4_DI MPU6050::get_i2c_slv4_di_register() const noexcept
    {
        return std::bit_cast<I2C_SLV4_DI>(this->i2c_device_.read_byte(std::to_underlying(RA::I2C_SLV4_DI)));
    }

    I2C_MST_STATUS MPU6050::get_i2c_mst_status_register() const noexcept
    {
        return std::bit_cast<I2C_MST_STATUS>(this->i2c_device_.read_byte(std::to_underlying(RA::I2C_MST_STATUS)));
    }

    void MPU6050::set_int_pin_cfg_register(INT_PIN_CFG const int_pin_cfg) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::INT_PIN_CFG), std::bit_cast<std::uint8_t>(int_pin_cfg));
    }

    void MPU6050::set_int_enable_register(INT_ENABLE const int_enable) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::INT_ENABLE), std::bit_cast<std::uint8_t>(int_enable));
    }

    DMP_INT_STATUS MPU6050::get_dmp_int_status_register() const noexcept
    {
        return std::bit_cast<DMP_INT_STATUS>(this->i2c_device_.read_byte(std::to_underlying(RA::DMP_INT_STATUS)));
    }

    TC MPU6050::get_tc_register() const noexcept
    {
        //  return std::bit_cast<TC>(this->i2c_device_.read_byte(std::to_underlying(RA::TC)));
    }

    INT_STATUS MPU6050::get_int_status_register() const noexcept
    {
        return std::bit_cast<INT_STATUS>(this->i2c_device_.read_byte(std::to_underlying(RA::INT_STATUS)));
    }

    ACCEL_XOUT MPU6050::get_accel_xout_registers() const noexcept
    {
        return std::bit_cast<ACCEL_XOUT>(this->i2c_device_.read_word(std::to_underlying(RA::ACCEL_XOUT_H)));
    }

    ACCEL_YOUT MPU6050::get_accel_yout_registers() const noexcept
    {
        return std::bit_cast<ACCEL_YOUT>(this->i2c_device_.read_word(std::to_underlying(RA::ACCEL_YOUT_H)));
    }

    ACCEL_ZOUT MPU6050::get_accel_zout_registers() const noexcept
    {
        return std::bit_cast<ACCEL_ZOUT>(this->i2c_device_.read_word(std::to_underlying(RA::ACCEL_ZOUT_H)));
    }

    TEMP_OUT MPU6050::get_temp_out_registers() const noexcept
    {
        return std::bit_cast<TEMP_OUT>(this->i2c_device_.read_word(std::to_underlying(RA::TEMP_OUT_H)));
    }

    GYRO_XOUT MPU6050::get_gyro_xout_registers() const noexcept
    {
        return std::bit_cast<GYRO_XOUT>(this->i2c_device_.read_word(std::to_underlying(RA::GYRO_XOUT_H)));
    }

    GYRO_YOUT MPU6050::get_gyro_yout_registers() const noexcept
    {
        return std::bit_cast<GYRO_YOUT>(this->i2c_device_.read_word(std::to_underlying(RA::GYRO_YOUT_H)));
    }

    GYRO_ZOUT MPU6050::get_gyro_zout_registers() const noexcept
    {
        return std::bit_cast<GYRO_ZOUT>(this->i2c_device_.read_word(std::to_underlying(RA::GYRO_ZOUT_H)));
    }

    EXT_SENS_DATA MPU6050::get_ext_sens_data_register(std::uint8_t const position) const noexcept
    {
        return std::bit_cast<EXT_SENS_DATA>(
            this->i2c_device_.read_byte(std::to_underlying(RA::EXT_SENS_DATA_00) + position));
    }

    MOT_DETECT_STATUS MPU6050::get_mot_detect_status_registet() const noexcept
    {
        return std::bit_cast<MOT_DETECT_STATUS>(this->i2c_device_.read_byte(std::to_underlying(RA::MOT_DETECT_STATUS)));
    }

    void MPU6050::set_i2c_slv_do_register(SlaveNum const slave_num, I2C_SLV_DO const i2c_slv_do) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::I2C_SLV0_DO) + 3 * std::to_underlying(slave_num),
                                     std::bit_cast<std::uint8_t>(i2c_slv_do));
    }

    void MPU6050::set_i2c_mst_delay_ctrl_register(I2C_MST_DELAY_CTRL const i2c_mst_delay_ctrl) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::I2C_MST_DELAY_CTRL),
                                     std::bit_cast<std::uint8_t>(i2c_mst_delay_ctrl));
    }

    void MPU6050::set_signal_path_reset_register(SIGNAL_PATH_RESET const signal_path_reset) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::SIGNAL_PATH_RESET),
                                     std::bit_cast<std::uint8_t>(signal_path_reset));
    }

    void MPU6050::set_mot_detect_ctrl_register(MOT_DETECT_CTRL const mot_detect_ctrl) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::MOT_DETECT_CTRL),
                                     std::bit_cast<std::uint8_t>(mot_detect_ctrl));
    }

    void MPU6050::set_user_ctrl_register(USER_CTRL const user_ctrl) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::USER_CTRL), std::bit_cast<std::uint8_t>(user_ctrl));
    }

    void MPU6050::set_pwr_mgmt_1_register(PWR_MGMT_1 const pwr_mgmt_1) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::PWR_MGMT_1), std::bit_cast<std::uint8_t>(pwr_mgmt_1));
    }

    void MPU6050::set_pwr_mgmt_2_register(PWR_MGMT_2 const pwr_mgmt_2) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::PWR_MGMT_2), std::bit_cast<std::uint8_t>(pwr_mgmt_2));
    }

    void MPU6050::set_bank_sel_register(BANK_SEL const bank_sel) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::BANK_SEL), std::bit_cast<std::uint8_t>(bank_sel));
    }

    void MPU6050::set_mem_start_addr_register(MEM_START_ADDR const mem_start_addr) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::MEM_START_ADDR),
                                     std::bit_cast<std::uint8_t>(mem_start_addr));
    }

    void MPU6050::set_mem_r_w_register(MEM_R_W const mem_r_w) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::MEM_R_W), std::bit_cast<std::uint8_t>(mem_r_w));
    }

    MEM_R_W MPU6050::get_mem_r_w_register() const noexcept
    {
        return std::bit_cast<MEM_R_W>(this->i2c_device_.read_byte(std::to_underlying(RA::MEM_R_W)));
    }

    void MPU6050::set_dmp_cfg_1_register(DMP_CFG_1 const dmp_cfg_1) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::DMP_CFG_1), std::bit_cast<std::uint8_t>(dmp_cfg_1));
    }

    void MPU6050::set_dmp_cfg_2_register(DMP_CFG_2 const dmp_cfg_2) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::DMP_CFG_2), std::bit_cast<std::uint8_t>(dmp_cfg_2));
    }

    FIFO_COUNT MPU6050::get_fifo_count_registers() const noexcept
    {
        return std::bit_cast<FIFO_COUNT>(this->i2c_device_.read_word(std::to_underlying(RA::FIFO_COUNTH)));
    }

    void MPU6050::set_fifo_r_w_register(FIFO_R_W const fifo_r_w) const noexcept
    {
        this->i2c_device_.write_byte(std::to_underlying(RA::FIFO_R_W), std::bit_cast<std::uint8_t>(fifo_r_w));
    }

    FIFO_R_W MPU6050::get_fifo_r_w_register() const noexcept
    {
        return std::bit_cast<FIFO_R_W>(this->i2c_device_.read_byte(std::to_underlying(RA::WHO_AM_I)));
    }

    WHO_AM_I MPU6050::get_who_am_i_register() const noexcept
    {
        return std::bit_cast<WHO_AM_I>(this->i2c_device_.read_byte(std::to_underlying(RA::WHO_AM_I)));
    }

}; // namespace InvertedSway::BitFields

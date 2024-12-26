#include "mpu6050_dmp.hpp"
#include "mpu6050.hpp"

using namespace InvertedSway;
using Scaled = MPU6050_DMP::Scaled;
using GyroScaled = MPU6050_DMP::GyroScaled;
using AccelScaled = MPU6050_DMP::AccelScaled;
using RollPitchYaw = MPU6050_DMP::RollPitchYaw;
using Raw = MPU6050_DMP::Raw;
using GyroRaw = MPU6050_DMP::GyroRaw;
using AccelRaw = MPU6050_DMP::AccelRaw;
using DevAddress = MPU6050_DMP::DevAddress;
using RegAddress = MPU6050_DMP::RegAddress;
using Power1 = MPU6050_DMP::Power1;
using Power2 = MPU6050_DMP::Power2;
using Clock = MPU6050_DMP::Clock;
using Interrupt = MPU6050_DMP::Interrupt;
using IntrLatch = MPU6050_DMP::IntrLatch;
using IntrDrive = MPU6050_DMP::IntrDrive;
using IntrMode = MPU6050_DMP::IntrMode;
using IntrCfg = MPU6050_DMP::IntrCfg;
using IntrClear = MPU6050_DMP::IntrClear;
using IntrDMP = MPU6050_DMP::IntrDMP;
using TC = MPU6050_DMP::TC;

namespace InvertedSway {

    MPU6050_DMP::MPU6050_DMP(MPU6050&& mpu6050) noexcept : mpu6050_{std::forward<MPU6050>(mpu6050)}
    {
        this->initialize();
    }

    MPU6050_DMP::~MPU6050_DMP() noexcept
    {
        this->deinitialize();
    }

    void MPU6050_DMP::initialize() noexcept
    {
        this->set_memory_bank(0x10, true, true);
        this->set_memory_start_address(0x06);
        this->set_memory_bank(0, false, false);
        this->get_otp_bank_valid();

        this->mpu6050_.set_slave_address(0, 0x7F);
        this->mpu6050_.set_i2c_master_mode_enabled(false);
        this->mpu6050_.set_slave_address(0, 0x68);
        this->mpu6050_.reset_i2c_master();
        this->write_memory_block(dmp_memory, sizeof(dmp_memory), 0x00, 0x00);

        std::uint8_t dmp_update[]{0x00, 0x01};
        this->write_memory_block(dmp_update, sizeof(dmp_update), 0x02, 0x16);
        this->set_dmp_config1(0x03);
        this->set_dmp_config2(0x00);
        this->set_otp_bank_valid(false);

        this->mpu6050_.set_fifo_enabled(true);
        this->reset_dmp();
        this->set_dmp_enabled(false);
        this->mpu6050_.reset_fifo();
        this->mpu6050_.get_int_status();
    }

    void MPU6050_DMP::deinitialize() noexcept
    {}

    bool MPU6050_DMP::get_otp_bank_valid() const noexcept
    {
        return this->mpu6050_.i2c_device_.read_bit(std::to_underlying(RegAddress::XG_OFFS_TC),
                                                   std::to_underlying(TC::OTP_BNK_VLD_BIT));
    }

    void MPU6050_DMP::set_otp_bank_valid(bool const valid) const noexcept
    {
        this->mpu6050_.i2c_device_.write_bit(std::to_underlying(RegAddress::XG_OFFS_TC),
                                             valid,
                                             std::to_underlying(TC::OTP_BNK_VLD_BIT));
    }

    void MPU6050_DMP::set_gyro_x_offset_tc(std::uint8_t const offset) const noexcept
    {
        this->mpu6050_.i2c_device_.write_bits(std::to_underlying(RegAddress::XG_OFFS_TC),
                                              offset,
                                              std::to_underlying(TC::OFFSET_BIT),
                                              std::to_underlying(TC::OFFSET_LENGTH));
    }

    void MPU6050_DMP::set_gyro_y_offset_tc(std::uint8_t const offset) const noexcept
    {
        this->mpu6050_.i2c_device_.write_bits(std::to_underlying(RegAddress::YG_OFFS_TC),
                                              offset,
                                              std::to_underlying(TC::OFFSET_BIT),
                                              std::to_underlying(TC::OFFSET_LENGTH));
    }

    void MPU6050_DMP::set_gyro_z_offset_tc(std::uint8_t const offset) const noexcept
    {
        this->mpu6050_.i2c_device_.write_bits(std::to_underlying(RegAddress::ZG_OFFS_TC),
                                              offset,
                                              std::to_underlying(TC::OFFSET_BIT),
                                              std::to_underlying(TC::OFFSET_LENGTH));
    }

    void MPU6050_DMP::set_x_fine_gain(std::uint8_t const gain) const noexcept
    {
        this->mpu6050_.i2c_device_.write_byte(std::to_underlying(RegAddress::X_FINE_GAIN), gain);
    }

    void MPU6050_DMP::set_y_fine_gain(std::uint8_t const gain) const noexcept
    {
        this->mpu6050_.i2c_device_.write_byte(std::to_underlying(RegAddress::Y_FINE_GAIN), gain);
    }

    void MPU6050_DMP::set_z_fine_gain(std::uint8_t const gain) const noexcept
    {
        this->mpu6050_.i2c_device_.write_byte(std::to_underlying(RegAddress::Z_FINE_GAIN), gain);
    }

    void MPU6050_DMP::set_accel_x_offset(std::uint16_t const offset) const noexcept
    {
        this->mpu6050_.i2c_device_.write_word(std::to_underlying(RegAddress::XA_OFFS_H), offset);
    }

    void MPU6050_DMP::set_accel_y_offset(std::uint16_t const offset) const noexcept
    {
        this->mpu6050_.i2c_device_.write_word(std::to_underlying(RegAddress::YA_OFFS_H), offset);
    }

    void MPU6050_DMP::set_accel_z_offset(std::uint16_t const offset) const noexcept
    {
        this->mpu6050_.i2c_device_.write_word(std::to_underlying(RegAddress::ZA_OFFS_H), offset);
    }

    void MPU6050_DMP::set_gyro_x_offset(std::uint16_t const offset) const noexcept
    {
        this->mpu6050_.i2c_device_.write_word(std::to_underlying(RegAddress::XG_OFFS_USRH), offset);
    }

    void MPU6050_DMP::set_gyro_y_offset(std::uint16_t const offset) const noexcept
    {
        this->mpu6050_.i2c_device_.write_word(std::to_underlying(RegAddress::YG_OFFS_USRH), offset);
    }

    void MPU6050_DMP::set_gyro_z_offset(std::uint16_t const offset) const noexcept
    {
        this->mpu6050_.i2c_device_.write_word(std::to_underlying(RegAddress::ZG_OFFS_USRH), offset);
    }

    void MPU6050_DMP::set_int_pll_ready_enabled(bool const enabled) const noexcept
    {
        this->mpu6050_.i2c_device_.write_bit(std::to_underlying(RegAddress::INT_ENABLE),
                                             enabled,
                                             std::to_underlying(Interrupt::PLL_RDY_INT_BIT));
    }

    void MPU6050_DMP::set_int_dmp_enabled(bool const enabled) const noexcept
    {
        this->mpu6050_.i2c_device_.write_bit(std::to_underlying(RegAddress::INT_ENABLE),
                                             enabled,
                                             std::to_underlying(Interrupt::DMP_INT_BIT));
    }

    bool MPU6050_DMP::get_dmp_int_5_status() const noexcept
    {
        return this->mpu6050_.i2c_device_.read_bit(std::to_underlying(RegAddress::DMP_INT_STATUS),
                                                   std::to_underlying(IntrDMP::DMPINT_5_BIT));
    }

    bool MPU6050_DMP::get_dmp_int_4_status() const noexcept
    {
        return this->mpu6050_.i2c_device_.read_bit(std::to_underlying(RegAddress::DMP_INT_STATUS),
                                                   std::to_underlying(IntrDMP::DMPINT_4_BIT));
    }

    bool MPU6050_DMP::get_dmp_int_3_status() const noexcept
    {
        return this->mpu6050_.i2c_device_.read_bit(std::to_underlying(RegAddress::DMP_INT_STATUS),
                                                   std::to_underlying(IntrDMP::DMPINT_3_BIT));
    }

    bool MPU6050_DMP::get_dmp_int_2_status() const noexcept
    {
        return this->mpu6050_.i2c_device_.read_bit(std::to_underlying(RegAddress::DMP_INT_STATUS),
                                                   std::to_underlying(IntrDMP::DMPINT_2_BIT));
    }

    bool MPU6050_DMP::get_dmp_int_1_status() const noexcept
    {
        return this->mpu6050_.i2c_device_.read_bit(std::to_underlying(RegAddress::DMP_INT_STATUS),
                                                   std::to_underlying(IntrDMP::DMPINT_1_BIT));
    }

    bool MPU6050_DMP::get_dmp_int_0_status() const noexcept
    {
        return this->mpu6050_.i2c_device_.read_bit(std::to_underlying(RegAddress::DMP_INT_STATUS),
                                                   std::to_underlying(IntrDMP::DMPINT_0_BIT));
    }

    bool MPU6050_DMP::get_int_pll_ready_status() const noexcept
    {
        return this->mpu6050_.i2c_device_.read_bit(std::to_underlying(RegAddress::INT_STATUS),
                                                   std::to_underlying(Interrupt::PLL_RDY_INT_BIT));
    }

    bool MPU6050_DMP::get_int_dmp_status() const noexcept
    {
        return this->mpu6050_.i2c_device_.read_bit(std::to_underlying(RegAddress::INT_STATUS),
                                                   std::to_underlying(Interrupt::DMP_INT_BIT));
    }

    void MPU6050_DMP::set_dmp_enabled(bool const enabled) const noexcept
    {
        this->mpu6050_.i2c_device_.write_bit(std::to_underlying(RegAddress::USER_CTRL),
                                             enabled,
                                             std::to_underlying(UserCtrl::DMP_EN_BIT));
    }

    void MPU6050_DMP::reset_dmp() const noexcept
    {
        this->mpu6050_.i2c_device_.write_bit(std::to_underlying(RegAddress::USER_CTRL),
                                             true,
                                             std::to_underlying(UserCtrl::DMP_RESET_BIT));
    }

    void MPU6050_DMP::set_memory_bank(std::uint8_t const bank,
                                      bool const prefetch_enabled,
                                      bool const user_bank) const noexcept
    {
        std::uint8_t data = bank & 0x1F;
        if (user_bank)
            data |= 0x20;
        if (prefetch_enabled)
            data |= 0x40;
        this->mpu6050_.i2c_device_.write_byte(std::to_underlying(RegAddress::BANK_SEL), data);
    }

    void MPU6050_DMP::set_memory_start_address(std::uint8_t const address) const noexcept
    {
        this->mpu6050_.i2c_device_.write_byte(std::to_underlying(RegAddress::MEM_START_ADDR), address);
    }

    std::uint8_t MPU6050_DMP::read_memory_byte() const noexcept
    {
        return this->mpu6050_.i2c_device_.read_byte(std::to_underlying(RegAddress::MEM_R_W));
    }

    void MPU6050_DMP::write_memory_byte(std::uint8_t const data) const noexcept
    {
        this->mpu6050_.i2c_device_.write_byte(std::to_underlying(RegAddress::MEM_R_W), data);
    }

    void MPU6050_DMP::read_memory_block(std::uint8_t* read_data,
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

            this->mpu6050_.i2c_device_.read_bytes(std::to_underlying(RegAddress::MEM_R_W), read_data + i, chunk_size);
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

    void MPU6050_DMP::write_memory_block(std::uint8_t* write_data,
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

            this->mpu6050_.i2c_device_.write_bytes(std::to_underlying(RegAddress::MEM_R_W), write_data + i, chunk_size);
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

    void MPU6050_DMP::write_dmp_configuration_set(std::uint8_t* write_data, std::size_t const write_size) const noexcept
    {
        for (std::uint16_t i = 0; i < write_size;) {
            std::uint8_t bank = write_data[i++];
            std::uint8_t offset = write_data[i++];
            std::uint8_t length = write_data[i++];

            if (length > 0) {
                this->write_memory_block(write_data + i, length, bank, offset);
                i += length;
            } else {
                if (write_data[i++] == 0x01) {
                    this->mpu6050_.i2c_device_.write_byte(std::to_underlying(RegAddress::INT_ENABLE), 0x32);
                }
            }
        }
    }

    void MPU6050_DMP::set_dmp_config1(std::uint8_t const config) const noexcept
    {
        this->mpu6050_.i2c_device_.write_byte(std::to_underlying(RegAddress::DMP_CFG_1), config);
    }

    void MPU6050_DMP::set_dmp_config2(std::uint8_t const config) const noexcept
    {
        this->mpu6050_.i2c_device_.write_byte(std::to_underlying(RegAddress::DMP_CFG_2), config);
    }
}; // namespace InvertedSway
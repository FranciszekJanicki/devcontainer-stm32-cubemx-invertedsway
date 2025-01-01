#include "mpu_dmp.hpp"
#include "mpu6050.hpp"
#include "mpu_dmp_memory.hpp"
#include "mpu_register_map.hpp"

using namespace InvertedSway;
using Scaled = MPU_DMP::Scaled;
using Raw = MPU_DMP::Raw;
using QuaternionRaw = MPU_DMP::QuaternionRaw;
using QuaternionScaled = MPU_DMP::QuaternionScaled;
using RollPitchYaw = MPU_DMP::RollPitchYaw;
using Gravity = MPU_DMP::Gravity;
using DMP_Packet = MPU_DMP::DMP_Packet;

namespace InvertedSway {

    Gravity MPU_DMP::quaternion_to_gravity(QuaternionScaled const& quaternion) noexcept
    {
        return Gravity{2 * (quaternion.x * quaternion.z - quaternion.w * quaternion.y),
                       2 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z),
                       quaternion.w * quaternion.w - quaternion.x * quaternion.x - quaternion.y * quaternion.y +
                           quaternion.z * quaternion.z};
    }

    RollPitchYaw MPU_DMP::quaternion_to_roll_pitch_yaw(QuaternionScaled const& quaternion) noexcept
    {
        return RollPitchYaw{quaternion_to_roll(quaternion),
                            quaternion_to_pitch(quaternion),
                            quaternion_to_yaw(quaternion)};
    }

    Scaled MPU_DMP::quaternion_to_roll(QuaternionScaled const& quaternion) noexcept
    {
        auto const gravity{quaternion_to_gravity(quaternion)};
        return std::atan2(gravity.y, gravity.z);
    }

    Scaled MPU_DMP::quaternion_to_pitch(QuaternionScaled const& quaternion) noexcept
    {
        auto const gravity{quaternion_to_gravity(quaternion)};
        auto const pitch{std::atan2(gravity.x, sqrt(gravity.y * gravity.y + gravity.z * gravity.z))};
        return (gravity.z < 0) ? (pitch > 0 ? PI - pitch : -PI - pitch) : pitch;
    }

    Scaled MPU_DMP::quaternion_to_yaw(QuaternionScaled const& quaternion) noexcept
    {
        return std::atan2(2 * quaternion.x * quaternion.y - 2 * quaternion.w * quaternion.z,
                          2 * quaternion.w * quaternion.w + 2 * quaternion.x * quaternion.x - 1);
    }

    MPU_DMP::MPU_DMP(MPU6050&& mpu6050) noexcept : mpu6050_{std::forward<MPU6050>(mpu6050)}
    {
        this->initialize();
    }

    MPU_DMP::~MPU_DMP() noexcept
    {
        this->deinitialize();
    }

    void MPU_DMP::initialize() noexcept
    {
        if (this->mpu6050_.initialized_) {
            this->initialize_offsets();
            this->initialize_dmp();
            this->initialized_ = true;
        }
    }

    void MPU_DMP::initialize_dmp() const noexcept
    {
        this->mpu6050_.device_wake_up();
        HAL_Delay(200);
        this->mpu6050_.set_sleep_enabled(false);
        this->set_memory_bank(0x10, true, true);
        this->set_memory_start_address(0x06);
        this->set_memory_bank(0, false, false);
        this->get_otp_bank_valid();

        this->mpu6050_.set_slave_address(0, 0x7F);
        this->mpu6050_.set_i2c_master_mode_enabled(false);
        this->mpu6050_.set_slave_address(0, 0x68);
        this->mpu6050_.reset_i2c_master();
        this->mpu6050_.set_clock_source(MPU6050::Clock::PLL_ZGYRO);
        this->set_int_dmp_enabled(true);
        this->mpu6050_.set_int_fifo_overflow_enabled(true);
        this->mpu6050_.set_sampling_rate(200, MPU6050::DLPF::BW_42); // 1 / (1 + 4) = 200 Hz
        this->mpu6050_.set_external_frame_sync(MPU6050::ExtSync::TEMP_OUT_L);
        this->mpu6050_.set_dlpf_mode(MPU6050::DLPF::BW_42);
        this->mpu6050_.set_full_scale_gyro_range(MPU6050::GyroRange::GYRO_FS_2000);
        this->write_memory_block(dmp_memory.data(), dmp_memory.size(), 0x00, 0x00);

        std::array<std::uint8_t, 2UL> dmp_update{0x00, 0x01};
        this->write_memory_block(dmp_update.data(), 0x02, 0x02, 0x16);
        this->set_dmp_config1(0x03);
        this->set_dmp_config2(0x00);
        this->set_otp_bank_valid(false);

        this->mpu6050_.set_motion_detection_threshold(2);
        this->mpu6050_.set_zero_motion_detection_threshold(156);
        this->mpu6050_.set_motion_detection_duration(80);
        this->mpu6050_.set_zero_motion_detection_duration(0);
        this->mpu6050_.set_fifo_enabled(true);
        this->reset_dmp();
        this->set_dmp_enabled(false);
        this->mpu6050_.get_int_status();
        this->mpu6050_.reset_fifo();
        this->set_dmp_enabled(true);
    }

    void MPU_DMP::initialize_offsets() const noexcept
    {
        this->set_x_gyro_offset(220);
        this->set_y_gyro_offset(76);
        this->set_z_gyro_offset(-85);
        this->set_z_accel_offset(1788);
    }

    void MPU_DMP::deinitialize() noexcept
    {
        this->initialized_ = false;
    }

    bool MPU_DMP::get_otp_bank_valid() const noexcept
    {
        return this->mpu6050_.i2c_device_.read_bit(std::to_underlying(RA::XG_OFFS_TC),
                                                   std::to_underlying(TC::OTP_BNK_VLD_BIT));
    }

    void MPU_DMP::set_otp_bank_valid(bool const valid) const noexcept
    {
        this->mpu6050_.i2c_device_.write_bit(std::to_underlying(RA::XG_OFFS_TC),
                                             valid,
                                             std::to_underlying(TC::OTP_BNK_VLD_BIT));
    }

    void MPU_DMP::set_x_gyro_offset_tc(std::uint8_t const offset) const noexcept
    {
        this->mpu6050_.i2c_device_.write_bits(std::to_underlying(RA::XG_OFFS_TC),
                                              offset,
                                              std::to_underlying(TC::OFFSET_BIT),
                                              std::to_underlying(TC::OFFSET_LENGTH));
    }

    void MPU_DMP::set_y_gyro_offset_tc(std::uint8_t const offset) const noexcept
    {
        this->mpu6050_.i2c_device_.write_bits(std::to_underlying(RA::YG_OFFS_TC),
                                              offset,
                                              std::to_underlying(TC::OFFSET_BIT),
                                              std::to_underlying(TC::OFFSET_LENGTH));
    }

    void MPU_DMP::set_z_gyro_offset_tc(std::uint8_t const offset) const noexcept
    {
        this->mpu6050_.i2c_device_.write_bits(std::to_underlying(RA::ZG_OFFS_TC),
                                              offset,
                                              std::to_underlying(TC::OFFSET_BIT),
                                              std::to_underlying(TC::OFFSET_LENGTH));
    }

    void MPU_DMP::set_x_fine_gain(std::uint8_t const gain) const noexcept
    {
        this->mpu6050_.i2c_device_.write_byte(std::to_underlying(RA::X_FINE_GAIN), gain);
    }

    void MPU_DMP::set_y_fine_gain(std::uint8_t const gain) const noexcept
    {
        this->mpu6050_.i2c_device_.write_byte(std::to_underlying(RA::Y_FINE_GAIN), gain);
    }

    void MPU_DMP::set_z_fine_gain(std::uint8_t const gain) const noexcept
    {
        this->mpu6050_.i2c_device_.write_byte(std::to_underlying(RA::Z_FINE_GAIN), gain);
    }

    void MPU_DMP::set_x_accel_offset(std::int16_t const offset) const noexcept
    {
        this->mpu6050_.i2c_device_.write_word(std::to_underlying(RA::XA_OFFS_H), offset);
    }

    void MPU_DMP::set_y_accel_offset(std::int16_t const offset) const noexcept
    {
        this->mpu6050_.i2c_device_.write_word(std::to_underlying(RA::YA_OFFS_H), offset);
    }

    void MPU_DMP::set_z_accel_offset(std::int16_t const offset) const noexcept
    {
        this->mpu6050_.i2c_device_.write_word(std::to_underlying(RA::ZA_OFFS_H), offset);
    }

    void MPU_DMP::set_x_gyro_offset(std::int16_t const offset) const noexcept
    {
        this->mpu6050_.i2c_device_.write_word(std::to_underlying(RA::XG_OFFS_USRH), offset);
    }

    void MPU_DMP::set_y_gyro_offset(std::int16_t const offset) const noexcept
    {
        this->mpu6050_.i2c_device_.write_word(std::to_underlying(RA::YG_OFFS_USRH), offset);
    }

    void MPU_DMP::set_z_gyro_offset(std::int16_t const offset) const noexcept
    {
        this->mpu6050_.i2c_device_.write_word(std::to_underlying(RA::ZG_OFFS_USRH), offset);
    }

    DMP_Packet MPU_DMP::get_dmp_packet() const noexcept
    {
        DMP_Packet dmp_packet{};

        if (this->get_int_dmp_status()) {
            auto fifo_count{this->mpu6050_.get_fifo_count()};
            if (fifo_count == FIFO_MAX_COUNT) {
                this->mpu6050_.reset_fifo();
            }

            while (fifo_count < dmp_packet.size()) {
            }

            for (auto i{0}; i < fifo_count / dmp_packet.size(); ++i) {
                this->mpu6050_.get_fifo_bytes(dmp_packet.data(), dmp_packet.size());
            }
        }

        return dmp_packet;
    }

    QuaternionRaw MPU_DMP::get_quaternion_raw() const noexcept
    {
        auto packet{this->get_dmp_packet()};
        return QuaternionRaw{(static_cast<Raw>(packet[0]) << 8) | static_cast<Raw>(packet[1]),
                             (static_cast<Raw>(packet[4]) << 8) | static_cast<Raw>(packet[5]),
                             (static_cast<Raw>(packet[8]) << 8) | static_cast<Raw>(packet[9]),
                             (static_cast<Raw>(packet[12]) << 8) | static_cast<Raw>(packet[13])};
    }

    QuaternionScaled MPU_DMP::get_quaternion_scaled() const noexcept
    {
        return static_cast<QuaternionScaled>(this->get_quaternion_raw()) / this->mpu6050_.accel_scale_;
    }

    Gravity MPU_DMP::get_gravity() const noexcept
    {
        return quaternion_to_gravity(this->get_quaternion_scaled());
    }

    Scaled MPU_DMP::get_roll() const noexcept
    {
        return quaternion_to_roll(this->get_quaternion_scaled());
    }

    Scaled MPU_DMP::get_pitch() const noexcept
    {
        return quaternion_to_pitch(this->get_quaternion_scaled());
    }

    Scaled MPU_DMP::get_yaw() const noexcept
    {
        return quaternion_to_yaw(this->get_quaternion_scaled());
    }

    RollPitchYaw MPU_DMP::get_roll_pitch_yaw() const noexcept
    {
        return quaternion_to_roll_pitch_yaw(this->get_quaternion_scaled());
    }

    void MPU_DMP::set_int_pll_ready_enabled(bool const enabled) const noexcept
    {
        this->mpu6050_.i2c_device_.write_bit(std::to_underlying(RA::INT_ENABLE),
                                             enabled,
                                             std::to_underlying(INT_STATUS::PLL_RDY_INT_BIT));
    }

    void MPU_DMP::set_int_dmp_enabled(bool const enabled) const noexcept
    {
        this->mpu6050_.i2c_device_.write_bit(std::to_underlying(RA::INT_ENABLE),
                                             enabled,
                                             std::to_underlying(INT_STATUS::DMP_INT_BIT));
    }

    bool MPU_DMP::get_dmp_int_5_status() const noexcept
    {
        return this->mpu6050_.i2c_device_.read_bit(std::to_underlying(RA::DMP_INT_STATUS),
                                                   std::to_underlying(DMP_INT::DMPINT_5_BIT));
    }

    bool MPU_DMP::get_dmp_int_4_status() const noexcept
    {
        return this->mpu6050_.i2c_device_.read_bit(std::to_underlying(RA::DMP_INT_STATUS),
                                                   std::to_underlying(DMP_INT::DMPINT_4_BIT));
    }

    bool MPU_DMP::get_dmp_int_3_status() const noexcept
    {
        return this->mpu6050_.i2c_device_.read_bit(std::to_underlying(RA::DMP_INT_STATUS),
                                                   std::to_underlying(DMP_INT::DMPINT_3_BIT));
    }

    bool MPU_DMP::get_dmp_int_2_status() const noexcept
    {
        return this->mpu6050_.i2c_device_.read_bit(std::to_underlying(RA::DMP_INT_STATUS),
                                                   std::to_underlying(DMP_INT::DMPINT_2_BIT));
    }

    bool MPU_DMP::get_dmp_int_1_status() const noexcept
    {
        return this->mpu6050_.i2c_device_.read_bit(std::to_underlying(RA::DMP_INT_STATUS),
                                                   std::to_underlying(DMP_INT::DMPINT_1_BIT));
    }

    bool MPU_DMP::get_dmp_int_0_status() const noexcept
    {
        return this->mpu6050_.i2c_device_.read_bit(std::to_underlying(RA::DMP_INT_STATUS),
                                                   std::to_underlying(DMP_INT::DMPINT_0_BIT));
    }

    bool MPU_DMP::get_int_pll_ready_status() const noexcept
    {
        return this->mpu6050_.i2c_device_.read_bit(std::to_underlying(RA::INT_STATUS),
                                                   std::to_underlying(INT_STATUS::PLL_RDY_INT_BIT));
    }

    bool MPU_DMP::get_int_dmp_status() const noexcept
    {
        return this->mpu6050_.i2c_device_.read_bit(std::to_underlying(RA::INT_STATUS),
                                                   std::to_underlying(INT_STATUS::DMP_INT_BIT));
    }

    void MPU_DMP::set_dmp_enabled(bool const enabled) const noexcept
    {
        this->mpu6050_.i2c_device_.write_bit(std::to_underlying(RA::USER_CTRL),
                                             enabled,
                                             std::to_underlying(USER_CTRL::DMP_EN_BIT));
    }

    void MPU_DMP::reset_dmp() const noexcept
    {
        this->mpu6050_.i2c_device_.write_bit(std::to_underlying(RA::USER_CTRL),
                                             true,
                                             std::to_underlying(USER_CTRL::DMP_RESET_BIT));
    }

    void
    MPU_DMP::set_memory_bank(std::uint8_t const bank, bool const prefetch_enabled, bool const user_bank) const noexcept
    {
        std::uint8_t data = bank & 0x1F;
        if (user_bank)
            data |= 0x20;
        if (prefetch_enabled)
            data |= 0x40;
        this->mpu6050_.i2c_device_.write_byte(std::to_underlying(RA::BANK_SEL), data);
    }

    void MPU_DMP::set_memory_start_address(std::uint8_t const address) const noexcept
    {
        this->mpu6050_.i2c_device_.write_byte(std::to_underlying(RA::MEM_START_ADDR), address);
    }

    std::uint8_t MPU_DMP::read_memory_byte() const noexcept
    {
        return this->mpu6050_.i2c_device_.read_byte(std::to_underlying(RA::MEM_R_W));
    }

    void MPU_DMP::write_memory_byte(std::uint8_t const data) const noexcept
    {
        this->mpu6050_.i2c_device_.write_byte(std::to_underlying(RA::MEM_R_W), data);
    }

    void MPU_DMP::read_memory_block(std::uint8_t* read_data,
                                    std::size_t const read_size,
                                    std::uint8_t bank,
                                    std::uint8_t address) const noexcept
    {
        this->set_memory_bank(bank);
        this->set_memory_start_address(address);

        for (std::int16_t i = 0; i < read_size;) {
            std::uint8_t chunk_size = DMP_MEMORY_CHUNK_SIZE;

            if (i + chunk_size > read_size) {
                chunk_size = read_size - i;
            }
            if (chunk_size > 256 - address) {
                chunk_size = 256 - address;
            }

            this->mpu6050_.i2c_device_.read_bytes(std::to_underlying(RA::MEM_R_W), read_data + i, chunk_size);
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

    void MPU_DMP::write_memory_block(std::uint8_t* write_data,
                                     std::size_t const write_size,
                                     std::uint8_t bank,
                                     std::uint8_t address) const noexcept
    {
        this->set_memory_bank(bank);
        this->set_memory_start_address(address);

        for (std::int16_t i = 0; i < write_size;) {
            std::uint8_t chunk_size = DMP_MEMORY_CHUNK_SIZE;

            if (i + chunk_size > write_size) {
                chunk_size = write_size - i;
            }
            if (chunk_size > 256 - address) {
                chunk_size = 256 - address;
            }

            this->mpu6050_.i2c_device_.write_bytes(std::to_underlying(RA::MEM_R_W), write_data + i, chunk_size);
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

    void MPU_DMP::write_dmp_configuration_set(std::uint8_t* write_data, std::size_t const write_size) const noexcept
    {
        for (std::int16_t i = 0; i < write_size;) {
            std::uint8_t bank = write_data[i++];
            std::uint8_t offset = write_data[i++];
            std::uint8_t length = write_data[i++];

            if (length > 0) {
                this->write_memory_block(write_data + i, length, bank, offset);
                i += length;
            } else {
                if (write_data[i++] == 0x01) {
                    this->mpu6050_.i2c_device_.write_byte(std::to_underlying(RA::INT_ENABLE), 0x32);
                }
            }
        }
    }

    void MPU_DMP::set_dmp_config1(std::uint8_t const config) const noexcept
    {
        this->mpu6050_.i2c_device_.write_byte(std::to_underlying(RA::DMP_CFG_1), config);
    }

    void MPU_DMP::set_dmp_config2(std::uint8_t const config) const noexcept
    {
        this->mpu6050_.i2c_device_.write_byte(std::to_underlying(RA::DMP_CFG_2), config);
    }

}; // namespace InvertedSway
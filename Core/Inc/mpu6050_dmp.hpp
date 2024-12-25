#ifndef MPU6050_DMP_HPP
#define MPU6050_DMP_HPP

#include "mpu6050.hpp"

namespace InvertedSway {

    struct MPU6050_DMP {
    public:
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
        using UserCtrl = MPU6050::UserCtrl;
        using Interrupt = MPU6050::Interrupt;
        using IntrLatch = MPU6050::IntrLatch;
        using IntrDrive = MPU6050::IntrDrive;
        using IntrMode = MPU6050::IntrMode;
        using IntrCfg = MPU6050::IntrCfg;
        using IntrClear = MPU6050::IntrClear;
        using IntrDMP = MPU6050::IntrDMP;
        using TC = MPU6050::TC;

        MPU6050_DMP() noexcept = default;

        MPU6050_DMP(MPU6050&& mpu6050) noexcept;

        MPU6050_DMP(MPU6050_DMP const& other) noexcept = delete;
        MPU6050_DMP(MPU6050_DMP&& other) noexcept = default;

        MPU6050_DMP& operator=(MPU6050_DMP const& other) noexcept = delete;
        MPU6050_DMP& operator=(MPU6050_DMP&& other) noexcept = default;

        ~MPU6050_DMP() noexcept;

    private:
        static constexpr std::uint8_t DMP_MEMORY_BANKS{8};
        static constexpr std::size_t DMP_MEMORY_BANK_SIZE{256};
        static constexpr std::size_t DMP_MEMORY_CHUNK_SIZE{16};
        static constexpr auto FIFO_DEFAULT_TIMEOUT{11000};

        bool get_otp_bank_valid() const noexcept;
        void set_otp_bank_valid(bool const enabled) const noexcept;

        void set_gyro_x_offset_tc(std::uint8_t const offset) const noexcept;
        void set_gyro_y_offset_tc(std::uint8_t const offset) const noexcept;
        void set_gyro_z_offset_tc(std::uint8_t const offset) const noexcept;

        void set_x_fine_gain(std::uint8_t const gain) const noexcept;
        void set_y_fine_gain(std::uint8_t const gain) const noexcept;
        void set_z_fine_gain(std::uint8_t const gain) const noexcept;

        void set_accel_x_offset(std::uint16_t const offset) const noexcept;
        void set_accel_y_offset(std::uint16_t const offset) const noexcept;
        void set_accel_z_offset(std::uint16_t const offset) const noexcept;

        void set_gyro_x_offset(std::uint16_t const offset) const noexcept;
        void set_gyro_y_offset(std::uint16_t const offset) const noexcept;
        void set_gyro_z_offset(std::uint16_t const offset) const noexcept;

        void set_int_pll_ready_enabled(bool const enabled) const noexcept;
        void set_int_dmp_enabled(bool const enabled) const noexcept;

        bool get_dmp_int_5_status() const noexcept;
        bool get_dmp_int_4_status() const noexcept;
        bool get_dmp_int_3_status() const noexcept;
        bool get_dmp_int_2_status() const noexcept;
        bool get_dmp_int_1_status() const noexcept;
        bool get_dmp_int_0_status() const noexcept;

        bool get_int_pll_ready_status() const noexcept;
        bool get_int_dmp_status() const noexcept;

        void set_dmp_enabled(bool const enabled) const noexcept;
        void reset_dmp() const noexcept;

        void set_memory_bank(std::uint8_t const bank,
                             bool const prefetch_enabled = false,
                             bool const user_bank = false) const noexcept;
        void set_memory_start_address(std::uint8_t const address) const noexcept;

        std::uint8_t read_memory_byte() const noexcept;

        void write_memory_byte(std::uint8_t write_data) const noexcept;

        void read_memory_block(std::uint8_t* read_data,
                               std::size_t const read_size,
                               std::uint8_t bank,
                               std::uint8_t address) const noexcept;

        void write_memory_block(std::uint8_t* write_data,
                                std::size_t const write_size,
                                std::uint8_t bank,
                                std::uint8_t address) const noexcept;

        void write_dmp_configuration_set(std::uint8_t* write_data, std::size_t const write_size) const noexcept;

        void set_dmp_config1(std::uint8_t const config) const noexcept;

        void set_dmp_config2(std::uint8_t const config) const noexcept;

        MPU6050 mpu6050_{};
    };

}; // namespace InvertedSway

#endif // MPU6050_DMP_HPP
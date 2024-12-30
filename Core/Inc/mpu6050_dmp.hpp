#ifndef MPU6050_DMP_HPP
#define MPU6050_DMP_HPP

#include "mpu6050.hpp"
#include "quaternion3d.hpp"
#include "vector3d.hpp"
#include <array>
#include <cstdint>

namespace InvertedSway {

    struct MPU6050_DMP {
    public:
        enum struct IntrDMP : std::uint8_t {
            DMPINT_5_BIT = 5,
            DMPINT_4_BIT = 4,
            DMPINT_3_BIT = 3,
            DMPINT_2_BIT = 2,
            DMPINT_1_BIT = 1,
            DMPINT_0_BIT = 0,
        };

        enum struct TC : std::uint8_t {
            PWR_MODE_BIT = 7,
            OFFSET_BIT = 6,
            OFFSET_LENGTH = 6,
            OTP_BNK_VLD_BIT = 0,
        };

        using Scaled = MPU6050::Scaled;
        using Raw = MPU6050::Raw;
        using QuaternionRaw = Linalg::Quaternion3D<Raw>;
        using QuaternionScaled = Linalg::Quaternion3D<Scaled>;
        using RollPitchYaw = Linalg::Vector3D<Scaled>;
        using Gravity = Linalg::Vector3D<Scaled>;
        using UserCtrl = MPU6050::UserCtrl;
        using RegAddress = MPU6050::RegAddress;
        using Interrupt = MPU6050::Interrupt;

        static constexpr auto DMP_PACKET_SIZE{42UL};
        using DMP_Packet = std::array<std::uint8_t, DMP_PACKET_SIZE>;

        MPU6050_DMP() noexcept = default;

        MPU6050_DMP(MPU6050&& mpu6050) noexcept;

        MPU6050_DMP(MPU6050_DMP const& other) noexcept = delete;
        MPU6050_DMP(MPU6050_DMP&& other) noexcept = default;

        MPU6050_DMP& operator=(MPU6050_DMP const& other) noexcept = delete;
        MPU6050_DMP& operator=(MPU6050_DMP&& other) noexcept = default;

        ~MPU6050_DMP() noexcept;

        QuaternionRaw get_quaternion_raw() const noexcept;
        QuaternionScaled get_quaternion_scaled() const noexcept;
        Gravity get_gravity() const noexcept;
        RollPitchYaw get_roll_pitch_yaw() const noexcept;

    private:
        static Gravity quaternion_to_gravity(QuaternionScaled const quaternion) noexcept;
        static RollPitchYaw quaternion_to_roll_pitch_yaw(QuaternionScaled const quaternion) noexcept;

        static constexpr auto DMP_MEMORY_BANKS{8};
        static constexpr auto DMP_MEMORY_BANK_SIZE{256UL};
        static constexpr auto DMP_MEMORY_CHUNK_SIZE{16UL};
        static constexpr auto FIFO_DEFAULT_TIMEOUT{11000};
        static constexpr auto FIFO_MAX_COUNT{1024UL};
        static constexpr auto PI{MPU6050::PI};

        void initialize() noexcept;
        void initialize_dmp() const noexcept;
        void initialize_offsets() const noexcept;
        void deinitialize() noexcept;

        bool get_otp_bank_valid() const noexcept;
        void set_otp_bank_valid(bool const enabled) const noexcept;

        void set_x_gyro_offset_tc(std::uint8_t const offset) const noexcept;
        void set_y_gyro_offset_tc(std::uint8_t const offset) const noexcept;
        void set_z_gyro_offset_tc(std::uint8_t const offset) const noexcept;

        void set_x_fine_gain(std::uint8_t const gain) const noexcept;
        void set_y_fine_gain(std::uint8_t const gain) const noexcept;
        void set_z_fine_gain(std::uint8_t const gain) const noexcept;

        void set_x_accel_offset(std::int16_t const offset) const noexcept;
        void set_y_accel_offset(std::int16_t const offset) const noexcept;
        void set_z_accel_offset(std::int16_t const offset) const noexcept;

        void set_x_gyro_offset(std::int16_t const offset) const noexcept;
        void set_y_gyro_offset(std::int16_t const offset) const noexcept;
        void set_z_gyro_offset(std::int16_t const offset) const noexcept;

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

        DMP_Packet get_dmp_packet() const noexcept;

        bool initialized_{false};

        MPU6050 mpu6050_{};
    };

}; // namespace InvertedSway

#endif // MPU6050_DMP_HPP
#ifndef MPU6050_HPP
#define MPU6050_HPP

#include "common.hpp"
#include "stm32l4xx_hal.h"
#include "vector3d.hpp"
#include <cstddef>
#include <cstdint>

namespace InvertedSway {
    struct MPU6050 {
    public:
        enum struct DevAddress : std::uint16_t {
            AD0_LOW = 0x68,
            AD0_HIGH = 0x69,
        };

        enum struct GyroRange : std::uint8_t {
            GYRO_FS_250 = 0x00,
            GYRO_FS_500 = 0x01,
            GYRO_FS_1000 = 0x02,
            GYRO_FS_2000 = 0x03,
        };

        enum struct AccelRange : std::uint8_t {
            ACCEL_FS_2 = 0x00,
            ACCEL_FS_4 = 0x01,
            ACCEL_FS_8 = 0x02,
            ACCEL_FS_16 = 0x03,
        };

        enum struct RegAddress : std::uint8_t {
            XG_OFFS_TC = 0x00,
            YG_OFFS_TC = 0x01,
            ZG_OFFS_TC = 0x02,
            X_FINE_GAIN = 0x03,
            Y_FINE_GAIN = 0x04,
            Z_FINE_GAIN = 0x05,
            XA_OFFS_H = 0x06,
            XA_OFFS_L_TC = 0x07,
            YA_OFFS_H = 0x08,
            YA_OFFS_L_TC = 0x09,
            ZA_OFFS_H = 0x0A,
            ZA_OFFS_L_TC = 0x0B,
            SELF_TEST_X = 0x0D,
            SELF_TEST_Y = 0x0E,
            SELF_TEST_Z = 0x0F,
            SELF_TEST_A = 0x10,
            XG_OFFS_USRH = 0x13,
            XG_OFFS_USRL = 0x14,
            YG_OFFS_USRH = 0x15,
            YG_OFFS_USRL = 0x16,
            ZG_OFFS_USRH = 0x1,
            ZG_OFFS_USRL = 0x18,
            SMPLRT_DIV = 0x19,
            CONFIG = 0x1A,
            GYRO_CONFIG = 0x1B,
            ACCEL_CONFIG = 0x1C,
            FF_THR = 0x1D,
            FF_DUR = 0x1E,
            MOT_THR = 0x1F,
            MOT_DUR = 0x20,
            ZRMOT_THR = 0x21,
            ZRMOT_DUR = 0x22,
            FIFO_EN = 0x23,
            I2C_MST_CTRL = 0x24,
            I2C_SLV0_ADDR = 0x25,
            I2C_SLV0_REG = 0x26,
            I2C_SLV0_CTRL = 0x27,
            I2C_SLV1_ADDR = 0x28,
            I2C_SLV1_REG = 0x29,
            I2C_SLV1_CTRL = 0x2A,
            I2C_SLV2_ADDR = 0x2B,
            I2C_SLV2_REG = 0x2C,
            I2C_SLV2_CTRL = 0x2D,
            I2C_SLV3_ADDR = 0x2E,
            I2C_SLV3_REG = 0x2F,
            I2C_SLV3_CTRL = 0x30,
            I2C_SLV4_ADDR = 0x31,
            I2C_SLV4_REG = 0x32,
            I2C_SLV4_DO = 0x33,
            I2C_SLV4_CTRL = 0x34,
            I2C_SLV4_DI = 0x35,
            I2C_MST_STATUS = 0x36,
            INT_PIN_CFG = 0x37,
            INT_ENABLE = 0x38,
            DMP_INT_STATUS = 0x39,
            INT_STATUS = 0x3A,
            ACCEL_XOUT_H = 0x3B,
            ACCEL_XOUT_L = 0x3C,
            ACCEL_YOUT_H = 0x3D,
            ACCEL_YOUT_L = 0x3E,
            ACCEL_ZOUT_H = 0x3F,
            ACCEL_ZOUT_L = 0x40,
            TEMP_OUT_H = 0x41,
            TEMP_OUT_L = 0x42,
            GYRO_XOUT_H = 0x43,
            GYRO_XOUT_L = 0x44,
            GYRO_YOUT_H = 0x45,
            GYRO_YOUT_L = 0x46,
            GYRO_ZOUT_H = 0x47,
            GYRO_ZOUT_L = 0x48,
            EXT_SENS_DATA_00 = 0x49,
            EXT_SENS_DATA_01 = 0x4A,
            EXT_SENS_DATA_02 = 0x4B,
            EXT_SENS_DATA_03 = 0x4C,
            EXT_SENS_DATA_04 = 0x4D,
            EXT_SENS_DATA_05 = 0x4E,
            EXT_SENS_DATA_06 = 0x4F,
            EXT_SENS_DATA_07 = 0x50,
            EXT_SENS_DATA_08 = 0x51,
            EXT_SENS_DATA_09 = 0x52,
            EXT_SENS_DATA_10 = 0x53,
            EXT_SENS_DATA_11 = 0x54,
            EXT_SENS_DATA_12 = 0x55,
            EXT_SENS_DATA_13 = 0x56,
            EXT_SENS_DATA_14 = 0x57,
            EXT_SENS_DATA_15 = 0x58,
            EXT_SENS_DATA_16 = 0x59,
            EXT_SENS_DATA_17 = 0x5A,
            EXT_SENS_DATA_18 = 0x5B,
            EXT_SENS_DATA_19 = 0x5C,
            EXT_SENS_DATA_20 = 0x5D,
            EXT_SENS_DATA_21 = 0x5E,
            EXT_SENS_DATA_22 = 0x5F,
            EXT_SENS_DATA_23 = 0x60,
            MOT_DETECT_STATUS = 0x61,
            I2C_SLV0_DO = 0x63,
            I2C_SLV1_DO = 0x64,
            I2C_SLV2_DO = 0x65,
            I2C_SLV3_DO = 0x66,
            I2C_MST_DELAY_CTRL = 0x67,
            SIGNAL_PATH_RESET = 0x68,
            MOT_DETECT_CTRL = 0x69,
            USER_CTRL = 0x6A,
            PWR_MGMT_1 = 0x6B,
            PWR_MGMT_2 = 0x6C,
            BANK_SEL = 0x6D,
            MEM_START_ADDR = 0x6E,
            MEM_R_W = 0x6F,
            DMP_CFG_1 = 0x70,
            DMP_CFG_2 = 0x71,
            FIFO_COUNTH = 0x72,
            FIFO_COUNTL = 0x73,
            FIFO_R_W = 0x74,
            WHO_AM_I = 0x75,
        };

        enum struct SelfTest : std::uint8_t {
            XA_1_BIT = 0x07,
            XA_1_LENGTH = 0x03,
            XA_2_BIT = 0x05,
            XA_2_LENGTH = 0x02,
            YA_1_BIT = 0x07,
            YA_1_LENGTH = 0x03,
            YA_2_BIT = 0x03,
            YA_2_LENGTH = 0x02,
            ZA_1_BIT = 0x07,
            ZA_1_LENGTH = 0x03,
            ZA_2_BIT = 0x01,
            ZA_2_LENGTH = 0x02,
            XG_1_BIT = 0x04,
            XG_1_LENGTH = 0x05,
            YG_1_BIT = 0x04,
            YG_1_LENGTH = 0x05,
            ZG_1_BIT = 0x04,
            ZG_1_LENGTH = 0x05,
        };

        enum struct Config : std::uint8_t {
            EXT_SYNC_SET_BIT = 5,
            EXT_SYNC_SET_LENGTH = 3,
            DLPF_CFG_BIT = 2,
            DLPF_CFG_LENGTH = 3,
        };

        enum struct ExtSync : std::uint8_t {
            DISABLED = 0x0,
            TEMP_OUT_L = 0x1,
            GYRO_XOUT_L = 0x2,
            GYRO_YOUT_L = 0x3,
            GYRO_ZOUT_L = 0x4,
            ACCEL_XOUT_L = 0x5,
            ACCEL_YOUT_L = 0x6,
            ACCEL_ZOUT_L = 0x7,
        };

        enum struct DLPF : std::uint8_t {
            BW_256 = 0x00,
            BW_188 = 0x01,
            BW_98 = 0x02,
            BW_42 = 0x03,
            BW_20 = 0x04,
            BW_10 = 0x05,
            BW_5 = 0x06,
        };

        enum struct TC : std::uint8_t {
            PWR_MODE_BIT = 7,
            OFFSET_BIT = 6,
            OFFSET_LENGTH = 6,
            OTP_BNK_VLD_BIT = 0,
        };

        enum struct GyroConfig : std::uint8_t {
            FS_SEL_BIT = 4,
            FS_SEL_LENGTH = 2,
        };

        enum struct AccelConfig : std::uint8_t {
            XA_ST_BIT = 7,
            YA_ST_BIT = 6,
            ZA_ST_BIT = 5,
            AFS_SEL_BIT = 4,
            AFS_SEL_LENGTH = 2,
            ACCEL_HPF_BIT = 2,
            ACCEL_HPF_LENGTH = 3,
        };

        enum struct DHPF : std::uint8_t {
            DHPF_RESET = 0x00,
            DHPF_5 = 0x01,
            DHPF_2P5 = 0x02,
            DHPF_1P25 = 0x03,
            DHPF_0P63 = 0x04,
            DHPF_HOLD = 0x07,
        };

        enum struct FIFO : std::uint8_t {
            TEMP_EN_BIT = 7,
            XG_EN_BIT = 6,
            YG_EN_BIT = 5,
            ZG_EN_BIT = 4,
            ACCEL_EN_BIT = 3,
            SLV2_EN_BIT = 2,
            SLV1_EN_BIT = 1,
            SLV0_EN_BIT = 0,
        };

        enum struct ClockDiv : std::uint8_t {
            DIV_500 = 0x9,
            DIV_471 = 0xA,
            DIV_444 = 0xB,
            DIV_421 = 0xC,
            DIV_400 = 0xD,
            DIV_381 = 0xE,
            DIV_364 = 0xF,
            DIV_348 = 0x0,
            DIV_333 = 0x1,
            DIV_320 = 0x2,
            DIV_308 = 0x3,
            DIV_296 = 0x4,
            DIV_286 = 0x5,
            DIV_276 = 0x6,
            DIV_267 = 0x7,
            DIV_258 = 0x8,
        };

        enum struct I2CSlave : std::uint8_t {
            SLV_RW_BIT = 7,
            SLV_ADDR_BIT = 6,
            SLV_ADDR_LENGTH = 7,
            SLV_EN_BIT = 7,
            SLV_BYTE_SW_BIT = 6,
            SLV_REG_DIS_BIT = 5,
            SLV_GRP_BIT = 4,
            SLV_LEN_BIT = 3,
            SLV_LEN_LENGTH = 4,
            SLV_3_FIFO_EN_BIT = 5,
        };

        enum struct I2CSlave4 : std::uint8_t {
            SLV4_RW_BIT = 7,
            SLV4_ADDR_BIT = 6,
            SLV4_ADDR_LENGTH = 7,
            SLV4_EN_BIT = 7,
            SLV4_INT_EN_BIT = 6,
            SLV4_REG_DIS_BIT = 5,
            SLV4_MST_DLY_BIT = 4,
            SLV4_MST_DLY_LENGTH = 5,
        };

        enum struct I2CMaster : std::uint8_t {
            PASS_THROUGH_BIT = 7,
            SLV4_DONE_BIT = 6,
            LOST_ARB_BIT = 5,
            SLV4_NACK_BIT = 4,
            SLV3_NACK_BIT = 3,
            SLV2_NACK_BIT = 2,
            SLV1_NACK_BIT = 1,
            SLV0_NACK_BIT = 0,
        };

        enum struct I2C : std::uint8_t {
            MULT_MST_EN_BIT = 7,
            MST_CLK_LENGTH = 4,
            MST_P_NSR_BIT = 4,
            MST_CLK_BIT = 3,
            WAIT_FOR_ES_BIT = 6,
        };

        enum struct IntrCfg : std::uint8_t {
            INT_LEVEL_BIT = 7,
            INT_OPEN_BIT = 6,
            LATCH_INT_EN_BIT = 5,
            INT_RD_CLEAR_BIT = 4,
            FSYNC_INT_LEVEL_BIT = 3,
            FSYNC_INT_EN_BIT = 2,
            I2C_BYPASS_EN_BIT = 1,
        };

        enum struct IntrMode : std::uint8_t {
            ACTIVEHIGH = 0x00,
            ACTIVELOW = 0x01,
        };

        enum struct IntrDrive : std::uint8_t {
            PUSHPULL = 0x00,
            OPENDRAIN = 0x01,
        };

        enum struct IntrLatch : std::uint8_t {
            PULSE50US = 0x00,
            WAITCLEAR = 0x01,
        };

        enum struct IntrClear : std::uint8_t {
            STATUSREAD = 0x00,
            ANYREAD = 0x01,
        };

        enum struct Interrupt : std::uint8_t {
            FF_BIT = 7,
            MOT_BIT = 6,
            ZMOT_BIT = 5,
            FIFO_OFLOW_BIT = 4,
            I2C_MST_INT_BIT = 3,
            PLL_RDY_INT_BIT = 2,
            DMP_INT_BIT = 1,
            DATA_RDY_BIT = 0,
        };

        enum struct IntrDMP : std::uint8_t {
            DMPINT_5_BIT = 5,
            DMPINT_4_BIT = 4,
            DMPINT_3_BIT = 3,
            DMPINT_2_BIT = 2,
            DMPINT_1_BIT = 1,
            DMPINT_0_BIT = 0,
        };

        enum struct Motion : std::uint8_t {
            MOT_XNEG_BIT = 7,
            MOT_XPOS_BIT = 6,
            MOT_YNEG_BIT = 5,
            MOT_YPOS_BIT = 4,
            MOT_ZNEG_BIT = 3,
            MOT_ZPOS_BIT = 2,
            MOT_ZRMOT_BIT = 0,
        };

        enum struct DelayCtrl : std::uint8_t {
            DELAY_ES_SHADOW_BIT = 7,
            I2C_SLV4_DLY_EN_BIT = 4,
            I2C_SLV3_DLY_EN_BIT = 3,
            I2C_SLV2_DLY_EN_BIT = 2,
            I2C_SLV1_DLY_EN_BIT = 1,
            I2C_SLV0_DLY_EN_BIT = 0,
        };

        enum struct PathReset : std::uint8_t {
            GYRO_RESET_BIT = 2,
            ACCEL_RESET_BIT = 1,
            TEMP_RESET_BIT = 0,
        };

        enum struct Detect : std::uint8_t {
            ACCEL_ON_DELAY_BIT = 5,
            ACCEL_ON_DELAY_LENGTH = 2,
            FF_COUNT_BIT = 3,
            FF_COUNT_LENGTH = 2,
            MOT_COUNT_BIT = 1,
            MOT_COUNT_LENGTH = 2,
        };

        enum struct DetectDecrement : std::uint8_t {
            DECREMENT_RESET = 0x0,
            DECREMENT_1 = 0x1,
            DECREMENT_2 = 0x2,
            DECREMENT_4 = 0x3,
        };

        enum struct Delay : std::uint8_t {
            DELAY_3MS = 0b11,
            DELAY_2MS = 0b10,
            DELAY_1MS = 0b01,
            NO_DELAY = 0b00,
        };

        enum struct UserCtrl : std::uint8_t {
            DMP_EN_BIT = 7,
            FIFO_EN_BIT = 6,
            I2C_MST_EN_BIT = 5,
            I2C_IF_DIS_BIT = 4,
            FIFO_RESET_BIT = 2,
            DMP_RESET_BIT = 3,
            I2C_MST_RESET_BIT = 1,
            SIG_COND_RESET_BIT = 0,
        };

        enum struct Power1 : std::uint8_t {
            DEVICE_RESET_BIT = 7,
            SLEEP_BIT = 6,
            CYCLE_BIT = 5,
            TEMP_DIS_BIT = 3,
            CLKSEL_BIT = 2,
            CLKSEL_LENGTH = 3,
        };

        enum struct Clock : std::uint8_t {
            INTERNAL = 0x00,
            PLL_XGYRO = 0x01,
            PLL_YGYRO = 0x02,
            PLL_ZGYRO = 0x03,
            PLL_EXT32K = 0x04,
            PLL_EXT19M = 0x05,
            KEEP_RESET = 0x07,
        };

        enum struct Power2 : std::uint8_t {
            LP_WAKE_CTRL_BIT = 7,
            LP_WAKE_CTRL_LENGTH = 2,
            STBY_XA_BIT = 5,
            STBY_YA_BIT = 4,
            STBY_ZA_BIT = 3,
            STBY_XG_BIT = 2,
            STBY_YG_BIT = 1,
            STBY_ZG_BIT = 0,
        };

        enum struct WakeFreq : std::uint8_t {
            FREQ_1P25 = 0x0,
            FREQ_5 = 0x1,
            FREQ_20 = 0x2,
            FREQ_40 = 0x3,
        };

        enum struct WhoAmI : std::uint8_t {
            BIT = 6,
            LENGTH = 6,
        };

        using Scaled = float;
        using GyroScaled = Linalg::Vector3D<Scaled>;   // radians
        using AccelScaled = Linalg::Vector3D<Scaled>;  // m/s^2
        using RollPitchYaw = Linalg::Vector3D<Scaled>; // degrees
        using TempScaled = float;                      // celsius
        using Raw = std::uint16_t;
        using GyroRaw = Linalg::Vector3D<Raw>;
        using AccelRaw = Linalg::Vector3D<Raw>;
        using TempRaw = std::uint16_t;

        MPU6050() noexcept = default;

        MPU6050(I2CHandle const i2c_bus,
                DevAddress const device_address,
                GyroRange const gyro_range,
                AccelRange const accel_range,
                std::uint32_t const sampling_rate) noexcept;

        MPU6050(MPU6050 const& other) noexcept = delete;
        MPU6050(MPU6050&& other) noexcept = default;

        MPU6050& operator=(MPU6050 const& other) noexcept = delete;
        MPU6050& operator=(MPU6050&& other) noexcept = default;

        ~MPU6050() noexcept;

        /* celsius */
        [[nodiscard]] TempScaled get_temperature_celsius() const noexcept;

        /* meters per square second */
        [[nodiscard]] AccelScaled get_acceleration_scaled() const noexcept;
        [[nodiscard]] Scaled get_acceleration_x_scaled() const noexcept;
        [[nodiscard]] Scaled get_acceleration_y_scaled() const noexcept;
        [[nodiscard]] Scaled get_acceleration_z_scaled() const noexcept;

        /* radians */
        [[nodiscard]] GyroScaled get_rotation_scaled() const noexcept;
        [[nodiscard]] Scaled get_rotation_x_scaled() const noexcept;
        [[nodiscard]] Scaled get_rotation_y_scaled() const noexcept;
        [[nodiscard]] Scaled get_rotation_z_scaled() const noexcept;

        /* degrees */
        [[nodiscard]] RollPitchYaw get_roll_pitch_yaw() const noexcept;
        [[nodiscard]] Scaled get_roll() const noexcept;
        [[nodiscard]] Scaled get_pitch() const noexcept;
        [[nodiscard]] Scaled get_yaw() const noexcept;

    private:
        static Scaled gyro_range_to_scale(GyroRange const gyro_range) noexcept;
        static Scaled accel_range_to_scale(AccelRange const accel_range) noexcept;

        static std::uint8_t get_sampling_divider(std::uint32_t const rate, DLPF const dlpf) noexcept;

        static constexpr Scaled PI{3.1415f};
        static constexpr std::uint32_t I2C_TIMEOUT{100};
        static constexpr std::uint32_t GYRO_OUTPUT_RATE_DLPF_EN_HZ{1000};
        static constexpr std::uint32_t GYRO_OUTPUT_RATE_DLPF_DIS_HZ{8000};
        static constexpr std::uint32_t ACCEL_OUTPUT_RATE_HZ{1000};

        static constexpr std::uint8_t DMP_MEMORY_BANKS{8};
        static constexpr std::size_t DMP_MEMORY_BANK_SIZE{256};
        static constexpr std::size_t DMP_MEMORY_CHUNK_SIZE{16};
        static constexpr auto FIFO_DEFAULT_TIMEOUT{11000};

        void initialize(std::uint32_t const sampling_rate) noexcept;
        void deinitialize() noexcept;

        void i2c_write_words(RegAddress const reg_address,
                             std::uint16_t* write_data,
                             std::uint8_t write_size) const noexcept;

        void i2c_write_word(RegAddress const reg_address, std::uint16_t write_data) const noexcept;

        void i2c_write_bytes(RegAddress const reg_address,
                             std::uint8_t* write_data,
                             std::size_t const write_size) const noexcept;

        void i2c_write_byte(RegAddress const reg_address, std::uint8_t write_data) const noexcept;

        void i2c_write_bit(RegAddress const reg_address,
                           bool const write_data,
                           std::uint8_t const write_position) const noexcept;

        void i2c_write_bits(RegAddress const reg_address,
                            std::uint8_t const write_data,
                            std::uint8_t const write_position,
                            std::uint8_t const write_size) const noexcept;

        void i2c_read_words(RegAddress const reg_address,
                            std::uint16_t* read_data,
                            std::uint8_t const read_size) const noexcept;

        std::uint16_t i2c_read_word(RegAddress const reg_address) const noexcept;

        void i2c_read_bytes(RegAddress const reg_address,
                            std::uint8_t* read_data,
                            std::size_t const read_size) const noexcept;

        std::uint8_t i2c_read_byte(RegAddress const reg_address) const noexcept;

        bool i2c_read_bit(RegAddress const reg_address, std::uint8_t const read_position) const noexcept;

        std::uint8_t i2c_read_bits(RegAddress const reg_address,
                                   std::uint8_t const read_position,
                                   std::uint8_t const read_size) const noexcept;

        void set_sampling_divider(std::uint8_t const divider) const noexcept;

        void set_external_frame_sync(std::uint8_t const frame_sync) const noexcept;
        void set_dlpf_mode(DLPF const dlpf) const noexcept;

        void set_full_scale_gyro_range(GyroRange const range) const noexcept;

        void set_full_scale_accel_range(AccelRange const range) const noexcept;
        void set_dhpf_mode(DHPF const dhpf) const noexcept;

        void set_free_fall_detection_threshold(std::uint8_t const threshold) const noexcept;
        void set_free_fall_detection_duration(std::uint8_t const duration) const noexcept;

        void set_motion_detection_threshold(std::uint8_t const threshold) const noexcept;
        void set_motion_detection_duration(std::uint8_t const duration) const noexcept;

        void set_zero_motion_detection_threshold(std::uint8_t const threshold) const noexcept;
        void set_zero_motion_detection_duration(std::uint8_t const duration) const noexcept;

        void set_temp_fifo_enabled(bool const enabled) const noexcept;
        void set_gyro_x_fifo_enabled(bool const enabled) const noexcept;
        void set_gyro_y_fifo_enabled(bool const enabled) const noexcept;
        void set_gyro_z_fifo_enabled(bool const enabled) const noexcept;
        void set_accel_fifo_enabled(bool const enabled) const noexcept;

        void set_interrupt() const noexcept;
        void set_interrupt_mode(IntrMode const mode) const noexcept;
        void set_interrupt_drive(IntrDrive const drive) const noexcept;
        void set_interrupt_latch(IntrLatch const latch) const noexcept;
        void set_interrupt_latch_clear(IntrClear const clear) const noexcept;

        void set_motion_interrupt() const noexcept;
        void set_int_enabled(std::uint8_t const enabled) const noexcept;
        void set_int_free_fall_enabled(bool const enabled) const noexcept;
        void set_int_motion_enabled(bool const enabled) const noexcept;
        void set_int_zero_motion_enabled(bool const enabled) const noexcept;
        void set_int_fifo_overflow_enabled(bool const enabled) const noexcept;
        void set_int_i2c_master_enabled(bool const enabled) const noexcept;
        void set_int_data_ready_enabled(bool const enabled) const noexcept;

        std::uint8_t get_int_status() const noexcept;
        bool get_int_free_fall_status() const noexcept;
        bool get_int_motion_status() const noexcept;
        bool get_int_zero_motion_status() const noexcept;
        bool get_int_fifo_overflow_status() const noexcept;
        bool get_int_i2c_master_status() const noexcept;
        bool get_int_data_ready_status() const noexcept;

        AccelRaw get_acceleration_raw() const noexcept;
        Raw get_acceleration_x_raw() const noexcept;
        Raw get_acceleration_y_raw() const noexcept;
        Raw get_acceleration_z_raw() const noexcept;

        Raw get_temperature_raw() const noexcept;

        GyroRaw get_rotation_raw() const noexcept;
        Raw get_rotation_x_raw() const noexcept;
        Raw get_rotation_y_raw() const noexcept;
        Raw get_rotation_z_raw() const noexcept;

        std::uint8_t get_motion_status() const noexcept;
        bool get_x_neg_motion_detected() const noexcept;
        bool get_x_pos_motion_detected() const noexcept;
        bool get_y_neg_motion_detected() const noexcept;
        bool get_y_pos_motion_detected() const noexcept;
        bool get_z_neg_motion_detected() const noexcept;
        bool get_z_pos_motion_detected() const noexcept;

        void reset_gyro_path() const noexcept;
        void reset_accel_path() const noexcept;
        void reset_temperature_path() const noexcept;

        void set_accel_power_on_delay(Delay const delay) const noexcept;
        void set_free_fall_detection_counter_decrement(DetectDecrement const decrement) const noexcept;
        void set_motion_detection_counter_decrement(DetectDecrement const decrement) const noexcept;

        void set_fifo_enabled(bool const enabled) const noexcept;
        void set_i2c_master_mode_enabled(bool const enabled) const noexcept;
        void reset_fifo() const noexcept;
        void reset_i2c_master() const noexcept;
        void reset_sensors() const noexcept;

        void device_reset() const noexcept;
        void set_clock_source(Clock const source) const noexcept;
        void set_sleep_enabled(bool const enabled) const noexcept;
        void set_wake_cycle_enabled(bool const enabled) const noexcept;
        void set_temperature_sensor_enabled(bool const enabled) const noexcept;

        void set_wake_up_frequency(WakeFreq const frequency) const noexcept;
        void set_accel_x_axis_standby(bool const standby) const noexcept;
        void set_accel_y_axis_standby(bool const standby) const noexcept;
        void set_accel_z_axis_standby(bool const standby) const noexcept;
        void set_gyro_x_axis_standby(bool const standby) const noexcept;
        void set_gyro_y_axis_standby(bool const standby) const noexcept;
        void set_gyro_z_axis_standby(bool const standby) const noexcept;

        std::uint16_t get_fifo_count() const noexcept;
        std::uint8_t get_fifo_byte() const noexcept;
        void get_fifo_bytes(std::uint8_t* read_data, std::uint8_t const read_size) const noexcept;
        void set_fifo_byte(std::uint8_t const write_data) const noexcept;
        void set_fifo_bytes(std::uint8_t* write_data, std::uint8_t const write_size) const noexcept;

        std::uint8_t get_device_id() const noexcept;

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
                               std::uint8_t const read_size,
                               std::uint8_t bank,
                               std::uint8_t address) const noexcept;

        void write_memory_block(std::uint8_t* write_data,
                                std::uint8_t const write_size,
                                std::uint8_t bank,
                                std::uint8_t address) const noexcept;

        void write_dmp_configuration_set(std::uint8_t* write_data, std::uint8_t const write_size) const noexcept;

        void set_dmp_config1(std::uint8_t const config) const noexcept;

        void set_dmp_config2(std::uint8_t const config) const noexcept;

        bool initialized_{false};

        I2CHandle i2c_bus_{nullptr};
        DevAddress device_address_{};

        GyroRange gyro_range_{};
        AccelRange accel_range_{};
    };

}; // namespace InvertedSway

#endif // MPU6050_HPP
#ifndef MPU6050_HPP
#define MPU6050_HPP

#include "common.hpp"
#include "filters.hpp"
#include "stm32l4xx_hal.h"
#include "vector3d.hpp"
#include <cstddef>
#include <cstdint>
#include <expected>
#include <functional>

namespace InvertedSway {

    struct MPU6050 {
    public:
        enum struct Error {
            OK,
            INIT,
            GYRO,
            ACCEL,
            TEMP,
            DEINIT,
        };

        enum Address : std::uint8_t {
            ADDRESS = 0xD0,  // AD0 low
            ADDRESS2 = 0xD1, // AD0 high
        };

        enum GyroRange : std::uint8_t {
            GYRO_FS_250 = 0x00,
            GYRO_FS_500 = 0x01,
            GYRO_FS_1000 = 0x02,
            GYRO_FS_2000 = 0x03,
        };

        enum AccelRange : std::uint8_t {
            ACCEL_FS_2 = 0x00,
            ACCEL_FS_4 = 0x01,
            ACCEL_FS_8 = 0x02,
            ACCEL_FS_16 = 0x03,
        };

        using Scaled = float;
        using GyroScaled = Linalg::Vector3D<Scaled>;   // radians
        using AccelScaled = Linalg::Vector3D<Scaled>;  // m/s^2
        using RollPitchYaw = Linalg::Vector3D<Scaled>; // degrees
        using TempScaled = float;                      // celsius
        using Raw = std::int16_t;
        using GyroRaw = Linalg::Vector3D<Raw>;
        using AccelRaw = Linalg::Vector3D<Raw>;
        using TempRaw = std::int16_t;

        [[nodiscard]] static const char* error_to_string(const Error error) noexcept;

        static constexpr std::uint32_t SAMPLING_RATE_HZ{8000};
        static constexpr float SAMPLING_TIME_S{1.f / static_cast<float>(SAMPLING_RATE_HZ)};

        MPU6050() noexcept = default;

        MPU6050(I2cHandle i2c, const Address addres, const GyroRange gyro_range, const AccelRange accel_range) noexcept;

        MPU6050(const MPU6050& other) noexcept = default;
        MPU6050(MPU6050&& other) noexcept = default;

        MPU6050& operator=(const MPU6050& other) noexcept = default;
        MPU6050& operator=(MPU6050&& other) noexcept = default;

        ~MPU6050() noexcept;

        /* celcius */
        [[nodiscard]] TempScaled get_temperature_celsius() const noexcept;

        /* meters per square second */
        [[nodiscard]] AccelScaled get_accelerometer_scaled() const noexcept;
        [[nodiscard]] Scaled get_acceleration_x_scaled() const noexcept;
        [[nodiscard]] Scaled get_acceleration_y_scaled() const noexcept;
        [[nodiscard]] Scaled get_acceleration_z_scaled() const noexcept;

        /* radians */
        [[nodiscard]] GyroScaled get_gyroscope_scaled() const noexcept;
        [[nodiscard]] Scaled get_rotation_x_scaled() const noexcept;
        [[nodiscard]] Scaled get_rotation_y_scaled() const noexcept;
        [[nodiscard]] Scaled get_rotation_z_scaled() const noexcept;

        /* degrees */
        [[nodiscard]] RollPitchYaw get_roll_pitch_yaw() const noexcept;
        [[nodiscard]] Scaled get_roll() const noexcept;
        [[nodiscard]] Scaled get_pitch() const noexcept;
        [[nodiscard]] Scaled get_yaw() const noexcept;

    private:
        enum RegAddress : std::uint8_t {
            RA_SELF_TEST_X = 0x0D,
            RA_SELF_TEST_Y = 0x0E,
            RA_SELF_TEST_Z = 0x0F,
            RA_SELF_TEST_A = 0x10,
            RA_SMPLRT_DIV = 0x19,
            RA_CONFIG = 0x1A,
            RA_GYRO_CONFIG = 0x1B,
            RA_ACCEL_CONFIG = 0x1C,
            RA_FF_THR = 0x1D,
            RA_FF_DUR = 0x1E,
            RA_MOT_THR = 0x1F,
            RA_MOT_DUR = 0x20,
            RA_ZRMOT_THR = 0x21,
            RA_ZRMOT_DUR = 0x22,
            RA_FIFO_EN = 0x23,
            RA_I2C_MST_CTRL = 0x24,
            RA_I2C_SLV0_ADDR = 0x25,
            RA_I2C_SLV0_REG = 0x26,
            RA_I2C_SLV0_CTRL = 0x27,
            RA_I2C_SLV1_ADDR = 0x28,
            RA_I2C_SLV1_REG = 0x29,
            RA_I2C_SLV1_CTRL = 0x2A,
            RA_I2C_SLV2_ADDR = 0x2B,
            RA_I2C_SLV2_REG = 0x2C,
            RA_I2C_SLV2_CTRL = 0x2D,
            RA_I2C_SLV3_ADDR = 0x2E,
            RA_I2C_SLV3_REG = 0x2F,
            RA_I2C_SLV3_CTRL = 0x30,
            RA_I2C_SLV4_ADDR = 0x31,
            RA_I2C_SLV4_REG = 0x32,
            RA_I2C_SLV4_DO = 0x33,
            RA_I2C_SLV4_CTRL = 0x34,
            RA_I2C_SLV4_DI = 0x35,
            RA_I2C_MST_STATUS = 0x36,
            RA_INT_PIN_CFG = 0x37,
            RA_INT_ENABLE = 0x38,
            RA_DMP_INT_STATUS = 0x39,
            RA_INT_STATUS = 0x3A,
            RA_ACCEL_XOUT_H = 0x3B,
            RA_ACCEL_XOUT_L = 0x3C,
            RA_ACCEL_YOUT_H = 0x3D,
            RA_ACCEL_YOUT_L = 0x3E,
            RA_ACCEL_ZOUT_H = 0x3F,
            RA_ACCEL_ZOUT_L = 0x40,
            RA_TEMP_OUT_H = 0x41,
            RA_TEMP_OUT_L = 0x42,
            RA_GYRO_XOUT_H = 0x43,
            RA_GYRO_XOUT_L = 0x44,
            RA_GYRO_YOUT_H = 0x45,
            RA_GYRO_YOUT_L = 0x46,
            RA_GYRO_ZOUT_H = 0x47,
            RA_GYRO_ZOUT_L = 0x48,
            RA_EXT_SENS_DATA_00 = 0x49,
            RA_EXT_SENS_DATA_01 = 0x4A,
            RA_EXT_SENS_DATA_02 = 0x4B,
            RA_EXT_SENS_DATA_03 = 0x4C,
            RA_EXT_SENS_DATA_04 = 0x4D,
            RA_EXT_SENS_DATA_05 = 0x4E,
            RA_EXT_SENS_DATA_06 = 0x4F,
            RA_EXT_SENS_DATA_07 = 0x50,
            RA_EXT_SENS_DATA_08 = 0x51,
            RA_EXT_SENS_DATA_09 = 0x52,
            RA_EXT_SENS_DATA_10 = 0x53,
            RA_EXT_SENS_DATA_11 = 0x54,
            RA_EXT_SENS_DATA_12 = 0x55,
            RA_EXT_SENS_DATA_13 = 0x56,
            RA_EXT_SENS_DATA_14 = 0x57,
            RA_EXT_SENS_DATA_15 = 0x58,
            RA_EXT_SENS_DATA_16 = 0x59,
            RA_EXT_SENS_DATA_17 = 0x5A,
            RA_EXT_SENS_DATA_18 = 0x5B,
            RA_EXT_SENS_DATA_19 = 0x5C,
            RA_EXT_SENS_DATA_20 = 0x5D,
            RA_EXT_SENS_DATA_21 = 0x5E,
            RA_EXT_SENS_DATA_22 = 0x5F,
            RA_EXT_SENS_DATA_23 = 0x60,
            RA_MOT_DETECT_STATUS = 0x61,
            RA_I2C_SLV0_DO = 0x63,
            RA_I2C_SLV1_DO = 0x64,
            RA_I2C_SLV2_DO = 0x65,
            RA_I2C_SLV3_DO = 0x66,
            RA_I2C_MST_DELAY_CTRL = 0x67,
            RA_SIGNAL_PATH_RESET = 0x68,
            RA_MOT_DETECT_CTRL = 0x69,
            RA_USER_CTRL = 0x6A,
            RA_PWR_MGMT_1 = 0x6B,
            RA_PWR_MGMT_2 = 0x6C,
            RA_FIFO_COUNTH = 0x72,
            RA_FIFO_COUNTL = 0x73,
            RA_FIFO_R_W = 0x74,
            RA_WHO_AM_I = 0x75,
        };

        enum SelfTest : std::uint8_t {
            SELF_TEST_XA_1_BIT = 0x07,
            SELF_TEST_XA_1_LENGTH = 0x03,
            SELF_TEST_XA_2_BIT = 0x05,
            SELF_TEST_XA_2_LENGTH = 0x02,
            SELF_TEST_YA_1_BIT = 0x07,
            SELF_TEST_YA_1_LENGTH = 0x03,
            SELF_TEST_YA_2_BIT = 0x03,
            SELF_TEST_YA_2_LENGTH = 0x02,
            SELF_TEST_ZA_1_BIT = 0x07,
            SELF_TEST_ZA_1_LENGTH = 0x03,
            SELF_TEST_ZA_2_BIT = 0x01,
            SELF_TEST_ZA_2_LENGTH = 0x02,
            SELF_TEST_XG_1_BIT = 0x04,
            SELF_TEST_XG_1_LENGTH = 0x05,
            SELF_TEST_YG_1_BIT = 0x04,
            SELF_TEST_YG_1_LENGTH = 0x05,
            SELF_TEST_ZG_1_BIT = 0x04,
            SELF_TEST_ZG_1_LENGTH = 0x05,
        };

        enum Config : std::uint8_t {
            CFG_EXT_SYNC_SET_BIT = 5,
            CFG_EXT_SYNC_SET_LENGTH = 3,
            CFG_DLPF_CFG_BIT = 2,
            CFG_DLPF_CFG_LENGTH = 3,
        };

        enum ExtSync : std::uint8_t {
            EXT_SYNC_DISABLED = 0x0,
            EXT_SYNC_TEMP_OUT_L = 0x1,
            EXT_SYNC_GYRO_XOUT_L = 0x2,
            EXT_SYNC_GYRO_YOUT_L = 0x3,
            EXT_SYNC_GYRO_ZOUT_L = 0x4,
            EXT_SYNC_ACCEL_XOUT_L = 0x5,
            EXT_SYNC_ACCEL_YOUT_L = 0x6,
            EXT_SYNC_ACCEL_ZOUT_L = 0x7,
        };

        enum DLPF : std::uint8_t {
            DLPF_BW_256 = 0x00,
            DLPF_BW_188 = 0x01,
            DLPF_BW_98 = 0x02,
            DLPF_BW_42 = 0x03,
            DLPF_BW_20 = 0x04,
            DLPF_BW_10 = 0x05,
            DLPF_BW_5 = 0x06,
        };

        enum GyroConfig : std::uint8_t {
            GCONFIG_FS_SEL_BIT = 4,
            GCONFIG_FS_SEL_LENGTH = 2,
        };

        enum AccelConfig : std::uint8_t {
            ACONFIG_XA_ST_BIT = 7,
            ACONFIG_YA_ST_BIT = 6,
            ACONFIG_ZA_ST_BIT = 5,
            ACONFIG_AFS_SEL_BIT = 4,
            ACONFIG_AFS_SEL_LENGTH = 2,
            ACONFIG_ACCEL_HPF_BIT = 2,
            ACONFIG_ACCEL_HPF_LENGTH = 3,
        };

        enum DHPF : std::uint8_t {
            DHPF_RESET = 0x00,
            DHPF_5 = 0x01,
            DHPF_2P5 = 0x02,
            DHPF_1P25 = 0x03,
            DHPF_0P63 = 0x04,
            DHPF_HOLD = 0x07,
        };

        enum FIFO : std::uint8_t {
            TEMP_FIFO_EN_BIT = 7,
            XG_FIFO_EN_BIT = 6,
            YG_FIFO_EN_BIT = 5,
            ZG_FIFO_EN_BIT = 4,
            ACCEL_FIFO_EN_BIT = 3,
            SLV2_FIFO_EN_BIT = 2,
            SLV1_FIFO_EN_BIT = 1,
            SLV0_FIFO_EN_BIT = 0,
        };

        enum ClockDiv : std::uint8_t {
            CLOCK_DIV_500 = 0x9,
            CLOCK_DIV_471 = 0xA,
            CLOCK_DIV_444 = 0xB,
            CLOCK_DIV_421 = 0xC,
            CLOCK_DIV_400 = 0xD,
            CLOCK_DIV_381 = 0xE,
            CLOCK_DIV_364 = 0xF,
            CLOCK_DIV_348 = 0x0,
            CLOCK_DIV_333 = 0x1,
            CLOCK_DIV_320 = 0x2,
            CLOCK_DIV_308 = 0x3,
            CLOCK_DIV_296 = 0x4,
            CLOCK_DIV_286 = 0x5,
            CLOCK_DIV_276 = 0x6,
            CLOCK_DIV_267 = 0x7,
            CLOCK_DIV_258 = 0x8,
        };

        enum Slave : std::uint8_t {
            I2C_SLV_RW_BIT = 7,
            I2C_SLV_ADDR_BIT = 6,
            I2C_SLV_ADDR_LENGTH = 7,
            I2C_SLV_EN_BIT = 7,
            I2C_SLV_BYTE_SW_BIT = 6,
            I2C_SLV_REG_DIS_BIT = 5,
            I2C_SLV_GRP_BIT = 4,
            I2C_SLV_LEN_BIT = 3,
            I2C_SLV_LEN_LENGTH = 4,
            I2C_SLV4_RW_BIT = 7,
            I2C_SLV4_ADDR_BIT = 6,
            I2C_SLV4_ADDR_LENGTH = 7,
            I2C_SLV4_EN_BIT = 7,
            I2C_SLV4_INT_EN_BIT = 6,
            I2C_SLV4_REG_DIS_BIT = 5,
            I2C_SLV4_MST_DLY_BIT = 4,
            I2C_SLV4_MST_DLY_LENGTH = 5,
            SLV_3_FIFO_EN_BIT = 5,
        };

        enum Master : std::uint8_t {
            MST_PASS_THROUGH_BIT = 7,
            MST_I2C_SLV4_DONE_BIT = 6,
            MST_I2C_LOST_ARB_BIT = 5,
            MST_I2C_SLV4_NACK_BIT = 4,
            MST_I2C_SLV3_NACK_BIT = 3,
            MST_I2C_SLV2_NACK_BIT = 2,
            MST_I2C_SLV1_NACK_BIT = 1,
            MST_I2C_SLV0_NACK_BIT = 0,
            MULT_MST_EN_BIT = 7,
            I2C_MST_CLK_LENGTH = 4,
            I2C_MST_P_NSR_BIT = 4,
            I2C_MST_CLK_BIT = 3,
            WAIT_FOR_ES_BIT = 6,
        };

        enum IntrCfg : std::uint8_t {
            INTCFG_INT_LEVEL_BIT = 7,
            INTCFG_INT_OPEN_BIT = 6,
            INTCFG_LATCH_INT_EN_BIT = 5,
            INTCFG_INT_RD_CLEAR_BIT = 4,
            INTCFG_FSYNC_INT_LEVEL_BIT = 3,
            INTCFG_FSYNC_INT_EN_BIT = 2,
            INTCFG_I2C_BYPASS_EN_BIT = 1,
        };

        enum IntrMode : std::uint8_t {
            INTMODE_ACTIVEHIGH = 0x00,
            INTMODE_ACTIVELOW = 0x01,
        };

        enum IntrDrive : std::uint8_t {
            INTDRV_PUSHPULL = 0x00,
            INTDRV_OPENDRAIN = 0x01,
        };

        enum IntrLatch : std::uint8_t {
            INTLATCH_50USPULSE = 0x00,
            INTLATCH_WAITCLEAR = 0x01,
        };

        enum IntrClear : std::uint8_t {
            INTCLEAR_STATUSREAD = 0x00,
            INTCLEAR_ANYREAD = 0x01,
        };

        enum Interrupt : std::uint8_t {
            INTERRUPT_FF_BIT = 7,
            INTERRUPT_MOT_BIT = 6,
            INTERRUPT_ZMOT_BIT = 5,
            INTERRUPT_FIFO_OFLOW_BIT = 4,
            INTERRUPT_I2C_MST_INT_BIT = 3,
            INTERRUPT_DATA_RDY_BIT = 0,
        };

        enum Motion : std::uint8_t {
            MOTION_MOT_XNEG_BIT = 7,
            MOTION_MOT_XPOS_BIT = 6,
            MOTION_MOT_YNEG_BIT = 5,
            MOTION_MOT_YPOS_BIT = 4,
            MOTION_MOT_ZNEG_BIT = 3,
            MOTION_MOT_ZPOS_BIT = 2,
            MOTION_MOT_ZRMOT_BIT = 0,
        };

        enum DelayCtrl : std::uint8_t {
            DELAYCTRL_DELAY_ES_SHADOW_BIT = 7,
            DELAYCTRL_I2C_SLV4_DLY_EN_BIT = 4,
            DELAYCTRL_I2C_SLV3_DLY_EN_BIT = 3,
            DELAYCTRL_I2C_SLV2_DLY_EN_BIT = 2,
            DELAYCTRL_I2C_SLV1_DLY_EN_BIT = 1,
            DELAYCTRL_I2C_SLV0_DLY_EN_BIT = 0,
        };

        enum PathReset : std::uint8_t {
            PATHRESET_GYRO_RESET_BIT = 2,
            PATHRESET_ACCEL_RESET_BIT = 1,
            PATHRESET_TEMP_RESET_BIT = 0,
        };

        enum Detect : std::uint8_t {
            DETECT_ACCEL_ON_DELAY_BIT = 5,
            DETECT_ACCEL_ON_DELAY_LENGTH = 2,
            DETECT_FF_COUNT_BIT = 3,
            DETECT_FF_COUNT_LENGTH = 2,
            DETECT_MOT_COUNT_BIT = 1,
            DETECT_MOT_COUNT_LENGTH = 2,
            DETECT_DECREMENT_RESET = 0x0,
            DETECT_DECREMENT_1 = 0x1,
            DETECT_DECREMENT_2 = 0x2,
            DETECT_DECREMENT_4 = 0x3,
        };

        enum Delay : std::uint8_t {
            DELAY_3MS = 0b11,
            DELAY_2MS = 0b10,
            DELAY_1MS = 0b01,
            NO_DELAY = 0b00,
        };

        enum UserCtrl : std::uint8_t {
            USERCTRL_FIFO_EN_BIT = 6,
            USERCTRL_I2C_MST_EN_BIT = 5,
            USERCTRL_I2C_IF_DIS_BIT = 4,
            USERCTRL_FIFO_RESET_BIT = 2,
            USERCTRL_I2C_MST_RESET_BIT = 1,
            USERCTRL_SIG_COND_RESET_BIT = 0,
        };

        enum Power1 : std::uint8_t {
            PWR1_DEVICE_RESET_BIT = 7,
            PWR1_SLEEP_BIT = 6,
            PWR1_CYCLE_BIT = 5,
            PWR1_TEMP_DIS_BIT = 3,
            PWR1_CLKSEL_BIT = 2,
            PWR1_CLKSEL_LENGTH = 3,
        };

        enum Clock : std::uint8_t {
            CLOCK_INTERNAL = 0x00,
            CLOCK_PLL_XGYRO = 0x01,
            CLOCK_PLL_YGYRO = 0x02,
            CLOCK_PLL_ZGYRO = 0x03,
            CLOCK_PLL_EXT32K = 0x04,
            CLOCK_PLL_EXT19M = 0x05,
            CLOCK_KEEP_RESET = 0x07,
        };

        enum Power2 : std::uint8_t {
            PWR2_LP_WAKE_CTRL_BIT = 7,
            PWR2_LP_WAKE_CTRL_LENGTH = 2,
            PWR2_STBY_XA_BIT = 5,
            PWR2_STBY_YA_BIT = 4,
            PWR2_STBY_ZA_BIT = 3,
            PWR2_STBY_XG_BIT = 2,
            PWR2_STBY_YG_BIT = 1,
            PWR2_STBY_ZG_BIT = 0,
        };

        enum WakeFreq : std::uint8_t {
            WAKE_FREQ_1P25 = 0x0,
            WAKE_FREQ_5 = 0x1,
            WAKE_FREQ_20 = 0x2,
            WAKE_FREQ_40 = 0x3,
        };

        enum WhoAmI : std::uint8_t {
            WHO_AM_I_BIT = 6,
            WHO_AM_I_LENGTH = 6,
        };

        static Scaled gyro_range_to_scale(const GyroRange gyro_range) noexcept;
        static Scaled accel_range_to_scale(const AccelRange accel_range) noexcept;

        static constexpr Scaled M_PI{3.14};
        static constexpr std::uint32_t I2C_TIMEOUT{100};
        static constexpr std::uint32_t GYRO_OUTPUT_RATE_DLPF_EN_HZ{1000};
        static constexpr std::uint32_t GYRO_OUTPUT_RATE_DLPF_DIS_HZ{8000};
        static constexpr std::uint32_t ACCEL_OUTPUT_RATE_HZ{1000};

        void initialize() noexcept;
        void deinitialize() noexcept;

        std::uint8_t get_device_id() const noexcept;

        void device_reset() const noexcept;
        void set_address_pin(GpioHandle gpio, const std::uint16_t address_pin) const noexcept;
        void set_sampling_rate_and_dlpf(const std::uint32_t rate, const std::uint8_t dlpf) const noexcept;
        void set_dlpf(const std::uint8_t value) const noexcept;
        void set_clock_source(const std::uint8_t source) const noexcept;
        void set_sleep_enabled(const std::uint8_t enable) const noexcept;
        void set_cycle_enabled(const std::uint8_t enable) const noexcept;
        void set_temperature_sensor_disabled(const std::uint8_t disable) const noexcept;
        void set_low_power_wake_up_frequency(const std::uint8_t frequency) const noexcept;

        void accelerometer_axis_standby(const std::uint8_t x_accel_standby,
                                        const std::uint8_t y_accel_standby,
                                        const std::uint8_t z_accel_standby) const noexcept;
        void gyroscope_axis_standby(const std::uint8_t x_gyro_standby,
                                    const std::uint8_t y_gyro_standby,
                                    const std::uint8_t z_gyro_standby) const noexcept;

        void set_full_scale_gyro_range(const GyroRange range) const noexcept;
        void set_full_scale_accel_range(const AccelRange range) const noexcept;

        Raw get_temperature_raw() const noexcept;

        Raw get_acceleration_x_raw() const noexcept;
        Raw get_acceleration_y_raw() const noexcept;
        Raw get_acceleration_z_raw() const noexcept;
        AccelRaw get_accelerometer_raw() const noexcept;

        Raw get_rotation_x_raw() const noexcept;
        Raw get_rotation_y_raw() const noexcept;
        Raw get_rotation_z_raw() const noexcept;
        GyroRaw get_gyroscope_raw() const noexcept;

        void set_interrupt() const noexcept;
        void set_interrupt_mode(const std::uint8_t mode) const noexcept;
        void set_interrupt_drive(const std::uint8_t drive) const noexcept;
        void set_interrupt_latch(const std::uint8_t latch) const noexcept;
        void set_interrupt_latch_clear(const std::uint8_t clear) const noexcept;
        void set_int_enable_register(std::uint8_t value) const noexcept;
        void set_int_data_ready_enabled(const std::uint8_t enable) const noexcept;

        std::uint8_t get_int_status_register() const noexcept;
        std::uint8_t get_motion_status_register() const noexcept;

        void set_dhpf_mode(const std::uint8_t dhpf) const noexcept;
        void set_int_zero_motion_enabled(const std::uint8_t enable) const noexcept;
        void set_int_motion_enabled(const std::uint8_t enable) const noexcept;
        void set_int_free_fall_enabled(const std::uint8_t enable) const noexcept;

        void set_motion_detection_threshold(std::uint8_t threshold) const noexcept;
        void set_motion_detection_duration(std::uint8_t duration) const noexcept;

        void set_zero_motion_detection_threshold(std::uint8_t threshold) const noexcept;
        void set_zero_motion_detection_duration(std::uint8_t duration) const noexcept;

        void set_free_fall_detection_threshold(std::uint8_t threshold) const noexcept;
        void set_free_fall_detection_duration(std::uint8_t duration) const noexcept;

        bool initialized_{false};

        I2cHandle i2c_{nullptr};

        Address address_{};
        GyroRange gyro_range_{};
        AccelRange accel_range_{};
    };

}; // namespace InvertedSway

#endif // MPU6050_HPP
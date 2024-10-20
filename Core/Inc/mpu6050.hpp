#ifndef MPU6050_HPP
#define MPU6050_HPP

#include "stm32l4xx_hal.h"
#include "vector3d.hpp"
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <expected>

struct MPU6050 {
    using Scaled = std::double_t;
    using GyroScaled = Linalg::Vector3D<Scaled>;
    using AccelScaled = Linalg::Vector3D<Scaled>;
    using RollPitchYaw = Linalg::Vector3D<Scaled>;

    using Raw = std::int32_t;
    using GyroRaw = Linalg::Vector3D<Raw>;
    using AccelRaw = Linalg::Vector3D<Raw>;

    using TempScaled = std::double_t;
    using TempRaw = std::int32_t;

    using I2C_Handle = I2C_HandleTypeDef*;

    MPU6050(I2C_Handle i2c,
            const std::uint8_t addres,
            const std::uint8_t gyro_range,
            const std::uint8_t accel_range) noexcept;

    MPU6050(const MPU6050& other) = delete;
    MPU6050(MPU6050&& other) = delete;

    MPU6050& operator=(const MPU6050& other) = delete;
    MPU6050& operator=(MPU6050&& other) = delete;

    ~MPU6050() noexcept = default;

    std::uint8_t get_device_id() noexcept;

    void set_dlpf(const std::uint8_t value) noexcept;

    //	PWR_MGMT_12
    void device_reset(const std::uint8_t reset) noexcept;
    void set_clock_source(const std::uint8_t source) noexcept;
    void set_sleep_enabled(const std::uint8_t enable) noexcept;
    void set_cycle_enabled(const std::uint8_t enable) noexcept;
    void set_temperature_sensor_disabled(const std::uint8_t disable) noexcept;

    //	PWR_MGMT_2
    void set_low_power_wake_up_frequency(const std::uint8_t frequency) noexcept;
    void accelerometer_axis_standby(const std::uint8_t x_accel_standby,
                                    const std::uint8_t y_accel_standby,
                                    const std::uint8_t z_accel_standby) noexcept;
    void gyroscope_axis_standby(const std::uint8_t x_gyro_standby,
                                const std::uint8_t y_gyro_standby,
                                const std::uint8_t z_gyro_standby) noexcept;

    //	Measurement scale configuration
    void set_full_scale_gyro_range(const std::uint8_t range) noexcept;
    void set_full_scale_accel_range(const std::uint8_t range) noexcept;

    // Reading data
    Raw get_temperature_raw() noexcept;
    TempScaled get_temperature_celsius() noexcept;

    Raw get_acceleration_x_raw() noexcept;
    Raw get_acceleration_y_raw() noexcept;
    Raw get_acceleration_z_raw() noexcept;
    AccelRaw get_accelerometer_raw() noexcept;
    AccelScaled get_accelerometer_scaled() noexcept;

    Raw get_rotation_x_raw() noexcept;
    Raw get_rotation_y_raw() noexcept;
    Raw get_rotation_z_raw() noexcept;
    GyroRaw get_gyroscope_raw() noexcept;
    GyroScaled get_gyroscope_scaled() noexcept;

    RollPitchYaw get_roll_pitch_yaw() noexcept;

    //	setting INT pin
    // INT_PIN_CFG register
    void set_interrupt_mode(const std::uint8_t mode) noexcept;
    void set_interrupt_drive(const std::uint8_t drive) noexcept;
    void set_interrupt_latch(const std::uint8_t latch) noexcept;
    void set_interrupt_latch_clear(const std::uint8_t clear) noexcept;
    // INT_ENABLE register
    void set_int_enable_register(std::uint8_t value) noexcept;
    void set_int_data_ready_enabled(const std::uint8_t enable) noexcept;
    // INT_STATUS register
    std::uint8_t get_int_status_register() noexcept;

    //	motion_ functions - not included in documentation/register map
    std::uint8_t get_motion_status_register() noexcept;

    void set_dhpf_mode(const std::uint8_t dhpf) noexcept;
    // INT_ENABLE register
    void set_int_zero_motion_enabled(const std::uint8_t enable) noexcept;
    void set_int_motion_enabled(const std::uint8_t enable) noexcept;
    void set_int_free_fall_enabled(const std::uint8_t enable) noexcept;

    void set_motion_detection_threshold(std::uint8_t threshold) noexcept;
    void set_motion_detection_duration(std::uint8_t duration) noexcept;

    void set_zero_motion_detection_threshold(std::uint8_t threshold) noexcept;
    void set_zero_motion_detection_duration(std::uint8_t duration) noexcept;

    void set_free_fall_detection_threshold(std::uint8_t threshold) noexcept;
    void set_free_fall_detection_duration(std::uint8_t duration) noexcept;

    std::uint8_t gyro_range{};
    std::uint8_t accel_range{};
    std::uint8_t addres{};
    I2C_Handle i2c{nullptr};

    static Scaled gyro_range_to_scale(const std::uint8_t gyro_range) noexcept
    {
        switch (gyro_range) {
            case GYRO_FS_250:
                return 0.007633;
            case GYRO_FS_500:
                return 0.015267;
            case GYRO_FS_1000:
                return 0.030487;
            case GYRO_FS_2000:
                return 0.060975;
            default:
                return 0.0;
        }
    }

    static Scaled accel_range_to_scale(const std::uint8_t accel_range) noexcept
    {
        switch (accel_range) {
            case ACCEL_FS_2:
                return 0.000061;
            case ACCEL_FS_4:
                return 0.000122;
            case ACCEL_FS_8:
                return 0.000244;
            case ACCEL_FS_16:
                return 0.0004882;
            default:
                return 0.0;
        }
    }

    static inline std::uint32_t i2c_TIMEOUT{10};

    static constexpr std::uint8_t ADDRESS{0xD0}; // AD0 low
                                                 //  static constexpr std::uint8_t ADDRESS { 0xD1	// AD0 high
                                                 //
                                                 //	Registers addresses
                                                 //
    static constexpr std::uint8_t RA_SELF_TEST_X{0x0D};
    static constexpr std::uint8_t RA_SELF_TEST_Y{0x0E};
    static constexpr std::uint8_t RA_SELF_TEST_Z{0x0F};
    static constexpr std::uint8_t RA_SELF_TEST_A{0x10};
    static constexpr std::uint8_t RA_SMPLRT_DIV{0x19};
    static constexpr std::uint8_t RA_CONFIG{0x1A};
    static constexpr std::uint8_t RA_GYRO_CONFIG{0x1B};
    static constexpr std::uint8_t RA_ACCEL_CONFIG{0x1C};
    // Not in documentation

    static constexpr std::uint8_t RA_FF_THR{0x1D};
    static constexpr std::uint8_t RA_FF_DUR{0x1E};
    static constexpr std::uint8_t RA_MOT_THR{0x1F};
    static constexpr std::uint8_t RA_MOT_DUR{0x20};
    static constexpr std::uint8_t RA_ZRMOT_THR{0x21};
    static constexpr std::uint8_t RA_ZRMOT_DUR{0x22};
    // Not in documentation end

    static constexpr std::uint8_t RA_FIFO_EN{0x23};
    static constexpr std::uint8_t RA_I2C_MST_CTRL{0x24};
    static constexpr std::uint8_t RA_I2C_SLV0_ADDR{0x25};
    static constexpr std::uint8_t RA_I2C_SLV0_REG{0x26};
    static constexpr std::uint8_t RA_I2C_SLV0_CTRL{0x27};
    static constexpr std::uint8_t RA_I2C_SLV1_ADDR{0x28};
    static constexpr std::uint8_t RA_I2C_SLV1_REG{0x29};
    static constexpr std::uint8_t RA_I2C_SLV1_CTRL{0x2A};
    static constexpr std::uint8_t RA_I2C_SLV2_ADDR{0x2B};
    static constexpr std::uint8_t RA_I2C_SLV2_REG{0x2C};
    static constexpr std::uint8_t RA_I2C_SLV2_CTRL{0x2D};
    static constexpr std::uint8_t RA_I2C_SLV3_ADDR{0x2E};
    static constexpr std::uint8_t RA_I2C_SLV3_REG{0x2F};
    static constexpr std::uint8_t RA_I2C_SLV3_CTRL{0x30};
    static constexpr std::uint8_t RA_I2C_SLV4_ADDR{0x31};
    static constexpr std::uint8_t RA_I2C_SLV4_REG{0x32};
    static constexpr std::uint8_t RA_I2C_SLV4_DO{0x33};
    static constexpr std::uint8_t RA_I2C_SLV4_CTRL{0x34};
    static constexpr std::uint8_t RA_I2C_SLV4_DI{0x35};
    static constexpr std::uint8_t RA_I2C_MST_STATUS{0x36};
    static constexpr std::uint8_t RA_INT_PIN_CFG{0x37};
    static constexpr std::uint8_t RA_INT_ENABLE{0x38};
    // Not in documentation

    static constexpr std::uint8_t RA_DMP_INT_STATUS{0x39};
    // Not in documentation end

    static constexpr std::uint8_t RA_INT_STATUS{0x3A};
    static constexpr std::uint8_t RA_ACCEL_XOUT_H{0x3B};
    static constexpr std::uint8_t RA_ACCEL_XOUT_L{0x3C};
    static constexpr std::uint8_t RA_ACCEL_YOUT_H{0x3D};
    static constexpr std::uint8_t RA_ACCEL_YOUT_L{0x3E};
    static constexpr std::uint8_t RA_ACCEL_ZOUT_H{0x3F};
    static constexpr std::uint8_t RA_ACCEL_ZOUT_L{0x40};
    static constexpr std::uint8_t RA_TEMP_OUT_H{0x41};
    static constexpr std::uint8_t RA_TEMP_OUT_L{0x42};
    static constexpr std::uint8_t RA_GYRO_XOUT_H{0x43};
    static constexpr std::uint8_t RA_GYRO_XOUT_L{0x44};
    static constexpr std::uint8_t RA_GYRO_YOUT_H{0x45};
    static constexpr std::uint8_t RA_GYRO_YOUT_L{0x46};
    static constexpr std::uint8_t RA_GYRO_ZOUT_H{0x47};
    static constexpr std::uint8_t RA_GYRO_ZOUT_L{0x48};
    static constexpr std::uint8_t RA_EXT_SENS_DATA_00{0x49};
    static constexpr std::uint8_t RA_EXT_SENS_DATA_01{0x4A};
    static constexpr std::uint8_t RA_EXT_SENS_DATA_02{0x4B};
    static constexpr std::uint8_t RA_EXT_SENS_DATA_03{0x4C};
    static constexpr std::uint8_t RA_EXT_SENS_DATA_04{0x4D};
    static constexpr std::uint8_t RA_EXT_SENS_DATA_05{0x4E};
    static constexpr std::uint8_t RA_EXT_SENS_DATA_06{0x4F};
    static constexpr std::uint8_t RA_EXT_SENS_DATA_07{0x50};
    static constexpr std::uint8_t RA_EXT_SENS_DATA_08{0x51};
    static constexpr std::uint8_t RA_EXT_SENS_DATA_09{0x52};
    static constexpr std::uint8_t RA_EXT_SENS_DATA_10{0x53};
    static constexpr std::uint8_t RA_EXT_SENS_DATA_11{0x54};
    static constexpr std::uint8_t RA_EXT_SENS_DATA_12{0x55};
    static constexpr std::uint8_t RA_EXT_SENS_DATA_13{0x56};
    static constexpr std::uint8_t RA_EXT_SENS_DATA_14{0x57};
    static constexpr std::uint8_t RA_EXT_SENS_DATA_15{0x58};
    static constexpr std::uint8_t RA_EXT_SENS_DATA_16{0x59};
    static constexpr std::uint8_t RA_EXT_SENS_DATA_17{0x5A};
    static constexpr std::uint8_t RA_EXT_SENS_DATA_18{0x5B};
    static constexpr std::uint8_t RA_EXT_SENS_DATA_19{0x5C};
    static constexpr std::uint8_t RA_EXT_SENS_DATA_20{0x5D};
    static constexpr std::uint8_t RA_EXT_SENS_DATA_21{0x5E};
    static constexpr std::uint8_t RA_EXT_SENS_DATA_22{0x5F};
    static constexpr std::uint8_t RA_EXT_SENS_DATA_23{0x60};
    // Not in documentation

    static constexpr std::uint8_t RA_MOT_DETECT_STATUS{0x61};
    // Not in documentation end

    static constexpr std::uint8_t RA_I2C_SLV0_DO{0x63};
    static constexpr std::uint8_t RA_I2C_SLV1_DO{0x64};
    static constexpr std::uint8_t RA_I2C_SLV2_DO{0x65};
    static constexpr std::uint8_t RA_I2C_SLV3_DO{0x66};
    static constexpr std::uint8_t RA_I2C_MST_DELAY_CTRL{0x67};
    static constexpr std::uint8_t RA_SIGNAL_PATH_RESET{0x68};
    // Not in documentation

    static constexpr std::uint8_t RA_MOT_DETECT_CTRL{0x69};
    // Not in documentation end

    static constexpr std::uint8_t RA_USER_CTRL{0x6A};
    static constexpr std::uint8_t RA_PWR_MGMT_1{0x6B};
    static constexpr std::uint8_t RA_PWR_MGMT_2{0x6C};
    static constexpr std::uint8_t RA_FIFO_COUNTH{0x72};
    static constexpr std::uint8_t RA_FIFO_COUNTL{0x73};
    static constexpr std::uint8_t RA_FIFO_R_W{0x74};
    static constexpr std::uint8_t RA_WHO_AM_I{0x75};
    //
    //	Registers 13 to 16 � Self Test Registers
    //	SELF_TEST_X, SELF_TEST_Y, SELF_TEST_Z, and SELF_TEST_A
    //
    static constexpr std::uint8_t SELF_TEST_XA_1_BIT{0x07};
    static constexpr std::uint8_t SELF_TEST_XA_1_LENGTH{0x03};
    static constexpr std::uint8_t SELF_TEST_XA_2_BIT{0x05};
    static constexpr std::uint8_t SELF_TEST_XA_2_LENGTH{0x02};
    static constexpr std::uint8_t SELF_TEST_YA_1_BIT{0x07};
    static constexpr std::uint8_t SELF_TEST_YA_1_LENGTH{0x03};
    static constexpr std::uint8_t SELF_TEST_YA_2_BIT{0x03};
    static constexpr std::uint8_t SELF_TEST_YA_2_LENGTH{0x02};
    static constexpr std::uint8_t SELF_TEST_ZA_1_BIT{0x07};
    static constexpr std::uint8_t SELF_TEST_ZA_1_LENGTH{0x03};
    static constexpr std::uint8_t SELF_TEST_ZA_2_BIT{0x01};
    static constexpr std::uint8_t SELF_TEST_ZA_2_LENGTH{0x02};

    static constexpr std::uint8_t SELF_TEST_XG_1_BIT{0x04};
    static constexpr std::uint8_t SELF_TEST_XG_1_LENGTH{0x05};
    static constexpr std::uint8_t SELF_TEST_YG_1_BIT{0x04};
    static constexpr std::uint8_t SELF_TEST_YG_1_LENGTH{0x05};
    static constexpr std::uint8_t SELF_TEST_ZG_1_BIT{0x04};
    static constexpr std::uint8_t SELF_TEST_ZG_1_LENGTH{0x05};
    //
    //	Register 26 � Configuration
    //	CONFIG
    //
    static constexpr std::uint8_t CFG_EXT_SYNC_SET_BIT{5};
    static constexpr std::uint8_t CFG_EXT_SYNC_SET_LENGTH{3};
    static constexpr std::uint8_t CFG_DLPF_CFG_BIT{2};
    static constexpr std::uint8_t CFG_DLPF_CFG_LENGTH{3};

    static constexpr std::uint8_t EXT_SYNC_DISABLED{0x0};
    static constexpr std::uint8_t EXT_SYNC_TEMP_OUT_L{0x1};
    static constexpr std::uint8_t EXT_SYNC_GYRO_XOUT_L{0x2};
    static constexpr std::uint8_t EXT_SYNC_GYRO_YOUT_L{0x3};
    static constexpr std::uint8_t EXT_SYNC_GYRO_ZOUT_L{0x4};
    static constexpr std::uint8_t EXT_SYNC_ACCEL_XOUT_L{0x5};
    static constexpr std::uint8_t EXT_SYNC_ACCEL_YOUT_L{0x6};
    static constexpr std::uint8_t EXT_SYNC_ACCEL_ZOUT_L{0x7};

    static constexpr std::uint8_t DLPF_BW_256{0x00};
    static constexpr std::uint8_t DLPF_BW_188{0x01};
    static constexpr std::uint8_t DLPF_BW_98{0x02};
    static constexpr std::uint8_t DLPF_BW_42{0x03};
    static constexpr std::uint8_t DLPF_BW_20{0x04};
    static constexpr std::uint8_t DLPF_BW_10{0x05};
    static constexpr std::uint8_t DLPF_BW_5{0x06};
    //
    //	Register 27 � Gyroscope Configuration
    //	GYRO_CONFIG
    //
    static constexpr std::uint8_t GCONFIG_FS_SEL_BIT{4};
    static constexpr std::uint8_t GCONFIG_FS_SEL_LENGTH{2};

    static constexpr std::uint8_t GYRO_FS_250{0x00};
    static constexpr std::uint8_t GYRO_FS_500{0x01};
    static constexpr std::uint8_t GYRO_FS_1000{0x02};
    static constexpr std::uint8_t GYRO_FS_2000{0x03};
    //
    //	Register 28 � Accelerometer Configuration
    //	ACCEL_CONFIG
    //
    static constexpr std::uint8_t ACONFIG_XA_ST_BIT{7};
    static constexpr std::uint8_t ACONFIG_YA_ST_BIT{6};
    static constexpr std::uint8_t ACONFIG_ZA_ST_BIT{5};
    static constexpr std::uint8_t ACONFIG_AFS_SEL_BIT{4};
    static constexpr std::uint8_t ACONFIG_AFS_SEL_LENGTH{2};
    static constexpr std::uint8_t ACONFIG_ACCEL_HPF_BIT{2};
    static constexpr std::uint8_t ACONFIG_ACCEL_HPF_LENGTH{3};

    static constexpr std::uint8_t ACCEL_FS_2{0x00};
    static constexpr std::uint8_t ACCEL_FS_4{0x01};
    static constexpr std::uint8_t ACCEL_FS_8{0x02};
    static constexpr std::uint8_t ACCEL_FS_16{0x03};
    //
    //
    //
    static constexpr std::uint8_t DHPF_RESET{0x00};
    static constexpr std::uint8_t DHPF_5{0x01};
    static constexpr std::uint8_t DHPF_2P5{0x02};
    static constexpr std::uint8_t DHPF_1P25{0x03};
    static constexpr std::uint8_t DHPF_0P63{0x04};
    static constexpr std::uint8_t DHPF_HOLD{0x07};
    //
    //	Register 35 � FIFO Enable
    //	FIFO_EN
    //
    static constexpr std::uint8_t TEMP_FIFO_EN_BIT{7};
    static constexpr std::uint8_t XG_FIFO_EN_BIT{6};
    static constexpr std::uint8_t YG_FIFO_EN_BIT{5};
    static constexpr std::uint8_t ZG_FIFO_EN_BIT{4};
    static constexpr std::uint8_t ACCEL_FIFO_EN_BIT{3};
    static constexpr std::uint8_t SLV2_FIFO_EN_BIT{2};
    static constexpr std::uint8_t SLV1_FIFO_EN_BIT{1};
    static constexpr std::uint8_t SLV0_FIFO_EN_BIT{0};
    //
    //	Register 36 � I2C Master Control
    //	I2C_MST_CTRL
    //
    static constexpr std::uint8_t MULT_MST_EN_BIT{7};
    static constexpr std::uint8_t WAIT_FOR_ES_BIT{6};
    static constexpr std::uint8_t SLV_3_FIFO_EN_BIT{5};
    static constexpr std::uint8_t I2C_MST_P_NSR_BIT{4};
    static constexpr std::uint8_t I2C_MST_CLK_BIT{3};
    static constexpr std::uint8_t I2C_MST_CLK_LENGTH{4};

    static constexpr std::uint8_t CLOCK_DIV_500{0x9};
    static constexpr std::uint8_t CLOCK_DIV_471{0xA};
    static constexpr std::uint8_t CLOCK_DIV_444{0xB};
    static constexpr std::uint8_t CLOCK_DIV_421{0xC};
    static constexpr std::uint8_t CLOCK_DIV_400{0xD};
    static constexpr std::uint8_t CLOCK_DIV_381{0xE};
    static constexpr std::uint8_t CLOCK_DIV_364{0xF};
    static constexpr std::uint8_t CLOCK_DIV_348{0x0};
    static constexpr std::uint8_t CLOCK_DIV_333{0x1};
    static constexpr std::uint8_t CLOCK_DIV_320{0x2};
    static constexpr std::uint8_t CLOCK_DIV_308{0x3};
    static constexpr std::uint8_t CLOCK_DIV_296{0x4};
    static constexpr std::uint8_t CLOCK_DIV_286{0x5};
    static constexpr std::uint8_t CLOCK_DIV_276{0x6};
    static constexpr std::uint8_t CLOCK_DIV_267{0x7};
    static constexpr std::uint8_t CLOCK_DIV_258{0x8};
    //
    //	Registers 37 to 39 � I2C Slave 0 Control
    //	I2C_SLV0_ADDR, I2C_SLV0_REG, and I2C_SLV0_CTRL
    //
    //	Registers 40 to 42 � I2C Slave 1 Control
    //	I2C_SLV1_ADDR, I2C_SLV1_REG, and I2C_SLV1_CTRL
    //
    //	Registers 43 to 45 � I2C Slave 2 Control
    //	I2C_SLV2_ADDR, I2C_SLV2_REG, and I2C_SLV2_CTRL
    //
    //	Registers 46 to 48 � I2C Slave 3 Control
    //	I2C_SLV3_ADDR, I2C_SLV3_REG, and I2C_SLV3_CTRL
    //
    //	Same structure for these registers
    //
    static constexpr std::uint8_t I2C_SLV_RW_BIT{7};
    static constexpr std::uint8_t I2C_SLV_ADDR_BIT{6};
    static constexpr std::uint8_t I2C_SLV_ADDR_LENGTH{7};
    static constexpr std::uint8_t I2C_SLV_EN_BIT{7};
    static constexpr std::uint8_t I2C_SLV_BYTE_SW_BIT{6};
    static constexpr std::uint8_t I2C_SLV_REG_DIS_BIT{5};
    static constexpr std::uint8_t I2C_SLV_GRP_BIT{4};
    static constexpr std::uint8_t I2C_SLV_LEN_BIT{3};
    static constexpr std::uint8_t I2C_SLV_LEN_LENGTH{4};
    //
    //	Registers 49 to 53 � I2C Slave 4 Control
    //	I2C_SLV4_ADDR, I2C_SLV4_REG, I2C_SLV4_DO, I2C_SLV4_CTRL, and I2C_SLV4_DI
    //
    static constexpr std::uint8_t I2C_SLV4_RW_BIT{7};
    static constexpr std::uint8_t I2C_SLV4_ADDR_BIT{6};
    static constexpr std::uint8_t I2C_SLV4_ADDR_LENGTH{7};
    static constexpr std::uint8_t I2C_SLV4_EN_BIT{7};
    static constexpr std::uint8_t I2C_SLV4_INT_EN_BIT{6};
    static constexpr std::uint8_t I2C_SLV4_REG_DIS_BIT{5};
    static constexpr std::uint8_t I2C_SLV4_MST_DLY_BIT{4};
    static constexpr std::uint8_t I2C_SLV4_MST_DLY_LENGTH{5};
    //
    //	Register 54 � I2C Master Status
    //	I2C_MST_STATUS
    //
    static constexpr std::uint8_t MST_PASS_THROUGH_BIT{7};
    static constexpr std::uint8_t MST_I2C_SLV4_DONE_BIT{6};
    static constexpr std::uint8_t MST_I2C_LOST_ARB_BIT{5};
    static constexpr std::uint8_t MST_I2C_SLV4_NACK_BIT{4};
    static constexpr std::uint8_t MST_I2C_SLV3_NACK_BIT{3};
    static constexpr std::uint8_t MST_I2C_SLV2_NACK_BIT{2};
    static constexpr std::uint8_t MST_I2C_SLV1_NACK_BIT{1};
    static constexpr std::uint8_t MST_I2C_SLV0_NACK_BIT{0};

    //
    //	Register 55 � INT Pin / Bypass Enable Configuration
    //	INT_PIN_CFG
    //
    static constexpr std::uint8_t INTCFG_INT_LEVEL_BIT{7};
    static constexpr std::uint8_t INTCFG_INT_OPEN_BIT{6};
    static constexpr std::uint8_t INTCFG_LATCH_INT_EN_BIT{5};
    static constexpr std::uint8_t INTCFG_INT_RD_CLEAR_BIT{4};
    static constexpr std::uint8_t INTCFG_FSYNC_INT_LEVEL_BIT{3};
    static constexpr std::uint8_t INTCFG_FSYNC_INT_EN_BIT{2};
    static constexpr std::uint8_t INTCFG_I2C_BYPASS_EN_BIT{1};

    static constexpr std::uint8_t INTMODE_ACTIVEHIGH{0x00};
    static constexpr std::uint8_t INTMODE_ACTIVELOW{0x01};

    static constexpr std::uint8_t INTDRV_PUSHPULL{0x00};
    static constexpr std::uint8_t INTDRV_OPENDRAIN{0x01};

    static constexpr std::uint8_t INTLATCH_50USPULSE{0x00};
    static constexpr std::uint8_t INTLATCH_WAITCLEAR{0x01};

    static constexpr std::uint8_t INTCLEAR_STATUSREAD{0x00};
    static constexpr std::uint8_t INTCLEAR_ANYREAD{0x01};
    //
    //	Register 56 � Interrupt Enable
    //	INT_ENABLE
    //
    //	Register 58 � Interrupt Status
    //	INT_STATUS
    //

    // Not in documentation
    static constexpr std::uint8_t INTERRUPT_FF_BIT{7};
    static constexpr std::uint8_t INTERRUPT_MOT_BIT{6};
    static constexpr std::uint8_t INTERRUPT_ZMOT_BIT{5};
    // Not in documentation end
    static constexpr std::uint8_t INTERRUPT_FIFO_OFLOW_BIT{4};
    static constexpr std::uint8_t INTERRUPT_I2C_MST_INT_BIT{3};
    static constexpr std::uint8_t INTERRUPT_DATA_RDY_BIT{0};
    //
    //	Not documented
    //	Register 91 - Motion Status
    //
    static constexpr std::uint8_t MOTION_MOT_XNEG_BIT{7};
    static constexpr std::uint8_t MOTION_MOT_XPOS_BIT{6};
    static constexpr std::uint8_t MOTION_MOT_YNEG_BIT{5};
    static constexpr std::uint8_t MOTION_MOT_YPOS_BIT{4};
    static constexpr std::uint8_t MOTION_MOT_ZNEG_BIT{3};
    static constexpr std::uint8_t MOTION_MOT_ZPOS_BIT{2};
    static constexpr std::uint8_t MOTION_MOT_ZRMOT_BIT{0};
    //
    //	Register 103 � I2C Master Delay Control
    //	I2C_MST_DELAY_CTR
    //
    static constexpr std::uint8_t DELAYCTRL_DELAY_ES_SHADOW_BIT{7};
    static constexpr std::uint8_t DELAYCTRL_I2C_SLV4_DLY_EN_BIT{4};
    static constexpr std::uint8_t DELAYCTRL_I2C_SLV3_DLY_EN_BIT{3};
    static constexpr std::uint8_t DELAYCTRL_I2C_SLV2_DLY_EN_BIT{2};
    static constexpr std::uint8_t DELAYCTRL_I2C_SLV1_DLY_EN_BIT{1};
    static constexpr std::uint8_t DELAYCTRL_I2C_SLV0_DLY_EN_BIT{0};
    //
    //	Register 104 � Signal Path Reset
    //	SIGNAL_PATH_RESE
    //
    static constexpr std::uint8_t PATHRESET_GYRO_RESET_BIT{2};
    static constexpr std::uint8_t PATHRESET_ACCEL_RESET_BIT{1};
    static constexpr std::uint8_t PATHRESET_TEMP_RESET_BIT{0};
    //
    //	Not documented
    //	Register 105 - Motion Detect Control
    //
    static constexpr std::uint8_t DETECT_ACCEL_ON_DELAY_BIT{5};
    static constexpr std::uint8_t DETECT_ACCEL_ON_DELAY_LENGTH{2};
    static constexpr std::uint8_t DETECT_FF_COUNT_BIT{3};
    static constexpr std::uint8_t DETECT_FF_COUNT_LENGTH{2};
    static constexpr std::uint8_t DETECT_MOT_COUNT_BIT{1};
    static constexpr std::uint8_t DETECT_MOT_COUNT_LENGTH{2};

    static constexpr std::uint8_t DETECT_DECREMENT_RESET{0x0};
    static constexpr std::uint8_t DETECT_DECREMENT_1{0x1};
    static constexpr std::uint8_t DETECT_DECREMENT_2{0x2};
    static constexpr std::uint8_t DETECT_DECREMENT_4{0x3};

    static constexpr std::uint8_t DELAY_3MS{0b11};
    static constexpr std::uint8_t DELAY_2MS{0b10};
    static constexpr std::uint8_t DELAY_1MS{0b01};
    static constexpr std::uint8_t NO_DELAY{0b00};
    //
    //	Register 106 � User Control
    //	USER_CTRL
    //
    static constexpr std::uint8_t USERCTRL_FIFO_EN_BIT{6};
    static constexpr std::uint8_t USERCTRL_I2C_MST_EN_BIT{5};
    static constexpr std::uint8_t USERCTRL_I2C_IF_DIS_BIT{4};
    static constexpr std::uint8_t USERCTRL_FIFO_RESET_BIT{2};
    static constexpr std::uint8_t USERCTRL_I2C_MST_RESET_BIT{1};
    static constexpr std::uint8_t USERCTRL_SIG_COND_RESET_BIT{0};
    //
    //	Register 107 � Power Management 1
    //	PWR_MGMT_1
    //
    static constexpr std::uint8_t PWR1_DEVICE_RESET_BIT{7};
    static constexpr std::uint8_t PWR1_SLEEP_BIT{6};
    static constexpr std::uint8_t PWR1_CYCLE_BIT{5};
    static constexpr std::uint8_t PWR1_TEMP_DIS_BIT{3};
    static constexpr std::uint8_t PWR1_CLKSEL_BIT{2};
    static constexpr std::uint8_t PWR1_CLKSEL_LENGTH{3};

    static constexpr std::uint8_t CLOCK_INTERNAL{0x00};
    static constexpr std::uint8_t CLOCK_PLL_XGYRO{0x01};
    static constexpr std::uint8_t CLOCK_PLL_YGYRO{0x02};
    static constexpr std::uint8_t CLOCK_PLL_ZGYRO{0x03};
    static constexpr std::uint8_t CLOCK_PLL_EXT32K{0x04};
    static constexpr std::uint8_t CLOCK_PLL_EXT19M{0x05};
    static constexpr std::uint8_t CLOCK_KEEP_RESET{0x07};
    //
    //	Register 108 � Power Management 2
    //	PWR_MGMT_2
    //
    static constexpr std::uint8_t PWR2_LP_WAKE_CTRL_BIT{7};
    static constexpr std::uint8_t PWR2_LP_WAKE_CTRL_LENGTH{2};
    static constexpr std::uint8_t PWR2_STBY_XA_BIT{5};
    static constexpr std::uint8_t PWR2_STBY_YA_BIT{4};
    static constexpr std::uint8_t PWR2_STBY_ZA_BIT{3};
    static constexpr std::uint8_t PWR2_STBY_XG_BIT{2};
    static constexpr std::uint8_t PWR2_STBY_YG_BIT{1};
    static constexpr std::uint8_t PWR2_STBY_ZG_BIT{0};

    static constexpr std::uint8_t WAKE_FREQ_1P25{0x0};
    static constexpr std::uint8_t WAKE_FREQ_5{0x1};
    static constexpr std::uint8_t WAKE_FREQ_20{0x2};
    static constexpr std::uint8_t WAKE_FREQ_40{0x3};
    //
    //	Register 117 � Who Am I
    //	WHO_AM_I
    //
    static constexpr std::uint8_t WHO_AM_I_BIT{6};
    static constexpr std::uint8_t WHO_AM_I_LENGTH{6};
};

#endif // MPU6050_HPP
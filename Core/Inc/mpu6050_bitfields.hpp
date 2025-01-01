#ifndef MPU6050_BITFIELDS_HPP
#define MPU6050_BITFIELDS_HPP

#include "common.hpp"
#include "i2c_device.hpp"
#include "mpu_register_map.hpp"
#include "stm32l4xx_hal.h"
#include "vector3d.hpp"
#include <cstddef>
#include <cstdint>

namespace InvertedSway::BitFields {

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

        enum struct DHPF : std::uint8_t {
            DHPF_RESET = 0x00,
            DHPF_5 = 0x01,
            DHPF_2P5 = 0x02,
            DHPF_1P25 = 0x03,
            DHPF_0P63 = 0x04,
            DHPF_HOLD = 0x07,
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

        enum struct SlaveNum {
            SLAVE1 = 1,
            SLAVE2 = 2,
            SLAVE3 = 3,
        };

        enum struct IntMode : std::uint8_t {
            ACTIVEHIGH = 0x00,
            ACTIVELOW = 0x01,
        };

        enum struct IntDrive : std::uint8_t {
            PUSHPULL = 0x00,
            OPENDRAIN = 0x01,
        };

        enum struct IntLatch : std::uint8_t {
            PULSE50US = 0x00,
            WAITCLEAR = 0x01,
        };

        enum struct IntClear : std::uint8_t {
            STATUSREAD = 0x00,
            ANYREAD = 0x01,
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

        enum struct Clock : std::uint8_t {
            INTERNAL = 0x00,
            PLL_XGYRO = 0x01,
            PLL_YGYRO = 0x02,
            PLL_ZGYRO = 0x03,
            PLL_EXT32K = 0x04,
            PLL_EXT19M = 0x05,
            KEEP_RESET = 0x07,
        };

        enum struct WakeFreq : std::uint8_t {
            FREQ_1P25 = 0x0,
            FREQ_5 = 0x1,
            FREQ_20 = 0x2,
            FREQ_40 = 0x3,
        };

        using Scaled = float;
        using GyroScaled = Linalg::Vector3D<Scaled>;   // radians
        using AccelScaled = Linalg::Vector3D<Scaled>;  // m/s^2
        using RollPitchYaw = Linalg::Vector3D<Scaled>; // degrees
        using Raw = std::int16_t;
        using GyroRaw = Linalg::Vector3D<Raw>;
        using AccelRaw = Linalg::Vector3D<Raw>;

        MPU6050() noexcept = default;

        MPU6050(I2CDevice const i2c_device,
                std::uint32_t const sampling_rate,
                GyroRange const gyro_range,
                AccelRange const accel_range,
                DLPF const dlpf,
                DHPF const dhpf,
                ExtSync const ext_sync) noexcept;

        MPU6050(MPU6050 const& other) noexcept = delete;
        MPU6050(MPU6050&& other) noexcept = default;

        MPU6050& operator=(MPU6050 const& other) noexcept = delete;
        MPU6050& operator=(MPU6050&& other) noexcept = default;

        ~MPU6050() noexcept;

        /* meters per square second */
        [[nodiscard]] AccelScaled get_acceleration_scaled() const noexcept;
        [[nodiscard]] Scaled get_acceleration_x_scaled() const noexcept;
        [[nodiscard]] Scaled get_acceleration_y_scaled() const noexcept;
        [[nodiscard]] Scaled get_acceleration_z_scaled() const noexcept;

        /* celsius */
        [[nodiscard]] Scaled get_temperature_celsius() const noexcept;

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
        static std::uint8_t get_sampling_divider(std::uint32_t const sampling_rate, DLPF const dlpf) noexcept;

        static RollPitchYaw accel_to_roll_pitch_yaw(AccelScaled const& accel_scaled) noexcept;
        static Scaled accel_to_roll(AccelScaled const& accel_scaled) noexcept;
        static Scaled accel_to_pitch(AccelScaled const& accel_scaled) noexcept;
        static Scaled accel_to_yaw(AccelScaled const& accel_scaled) noexcept;

        static constexpr Scaled PI{3.1415f};
        static constexpr std::uint32_t GYRO_OUTPUT_RATE_DLPF_EN_HZ{1000};
        static constexpr std::uint32_t GYRO_OUTPUT_RATE_DLPF_DIS_HZ{8000};
        static constexpr std::uint32_t ACCEL_OUTPUT_RATE_HZ{1000};

        void device_reset() const noexcept;
        void device_wake_up() const noexcept;

        std::uint8_t get_device_id() const noexcept;
        bool is_valid_device_id() const noexcept;

        void set_smplrt_div(std::uint8_t const sampling_rate, DLPF const dlpf) const noexcept;
        void set_config(ExtSync const ext_sync, DLPF const dlpf) const noexcept;
        void set_gyro_config(bool const x_standby,
                             bool const y_standby,
                             bool const z_standby,
                             GyroRange const gyro_range) const noexcept;
        void set_accel_config(bool const x_standby,
                              bool const y_standby,
                              bool const z_standby,
                              AccelRange const accel_range,
                              DHPF const dhpf) const noexcept;

        AccelRaw get_acceleration_raw() const noexcept;
        Raw get_acceleration_x_raw() const noexcept;
        Raw get_acceleration_y_raw() const noexcept;
        Raw get_acceleration_z_raw() const noexcept;

        Raw get_temperature_raw() const noexcept;

        GyroRaw get_rotation_raw() const noexcept;
        Raw get_rotation_x_raw() const noexcept;
        Raw get_rotation_y_raw() const noexcept;
        Raw get_rotation_z_raw() const noexcept;

        void initialize(std::uint32_t const sampling_rate,
                        GyroRange const gyro_range,
                        AccelRange const accel_range,
                        DLPF const dlpf,
                        DHPF const dhpf,
                        ExtSync const ext_sync) noexcept;

        void deinitialize() noexcept;

        void set_xg_offs_register(XG_OFFS_TC const xg_offs_tc) const noexcept;
        void set_yg_offs_tc_register(YG_OFFS_TC const yg_offs_tc) const noexcept;
        void set_zg_offs_tc_register(ZG_OFFS_TC const zg_offs_tc) const noexcept;

        void set_x_fine_gain_register(X_FINE_GAIN const x_fine_gain) const noexcept;
        void set_y_fine_gain_register(Y_FINE_GAIN const y_fine_gain) const noexcept;
        void set_z_fine_gain_register(Z_FINE_GAIN const z_fine_gain) const noexcept;

        void set_xa_offs_registers(XA_OFFS const xa_offs) const noexcept;
        void set_ya_offs_registers(YA_OFFS const ya_offs) const noexcept;
        void set_za_offs_registers(ZA_OFFS const za_offs) const noexcept;

        void set_xg_offs_usr_registers(XG_OFFS_USR const xg_offs_usr) const noexcept;
        void set_yg_offs_usr_registers(YG_OFFS_USR const yg_offs_usr) const noexcept;
        void set_zg_offs_usr_registers(ZG_OFFS_USR const zg_offs_usr) const noexcept;

        void set_self_test_x_register(SELF_TEST_X const self_test_x) const noexcept;
        void set_self_test_y_register(SELF_TEST_Y const self_test_y) const noexcept;
        void set_self_test_x_register(SELF_TEST_Z const self_test_z) const noexcept;
        void set_self_test_a_register(SELF_TEST_A const self_test_a) const noexcept;

        void set_smplrt_div_register(SMPLRT_DIV const smplrt_div) const noexcept;
        void set_config_register(CONFIG const config) const noexcept;
        void set_gyro_config_register(GYRO_CONFIG const gyro_config) const noexcept;
        void set_accel_config_register(ACCEL_CONFIG const accel_config) const noexcept;

        void set_ff_thr_register(FF_THR const ff_thr) const noexcept;
        void set_ff_dur_register(FF_DUR const ff_dur) const noexcept;
        void set_mot_thr_register(MOT_THR const mot_thr) const noexcept;
        void set_mot_dur_register(MOT_DUR const mot_dur) const noexcept;
        void set_zrmot_thr_register(ZRMOT_THR const zrmot_thr) const noexcept;
        void set_zrmot_dur_register(ZRMOT_DUR const zrmot_dur) const noexcept;

        void set_fifo_en_register(FIFO_EN const fifo_en) const noexcept;

        void set_i2c_mst_ctrl_register(I2C_MST_CTRL const i2c_mst_ctrl) const noexcept;

        void set_i2c_slv_addr_register(SlaveNum const slave_num, I2C_SLV_ADDR const i2c_slv_addr) const noexcept;
        void set_i2c_slv_reg_register(SlaveNum const slave_num, I2C_SLV_REG const i2c_slv_reg) const noexcept;
        void set_i2c_slv_ctrl_register(SlaveNum const slave_num, I2C_SLV_CTRL const i2c_slv_ctrl) const noexcept;

        void set_i2c_slv4_addr_register(I2C_SLV4_ADDR const i2c_slv4_addr) const noexcept;
        void set_i2c_slv4_reg_register(I2C_SLV4_REG const i2c_slv4_reg) const noexcept;
        void set_i2c_slv4_ctrl_register(I2C_SLV4_CTRL const i2c_slv4_ctrl) const noexcept;
        void set_i2c_slv4_do_register(I2C_SLV4_DO const i2c_slv4_do) const noexcept;
        I2C_SLV4_DI get_i2c_slv4_di_register() const noexcept;

        I2C_MST_STATUS get_i2c_mst_status_register() const noexcept;

        void set_int_pin_cfg_register(INT_PIN_CFG const int_pin_cfg) const noexcept;
        void set_int_enable_register(INT_ENABLE const int_enable) const noexcept;

        DMP_INT_STATUS get_dmp_int_status_register() const noexcept;

        TC get_tc_register() const noexcept;

        INT_STATUS get_int_status_register() const noexcept;

        ACCEL_XOUT get_accel_xout_registers() const noexcept;
        ACCEL_YOUT get_accel_yout_registers() const noexcept;
        ACCEL_ZOUT get_accel_zout_registers() const noexcept;

        TEMP_OUT get_temp_out_registers() const noexcept;

        GYRO_XOUT get_gyro_xout_registers() const noexcept;
        GYRO_YOUT get_gyro_yout_registers() const noexcept;
        GYRO_ZOUT get_gyro_zout_registers() const noexcept;

        EXT_SENS_DATA get_ext_sens_data_register(std::uint8_t const position) const noexcept;

        MOT_DETECT_STATUS get_mot_detect_status_registet() const noexcept;

        void set_i2c_slv_do_register(SlaveNum const slave_num, I2C_SLV_DO const i2c_slv_do) const noexcept;

        void set_i2c_mst_delay_ctrl_register(I2C_MST_DELAY_CTRL const i2c_mst_delay_ctrl) const noexcept;

        void set_signal_path_reset_register(SIGNAL_PATH_RESET const signal_path_reset) const noexcept;

        void set_mot_detect_ctrl_register(MOT_DETECT_CTRL const mot_detect_ctrl) const noexcept;

        void set_user_ctrl_register(USER_CTRL const user_ctrl) const noexcept;

        void set_pwr_mgmt_1_register(PWR_MGMT_1 const pwr_mgmt_1) const noexcept;
        void set_pwr_mgmt_2_register(PWR_MGMT_2 const pwr_mgmt_2) const noexcept;

        void set_bank_sel_register(BANK_SEL const bank_sel) const noexcept;

        void set_mem_start_addr_register(MEM_START_ADDR const mem_start_addr) const noexcept;

        void set_mem_r_w_register(MEM_R_W const mem_r_w) const noexcept;
        MEM_R_W get_mem_r_w_register() const noexcept;

        void set_dmp_cfg_1_register(DMP_CFG_1 const dmp_cfg_1) const noexcept;
        void set_dmp_cfg_2_register(DMP_CFG_2 const dmp_cfg_2) const noexcept;

        FIFO_COUNT get_fifo_count_registers() const noexcept;

        void set_fifo_r_w_register(FIFO_R_W const fifo_r_w) const noexcept;
        FIFO_R_W get_fifo_r_w_register() const noexcept;

        WHO_AM_I get_who_am_i_register() const noexcept;

        bool initialized_{false};

        I2CDevice i2c_device_{};

        Scaled gyro_scale_{};
        Scaled accel_scale_{};
    };

}; // namespace InvertedSway::BitFields

#endif // MPU6050_BITFIELDS_HPP
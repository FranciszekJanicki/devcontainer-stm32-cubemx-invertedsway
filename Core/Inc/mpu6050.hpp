#ifndef MPU6050_HPP
#define MPU6050_HPP

#include "common.hpp"
#include "i2c_device.hpp"
#include "mpu_register_map.hpp"
#include "stm32l4xx_hal.h"
#include "vector3d.hpp"
#include <cstddef>
#include <cstdint>

namespace InvertedSway {
    struct MPU6050 {
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

        MPU6050(I2CDevice const& i2c_device,
                std::uint32_t const sampling_rate,
                GyroRange const gyro_range,
                AccelRange const accel_range,
                DLPF const dlpf,
                DHPF const dhpf) noexcept;

        MPU6050(MPU6050 const& other) noexcept = delete;
        MPU6050(MPU6050&& other) noexcept = default;

        MPU6050& operator=(MPU6050 const& other) noexcept = delete;
        MPU6050& operator=(MPU6050&& other) noexcept = default;

        ~MPU6050() noexcept;

        /* celsius */
        [[nodiscard]] Scaled get_temperature_celsius() const noexcept;

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

        static Scaled gyro_range_to_scale(GyroRange const gyro_range) noexcept;
        static Scaled accel_range_to_scale(AccelRange const accel_range) noexcept;
        static std::uint8_t get_sampling_divider(std::uint32_t const sampling_rate, DLPF const dlpf) noexcept;

        static RollPitchYaw accel_to_roll_pitch_yaw(AccelScaled const& accel_scaled) noexcept;
        static Scaled accel_to_roll(AccelScaled const& accel_scaled) noexcept;
        static Scaled accel_to_pitch(AccelScaled const& accel_scaled) noexcept;
        static Scaled accel_to_yaw(AccelScaled const& accel_scaled) noexcept;

        static std::uint8_t slave_num_to_address(std::uint8_t const num) noexcept;
        static std::uint8_t slave_num_to_register(std::uint8_t const num) noexcept;
        static std::uint8_t slave_num_to_control(std::uint8_t const num) noexcept;
        static std::uint8_t slave_num_to_output_byte(std::uint8_t const num) noexcept;

        static constexpr std::uint32_t GYRO_OUTPUT_RATE_DLPF_EN_HZ{1000U};
        static constexpr std::uint32_t GYRO_OUTPUT_RATE_DLPF_DIS_HZ{8000U};
        static constexpr std::uint32_t ACCEL_OUTPUT_RATE_HZ{1000U};

        bool is_valid_device_id() const noexcept;

        void initialize(std::uint32_t const sampling_rate,
                        GyroRange const gyro_range,
                        AccelRange const accel_range,
                        DLPF const dlpf,
                        DHPF const dhpf) noexcept;
        void initialize_base(GyroRange const gyro_range, AccelRange const accel_range) const noexcept;
        void initialize_advanced(std::uint32_t const sampling_rate, DLPF const dlpf, DHPF const dhpf) const noexcept;
        void initialize_interrupt() const noexcept;
        void initialize_data_ready_interrupt() const noexcept;
        void initialize_f_sync_interrupt() const noexcept;
        void initialize_motion_interrupt() const noexcept;
        void initialize_zero_motion_interrupt() const noexcept;
        void initialize_free_fall_interrupt() const noexcept;
        void deinitialize() noexcept;

        void set_sampling_rate(std::uint8_t const sampling_rate, DLPF const dlpf) const noexcept;
        void set_external_frame_sync(ExtSync const frame_sync) const noexcept;
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

        void set_fifo_enabled(std::uint8_t const fifo_enabled) const noexcept;
        void set_temp_fifo_enabled(bool const enabled) const noexcept;
        void set_x_gyro_fifo_enabled(bool const enabled) const noexcept;
        void set_y_gyro_fifo_enabled(bool const enabled) const noexcept;
        void set_z_gyro_fifo_enabled(bool const enabled) const noexcept;
        void set_accel_fifo_enabled(bool const enabled) const noexcept;
        void set_slave2_fifo_enabled(bool const enabled) const noexcept;
        void set_slave1_fifo_enabled(bool const enabled) const noexcept;
        void set_slave0_fifo_enabled(bool const enabled) const noexcept;

        void set_multi_master_enabled(bool const enabled) const noexcept;
        void set_wait_for_external_sensor_enabled(bool const enabled) const noexcept;
        void set_slave3_fifo_enabled(bool const enabled) const noexcept;
        void set_slave_read_write_transition_enabled(bool const enabled) const noexcept;
        void set_master_clock_speed(std::uint8_t const speed) const noexcept;

        void set_slave_address(std::uint8_t const num, std::uint8_t const address) const noexcept;
        void set_slave_register(std::uint8_t const num, std::uint8_t const reg) const noexcept;
        void set_slave_enabled(std::uint8_t const num, bool const enabled) const noexcept;
        void set_slave_word_byte_swap(std::uint8_t const num, bool const enabled) const noexcept;
        void set_slave_write_mode(std::uint8_t const num, bool const mode) const noexcept;
        void set_slave_word_group_offset(std::uint8_t const num, bool const enabled) const noexcept;
        void set_slave_data_length(std::uint8_t const num, std::uint8_t const length) const noexcept;

        void set_slave4_address(std::uint8_t const address) const noexcept;
        void set_slave4_register(std::uint8_t const reg) const noexcept;
        void set_slave4_output_byte(std::uint8_t const data) const noexcept;
        void set_slave4_enabled(bool const enabled) const noexcept;
        void set_slave4_interrupt_enabled(bool const enabled) const noexcept;
        void set_slave4_write_mode(bool const mode) const noexcept;
        void set_slave4_master_delay(std::uint8_t const delay) const noexcept;
        std::uint8_t get_slave4_input_byte() const noexcept;

        bool get_passthrough_status() const noexcept;
        bool get_slave4_is_done() const noexcept;
        bool get_lost_arbitration() const noexcept;
        bool get_slave4_nack() const noexcept;
        bool get_slave3_nack() const noexcept;
        bool get_slave2_nack() const noexcept;
        bool get_slave1_nack() const noexcept;
        bool get_slave0_nack() const noexcept;

        void set_interrupt(std::uint8_t const interrupt) const noexcept;
        void set_interrupt_mode(IntMode const mode) const noexcept;
        void set_interrupt_drive(IntDrive const drive) const noexcept;
        void set_interrupt_latch(IntLatch const latch) const noexcept;
        void set_interrupt_latch_clear(IntClear const clear) const noexcept;
        void set_f_sync_interrupt_mode(IntMode const mode) const noexcept;
        void set_f_sync_interrupt_enabled(bool const enabled) const noexcept;
        void set_i2c_bypass_enabled(bool const enabled) const noexcept;
        void set_clock_output_enabled(bool const enabled) const noexcept;

        void set_int_enabled(std::uint8_t const int_enabled) const noexcept;
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

        std::uint8_t get_external_sensor_byte(std::uint8_t const position) const noexcept;
        std::uint16_t get_external_sensor_word(std::uint8_t const position) const noexcept;
        std::uint32_t get_external_sensor_dword(std::uint8_t const position) const noexcept;

        std::uint8_t get_motion_status() const noexcept;
        bool get_x_neg_motion_detected() const noexcept;
        bool get_x_pos_motion_detected() const noexcept;
        bool get_y_neg_motion_detected() const noexcept;
        bool get_y_pos_motion_detected() const noexcept;
        bool get_z_neg_motion_detected() const noexcept;
        bool get_z_pos_motion_detected() const noexcept;
        bool get_zero_motion_detected() const noexcept;

        void set_slave_output_byte(std::uint8_t const num, std::uint8_t const data) const noexcept;
        void set_external_shadow_delay_enabled(bool const enabled) const noexcept;
        void set_slave_delay_enabled(std::uint8_t const num, bool const enabled) const noexcept;

        void reset_gyro_path() const noexcept;
        void reset_accel_path() const noexcept;
        void reset_temperature_path() const noexcept;

        void set_motion_detection_control(std::uint8_t const control) const noexcept;
        void set_accel_power_on_delay(Delay const delay) const noexcept;
        void set_free_fall_detection_counter_decrement(DetectDecrement const decrement) const noexcept;
        void set_motion_detection_counter_decrement(DetectDecrement const decrement) const noexcept;

        void set_fifo_enabled(bool const enabled) const noexcept;
        void set_i2c_master_mode_enabled(bool const enabled) const noexcept;
        void reset_fifo() const noexcept;
        void reset_i2c_master() const noexcept;
        void reset_sensors() const noexcept;

        void device_reset() const noexcept;
        void device_wake_up() const noexcept;
        void set_clock_source(Clock const source) const noexcept;
        void set_sleep_enabled(bool const enabled) const noexcept;
        void set_wake_cycle_enabled(bool const enabled) const noexcept;
        void set_temperature_sensor_enabled(bool const enabled) const noexcept;

        void set_wake_up_frequency(WakeFreq const frequency) const noexcept;
        void set_x_accel_standby(bool const standby) const noexcept;
        void set_y_accel_standby(bool const standby) const noexcept;
        void set_z_accel_standby(bool const standby) const noexcept;
        void set_x_gyro_standby(bool const standby) const noexcept;
        void set_y_gyro_standby(bool const standby) const noexcept;
        void set_z_gyro_standby(bool const standby) const noexcept;

        std::uint16_t get_fifo_count() const noexcept;
        std::uint8_t get_fifo_byte() const noexcept;
        void get_current_fifo_packet(std::uint8_t* packet_data, std::size_t const packet_size) const noexcept;
        void get_fifo_bytes(std::uint8_t* read_data, std::size_t const read_size) const noexcept;
        void set_fifo_byte(std::uint8_t const write_data) const noexcept;
        void set_fifo_bytes(std::uint8_t* write_data, std::size_t const write_size) const noexcept;

        std::uint8_t get_device_id() const noexcept;

        bool initialized_{false};

        I2CDevice i2c_device_{};

        Scaled gyro_scale_{};
        Scaled accel_scale_{};
    };

}; // namespace InvertedSway

#endif // MPU6050_HPP
#ifndef MPU6050_REGISTER_MAP_HPP
#define MPU6050_REGISTER_MAP_HPP

#include <cstdint>

#define packed __attribute__((__packed__))

struct SELF_TEST_X {
    uint8_t xa_test : 3;
    uint8_t xg_test : 5;
};

struct SELF_TEST_Y {
    uint8_t ya_test : 3;
    uint8_t yg_test : 5;
};

struct SELF_TEST_Z {
    uint8_t za_test : 3;
    uint8_t zg_test : 5;
};

struct SELF_TEST_A {
    uint8_t : 2;
    uint8_t xa_test : 2;
    uint8_t ya_test : 2;
    uint8_t za_test : 2;
};

struct SMPLRT_DIV {
    uint8_t smplrt_div : 8;
} packed;

struct CONFIG {
    uint8_t : 2;
    uint8_t ext_sync_set : 3;
    uint8_t dlpf_cfg : 3;
} packed;

struct GYRO_CONFIG {
    bool xg_st : 1;
    bool yg_st : 1;
    bool zg_st : 1;
    uint8_t fs_sel : 2;
    uint8_t : 3;
} packed;

struct ACCEL_CONFIG {
    bool xa_st : 1;
    bool ya_st : 1;
    bool za_st : 1;
    uint8_t afs_sel : 2;
    uint8_t : 3;
} packed;

struct FIFO_EN {
    bool temp_fifo_en : 1;
    bool xg_fifo_en : 1;
    bool yg_fifo_en : 1;
    bool zg_fifo_en : 1;
    bool accel_fifo_en : 1;
    bool slv2_fifo_en : 1;
    bool slv1_fifo_en : 1;
    bool slv0_fifo_en : 1;
} packed;

struct I2C_MST_CTRL {
    bool mult_mst_en : 1;
    bool wait_for_es : 1;
    bool slv3_fifo_en : 1;
    bool i2c_mst_p_nsr : 1;
    uint8_t i2c_mst_clk : 4;
} packed;

struct I2C_SLV0 {
    bool i2c_slv0_rw : 1;
    uint8_t i2c_slv0_addr : 7;
    uint8_t i2c_slv0_reg : 8;
    bool i2c_slv0_en : 1;
    bool i2c_slv0_byte_sw : 1;
    bool i2c_slv0_reg_dis : 1;
    bool i2c_slv0_grp : 1;
    uint8_t i2c_slv0_len : 4:
} packed;

struct I2C_SLV1 {
    bool i2c_slv1_rw : 1;
    uint8_t i2c_slv1_addr : 7;
    uint8_t i2c_slv1_reg : 8;
    bool i2c_slv1_en : 1;
    bool i2c_slv1_byte_sw : 1;
    bool i2c_slv1_reg_dis : 1;
    bool i2c_slv1_grp : 1;
    uint8_t i2c_slv1_len : 4:
} packed;

struct I2C_SLV2 {
    bool i2c_slv2_rw : 1;
    uint8_t i2c_slv2_addr : 7;
    uint8_t i2c_slv2_reg : 8;
    bool i2c_slv2_en : 1;
    bool i2c_slv2_byte_sw : 1;
    bool i2c_slv2_reg_dis : 1;
    bool i2c_slv2_grp : 1;
    uint8_t i2c_slv2_len : 4:
} packed;

struct I2C_SLV3 {
    bool i2c_slv3_rw : 1;
    uint8_t i2c_slv3_addr : 7;
    uint8_t i2c_slv3_reg : 8;
    bool i2c_slv3_en : 1;
    bool i2c_slv3_byte_sw : 1;
    bool i2c_slv3_reg_dis : 1;
    bool i2c_slv3_grp : 1;
    uint8_t i2c_slv3_len : 4:
} packed;

struct I2C_SLV4 {
    bool i2c_slv4_rw : 1;
    uint8_t i2c_slv4_addr : 7;
    uint8_t i2c_slv4_reg : 8;
    uint8_t i2c_slv4_do : 8;
    bool i2c_slv4_en : 1;
    bool i2c_slv4_int_en : 1;
    bool i2c_slv4_reg_dis : 1;
    uint8_t i2c_mst_dly : 5:
} packed;

struct INT_MST_STATUS {
    bool pass_through : 1;
    bool i2c_slv4_done : 1;
    bool i2c_lost_arb : 1;
    bool i2c_slv4_nack : 1;
    bool i2c_slv3_nack : 1;
    bool i2c_slv2_nack : 1;
    bool i2c_slv1_nack : 1;
    bool i2c_slv0_nack : 1;
} packed;

struct INT_ENABLE {
    bool int_level : 1;
    bool int_open : 1;
    bool latch_int_en : 1;
    bool int_rd_clear : 1;
    bool fsync_int_level : 1;
    bool i2c_bypass_en : 1;
    bool : 1;
} packed;

struct INT_ENABLE {
    uint8_t : 3;
    bool fifo_oflow_en : 1;
    bool i2c_mst_en : 1;
    uint8_t : 2;
    bool data_rdy_en : 1;
} packed;

struct INT_STATUS {
    uint8_t : 3;
    bool fifo_oflow_int : 1;
    bool i2c_mst_int : 1;
    uint8_t : 2;
    bool data_rdy_int : 1;
} packed;

struct ACCEL_XOUT {
    uint8_t aceel_x_out_h : 8;
    uint8_t aceel_x_out_l : 8;
} packed;

struct ACCEL_YOUT {
    uint8_t aceel_y_out_h : 8;
    uint8_t aceel_y_out_l : 8;
} packed;

struct ACCEL_ZOUT {
    uint8_t aceel_z_out_h : 8;
    uint8_t aceel_z_out_l : 8;
} packed;

struct TEMP_OUT {
    uint8_t temp_out_h : 8;
    uint8_t temp_out_l : 8;
} packed;

struct GYRO_XOUT {
    uint8_t gyro_x_out_h : 8;
    uint8_t gyro_x_out_l : 8;
} packed;

struct GYRO_YOUT {
    uint8_t gyro_y_out_h : 8;
    uint8_t gyro_y_out_l : 8;
} packed;

struct GYRO_ZOUT {
    uint8_t gyro_z_out_h : 8;
    uint8_t gyro_z_out_l : 8;
} packed;

struct EXT_SENS_DATA {
    uint8_t ext_sens_data_00 : 8;
    uint8_t ext_sens_data_01 : 8;
    uint8_t ext_sens_data_02 : 8;
    uint8_t ext_sens_data_03 : 8;
    uint8_t ext_sens_data_04 : 8;
    uint8_t ext_sens_data_05 : 8;
    uint8_t ext_sens_data_06 : 8;
    uint8_t ext_sens_data_07 : 8;
    uint8_t ext_sens_data_08 : 8;
    uint8_t ext_sens_data_09 : 8;
    uint8_t ext_sens_data_10 : 8;
    uint8_t ext_sens_data_11 : 8;
    uint8_t ext_sens_data_12 : 8;
    uint8_t ext_sens_data_13 : 8;
    uint8_t ext_sens_data_14 : 8;
    uint8_t ext_sens_data_15 : 8;
    uint8_t ext_sens_data_16 : 8;
    uint8_t ext_sens_data_17 : 8;
    uint8_t ext_sens_data_18 : 8;
    uint8_t ext_sens_data_19 : 8;
    uint8_t ext_sens_data_20 : 8;
    uint8_t ext_sens_data_21 : 8;
    uint8_t ext_sens_data_22 : 8;
    uint8_t ext_sens_data_23 : 8;
} packed;

struct I2C_SLV0DO {
    uint8_t i2c_slv0_do : 8;
} packed;

struct I2C_SLV1DO {
    uint8_t i2c_slv1_do : 8;
} packed;

struct I2C_SLV2_DO {
    uint8_t i2c_slv2_do : 8;
} packed;

struct I2C_SLV3_DO {
    uint8_t i2c_slv3_do : 8;
} packed;

struct I2C_MST_DELAY_CTRL {
    bool delay_es_shadow : 1;
    uint8_t : 2;
    bool i2c_slv4_dly_en : 1;
    bool i2c_slv3_dly_en : 1;
    bool i2c_slv2_dly_en : 1;
    bool i2c_slv1_dly_en : 1;
    bool i2c_slv0_dly_en : 1;
} packed;

struct SIGNAL_PATH_RESET {
    uint8_t : 5;
    bool gyro_reset : 1;
    bool accel_reset : 1;
    bool temp_reset : 1;
} packed;

struct USER_CTRL {
    bool : 1;
    bool fifo_en : 1;
    bool i2c_mst_en : 1;
    bool i2c_if_dis : 1;
    bool : 1;
    bool fifo_reset : 1;
    bool i2c_mst_reset : 1;
    bool sig_cond_reset : 1;
} packed;

struct PWR_MGMT_1 {
    bool device_reset : 1;
    bool sleep : 1;
    bool cycle : 1;
    bool : 1;
    bool temp_dis : 1;
    uint8_t clksel : 3;
} packed;

struct PWR_MGMT_2 {
    uint8_t lp_wake_ctrl : 2;
    bool stby_xa : 1;
    bool stby_ya : 1;
    bool stby_za : 1;
    bool stby_xg : 1;
    bool stby_yg : 1;
    bool stby_yz : 1;
} packed;

struct FIFO_COUNT {
    uint8_t fifo_count_h : 8;
    uint8_t fifo_count_l : 8;
} packed;

struct FIFO_R_W {
    uint8_t fifo_data : 8;
} packed;

struct WHO_AM_I {
    bool : 1;
    uint8_t who_am_i : 6;
    bool : 1;
} packed;

#endif // MPU6050_REGISTER_MAP_HPP
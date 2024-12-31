#ifndef MPU_REG_MAP_HPP
#define MPU_REG_MAP_HPP

#include <cstdint>

#define packed __attribute__((__packed__))

struct AUX_VDDIO {
    bool aux_vddio : 1;
    uint8_t xg_offs_tc : 7;
} packed;

struct YG_OFFS_TC {
    bool : 1;
    uint8_t yg_offs_tc : 7;
} packed;

struct ZG_OFFS_TC {
    bool : 1;
    uint8_t zg_offs_tc : 7;
} packed;

struct X_FINE_GAIN {
    uint8_t x_fine_gain : 8;
} packed;

struct Y_FINE_GAIN {
    uint8_t y_fine_gain : 8;
} packed;

struct Z_FINE_GAIN {
    uint8_t z_fine_gain : 8;
} packed;

struct XA_OFFS {
    uint8_t xa_offs_h : 8;
    uint8_t xa_offs_l_tc : 8;
} packed;

struct YA_OFFS {
    uint8_t ya_offs_h : 8;
    uint8_t ya_offs_l_tc : 8;
} packed;

struct ZA_OFFS {
    uint8_t za_offs_h : 8;
    uint8_t za_offs_l_tc : 8;
} packed;

struct XG_OFFS_USR {
    uint8_t xg_offs_usrh : 8;
    uint8_t xg_offs_usrhl : 8;
} packed;

struct YG_OFFS_USR {
    uint8_t yg_offs_usrh : 8;
    uint8_t yg_offs_usrhl : 8;
} packed;

struct ZG_OFFS_USR {
    uint8_t zg_offs_usrh : 8;
    uint8_t zg_offs_usrhl : 8;
} packed;

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

struct FF_THR {
    uint8_t ff_thr : 8;
} packed;

struct FF_DUR {
    uint8_t ff_dur : 8;
} packed;

struct MOT_THR {
    uint8_t mot_thr : 8;
} packed;

struct MOT_DUR {
    uint8_t mot_dur : 8;
} packed;

struct ZRMOT_THR {
    uint8_t zrmot_thr : 8;
} packed;

struct ZRMOT_DUR {
    uint8_t zrmot_dur : 8;
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

struct I2C_SLV0_ADDR {
    bool i2c_slv0_rw : 1;
    uint8_t i2c_slv0_addr : 7;
} packed;

struct I2C_SLV0_REG {
    uint8_t i2c_slv0_reg : 8;
} packed;

struct I2C_SLV0_CTRL {
    bool i2c_slv0_en : 1;
    bool i2c_slv0_byte_sw : 1;
    bool i2c_slv0_reg_dis : 1;
    bool i2c_slv0_grp : 1;
    uint8_t i2c_slv0_len : 4;
} packed;

struct I2C_SLV1_ADDR {
    bool i2c_slv1_rw : 1;
    uint8_t i2c_slv1_addr : 7;
} packed;

struct I2C_SLV1_REG {
    uint8_t i2c_slv1_reg : 8;
} packed;

struct I2C_SLV1_CTRL {
    bool i2c_slv1_en : 1;
    bool i2c_slv1_byte_sw : 1;
    bool i2c_slv1_reg_dis : 1;
    bool i2c_slv1_grp : 1;
    uint8_t i2c_slv1_len : 4;
} packed;

struct I2C_SLV2_ADDR {
    bool i2c_slv2_rw : 1;
    uint8_t i2c_slv2_addr : 7;
} packed;

struct I2C_SLV2_REG {
    uint8_t i2c_slv2_reg : 8;
} packed;

struct I2C_SLV2_CTRL {
    bool i2c_slv2_en : 1;
    bool i2c_slv2_byte_sw : 1;
    bool i2c_slv2_reg_dis : 1;
    bool i2c_slv2_grp : 1;
    uint8_t i2c_slv2_len : 4;
} packed;

struct I2C_SLV3_ADDR {
    bool i2c_slv3_rw : 1;
    uint8_t i2c_slv3_addr : 7;
} packed;

struct I2C_SLV3_REG {
    uint8_t i2c_slv3_reg : 8;
} packed;

struct I2C_SLV3_CTRL {
    bool i2c_slv3_en : 1;
    bool i2c_slv3_byte_sw : 1;
    bool i2c_slv3_reg_dis : 1;
    bool i2c_slv3_grp : 1;
    uint8_t i2c_slv3_len : 4;
} packed;

struct I2C_SLV4_ADDR {
    bool i2c_slv4_rw : 1;
    uint8_t i2c_slv4_addr : 7;
} packed;

struct I2C_SLV4_REG {
    uint8_t i2c_slv4_reg : 8;
} packed;

struct I2C_SLV4_DO {
    uint8_t i2c_slv4_do : 8;
} packed;

struct I2C_SLV4_CTRL {
    bool i2c_slv4_en : 1;
    bool i2c_slv4_int_en : 1;
    bool i2c_slv4_reg_dis : 1;
    uint8_t i2c_mst_dly : 5;
} packed;

struct I2C_SLV4_DI {
    uint8_t i2c_slv4_di : 8;
} packed;

struct I2C_MST_STATUS {
    bool pass_through : 1;
    bool i2c_slv4_done : 1;
    bool i2c_lost_arb : 1;
    bool i2c_slv4_nack : 1;
    bool i2c_slv3_nack : 1;
    bool i2c_slv2_nack : 1;
    bool i2c_slv1_nack : 1;
    bool i2c_slv0_nack : 1;
} packed;

struct INT_PIN_CFG {
    bool int_level : 1;
    bool int_open : 1;
    bool latch_int_en : 1;
    bool int_rd_clear : 1;
    bool fsync_int_level : 1;
    bool i2c_bypass_en : 1;
    bool : 1;
} packed;

struct INT_ENABLE {
    bool ff_en : 1;
    bool mot_en : 1;
    bool zmot_en : 1;
    bool fifo_oflow_en : 1;
    bool i2c_mst_en : 1;
    bool pll_rdy_int_en : 1;
    bool dmp_int_en : 1;
    bool raw_rdy_int_en : 1;
} packed;

struct DMP_INT_STATUS {
    uint8_t : 2;
    bool dmp_int_5 : 1;
    bool dmp_int_4 : 1;
    bool dmp_int_3 : 1;
    bool dmp_int_2 : 1;
    bool dmp_int_1 : 1;
    bool dmp_int_0 : 1;
} packed;

struct TC {
    bool pwr_mode : 1;
    uint8_t offset : 6;
    bool otp_bnk_vld : 1;
} packed;

struct INT_STATUS {
    bool ff_int : 1;
    bool mot_int : 1;
    bool zmot_int : 1;
    bool fifo_oflow_int : 1;
    bool i2c_mst_int : 1;
    bool pll_rdy_int : 1;
    bool dmp_int : 1;
    bool raw_rdy_int : 1;
} packed;

struct ACCEL_XOUT {
    uint8_t aceel_xout_h : 8;
    uint8_t aceel_xout_l : 8;
} packed;

struct ACCEL_YOUT {
    uint8_t aceel_yout_h : 8;
    uint8_t aceel_yout_l : 8;
} packed;

struct ACCEL_ZOUT {
    uint8_t aceel_zout_h : 8;
    uint8_t aceel_zout_l : 8;
} packed;

struct TEMP_OUT {
    uint8_t temp_out_h : 8;
    uint8_t temp_out_l : 8;
} packed;

struct GYRO_XOUT {
    uint8_t gyro_xout_h : 8;
    uint8_t gyro_xout_l : 8;
} packed;

struct GYRO_YOUT {
    uint8_t gyro_yout_h : 8;
    uint8_t gyro_yout_l : 8;
} packed;

struct GYRO_ZOUT {
    uint8_t gyro_zout_h : 8;
    uint8_t gyro_zout_l : 8;
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

struct MOT_DETECT_STATUS {
    bool mot_xneg : 1;
    bool mot_xpos : 1;
    bool mot_yneg : 1;
    bool mot_ypos : 1;
    bool mot_zneg : 1;
    bool mot_zpos : 1;
    bool : 1;
    bool mot_zrmot : 1;
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

struct MOT_DETECT_CTRL {
    uint8_t : 2;
    uint8_t accel_on_delay : 2;
    uint8_t ff_count : 2;
    uint8_t mot_count : 2;
} packed;

struct USER_CTRL {
    bool dmp_en : 1;
    bool fifo_en : 1;
    bool i2c_mst_en : 1;
    bool i2c_if_dis : 1;
    bool dmp_reset : 1;
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
    bool lp_wake_ctrl : 1;
    bool : 1;
    bool stby_xa : 1;
    bool stby_ya : 1;
    bool stby_za : 1;
    bool stby_xg : 1;
    bool stby_yg : 1;
    bool stby_yz : 1;
} packed;

struct BANK_SEL {
    bool : 1;
    bool prftch_en : 1;
    bool cfg_user_bank : 1;
    uint8_t mem_sel : 5;
} packed;

struct MEM_START_ADDR {
    uint8_t start_addr : 8;
} packed;

struct MEM_R_W {
    uint8_t mem_r_w : 8;
} packed;

struct DMP_CFG_1 {
    uint8_t dmp_cfg_1 : 8;
} packed;

struct DMP_CFG_2 {
    uint8_t dmp_cfg_2 : 8;
} packed;

struct FIFO_COUNT {
    uint8_t fifo_count_h : 8;
    uint8_t fifo_count_l : 8;
} packed;

struct FIFO_R_W {
    uint8_t fifo_r_w : 8;
} packed;

struct WHO_AM_I {
    bool : 1;
    uint8_t who_am_i : 6;
    bool : 1;
} packed;

#endif // MPU_REGISTERS_HPP
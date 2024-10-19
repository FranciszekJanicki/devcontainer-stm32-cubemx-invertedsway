#ifndef MPU6050_CONFIG_HPP
#define MPU6050_CONFIG_HPP

constexpr std::uint8_t ADDRESS = 0xD0; // AD0 low
// constexpr std::uint8_t ADDRESS = 0xD1	// AD0 high
//
//	Registers addresses
//
constexpr std::uint8_t RA_SELF_TEST_X = 0x0D;
constexpr std::uint8_t RA_SELF_TEST_Y = 0x0E;
constexpr std::uint8_t RA_SELF_TEST_Z = 0x0F;
constexpr std::uint8_t RA_SELF_TEST_A = 0x10;
constexpr std::uint8_t RA_SMPLRT_DIV = 0x19;
constexpr std::uint8_t RA_CONFIG = 0x1A;
constexpr std::uint8_t RA_GYRO_CONFIG = 0x1B;
constexpr std::uint8_t RA_ACCEL_CONFIG = 0x1C;
// Not in documentation

constexpr std::uint8_t RA_FF_THR = 0x1D;
constexpr std::uint8_t RA_FF_DUR = 0x1E;
constexpr std::uint8_t RA_MOT_THR = 0x1F;
constexpr std::uint8_t RA_MOT_DUR = 0x20;
constexpr std::uint8_t RA_ZRMOT_THR = 0x21;
constexpr std::uint8_t RA_ZRMOT_DUR = 0x22;
// Not in documentation end

constexpr std::uint8_t RA_FIFO_EN = 0x23;
constexpr std::uint8_t RA_I2C_MST_CTRL = 0x24;
constexpr std::uint8_t RA_I2C_SLV0_ADDR = 0x25;
constexpr std::uint8_t RA_I2C_SLV0_REG = 0x26;
constexpr std::uint8_t RA_I2C_SLV0_CTRL = 0x27;
constexpr std::uint8_t RA_I2C_SLV1_ADDR = 0x28;
constexpr std::uint8_t RA_I2C_SLV1_REG = 0x29;
constexpr std::uint8_t RA_I2C_SLV1_CTRL = 0x2A;
constexpr std::uint8_t RA_I2C_SLV2_ADDR = 0x2B;
constexpr std::uint8_t RA_I2C_SLV2_REG = 0x2C;
constexpr std::uint8_t RA_I2C_SLV2_CTRL = 0x2D;
constexpr std::uint8_t RA_I2C_SLV3_ADDR = 0x2E;
constexpr std::uint8_t RA_I2C_SLV3_REG = 0x2F;
constexpr std::uint8_t RA_I2C_SLV3_CTRL = 0x30;
constexpr std::uint8_t RA_I2C_SLV4_ADDR = 0x31;
constexpr std::uint8_t RA_I2C_SLV4_REG = 0x32;
constexpr std::uint8_t RA_I2C_SLV4_DO = 0x33;
constexpr std::uint8_t RA_I2C_SLV4_CTRL = 0x34;
constexpr std::uint8_t RA_I2C_SLV4_DI = 0x35;
constexpr std::uint8_t RA_I2C_MST_STATUS = 0x36;
constexpr std::uint8_t RA_INT_PIN_CFG = 0x37;
constexpr std::uint8_t RA_INT_ENABLE = 0x38;
// Not in documentation

constexpr std::uint8_t RA_DMP_INT_STATUS = 0x39;
// Not in documentation end

constexpr std::uint8_t RA_INT_STATUS = 0x3A;
constexpr std::uint8_t RA_ACCEL_XOUT_H = 0x3B;
constexpr std::uint8_t RA_ACCEL_XOUT_L = 0x3C;
constexpr std::uint8_t RA_ACCEL_YOUT_H = 0x3D;
constexpr std::uint8_t RA_ACCEL_YOUT_L = 0x3E;
constexpr std::uint8_t RA_ACCEL_ZOUT_H = 0x3F;
constexpr std::uint8_t RA_ACCEL_ZOUT_L = 0x40;
constexpr std::uint8_t RA_TEMP_OUT_H = 0x41;
constexpr std::uint8_t RA_TEMP_OUT_L = 0x42;
constexpr std::uint8_t RA_GYRO_XOUT_H = 0x43;
constexpr std::uint8_t RA_GYRO_XOUT_L = 0x44;
constexpr std::uint8_t RA_GYRO_YOUT_H = 0x45;
constexpr std::uint8_t RA_GYRO_YOUT_L = 0x46;
constexpr std::uint8_t RA_GYRO_ZOUT_H = 0x47;
constexpr std::uint8_t RA_GYRO_ZOUT_L = 0x48;
constexpr std::uint8_t RA_EXT_SENS_DATA_00 = 0x49;
constexpr std::uint8_t RA_EXT_SENS_DATA_01 = 0x4A;
constexpr std::uint8_t RA_EXT_SENS_DATA_02 = 0x4B;
constexpr std::uint8_t RA_EXT_SENS_DATA_03 = 0x4C;
constexpr std::uint8_t RA_EXT_SENS_DATA_04 = 0x4D;
constexpr std::uint8_t RA_EXT_SENS_DATA_05 = 0x4E;
constexpr std::uint8_t RA_EXT_SENS_DATA_06 = 0x4F;
constexpr std::uint8_t RA_EXT_SENS_DATA_07 = 0x50;
constexpr std::uint8_t RA_EXT_SENS_DATA_08 = 0x51;
constexpr std::uint8_t RA_EXT_SENS_DATA_09 = 0x52;
constexpr std::uint8_t RA_EXT_SENS_DATA_10 = 0x53;
constexpr std::uint8_t RA_EXT_SENS_DATA_11 = 0x54;
constexpr std::uint8_t RA_EXT_SENS_DATA_12 = 0x55;
constexpr std::uint8_t RA_EXT_SENS_DATA_13 = 0x56;
constexpr std::uint8_t RA_EXT_SENS_DATA_14 = 0x57;
constexpr std::uint8_t RA_EXT_SENS_DATA_15 = 0x58;
constexpr std::uint8_t RA_EXT_SENS_DATA_16 = 0x59;
constexpr std::uint8_t RA_EXT_SENS_DATA_17 = 0x5A;
constexpr std::uint8_t RA_EXT_SENS_DATA_18 = 0x5B;
constexpr std::uint8_t RA_EXT_SENS_DATA_19 = 0x5C;
constexpr std::uint8_t RA_EXT_SENS_DATA_20 = 0x5D;
constexpr std::uint8_t RA_EXT_SENS_DATA_21 = 0x5E;
constexpr std::uint8_t RA_EXT_SENS_DATA_22 = 0x5F;
constexpr std::uint8_t RA_EXT_SENS_DATA_23 = 0x60;
// Not in documentation

constexpr std::uint8_t RA_MOT_DETECT_STATUS = 0x61;
// Not in documentation end

constexpr std::uint8_t RA_I2C_SLV0_DO = 0x63;
constexpr std::uint8_t RA_I2C_SLV1_DO = 0x64;
constexpr std::uint8_t RA_I2C_SLV2_DO = 0x65;
constexpr std::uint8_t RA_I2C_SLV3_DO = 0x66;
constexpr std::uint8_t RA_I2C_MST_DELAY_CTRL = 0x67;
constexpr std::uint8_t RA_SIGNAL_PATH_RESET = 0x68;
// Not in documentation

constexpr std::uint8_t RA_MOT_DETECT_CTRL = 0x69;
// Not in documentation end

constexpr std::uint8_t RA_USER_CTRL = 0x6A;
constexpr std::uint8_t RA_PWR_MGMT_1 = 0x6B;
constexpr std::uint8_t RA_PWR_MGMT_2 = 0x6C;
constexpr std::uint8_t RA_FIFO_COUNTH = 0x72;
constexpr std::uint8_t RA_FIFO_COUNTL = 0x73;
constexpr std::uint8_t RA_FIFO_R_W = 0x74;
constexpr std::uint8_t RA_WHO_AM_I = 0x75;
//
//	Registers 13 to 16 � Self Test Registers
//	SELF_TEST_X, SELF_TEST_Y, SELF_TEST_Z, and SELF_TEST_A
//
constexpr std::uint8_t SELF_TEST_XA_1_BIT = 0x07;
constexpr std::uint8_t SELF_TEST_XA_1_LENGTH = 0x03;
constexpr std::uint8_t SELF_TEST_XA_2_BIT = 0x05;
constexpr std::uint8_t SELF_TEST_XA_2_LENGTH = 0x02;
constexpr std::uint8_t SELF_TEST_YA_1_BIT = 0x07;
constexpr std::uint8_t SELF_TEST_YA_1_LENGTH = 0x03;
constexpr std::uint8_t SELF_TEST_YA_2_BIT = 0x03;
constexpr std::uint8_t SELF_TEST_YA_2_LENGTH = 0x02;
constexpr std::uint8_t SELF_TEST_ZA_1_BIT = 0x07;
constexpr std::uint8_t SELF_TEST_ZA_1_LENGTH = 0x03;
constexpr std::uint8_t SELF_TEST_ZA_2_BIT = 0x01;
constexpr std::uint8_t SELF_TEST_ZA_2_LENGTH = 0x02;

constexpr std::uint8_t SELF_TEST_XG_1_BIT = 0x04;
constexpr std::uint8_t SELF_TEST_XG_1_LENGTH = 0x05;
constexpr std::uint8_t SELF_TEST_YG_1_BIT = 0x04;
constexpr std::uint8_t SELF_TEST_YG_1_LENGTH = 0x05;
constexpr std::uint8_t SELF_TEST_ZG_1_BIT = 0x04;
constexpr std::uint8_t SELF_TEST_ZG_1_LENGTH = 0x05;
//
//	Register 26 � Configuration
//	CONFIG
//
constexpr std::uint8_t CFG_EXT_SYNC_SET_BIT = 5;
constexpr std::uint8_t CFG_EXT_SYNC_SET_LENGTH = 3;
constexpr std::uint8_t CFG_DLPF_CFG_BIT = 2;
constexpr std::uint8_t CFG_DLPF_CFG_LENGTH = 3;

constexpr std::uint8_t EXT_SYNC_DISABLED = 0x0;
constexpr std::uint8_t EXT_SYNC_TEMP_OUT_L = 0x1;
constexpr std::uint8_t EXT_SYNC_GYRO_XOUT_L = 0x2;
constexpr std::uint8_t EXT_SYNC_GYRO_YOUT_L = 0x3;
constexpr std::uint8_t EXT_SYNC_GYRO_ZOUT_L = 0x4;
constexpr std::uint8_t EXT_SYNC_ACCEL_XOUT_L = 0x5;
constexpr std::uint8_t EXT_SYNC_ACCEL_YOUT_L = 0x6;
constexpr std::uint8_t EXT_SYNC_ACCEL_ZOUT_L = 0x7;

constexpr std::uint8_t DLPF_BW_256 = 0x00;
constexpr std::uint8_t DLPF_BW_188 = 0x01;
constexpr std::uint8_t DLPF_BW_98 = 0x02;
constexpr std::uint8_t DLPF_BW_42 = 0x03;
constexpr std::uint8_t DLPF_BW_20 = 0x04;
constexpr std::uint8_t DLPF_BW_10 = 0x05;
constexpr std::uint8_t DLPF_BW_5 = 0x06;
//
//	Register 27 � Gyroscope Configuration
//	GYRO_CONFIG
//
constexpr std::uint8_t GCONFIG_FS_SEL_BIT = 4;
constexpr std::uint8_t GCONFIG_FS_SEL_LENGTH = 2;

constexpr std::uint8_t GYRO_FS_250 = 0x00;
constexpr std::uint8_t GYRO_FS_500 = 0x01;
constexpr std::uint8_t GYRO_FS_1000 = 0x02;
constexpr std::uint8_t GYRO_FS_2000 = 0x03;
//
//	Register 28 � Accelerometer Configuration
//	ACCEL_CONFIG
//
constexpr std::uint8_t ACONFIG_XA_ST_BIT = 7;
constexpr std::uint8_t ACONFIG_YA_ST_BIT = 6;
constexpr std::uint8_t ACONFIG_ZA_ST_BIT = 5;
constexpr std::uint8_t ACONFIG_AFS_SEL_BIT = 4;
constexpr std::uint8_t ACONFIG_AFS_SEL_LENGTH = 2;
constexpr std::uint8_t ACONFIG_ACCEL_HPF_BIT = 2;
constexpr std::uint8_t ACONFIG_ACCEL_HPF_LENGTH = 3;

constexpr std::uint8_t ACCEL_FS_2 = 0x00;
constexpr std::uint8_t ACCEL_FS_4 = 0x01;
constexpr std::uint8_t ACCEL_FS_8 = 0x02;
constexpr std::uint8_t ACCEL_FS_16 = 0x03;
//
//
//
constexpr std::uint8_t DHPF_RESET = 0x00;
constexpr std::uint8_t DHPF_5 = 0x01;
constexpr std::uint8_t DHPF_2P5 = 0x02;
constexpr std::uint8_t DHPF_1P25 = 0x03;
constexpr std::uint8_t DHPF_0P63 = 0x04;
constexpr std::uint8_t DHPF_HOLD = 0x07;
//
//	Register 35 � FIFO Enable
//	FIFO_EN
//
constexpr std::uint8_t TEMP_FIFO_EN_BIT = 7;
constexpr std::uint8_t XG_FIFO_EN_BIT = 6;
constexpr std::uint8_t YG_FIFO_EN_BIT = 5;
constexpr std::uint8_t ZG_FIFO_EN_BIT = 4;
constexpr std::uint8_t ACCEL_FIFO_EN_BIT = 3;
constexpr std::uint8_t SLV2_FIFO_EN_BIT = 2;
constexpr std::uint8_t SLV1_FIFO_EN_BIT = 1;
constexpr std::uint8_t SLV0_FIFO_EN_BIT = 0;
//
//	Register 36 � I2C Master Control
//	I2C_MST_CTRL
//
constexpr std::uint8_t MULT_MST_EN_BIT = 7;
constexpr std::uint8_t WAIT_FOR_ES_BIT = 6;
constexpr std::uint8_t SLV_3_FIFO_EN_BIT = 5;
constexpr std::uint8_t I2C_MST_P_NSR_BIT = 4;
constexpr std::uint8_t I2C_MST_CLK_BIT = 3;
constexpr std::uint8_t I2C_MST_CLK_LENGTH = 4;

constexpr std::uint8_t CLOCK_DIV_500 = 0x9;
constexpr std::uint8_t CLOCK_DIV_471 = 0xA;
constexpr std::uint8_t CLOCK_DIV_444 = 0xB;
constexpr std::uint8_t CLOCK_DIV_421 = 0xC;
constexpr std::uint8_t CLOCK_DIV_400 = 0xD;
constexpr std::uint8_t CLOCK_DIV_381 = 0xE;
constexpr std::uint8_t CLOCK_DIV_364 = 0xF;
constexpr std::uint8_t CLOCK_DIV_348 = 0x0;
constexpr std::uint8_t CLOCK_DIV_333 = 0x1;
constexpr std::uint8_t CLOCK_DIV_320 = 0x2;
constexpr std::uint8_t CLOCK_DIV_308 = 0x3;
constexpr std::uint8_t CLOCK_DIV_296 = 0x4;
constexpr std::uint8_t CLOCK_DIV_286 = 0x5;
constexpr std::uint8_t CLOCK_DIV_276 = 0x6;
constexpr std::uint8_t CLOCK_DIV_267 = 0x7;
constexpr std::uint8_t CLOCK_DIV_258 = 0x8;
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
constexpr std::uint8_t I2C_SLV_RW_BIT = 7;
constexpr std::uint8_t I2C_SLV_ADDR_BIT = 6;
constexpr std::uint8_t I2C_SLV_ADDR_LENGTH = 7;
constexpr std::uint8_t I2C_SLV_EN_BIT = 7;
constexpr std::uint8_t I2C_SLV_BYTE_SW_BIT = 6;
constexpr std::uint8_t I2C_SLV_REG_DIS_BIT = 5;
constexpr std::uint8_t I2C_SLV_GRP_BIT = 4;
constexpr std::uint8_t I2C_SLV_LEN_BIT = 3;
constexpr std::uint8_t I2C_SLV_LEN_LENGTH = 4;
//
//	Registers 49 to 53 � I2C Slave 4 Control
//	I2C_SLV4_ADDR, I2C_SLV4_REG, I2C_SLV4_DO, I2C_SLV4_CTRL, and I2C_SLV4_DI
//
constexpr std::uint8_t I2C_SLV4_RW_BIT = 7;
constexpr std::uint8_t I2C_SLV4_ADDR_BIT = 6;
constexpr std::uint8_t I2C_SLV4_ADDR_LENGTH = 7;
constexpr std::uint8_t I2C_SLV4_EN_BIT = 7;
constexpr std::uint8_t I2C_SLV4_INT_EN_BIT = 6;
constexpr std::uint8_t I2C_SLV4_REG_DIS_BIT = 5;
constexpr std::uint8_t I2C_SLV4_MST_DLY_BIT = 4;
constexpr std::uint8_t I2C_SLV4_MST_DLY_LENGTH = 5;
//
//	Register 54 � I2C Master Status
//	I2C_MST_STATUS
//
constexpr std::uint8_t MST_PASS_THROUGH_BIT = 7;
constexpr std::uint8_t MST_I2C_SLV4_DONE_BIT = 6;
constexpr std::uint8_t MST_I2C_LOST_ARB_BIT = 5;
constexpr std::uint8_t MST_I2C_SLV4_NACK_BIT = 4;
constexpr std::uint8_t MST_I2C_SLV3_NACK_BIT = 3;
constexpr std::uint8_t MST_I2C_SLV2_NACK_BIT = 2;
constexpr std::uint8_t MST_I2C_SLV1_NACK_BIT = 1;
constexpr std::uint8_t MST_I2C_SLV0_NACK_BIT = 0;

//
//	Register 55 � INT Pin / Bypass Enable Configuration
//	INT_PIN_CFG
//
constexpr std::uint8_t INTCFG_INT_LEVEL_BIT = 7;
constexpr std::uint8_t INTCFG_INT_OPEN_BIT = 6;
constexpr std::uint8_t INTCFG_LATCH_INT_EN_BIT = 5;
constexpr std::uint8_t INTCFG_INT_RD_CLEAR_BIT = 4;
constexpr std::uint8_t INTCFG_FSYNC_INT_LEVEL_BIT = 3;
constexpr std::uint8_t INTCFG_FSYNC_INT_EN_BIT = 2;
constexpr std::uint8_t INTCFG_I2C_BYPASS_EN_BIT = 1;

constexpr std::uint8_t INTMODE_ACTIVEHIGH = 0x00;
constexpr std::uint8_t INTMODE_ACTIVELOW = 0x01;

constexpr std::uint8_t INTDRV_PUSHPULL = 0x00;
constexpr std::uint8_t INTDRV_OPENDRAIN = 0x01;

constexpr std::uint8_t INTLATCH_50USPULSE = 0x00;
constexpr std::uint8_t INTLATCH_WAITCLEAR = 0x01;

constexpr std::uint8_t INTCLEAR_STATUSREAD = 0x00;
constexpr std::uint8_t INTCLEAR_ANYREAD = 0x01;
//
//	Register 56 � Interrupt Enable
//	INT_ENABLE
//
//	Register 58 � Interrupt Status
//	INT_STATUS
//

// Not in documentation
constexpr std::uint8_t INTERRUPT_FF_BIT = 7;
constexpr std::uint8_t INTERRUPT_MOT_BIT = 6;
constexpr std::uint8_t INTERRUPT_ZMOT_BIT = 5;
// Not in documentation end
constexpr std::uint8_t INTERRUPT_FIFO_OFLOW_BIT = 4;
constexpr std::uint8_t INTERRUPT_I2C_MST_INT_BIT = 3;
constexpr std::uint8_t INTERRUPT_DATA_RDY_BIT = 0;
//
//	Not documented
//	Register 91 - Motion Status
//
constexpr std::uint8_t MOTION_MOT_XNEG_BIT = 7;
constexpr std::uint8_t MOTION_MOT_XPOS_BIT = 6;
constexpr std::uint8_t MOTION_MOT_YNEG_BIT = 5;
constexpr std::uint8_t MOTION_MOT_YPOS_BIT = 4;
constexpr std::uint8_t MOTION_MOT_ZNEG_BIT = 3;
constexpr std::uint8_t MOTION_MOT_ZPOS_BIT = 2;
constexpr std::uint8_t MOTION_MOT_ZRMOT_BIT = 0;
//
//	Register 103 � I2C Master Delay Control
//	I2C_MST_DELAY_CTR
//
constexpr std::uint8_t DELAYCTRL_DELAY_ES_SHADOW_BIT = 7;
constexpr std::uint8_t DELAYCTRL_I2C_SLV4_DLY_EN_BIT = 4;
constexpr std::uint8_t DELAYCTRL_I2C_SLV3_DLY_EN_BIT = 3;
constexpr std::uint8_t DELAYCTRL_I2C_SLV2_DLY_EN_BIT = 2;
constexpr std::uint8_t DELAYCTRL_I2C_SLV1_DLY_EN_BIT = 1;
constexpr std::uint8_t DELAYCTRL_I2C_SLV0_DLY_EN_BIT = 0;
//
//	Register 104 � Signal Path Reset
//	SIGNAL_PATH_RESE
//
constexpr std::uint8_t PATHRESET_GYRO_RESET_BIT = 2;
constexpr std::uint8_t PATHRESET_ACCEL_RESET_BIT = 1;
constexpr std::uint8_t PATHRESET_TEMP_RESET_BIT = 0;
//
//	Not documented
//	Register 105 - Motion Detect Control
//
constexpr std::uint8_t DETECT_ACCEL_ON_DELAY_BIT = 5;
constexpr std::uint8_t DETECT_ACCEL_ON_DELAY_LENGTH = 2;
constexpr std::uint8_t DETECT_FF_COUNT_BIT = 3;
constexpr std::uint8_t DETECT_FF_COUNT_LENGTH = 2;
constexpr std::uint8_t DETECT_MOT_COUNT_BIT = 1;
constexpr std::uint8_t DETECT_MOT_COUNT_LENGTH = 2;

constexpr std::uint8_t DETECT_DECREMENT_RESET = 0x0;
constexpr std::uint8_t DETECT_DECREMENT_1 = 0x1;
constexpr std::uint8_t DETECT_DECREMENT_2 = 0x2;
constexpr std::uint8_t DETECT_DECREMENT_4 = 0x3;

constexpr std::uint8_t DELAY_3MS = 0b11;
constexpr std::uint8_t DELAY_2MS = 0b10;
constexpr std::uint8_t DELAY_1MS = 0b01;
constexpr std::uint8_t NO_DELAY = 0b00;
//
//	Register 106 � User Control
//	USER_CTRL
//
constexpr std::uint8_t USERCTRL_FIFO_EN_BIT = 6;
constexpr std::uint8_t USERCTRL_I2C_MST_EN_BIT = 5;
constexpr std::uint8_t USERCTRL_I2C_IF_DIS_BIT = 4;
constexpr std::uint8_t USERCTRL_FIFO_RESET_BIT = 2;
constexpr std::uint8_t USERCTRL_I2C_MST_RESET_BIT = 1;
constexpr std::uint8_t USERCTRL_SIG_COND_RESET_BIT = 0;
//
//	Register 107 � Power Management 1
//	PWR_MGMT_1
//
constexpr std::uint8_t PWR1_DEVICE_RESET_BIT = 7;
constexpr std::uint8_t PWR1_SLEEP_BIT = 6;
constexpr std::uint8_t PWR1_CYCLE_BIT = 5;
constexpr std::uint8_t PWR1_TEMP_DIS_BIT = 3;
constexpr std::uint8_t PWR1_CLKSEL_BIT = 2;
constexpr std::uint8_t PWR1_CLKSEL_LENGTH = 3;

constexpr std::uint8_t CLOCK_INTERNAL = 0x00;
constexpr std::uint8_t CLOCK_PLL_XGYRO = 0x01;
constexpr std::uint8_t CLOCK_PLL_YGYRO = 0x02;
constexpr std::uint8_t CLOCK_PLL_ZGYRO = 0x03;
constexpr std::uint8_t CLOCK_PLL_EXT32K = 0x04;
constexpr std::uint8_t CLOCK_PLL_EXT19M = 0x05;
constexpr std::uint8_t CLOCK_KEEP_RESET = 0x07;
//
//	Register 108 � Power Management 2
//	PWR_MGMT_2
//
constexpr std::uint8_t PWR2_LP_WAKE_CTRL_BIT = 7;
constexpr std::uint8_t PWR2_LP_WAKE_CTRL_LENGTH = 2;
constexpr std::uint8_t PWR2_STBY_XA_BIT = 5;
constexpr std::uint8_t PWR2_STBY_YA_BIT = 4;
constexpr std::uint8_t PWR2_STBY_ZA_BIT = 3;
constexpr std::uint8_t PWR2_STBY_XG_BIT = 2;
constexpr std::uint8_t PWR2_STBY_YG_BIT = 1;
constexpr std::uint8_t PWR2_STBY_ZG_BIT = 0;

constexpr std::uint8_t WAKE_FREQ_1P25 = 0x0;
constexpr std::uint8_t WAKE_FREQ_5 = 0x1;
constexpr std::uint8_t WAKE_FREQ_20 = 0x2;
constexpr std::uint8_t WAKE_FREQ_40 = 0x3;
//
//	Register 117 � Who Am I
//	WHO_AM_I
//
constexpr std::uint8_t WHO_AM_I_BIT = 6;
constexpr std::uint8_t WHO_AM_I_LENGTH = 6;

#endif // MPU6050_CONFIG_HPP
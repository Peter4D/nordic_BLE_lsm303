/**
 * @file lm303_accel.h
 * @author Peter Medvesek (peter.medvesek@gorenje.com)
 * @brief 
 * @version 0.1
 * @date 2021-02-15
 * 
 * @copyright Copyright (c) 2021 Gorenje d.o.o
 * 
 */

#ifndef LM303_ACCEL_H
#define LM303_ACCEL_H

#include <stdint.h>
#include "nrfx_twi.h"
#include "app_error.h"

//#include "lsm303_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LSM303_ACCEL_ADDR    (0x32 >> 1)
//#define LSM303_MAG_ADDR      (0x3C >> 1)



typedef enum {                            
    LSM303_REG_ACCEL_WHO_AM_I = 0x0F,     // 00000111   rw
    LSM303_REG_ACCEL_CTRL_1 = 0x20,  // 00000111   rw
    LSM303_REG_ACCEL_CTRL_2 = 0x21,  // 00000000   rw
    LSM303_REG_ACCEL_CTRL_3 = 0x22,  // 00000000   rw
    LSM303_REG_ACCEL_CTRL_4 = 0x23,  // 00000000   rw
    LSM303_REG_ACCEL_CTRL_5 = 0x24,  // 00000000   rw
    LSM303_REG_ACCEL_CTRL_6 = 0x25,  // 00000000   rw
    LSM303_REG_ACCEL_REFERENCE = 0x26,  // 00000000   r
    LSM303_REG_ACCEL_STATUS_REG = 0x27, // 00000000   r
    LSM303_REG_ACCEL_OUT_X_L = 0x28,
    LSM303_REG_ACCEL_OUT_X_H = 0x29,
    LSM303_REG_ACCEL_OUT_Y_L = 0x2A,
    LSM303_REG_ACCEL_OUT_Y_H = 0x2B,
    LSM303_REG_ACCEL_OUT_Z_L = 0x2C,
    LSM303_REG_ACCEL_OUT_Z_H = 0x2D,
    LSM303_REG_ACCEL_FIFO_CTRL_REG = 0x2E,
    LSM303_REG_ACCEL_FIFO_SRC_REG = 0x2F,
    LSM303_REG_ACCEL_INT1_CFG = 0x30,
    LSM303_REG_ACCEL_INT1_SOURCE = 0x31,
    LSM303_REG_ACCEL_INT1_THS = 0x32,
    LSM303_REG_ACCEL_INT1_DURATION = 0x33,
    LSM303_REG_ACCEL_INT2_CFG = 0x34,
    LSM303_REG_ACCEL_INT2_SOURCE = 0x35,
    LSM303_REG_ACCEL_INT2_THS = 0x36,
    LSM303_REG_ACCEL_INT2_DURATION = 0x37,
    LSM303_REG_ACCEL_CLICK_CFG = 0x38,
    LSM303_REG_ACCEL_CLICK_SRC = 0x39,
    LSM303_REG_ACCEL_CLICK_THS = 0x3A,
    LSM303_REG_ACCEL_TIME_LIMIT = 0x3B,
    LSM303_REG_ACCEL_TIME_LATENCY = 0x3C,
    LSM303_REG_ACCEL_TIME_WINDOW = 0x3D
}lsm303_accel_reg_t;

typedef union _lsm303_accel_reg_int_src_t {
    struct {
        uint8_t X_L     :1; // X axis low event detected 
        uint8_t X_H     :1; // X axis high event detected 
        uint8_t Y_L     :1; // Y axis low event detected 
        uint8_t Y_H     :1; // Y axis high event detected 
        uint8_t Z_L     :1; // Z axis low event detected 
        uint8_t Z_H     :1; // Z axis high event detected 
        uint8_t I_A     :1; // Interrupt active 
        uint8_t         :1; //_reserved
    }bit;
    uint8_t reg;
}lsm303_accel_reg_int_src_t;

typedef union _lsm303_accel_reg_int_cfg_t {
    struct {
        uint8_t X_LIE     :1; // X low interrupt enable
        uint8_t X_HIE     :1; // x high interrupt enable
        uint8_t Y_LIE     :1; // y low interrupt enable
        uint8_t Y_HIE     :1; // y high interrupt enable
        uint8_t Z_LIE     :1; // z low interrupt enable
        uint8_t Z_HIE     :1; // z high interrupt enable
        uint8_t _6D       :1; // 6 direction detection enabled.
        uint8_t AOI       :1; // combination of events 
    }bit;
    uint8_t reg;
}lsm303_accel_reg_int_cfg_t;

#define ENABLE_ALL_AXIS_INT (0x3F)

typedef union _lsm303_accel_reg_ctrl_1_t {
    struct {
        uint8_t x_en     :1; // X axis enable
        uint8_t y_en     :1; // x axis enable
        uint8_t z_en     :1; // y axis enable
        uint8_t LP_en    :1; // Low power enable
        uint8_t ODR      :4; // outpput data rate
    }bit;
    uint8_t reg;
}lsm303_accel_reg_ctrl_1_t;

typedef union _lsm303_accel_reg_ctrl_2_t {
    struct {
        uint8_t HPIS_1     :1; //!< high-pass filter enable INT_1
        uint8_t HPIS_2     :1; //!< high-pass filter enable INT_2
        uint8_t HP_click   :1; //!< high-pass filter enable for click
        uint8_t FDS        :1; //!< Filtered data enable
        uint8_t HPCF       :2; //!< high-pass filter cutoff frequency
        uint8_t HPM        :2; //!< high-pass filter mode @ref HPM
    }bit;
    uint8_t reg;
}lsm303_accel_reg_ctrl_2_t;

/*
@section <HPM> 
HPM_1   | HPM_0 | comment
    0   |   0   | Normal mode (reset by reading the REFERENCE/DATACAPTURE_A (26h) register)
    0   |   1   | Reference signal for filtering
    1   |   0   | Normal mode
    1   |   1   | Autoreset on interrupt event
*/

typedef union _lsm303_accel_reg_ctrl_3_t {
    struct {
        uint8_t             :1; //!< _reserved
        uint8_t I1_overrun  :1; //!< FIFO overrun interrupt on INT1 pin. Default value 0. (0: disable; 1: enable)
        uint8_t I1_WTM      :1; //!< FIFO watermark interrupt on INT1 pin. Default value 0.(0: disable; 1: enable)
        uint8_t I1_DRDY_2   :1; //!< DRDY2 interrupt on INT1 pin. Default value 0.(0: disable; 1: enable)
        uint8_t I1_DRDY_1   :1; //!< DRDY1 interrupt on INT1 pin. Default value 0.(0: disable; 1: enable)
        uint8_t I1_AOI_2    :1; //!< AOI2 interrupt on INT1 pin. Default value 0.(0: disable; 1: enable)
        uint8_t I1_AOI_1    :1; //!< AOI1 interrupt on INT1 pin. Default value 0.(0: disable; 1: enable)
        uint8_t I1_click    :1; //!< CLICK interrupt on INT1 pin. Default value 0.(0: disable; 1: enable)
    }bit;
    uint8_t reg;
}lsm303_accel_reg_ctrl_3_t;

typedef union _lsm303_accel_reg_ctrl_4_t {
    struct {
        uint8_t SPI_EN  :1; //!< 3-wire SPI interface enable. Default: 0 (0: SPI 3-wire disabled; 1: SPI 3-wire enabled)
        uint8_t ST      :2; //!< Self-test enable. Default value: 00 (00: self-test disabled; other: see Table 43)
        uint8_t HR      :1; //!< Operating mode selection (refer to Section 4.2.1: Accelerometer power modes)
        uint8_t FS      :2; //!< Full-scale selection. Default value: 00(00: ±2g; 01: ±4g; 10: ±8g; 11: ±16g)
        uint8_t BLE     :1; //!< Big/Little Endian data selection. Default value: 0(0: data LSb at lower address; 1: data MSb at lower address) The BLE function can be activated only in high-resolution mode
        uint8_t BDU     :1; //!< Block data update. Default value: 0 (0: continuous update; 1: output registers not updated until MSB and LSB have been read)
    }bit;
    uint8_t reg;
}lsm303_accel_reg_ctrl_4_t;

typedef union _lsm303_accel_reg_ctrl_5_t {
    struct {
        uint8_t D4D_INT2    :1; //!< 4D enable: 4D detection is enabled on INT2 pin when 6D bit on INT2_CFG_A (34h) is set to 1.
        uint8_t LIR_INT2    :1; //!< Latch interrupt request on INT2_SRC_A (35h) register, with INT2_SRC_A (35h) register cleared by reading INT2_SRC_A (35h) itself. (0: interrupt request not latched; 1: interrupt request latched)
        uint8_t D4D_INT1    :1; //!< 4D enable: 4D detection is enabled on INT1 pin when 6D bit on INT1_CFG_A (30h) is set to 1.
        uint8_t LIR_INT1    :1; //!< Latch interrupt request on INT1_SRC_A (31h), with INT1_SRC_A (31h) register cleared by reading INT1_SRC_A (31h) itself. (0: interrupt request not latched; 1: interrupt request latched)
        uint8_t             :2; //!< _reserved
        uint8_t FIFO_EN     :1; //!< FIFO enable. Default value: 0(0: FIFO disabled; 1: FIFO enabled)
        uint8_t BOOT        :1; //!< Reboot accelerometer memory content. Default value: 0 (0: normal mode; 1: reboot memory content) 
    }bit;
    uint8_t reg;
}lsm303_accel_reg_ctrl_5_t;

typedef union _lsm303_accel_reg_ctrl_6_t {
    struct {
        uint8_t             :1; //!< _reserved
        uint8_t H_LACTIVE   :1; //!<  interrupt active. Default value: 0. (0: interrupt active-high; 1: interrupt active-low)
        uint8_t             :1; //!< _reserved
        uint8_t P2_ACT      :1; //!<  Activity interrupt enable on INT2 pin. Default value: 0. (0: disabled; 1:enabled)
        uint8_t BOOT_I_2    :1; //!<  Boot on INT2 pin enable. Default value: 0 (0: disabled; 1:enabled)
        uint8_t I2_INT_2    :1; //!<  Interrupt 2 function enable on INT2 pin. Default value: 0 (0: function disabled; 1: function enabled)
        uint8_t I2_INT_1    :1; //!<  Interrupt 1 function enable on INT2 pin. Default value: 0 (0: function disabled; 1: function enabled)
        uint8_t I2_CLICK_en :1; //!<  Click interrupt on INT2 pin. Default value: 0 (0: disabled; 1: enabled)
    }bit;
    uint8_t reg;
}lsm303_accel_reg_ctrl_6_t;

#define MMA7660_ADDR        (0x32 >> 1)
#define LSM303_ACCEL_ADDR    (0x32 >> 1)

/** @todo use type from lsm303_common.h file */
typedef union _lsm303AccelData_t {  
    struct {
        int16_t x; ///< x-axis data
        int16_t y; ///< y-axis data
        int16_t z; ///< z-axis data
    }axis;
    int16_t bytes[3];
}lsm303_accel_data_t;

typedef struct _LSM303_Accel_methods_t {
    lsm303_accel_data_t     (*raw_data_get)(void);
    uint8_t                 (*who_i_am_get)(void);
    void                    (*quick_setup)(void);
    void                    (*update)(void);
}LSM303_Accel_methods_t;

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

extern LSM303_Accel_methods_t LSM303_Accel;

//extern void read_accel_callback(ret_code_t result, void * p_user_data);


void twi_config(void);
void read_accel(void);

//void lms303_accel_vibration_trig_setup(void);





#ifdef __cplusplus
}
#endif

#endif /* LM303_ACCEL_H */

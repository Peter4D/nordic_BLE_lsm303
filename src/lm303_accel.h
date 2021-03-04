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

#define LSM303_ACCEL_INIT_TRANSFER_COUNT 1

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





#ifdef __cplusplus
}
#endif

#endif /* LM303_ACCEL_H */

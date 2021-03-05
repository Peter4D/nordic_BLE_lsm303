/**
 * @file lm303_mag.h
 * @author Peter Medvesek (peter.medvesek@gorenje.com)
 * @brief 
 * @version 0.1
 * @date 2021-02-15
 * 
 * @copyright Copyright (c) 2021 Gorenje d.o.o
 * 
 */
#ifndef LM303_MAG_H
#define LM303_MAG_H

#include <stdint.h>

//#include "lsm303_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    
    LSM303_REG_MAG_OUT_X_L = 0x45,
    LSM303_REG_MAG_OUT_X_H = 0x46,
    LSM303_REG_MAG_OUT_Y_L = 0x49,
    LSM303_REG_MAG_OUT_Y_H = 0x4A,
    LSM303_REG_MAG_OUT_Z_L = 0x47,
    LSM303_REG_MAG_OUT_Z_H = 0x48,

    LSM303_REG_MAG_WHO_AM_I = 0x4F,

    LSM303_REG_MAG_CFG_A = 0x60,
    LSM303_REG_MAG_CFG_B = 0x61,
    LSM303_REG_MAG_CFG_C = 0x62,

    LSM303_REG_INT_CTRL  = 0x63,
    LSM303_REG_INT_SRC  = 0x64,
    LSM303_REG_INT_THS_L  = 0x65,
    LSM303_REG_INT_THS_H  = 0x66,

    LSM303_REG_STATUS  = 0x67,

    LSM303_REG_OUT_X_L  = 0x68,
    LSM303_REG_OUT_X_H  = 0x69,
    LSM303_REG_OUT_Y_L  = 0x6A,
    LSM303_REG_OUT_Y_H  = 0x6B,
    LSM303_REG_OUT_Z_L  = 0x6C,
    LSM303_REG_OUT_Z_H  = 0x6D,

}lsm303_mag_reg_t;

#define LSM303_MAG_ADDR    (0X3C >> 1)


#ifdef __cplusplus
}
#endif

#endif /* LM303_MAG_H */

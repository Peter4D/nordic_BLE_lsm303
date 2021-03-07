/**
 * @file lsm303_drv.h
 * @author Peter Medvesek (peter.medvesek@gorenje.com)
 * @brief 
 * @version 0.1
 * @date 2021-03-04
 * 
 * @copyright Copyright (c) 2021 Gorenje d.o.o
 * 
 */
#ifndef LSM303_DRV_H
#define LSM303_DRV_H

#include <stdint.h>

#include <stdbool.h>
#include "boards.h"
#include "bsp.h"
#include "nrf_twi_mngr.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#ifdef __cplusplus
extern "C" {
#endif

typedef union _axis_data_t {
    struct {
        int16_t x;
        int16_t y;
        int16_t z;
    }axis;
    uint8_t bytes[6];
}axis_data_t;

void lsm303_accel_setup(void);
void lsm303_mag_setup(void);
void lms303_accel_vibration_trig_setup(void);

void twi_config(void);

void read_accel(void);

void read_mag(void);

#ifdef __cplusplus
}
#endif

#endif /* LSM303_DRV_H */

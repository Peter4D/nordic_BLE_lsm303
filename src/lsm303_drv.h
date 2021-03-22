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

typedef struct _axis_peak_detect_t {
    uint32_t time;
    int16_t value;
    int16_t angle;
    uint8_t neg_val_F;
    uint8_t peak_detected_F;
    const char* const p_name;
}axis_peak_detect_t;

#define AXIS_PEAK_DETECT_INIT(name)     \
    {                                   \
        .time = 0,                      \
        .value = 0,                     \
        .angle = 0,                     \
        .neg_val_F = 0,                 \
        .peak_detected_F = 0,           \
        .p_name = name                  \
    }

typedef struct _lsm303_data_t {
    axis_data_t accel;
    float accel_rad;
    uint16_t accel_rad_int;
    int16_t accel_angle;
    
    axis_data_t mag;
    axis_peak_detect_t peak_mag_x;
    axis_peak_detect_t peak_mag_z;
    uint8_t mag_dir;
}lsm303_data_t;


typedef struct _accel_t
{
    axis_data_t axis;
    float accel_rad;
    uint16_t accel_rad_int;
    uint16_t accel_angle;
}accel_t;

typedef struct _mag_t
{
    axis_data_t mag;
}mag_t;

typedef struct _lsm303_data_2_t {
    accel_t accel;
    mag_t   mag;
}lsm303_data_2_t;

void lsm303_accel_setup(void);
void lsm303_mag_setup(void);
void lms303_accel_vibration_trig_setup(void);

void twi_config(void);

void read_accel(void);

void read_mag(void);

lsm303_data_t* lsm303_data_p_get(void);

#ifdef __cplusplus
}
#endif

#endif /* LSM303_DRV_H */

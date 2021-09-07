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

#include "lm303_mag_reg.h"
#include "lm303_accel_reg.h"



#ifdef __cplusplus
extern "C" {
#endif


typedef enum {
    QD_A,
    QD_B,
}qd_sig_e_t;
typedef struct _lsm303_reg_dsc_t {
    uint8_t addr;
    uint8_t data;
    char* p_name;
}lsm303_reg_dsc_t;


typedef union _lsm303_reg_data_t {
    struct _reg{
        lsm303_reg_dsc_t who_i_am;
        lsm303_reg_dsc_t int1_src;

        lsm303_reg_dsc_t ctrl_1;
        lsm303_reg_dsc_t ctrl_2;
        lsm303_reg_dsc_t ctrl_3;
        lsm303_reg_dsc_t ctrl_4;
        lsm303_reg_dsc_t ctrl_5;
        lsm303_reg_dsc_t ctrl_6;

        lsm303_reg_dsc_t accel_int1_ths;
    }reg;
    lsm303_reg_dsc_t reg_array[9];
}lsm303_reg_data_t;


typedef union _axis_data_t {
    struct {
        int16_t x;
        int16_t y;
        int16_t z;
    }bit;
    int16_t bit_sum;
    uint8_t bytes[6];
}axis_data_t;

typedef union _axis_qd_ab_sig_t {
    struct {
        uint8_t b       :1;
        uint8_t a       :1;
        uint8_t b_old   :1;
        uint8_t a_old   :1;
        uint8_t         :4; //_reseved
    }bit;
    uint8_t byte;
}axis_qd_ab_sig_t;

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

typedef struct _qd_th_t {
    int32_t th;
    int32_t hysteresis;
}qd_th_t;
typedef struct _qd_desc_t {
    qd_th_t th_values[2];
    axis_qd_ab_sig_t qd;
    int32_t qd_cnt;
    int8_t qd_dir;
}qd_desc_t;

typedef struct _accel2_t
{
    qd_desc_t qd_data;

    axis_data_t axis;
    float rad;
    uint16_t rad_int;
    uint16_t angle;
    uint8_t area;
uint8_t area2;
}accel2_t;

typedef struct _mag2_t
{
    qd_desc_t qd_data;

    axis_data_t axis;
    axis_data_t axis_peak;
    uint16_t sum_abs_xyz;

}mag2_t;

typedef struct _lsm303_data_3_t {
    accel2_t accel;
    mag2_t   mag;
}lsm303_data_3_t;


// typedef struct _lsm303_data_t {
//     axis_data_t accel;
//     float accel_rad;
//     uint16_t accel_rad_int;
//     int16_t accel_angle;
    
//     axis_data_t mag;
//     axis_peak_detect_t peak_mag_x;
//     axis_peak_detect_t peak_mag_z;
//     uint8_t mag_dir;
// }lsm303_data_t;


typedef struct _accel_t
{
    axis_data_t axis;
    float rad;
    uint16_t rad_int;
    uint16_t angle;

    qd_th_t th_values;
    axis_qd_ab_sig_t qd;
    int32_t qd_cnt;
    int8_t qd_dir;
}accel_t;



// typedef struct _mag_t
// {
//     axis_data_t axis;
//     axis_data_t axis_peak;

//     qd_th_t th_values;
//     axis_qd_ab_sig_t qd;
//     int32_t qd_cnt;
//     int8_t qd_dir;
// }mag_t;

// typedef struct _lsm303_data_2_t {
//     accel_t accel;
//     mag_t   mag;

//     /* those are deprecated*/
//     axis_peak_detect_t peak_mag_x;
//     axis_peak_detect_t peak_mag_z;
//     uint8_t mag_dir;
// }lsm303_data_t;

typedef lsm303_data_3_t lsm303_data_t; 
typedef mag2_t mag_t; 


extern lsm303_reg_data_t lsm_reg_data;

void lsm303_accel_setup(void);
void lsm303_mag_setup(void);
void lms303_accel_vibration_trig_setup(void);

void lms303_accel_int_en(lsm303_accel_reg_int_cfg_t int_cfg);
void lms303_accel_int_disable(void);

void lsm303_setup_read_back_check(void);

void twi_config(void);

void read_accel(void);

void read_mag(void);

lsm303_data_t* lsm303_data_p_get(void);

void lsm303_read_reg(uint8_t* const p_reg_addr, uint8_t* p_data, size_t size,
                    void (*read_end_cb)(ret_code_t result, void * p_user_data)
                    );

#ifdef __cplusplus
}
#endif

#endif /* LSM303_DRV_H */

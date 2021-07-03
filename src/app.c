#include <math.h>
#include "app.h"
#include "lsm303_drv.h"


#define MAG_LOW_LEVEL_TH    3000
#define MAG_LOW_LEVEL_HYST  500

#define ANGLE_NOT_SET_VAL   0xFFFF

#define AXIS_CALIB_INIT()   \
    {   \
        .state              = CALIB_NONE,           \
        .side_aka_sign_F    = 0,                    \
        .value              = 0,                    \
        .angle_start        = ANGLE_NOT_SET_VAL,    \
        .angle              = ANGLE_NOT_SET_VAL,    \
        .angle_end          = ANGLE_NOT_SET_VAL,    \
        .sample_tm          = 0,                    \
        .curr_sample_tm     = 0                     \
    }

static calib_axis_t axis_calib_x = AXIS_CALIB_INIT();
static calib_axis_t axis_calib_y = AXIS_CALIB_INIT();
static calib_axis_t axis_calib_z = AXIS_CALIB_INIT();

void axis_calib_start(calib_axis_t* p_calib_axis) {
    p_calib_axis->state = CALIB_NONE;
}

void axis_calib_handle(calib_axis_t* p_calib_axis, uint32_t axis_data, uint16_t angle) {
    
    if(p_calib_axis->state == CALIB_NONE) {
        ++p_calib_axis->curr_sample_tm;

        switch (p_calib_axis->state) 
        {
            case CALIB_NONE:
            {

                p_calib_axis->value = 0;
                p_calib_axis->angle_start = ANGLE_NOT_SET_VAL;
                p_calib_axis->angle = 0;
                p_calib_axis->sample_tm = 0;
                p_calib_axis->curr_sample_tm = 0;
                p_calib_axis->side_aka_sign_F = 0;

                p_calib_axis->state = CALIB_LOW_LEVEL;
                break;
            }
            case CALIB_LOW_LEVEL:
            {
                if(abs(axis_data) < MAG_LOW_LEVEL_TH) {
                    p_calib_axis->state = CALIB_PEAK_DETECT;
                }
                break;
            }
            case CALIB_PEAK_DETECT:
            {   
                int32_t axis_data_abs = abs(axis_data);

                if(axis_data_abs > MAG_LOW_LEVEL_TH) {
                    
                    /* init angle value */
                    if(ANGLE_NOT_SET_VAL == p_calib_axis->angle_start) {
                        p_calib_axis->angle_start = angle;
                    }

                    if(axis_data_abs > p_calib_axis->value) {
                        
                        p_calib_axis->value = axis_data_abs;
                        if(axis_data < 0)   { p_calib_axis->side_aka_sign_F = 1; }
                        else                { p_calib_axis->side_aka_sign_F = 0; }

                        p_calib_axis->angle = angle;
                        p_calib_axis->sample_tm = p_calib_axis->curr_sample_tm;
                    }
                }else if(axis_data_abs < MAG_LOW_LEVEL_TH - MAG_LOW_LEVEL_HYST) {
                    p_calib_axis->angle_end = angle;
                    p_calib_axis->state = CALIB_OK;
                }
                break;
            }
            case CALIB_OK:
            {
                /* Do nothing ... peak detected. */
            }
        }
    }
}



void axis_calib_handle(calib_axis_t* p_calib_axis, lsm303_data_t* p_lsm303_data) {
    
    axis_data_t* p_mag_axis = &p_lsm303_data->mag.axis;

    p_lsm303_data->accel.angle;
    p_lsm303_data->mag.axis.bit.x;
    p_lsm303_data->mag.axis.bit.z;

    if(abs(p_mag_axis->bit.z) > MAG_LOW_LEVEL_TH) {
        
    }
}



void task_calib_handle(void) {

    lsm303_data_t* p_lsm303_data = lsm303_data_p_get();

    axis_calib_handle(&axis_calib_x, p_lsm303_data->mag.axis.bit.x, p_lsm303_data->accel.angle);
}
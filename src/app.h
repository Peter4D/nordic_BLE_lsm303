/**
 * @file app.h
 * @author Peter Medvesek (peter.medvesek@gorenje.com)
 * @brief 
 * @version 0.1
 * @date 2021-06-19
 * 
 * @copyright Copyright (c) 2021 Gorenje d.o.o
 * 
 */
#ifndef APP_H
#define APP_H

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

/*
We should be paying attention to 4 key angle of possible key insertion:
- 0   deg
- 90  deg
- 180 deg
- 270 deg

At those key angles then check:
- 0   deg: value of magX axis: magX > 0 then side_1  
- 90  deg: value of magZ axis: magZ > 0 then side_1  
- 180 deg: value of magX axis: magX < 0 then side_1
- 270 deg: value of magZ axis: magZ < 0 then side_1

Threshold should be set to 1200 (2G, 16bit result settings ).

Key insertion is detected by accel interrupt (interrupt is set to y axis).
After interrupt triggers then if abs(sum(magX, magY, magZ)) > KEY_INSERT_TH then
key is confirmed inserted.
After key insertion is confirmed then angle is read and check side of door where key is.

at calibration procedure we must save value of magZ axis.

if > 0 this is one
side of < 0 the key is inserted on other side of the door.
*/

typedef enum {
    CALIB_NONE = 0,
    CALIB_LOW_LEVEL,
    CALIB_PEAK_DETECT,
    CALIB_OK
}calib_axis_state_e_t;

typedef struct _calib_axis {
    calib_axis_state_e_t state;
    uint8_t  side_aka_sign_F;
    int32_t  value;
    uint16_t angle_start;
    uint16_t angle;
    uint16_t angle_end;
    uint32_t sample_tm;
    uint32_t curr_sample_tm;
}calib_axis_t;

void axis_calib_handle(calib_axis_t* p_calib_axis, uint32_t axis_data, uint16_t angle);

#endif /* APP_H */

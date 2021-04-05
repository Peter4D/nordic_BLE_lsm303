/**
 * Copyright (c) 2015 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 * @defgroup tw_sensor_example main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Sensor Example main file.
 *
 * This file contains the source code for a sample application using TWI.
 *
 */

#include <stdio.h>
#include <stdbool.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"

//#include "nrf_drv_twi.h"
#include "nrfx_twi.h"
#include "nrf_twi_mngr.h"

#include "nrf_delay.h"

//#include "nrf_drv_gpiote.h"
#include "nrfx_gpiote.h"
#include "bsp.h"
#include "nrf_drv_gpiote.h"



#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "app_timer.h"

#include "nrf_drv_clock.h"
#include "nrfx_clock.h"
#include "nrf_pwr_mgmt.h"

// #include "lm303_accel.h"
// #include "lm303_mag.h"
#include "lsm303_drv.h"

#include <math.h>

#ifndef DEBUG_APP_SHOW_QD
#define DEBUG_APP_SHOW_QD       1
#endif

#ifndef DEBUG_APP_SHOW_AXIS
#define DEBUG_APP_SHOW_AXIS     0
#endif


/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

#define BTN_ID_USER         0

/* Common addresses definition for temperature sensor. */
//#define LSM303_ACCEL_ADDR          (0x90U >> 1)
#define LSM303_ACCEL_ADDR    (0x32 >> 1)
#define LSM303_MAG_ADDR      (0x3C >> 1)

#define LM75B_REG_TEMP      0x00U
#define LM75B_REG_CONF      0x01U
#define LM75B_REG_THYST     0x02U
#define LM75B_REG_TOS       0x03U

/* Mode for LM75B. */
#define NORMAL_MODE 0U

static uint8_t m_who_i_am = 0xFF;

typedef struct  _app_CB_t
{
    int8_t key_side;
}app_CB_t;

static app_CB_t m_app_CB;

/* Buffer for samples read from accelerometer sensor. */

APP_TIMER_DEF(read_lsm303_tmr_id);
APP_TIMER_DEF(app_tmr_btn_long_press_id);
APP_TIMER_DEF(app_tmr_print_out_id);
APP_TIMER_DEF(app_tmr_calib_id);

void read_lsm303_tmr_handler(void* p_context);

#define APP_LED_RED BSP_LED_0

#ifdef BSP_BUTTON_0
    #define PIN_IN BSP_BUTTON_0
#endif
#ifndef PIN_IN
    #error "Please indicate input pin"
#endif

#ifdef APP_LED_RED
    #define PIN_OUT APP_LED_RED
#endif

#ifndef PIN_OUT
    #error "Please indicate output pin"
#endif

#ifndef PIN_OUT
    #error "Please indicate output pin"
#endif

#ifndef APP_LED_RED
#define APP_LED_RED BSP_BOARD_LED_0
#endif


void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    nrfx_gpiote_out_toggle(PIN_OUT);
}
/**
 * @brief Function for configuring: PIN_IN pin for input, PIN_OUT pin for output,
 * and configures GPIOTE to give an interrupt on pin change.
 */
static void gpio_init(void)
{
    ret_code_t err_code;

    err_code = nrfx_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrfx_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

    err_code = nrfx_gpiote_out_init(PIN_OUT, &out_config);
    APP_ERROR_CHECK(err_code);

    //nrfx_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);

    //nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_RAW_CONFIG_IN_SENSE_HITOLO(true);
    nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_RAW_CONFIG_IN_SENSE_HITOLO(false);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrfx_gpiote_in_init(PIN_IN, &in_config, in_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrfx_gpiote_in_event_enable(PIN_IN, true);
}


static void app_tmr_btn_long_press_handler(void* p_context) {

    if(bsp_button_is_pressed(0) == 1) {
        lsm303_data_2_t* p_lsm303_data = lsm303_data_p_get();

        NRF_LOG_INFO("btn long press\r\n");

        p_lsm303_data->mag.qd_cnt = 0;

        nrfx_gpiote_out_toggle(PIN_OUT);
    }
}

static void app_tmr_print_out_handler(void* p_context) {
    
    lsm303_data_2_t* p_lsm303_data = lsm303_data_p_get();
    //static uint32_t last_tick = 0;
    // uint32_t ticks = app_timer_cnt_get();
    // uint32_t delta = app_timer_cnt_diff_compute(ticks, last_tick);
    // last_tick = ticks;

    /* [angle],[mX],[my],[mZ]*/
    //NRF_LOG_INFO("angle/x/z | %3d,%4d,%4d\r",
    //NRF_LOG_RAW_INFO("angle/x/z/a/b/dir/cnt | %3d,%5d,%5d,%u,%u,%d,%d\r",

    #if( DEBUG_APP_SHOW_QD == 1)
    
    NRF_LOG_INFO("angle/a/b/dir/cnt/Y_peak | %3d,%u,%u,%d,%d,%d\r",
    p_lsm303_data->accel.angle, 
    // p_lsm303_data->mag.axis.bit.x,
    // p_lsm303_data->mag.axis.bit.z,
    p_lsm303_data->mag.qd.bit.a,
    p_lsm303_data->mag.qd.bit.b,
    p_lsm303_data->mag.qd_dir,
    p_lsm303_data->mag.qd_cnt,
    p_lsm303_data->mag.axis_peak.bit.y
    );

    #elif ( DEBUG_APP_SHOW_AXIS == 1)

    NRF_LOG_INFO("angle/x/z/dir/cnt/y | %3d,%d,%d,%d,%d,%d\r",
    p_lsm303_data->accel.angle, 
    p_lsm303_data->mag.axis.bit.x,
    p_lsm303_data->mag.axis.bit.z,
    // p_lsm303_data->mag.qd.bit.a,
    // p_lsm303_data->mag.qd.bit.b,
    p_lsm303_data->mag.qd_dir,
    p_lsm303_data->mag.qd_cnt,
    //p_lsm303_data->mag.axis_peak.bit.y
    p_lsm303_data->mag.axis.bit.y
    );

    #endif
}

static void app_tmr_calib_handler(void* p_context) {
    lsm303_data_2_t* p_lsm303_data = lsm303_data_p_get();
    int32_t qd_cnt = p_lsm303_data->mag.qd_cnt;
    

    if(abs(qd_cnt) > 4) {
        /* key has turned more than full circle */
        if(p_lsm303_data->mag.axis_peak.bit.y > 0) {
            m_app_CB.key_side = 0;
        }else {
            m_app_CB.key_side = -1;
        }
    }
}

void bsp_evt_handler(bsp_event_t bsp_event) {

    switch(bsp_event) 
    {
        case BSP_EVENT_KEY_0:
        {
            lsm303_data_2_t* p_lsm303_data = lsm303_data_p_get();
        
            NRF_LOG_INFO("btn short press\r\n");
            p_lsm303_data->mag.qd_cnt = 0;


            APP_ERROR_CHECK(app_timer_start(app_tmr_btn_long_press_id, APP_TIMER_TICKS(3000), NULL));

            break;
        }
    }
}

static void utils_setup(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);


    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_evt_handler);
    //APP_ERROR_CHECK(err_code);

    err_code = bsp_event_to_button_action_assign(BTN_ID_USER,
                                                 BSP_BUTTON_ACTION_PUSH,
                                                 BSP_EVENT_KEY_0);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


void read_lsm303_tmr_handler(void* p_context) {
    //static uint32_t heart_beat = 0;
    
    // Signal on LED that something is going on.
    bsp_board_led_invert(BSP_BOARD_LED_1);
    
    read_accel();
    read_mag();

    NRF_LOG_FLUSH();
}


void clock_event_handler(nrfx_clock_evt_type_t event)
{
}

/**@brief Function starting the internal LFCLK oscillator.
 *
 * @details This is needed by RTC1 which is used by the Application Timer
 *          (When SoftDevice is enabled the LFCLK is always running and this is not needed).
 */
static void lfclk_request(void)
{
    ret_code_t err_code;
    err_code = nrfx_clock_init(clock_event_handler);
    //err_code = nrf_drv_clock_init();
    
    APP_ERROR_CHECK(err_code);

    //nrf_drv_clock_lfclk_request(NULL);
    nrfx_clock_lfclk_start();
}


/**
 * @brief Function for main application entry.
 */
int main(void)
{
    ret_code_t err_code;

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("\r\nTWI sensor example started nRF52805. [%s] [%s]", __DATE__, __TIME__);
    //NRF_LOG_FLUSH();
    

    /* Configure board. */
    //bsp_board_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS);

    //twi_init();
    //lsm303_setup();
    utils_setup();
    
    lfclk_request();
    //APP_ERROR_CHECK(app_timer_init());
    //gpio_init();


    // err_code = nrf_pwr_mgmt_init();
    // APP_ERROR_CHECK(err_code);

    //twi_config();
    
    app_timer_create(&read_lsm303_tmr_id,
                        APP_TIMER_MODE_REPEATED,
                        read_lsm303_tmr_handler);

    app_timer_create(&app_tmr_btn_long_press_id,
                        //APP_TIMER_MODE_REPEATED,
                        APP_TIMER_MODE_SINGLE_SHOT,
                        app_tmr_btn_long_press_handler);
    
    app_timer_create(&app_tmr_print_out_id,
                        APP_TIMER_MODE_REPEATED,
                        app_tmr_print_out_handler);

    app_timer_create(&app_tmr_calib_id,
                        APP_TIMER_MODE_REPEATED,
                        app_tmr_calib_handler);
    
    APP_ERROR_CHECK(app_timer_start(read_lsm303_tmr_id, APP_TIMER_TICKS(10), NULL));
    APP_ERROR_CHECK(app_timer_start(app_tmr_print_out_id, APP_TIMER_TICKS(100), NULL));

    twi_config();
    
    lms303_accel_vibration_trig_setup();
    //lsm303_accel_setup();
    lsm303_mag_setup();

    lsm303_setup_read_back_check();

    while (true)
    {
        //__WFE();
        NRF_LOG_FLUSH();
        nrf_pwr_mgmt_run();
    }
}

/** @} */

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
#include "app_gpiote.h"

#include "nrf_drv_clock.h"
#include "nrfx_clock.h"
#include "nrf_pwr_mgmt.h"

// #include "lm303_accel.h"
// #include "lm303_mag.h"
#include "lsm303_drv.h"
#include "flash_storage.h"
#include <math.h>


#define goffsetof(TYPE, MEMBER) ((size_t) &((TYPE *)0)->MEMBER)
 // Obtain a pointer to the entire structure variable according to the "pointer (ptr) of the domain member variable (member)" in the "structure (type) variable"
#define gcontainer_of(ptr, type, member)     (type *)( (char *)ptr - goffsetof(type,member) );

/* angle/a/b/dir/cnt/Y_peak  */
#ifndef DEBUG_APP_SHOW_QD
#define DEBUG_APP_SHOW_QD 0
#endif


#ifndef DEBUG_APP_SHOW_QD_ACCEL
#define DEBUG_APP_SHOW_QD_ACCEL 0
#endif

#ifndef DEBUG_APP_SHOW_QD_AXIS_ACCEL
#define DEBUG_APP_SHOW_QD_AXIS_ACCEL 0
#endif

/* show angle/x/z/dir/cnt/y */
#ifndef DEBUG_APP_SHOW_AXIS_MAG
#define DEBUG_APP_SHOW_AXIS_MAG 1
#endif

/* show angle/x/z/a/b */
#ifndef DEBUG_APP_SHOW_MAG_AB
#define DEBUG_APP_SHOW_MAG_AB 0
#endif

/* show angle/x/y/z */
#ifndef DEBUG_APP_SHOW_AXIS_MAG_2
#define DEBUG_APP_SHOW_AXIS_MAG_2 0
#endif

#ifndef DEBUG_APP_SHOW_ACCEL_AXIS
#define DEBUG_APP_SHOW_ACCEL_AXIS 0
#endif


/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

/* UI */
#define BTN_PIN                   5
#define BTN_POLL_MS               100
#define LONG_BTN_TIMEOUT_MS       5000
#define LED_SHORT_BLINK_TIME_MS   100
#define LED_LONG_BLINK_TIME_MS    1000
#define LED_1                     4
#define LED_2                     20
#define LED_GREEN                 LED_1
#define LED_RED                   LED_2

/* Arbitary calibration constants */
#define CALIBRATION_TIMEOUT_MS    10000 /* Timeout after enabling calibration and no action was taken */

typedef enum{
  GREEN_OFF = 1 << 0,
  RED_OFF   = 1 << 1,
  BOTH_OFF  = GREEN_OFF | RED_OFF,
} led_timeout_handle_t;

#define led_off nrf_gpio_pin_clear
#define led_on nrf_gpio_pin_set
#define led_toggle nrf_gpio_pin_toggle

#define ACCEL_INT_PIN       14

#define LM75B_REG_TEMP      0x00U
#define LM75B_REG_CONF      0x01U
#define LM75B_REG_THYST     0x02U
#define LM75B_REG_TOS       0x03U

/* Mode for LM75B. */
#define NORMAL_MODE 0U

static uint8_t m_who_i_am = 0xFF;

typedef enum {
    STATE_IDLE = 0,
    STATE_CALIBRATING, 
    STATE_ACTIVE
}app_state_e_t;

typedef struct  _app_CB_t
{
    app_state_e_t app_state;
    int8_t key_side;
}app_CB_t;

typedef struct{
  uint8_t calibrated  :1;
  uint8_t calibrating :1;
  uint8_t locked      :1;
} application_state_t;

static app_CB_t m_app_CB = {
    .app_state = STATE_IDLE,
};

static application_state_t app_state;


/* Buffer for samples read from accelerometer sensor. */

APP_TIMER_DEF(read_lsm303_tmr_id);
APP_TIMER_DEF(app_tmr_btn_long_press_id);
APP_TIMER_DEF(app_tmr_print_out_id);
APP_TIMER_DEF(app_tmr_led_blink_id);
APP_TIMER_DEF(app_tmr_calibration_id);

void read_lsm303_tmr_handler(void* p_context);

#define APP_LED_RED BSP_LED_0
#define APP_GREEN_RED BSP_LED_1

#ifdef APP_LED_RED
    #define PIN_OUT APP_LED_RED
#endif

#ifndef PIN_OUT
    #error "Please indicate output pin"
#endif


#ifndef APP_LED_RED
#define APP_LED_RED BSP_BOARD_LED_0
#endif

/* test of lsm303 sensor response */
static void test_i2c_read_callback(ret_code_t result, void * p_user_data) { 
    NRF_LOG_INFO("who i am read %u", ((uint8_t*)p_user_data)[0]);
}

static void app_tmr_print_out_handler(void* p_context) {
    
    lsm303_data_t* p_lsm303_data = lsm303_data_p_get();
  
    #if( DEBUG_APP_SHOW_QD == 1)
    
    NRF_LOG_INFO("angle/a/b/dir/cnt/Y_peak | %3d,%u,%u,%d,%d,%d",
    p_lsm303_data->accel.angle, 

    p_lsm303_data->mag.qd_data.qd.bit.a,
    p_lsm303_data->mag.qd_data.qd.bit.b,
    p_lsm303_data->mag.qd_data.qd_dir,
    p_lsm303_data->mag.qd_data.qd_cnt,

    p_lsm303_data->mag.axis_peak.bit.y
    );
    
    #elif ( DEBUG_APP_SHOW_QD_ACCEL == 1 )

    NRF_LOG_INFO("angle/a/b/dir/cnt | %3d,%u,%u,%2d,%d",
    p_lsm303_data->accel.angle, 

    p_lsm303_data->accel.qd_data.qd.bit.a,
    p_lsm303_data->accel.qd_data.qd.bit.b,
    p_lsm303_data->accel.qd_data.qd_dir,
    p_lsm303_data->accel.qd_data.qd_cnt
    );

    #elif ( DEBUG_APP_SHOW_QD_AXIS_ACCEL == 1 )

    NRF_LOG_INFO("angle/a/b/x/z | %3d,%u,%u,%5d,%5d",
    p_lsm303_data->accel.angle, 

    p_lsm303_data->accel.qd_data.qd.bit.a,
    p_lsm303_data->accel.qd_data.qd.bit.b,
    abs(p_lsm303_data->accel.axis.bit.x),
    abs(p_lsm303_data->accel.axis.bit.z)
    );

    #elif ( DEBUG_APP_SHOW_AXIS_MAG == 1)

    NRF_LOG_INFO("angle/x/y/z/dir/cnt | %3d,%5d,%5d,%5d,%2d,%d",
    p_lsm303_data->accel.angle, 
    p_lsm303_data->mag.axis.bit.x,
    p_lsm303_data->mag.axis.bit.y,
    p_lsm303_data->mag.axis.bit.z,

    p_lsm303_data->mag.qd_data.qd_dir,
    p_lsm303_data->mag.qd_data.qd_cnt
    );

    #elif ( DEBUG_APP_SHOW_MAG_AB == 1)

    NRF_LOG_INFO("angle/x/z/a/b | %3d,%5d,%5d,%d,%d",
    p_lsm303_data->accel.angle, 
    p_lsm303_data->mag.axis.bit.x,
    p_lsm303_data->mag.axis.bit.z,

    p_lsm303_data->mag.qd_data.qd.bit.a,
    p_lsm303_data->mag.qd_data.qd.bit.b
    );
    
    #elif ( DEBUG_APP_SHOW_AXIS_MAG_2 == 1)

    NRF_LOG_INFO("angle/x/y/z | %3d,%5d,%5d,%5d",
    p_lsm303_data->accel.angle, 
    p_lsm303_data->mag.axis.bit.x,
    p_lsm303_data->mag.axis.bit.y,
    p_lsm303_data->mag.axis.bit.z
    );

    #elif ( DEBUG_APP_SHOW_ACCEL_AXIS == 1)

    NRF_LOG_INFO("A: angle/x/y/z | %3d, %5d, %5d, %5d",
    p_lsm303_data->accel.angle, 

    abs(p_lsm303_data->accel.axis.bit.x), 
    abs(p_lsm303_data->accel.axis.bit.y), 
    abs(p_lsm303_data->accel.axis.bit.z)
    );

    #endif

    // static uint8_t who_i_am_reg_addr = LSM303_REG_ACCEL_WHO_AM_I;
    // lsm303_read_reg(&who_i_am_reg_addr, &m_who_i_am, 1, test_i2c_read_callback);
}

static bool calib_handler(void* p_context) {
    lsm303_data_t* p_lsm303_data = lsm303_data_p_get();
    int32_t qd_cnt = p_lsm303_data->mag.qd_data.qd_cnt;
    

    if(abs(qd_cnt) > 4) {
        /* key has turned more than full circle */
        if(p_lsm303_data->mag.axis_peak.bit.y > 0) {
            m_app_CB.key_side = 0;
        }else {
            m_app_CB.key_side = -1;
        }
        return true;
    }
    return false;
}

static void lsm303_read_end_callback(ret_code_t result, void * p_user_data) {

    struct _lsm303_reg_dsc_t* p_my_container = {0};

    p_my_container = gcontainer_of(p_user_data, struct _lsm303_reg_dsc_t, data);
    
    NRF_LOG_INFO("reg: %s, val: 0x%X",  p_my_container->p_name, ((uint8_t*)p_user_data)[0]);

}


static uint8_t reg_data[1];
static uint8_t addr_reg = LSM303_REG_ACCEL_INT1_SOURCE;

static void calibration_timeout_handler(void* calibration_ctx_s)
{
    app_state.calibrating = false;
    NRF_LOG_INFO("Calibration timeout");
}

static void led_timeout_handler(void *led_state_s)
{
    led_timeout_handle_t led_state = (led_timeout_handle_t)led_state_s;
    if(led_state & GREEN_OFF){
      led_off(LED_GREEN);
    }
    if(led_state & RED_OFF){
      led_off(LED_RED);
    } 
}

static void button_short_press(void)
{
    NRF_LOG_INFO("Short press");
    // read_accel(); /* @note When SD will be running, this might not work as currently these are synchronous read requests */
    // read_mag();

    // Per spec, if key is in "reset" state then both leds blink
    uint32_t off_action;
    if(app_state.calibrated){
      led_on(LED_GREEN);
      led_on(LED_RED);
      off_action = BOTH_OFF;
    }
    else if(app_state.locked){
      led_on(LED_GREEN);
      off_action = GREEN_OFF;
      // Signal if door is locked or not
    }
    else{
      led_on(LED_RED);
      off_action = RED_OFF;
    }
    app_timer_start(app_tmr_led_blink_id,APP_TIMER_TICKS(LED_SHORT_BLINK_TIME_MS),(void*)off_action);
}

static void calibration_start(void)
{
    // lsm303_accel_reg_int_cfg_t cfg = {.reg = ENABLE_ALL_AXIS_INT};
    // lms303_accel_int_en(cfg);

    NRF_LOG_INFO("Calibration start");
    // read_accel(); /* @note When SD will be running, this might not work as currently these are synchronous read requests */
    // read_mag();
    
    app_state.calibrating = true;
    ret_code_t ret = app_timer_start(app_tmr_calibration_id,APP_TIMER_TICKS(CALIBRATION_TIMEOUT_MS),NULL);
    APP_ERROR_CHECK(ret);
}

static void button_long_press(void)
{
    if(!app_state.calibrating)
    {
      calibration_start();
      led_on(LED_GREEN);
      app_timer_start(app_tmr_led_blink_id,APP_TIMER_TICKS(LED_SHORT_BLINK_TIME_MS),(void*)GREEN_OFF);
    }
    else{
      NRF_LOG_INFO("Already calibrating");
    }
}

static void button_timeout_handler(void* unused)
{
  (void)unused;
  static uint8_t btn_hold_cnt = 0;
  if(app_button_is_pushed(0)) {
      if(++btn_hold_cnt >= LONG_BTN_TIMEOUT_MS/BTN_POLL_MS){
        btn_hold_cnt = 0;
        button_long_press();
      }
      else{
        app_timer_start(app_tmr_btn_long_press_id,APP_TIMER_TICKS(BTN_POLL_MS),NULL);
      }
  }
  else
  {
      btn_hold_cnt = 0;
      button_short_press();
  }
}

static void button_handler(uint8_t pin_no, uint8_t button_action) {
    (void)pin_no;
    if (button_action == APP_BUTTON_PUSH)
    {
        app_timer_start(app_tmr_btn_long_press_id, APP_TIMER_TICKS(BTN_POLL_MS),NULL); 
    }
}

static void bsp_evt_handler(bsp_event_t bsp_event) {}

static uint8_t int_counters[8];
static uint8_t compare_counters[8];

static void accel_src_handle(lsm303_accel_reg_int_src_t src)
{
    ASSERT(src.bit.I_A);
    static int cnt;
    if(src.bit.Y_H){
    //   read_accel(); /* @note When SD will be running, this might not work as currently these are synchronous read requests */
    //   read_mag();
      NRF_LOG_INFO("Int cnt %d, int val %u",cnt++,src.reg);
      app_tmr_print_out_handler(NULL);
      NRF_LOG_FLUSH();
    }
    else if(src.bit.Y_L){
      NRF_LOG_INFO("Y_L");
    }
    else if(src.bit.Z_H){
      NRF_LOG_INFO("Z_H");
    }
    else if(src.bit.Z_L){
      NRF_LOG_INFO("Z_L");
    }
    else if(src.bit.X_H){
      NRF_LOG_INFO("X_H");
    }
    else if(src.bit.X_L){
      NRF_LOG_INFO("X_L");
    }

}

static void accel_int_read_cb(ret_code_t result, void * p_user_data)
{
    if(result == NRF_SUCCESS)
    {
      struct _lsm303_reg_dsc_t* reg = gcontainer_of(p_user_data, struct _lsm303_reg_dsc_t, data);
      ASSERT(reg->addr == LSM303_REG_ACCEL_CLICK_SRC);
      accel_src_handle((lsm303_accel_reg_int_src_t)reg->data);
    }
    else
    {
      NRF_LOG_ERROR("Error in %s %08X",__FUNCTION__,result);
    }
}

static void accel_int_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    (void)pin;
    (void)action;
    
    NRF_LOG_ERROR("accel INT");
    lsm303_read_reg(&lsm_reg_data.reg.int1_src.addr, &lsm_reg_data.reg.int1_src.data, 1, accel_int_read_cb);
}

static void setup_accel_pin_int(void)
{
    nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    config.pull = NRF_GPIO_PIN_PULLUP;
    uint32_t err_code = nrf_drv_gpiote_in_init(ACCEL_INT_PIN, &config, accel_int_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(ACCEL_INT_PIN, true);
}

static void ui_init(void)
{
    static const app_button_cfg_t btn_cfg = {
      .pull_cfg = NRF_GPIO_PIN_PULLUP,
      .button_handler = button_handler,
      .active_state = 0,
      .pin_no = BTN_PIN,
    }; // Must be static
    app_timer_create(&app_tmr_btn_long_press_id,
                      APP_TIMER_MODE_SINGLE_SHOT,
                      button_timeout_handler);

    uint32_t err_code = app_button_init(&btn_cfg,1,BTN_POLL_MS);
    APP_ERROR_CHECK(err_code);

    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);

    nrf_gpio_cfg_output(LED_GREEN);
    nrf_gpio_cfg_output(LED_RED);
}

static void utils_setup(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    ui_init();

    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


void read_lsm303_tmr_handler(void* p_context) {
    
    read_accel();
    read_mag();
    lsm303_data_t* p_lsm303_data = lsm303_data_p_get();

    NRF_LOG_FLUSH();
}

void clock_event_handler(nrfx_clock_evt_type_t event){}

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

static void flash_storage_read_cb(void* data,int len)
{
    
}


APP_TIMER_DEF(reread_timer); // This is kinda hacky for now, because I don't want to change how read_accel works

static void calibration_handle(void) {
    static uint8_t init_step = 1;
    static uint16_t init_angle = 0;

    lsm303_data_t* p_lsm303_data = lsm303_data_p_get();

    if(init_step == 1) {
        /* get initial value of angle */
        init_angle = p_lsm303_data->accel.angle;

        init_step = 0;
    }
}

static bool timer_running = false;
void read_lsm303(void *unused)
{
    // read_accel();
    // read_mag();
    timer_running = false;

    if(m_app_CB.app_state == STATE_CALIBRATING) {
        calibration_handle();
    }
}



/**
 * @brief Function for main application entry.
 */
int main(void)
{
    STATIC_ASSERT_MSG(sizeof(app_state) <= 4,"Size too big");
    
    int32_t reset_reason = NRF_POWER->RESETREAS;

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    utils_setup();

    //APP_GPIOTE_INIT(1);
    
    lfclk_request();

    flash_storage_init(flash_storage_read_cb);

    app_timer_create(&read_lsm303_tmr_id,
                        APP_TIMER_MODE_REPEATED,
                        read_lsm303_tmr_handler);
    
    app_timer_create(&app_tmr_print_out_id,
                        APP_TIMER_MODE_REPEATED,
                        app_tmr_print_out_handler);

    app_timer_create(&app_tmr_led_blink_id,
                        APP_TIMER_MODE_SINGLE_SHOT,
                        led_timeout_handler);

    app_timer_create(&app_tmr_calibration_id,
                        APP_TIMER_MODE_SINGLE_SHOT,
                        calibration_timeout_handler);

    twi_config();

    nrf_delay_us(6000); // Datasheet says 5ms of boot time, stay safe
    
    setup_accel_pin_int();
    lms303_accel_vibration_trig_setup();
    lsm303_read_reg(&lsm_reg_data.reg.who_i_am.addr, &lsm_reg_data.reg.who_i_am.data, 1, lsm303_read_end_callback);

    lsm303_mag_setup();

    lsm303_setup_read_back_check();
    
    app_timer_start(read_lsm303_tmr_id,APP_TIMER_TICKS(10),NULL);
    app_timer_start(app_tmr_print_out_id,APP_TIMER_TICKS(100),NULL);

 
    app_timer_create(&reread_timer,
                    APP_TIMER_MODE_SINGLE_SHOT,
                    read_lsm303);

    
    
    while (true)
    {
        NRF_LOG_FLUSH();
        nrf_pwr_mgmt_run();
        if(app_state.calibrating)
        {
          bool calib_result;
          lsm303_data_t initial_data;
          lsm303_data_t* p_lsm303_data = lsm303_data_p_get();
          memcpy(&initial_data,p_lsm303_data,sizeof(initial_data));
          app_tmr_print_out_handler(NULL);
          app_timer_start(reread_timer,APP_TIMER_TICKS(10),NULL);
          timer_running = true;
          while(app_state.calibrating && !(calib_result = calib_handler(NULL)))
          {
            if(!timer_running){
              app_timer_start(reread_timer,APP_TIMER_TICKS(10),NULL);
              timer_running = true;
            }
            if(NRF_LOG_PROCESS() == false){
              nrf_pwr_mgmt_run();
            }
          }
          if(calib_result){
            NRF_LOG_INFO("Key turned full circle");
          }
         
          NRF_LOG_INFO("calibrating %d calib_result %d",app_state.calibrating,calib_result);
          app_timer_stop(app_tmr_calibration_id);
          app_state.calibrating = false;
          timer_running = false;
        }
    }
}

/** @} */

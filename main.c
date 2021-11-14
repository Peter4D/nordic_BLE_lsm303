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

#include "limits.h"

#define goffsetof(TYPE, MEMBER) ((size_t) & ((TYPE *)0)->MEMBER)
// Obtain a pointer to the entire structure variable according to the "pointer (ptr) of the domain member variable (member)" in the "structure (type) variable"
#define gcontainer_of(ptr, type, member) (type *)((char *)ptr - goffsetof(type, member));

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
#define DEBUG_APP_SHOW_AXIS_MAG 2
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
#define TWI_INSTANCE_ID 0

/* UI */
#define BTN_PIN 5
#define BTN_POLL_MS 100
#define LONG_BTN_TIMEOUT_MS 5000
#define LED_SHORT_BLINK_TIME_MS 100
#define LED_LONG_BLINK_TIME_MS 1000
#define LED_1 4
#define LED_2 20
#define LED_GREEN LED_1
#define LED_RED LED_2

/* ***************************************************** */
#define NUM_AREAS 36u
#define AREA_ANGLE ( (uint16_t) ( ((uint32_t) 360u) / ((uint16_t) NUM_AREAS) ) )
#define AREA_ANGLE_START (uint16_t) ((AREA_ANGLE/2)-1)
#define AREA_ANGLE_END (uint16_t) ((AREA_ANGLE/2))

/* ***************************************************** */

/* Arbitary calibration constants */
#define CALIBRATION_TIMEOUT_MS 20000 /* Timeout after enabling calibration and no action was taken */

typedef enum
{
  GREEN_OFF = 1 << 0,
  RED_OFF = 1 << 1,
  BOTH_OFF = GREEN_OFF | RED_OFF,
} led_timeout_handle_t;

#define led_off nrf_gpio_pin_clear
#define led_on nrf_gpio_pin_set
#define led_toggle nrf_gpio_pin_toggle

#define ACCEL_INT_PIN 14

#define LM75B_REG_TEMP 0x00U
#define LM75B_REG_CONF 0x01U
#define LM75B_REG_THYST 0x02U
#define LM75B_REG_TOS 0x03U

/* Mode for LM75B. */
#define NORMAL_MODE 0U

static uint8_t m_who_i_am = 0xFF;

typedef enum
{
  STATE_IDLE = 0,
  STATE_CALIBRATING,
  STATE_ACTIVE
} app_state_e_t;

typedef struct _app_CB_t
{
  app_state_e_t app_state;
  int8_t key_side;
} app_CB_t;

typedef struct
{
  uint8_t calibrated : 1;
  uint8_t calibrating : 1;
  uint8_t locked : 1;
  uint8_t lockedTwice : 1;
  uint8_t inserted : 1;
  uint16_t area : 1;
  uint8_t insertedInside : 1;
  uint16_t areaId;
} application_state_t;

static app_CB_t m_app_CB = {
    .app_state = STATE_IDLE,
};


void InitCalibParams(void);
void update_area_id(void);


static bool calibration_active = false;


static uint8_t step = 0u;
  static uint16_t curr_angle = 0u;
  static uint16_t num_of_meas = 0u;


typedef struct _cake_data_model_t {
    uint16_t start_angle_i;
    uint16_t end_angle_i; 
    uint16_t start_angle;
    uint16_t end_angle;
    uint16_t area_id;
    uint16_t angle_meas_taken_x;
    uint16_t angle_meas_taken_y;
    uint16_t angle_meas_taken_z;
    int16_t x_peak;
    int16_t y_peak;
    int16_t z_peak;
    int16_t x_avg;
    int16_t y_avg;
    int16_t z_avg;
    int16_t x_set_threshold_min;
    int16_t y_set_threshold_min;
    int16_t z_set_threshold_min;    

    int16_t x_set_threshold_min_from_avg;
    int16_t y_set_threshold_min_from_avg;
    int16_t z_set_threshold_min_from_avg;   

    int16_t x_set_threshold_max_from_avg;
    int16_t y_set_threshold_max_from_avg;
    int16_t z_set_threshold_max_from_avg;    

    int16_t x_set_threshold_max;
    int16_t y_set_threshold_max;
    int16_t z_set_threshold_max; 

    uint16_t sum_abs_xyz_peaks;  

    uint16_t sum_abs_xyz_peaks_min;
    uint16_t sum_abs_xyz_peaks_max; 

    uint16_t numOfMeasTaken; 
}cake_data_model_t;

typedef struct _cake_data_col_t {
    cake_data_model_t cake_data[NUM_AREAS];
    
    int16_t x_set_insert_threshold;
    uint16_t actual_angle;
    uint16_t actual_area_id;
}cake_data_col_t;

static cake_data_col_t cake_data_col =  {
  .cake_data[0].start_angle = 0u,
  .cake_data[0].end_angle = 0u,
  .cake_data[0].area_id = 0u,
  .cake_data[0].x_peak = 0u,
  .cake_data[0].y_peak = 0u,
  .cake_data[0].z_peak = 0u,
  .cake_data[0].x_set_threshold_min = 0u,
  .cake_data[0].y_set_threshold_min = 0u,
  .cake_data[0].z_set_threshold_min = 0u,
};


#define NUM_MEAS_AREA ( ( ( (uint32_t) AREA_ANGLE) / ( (uint16_t) 4u ) )  )
uint16_t angle_meas_taken[NUM_MEAS_AREA];


static application_state_t app_state;

static uint16_t cnt_x_axis_lowering = 0u;
static uint16_t cnt_x_axis_lowering_wait = 0u;
static int16_t old_x_axis = 0u;

static bool timer_running = false;

/* Buffer for samples read from accelerometer sensor. */

APP_TIMER_DEF(read_lsm303_tmr_id);
APP_TIMER_DEF(app_tmr_btn_long_press_id);
APP_TIMER_DEF(app_tmr_print_out_id);
APP_TIMER_DEF(app_tmr_led_blink_id);
APP_TIMER_DEF(app_tmr_calibration_id);

void read_lsm303_tmr_handler(void *p_context);
void check_key_inserted(void);
void check_area_changed(void);

void update_area(void);
void check_area_changed2(void);

static uint16_t increment_angle_step(uint16_t angle);
static uint16_t define_angle_limit(uint16_t angle, bool start_angle);






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
static void test_i2c_read_callback(ret_code_t result, void *p_user_data)
{
  NRF_LOG_INFO("who i am read %u", ((uint8_t *)p_user_data)[0]);
}

static void app_tmr_print_out_handler(void *p_context)
{

  lsm303_data_t *p_lsm303_data = lsm303_data_p_get();

#if (DEBUG_APP_SHOW_QD == 1)

  NRF_LOG_INFO("angle/a/b/dir/cnt/Y_peak | %3d,%u,%u,%d,%d,%d",
               p_lsm303_data->accel.angle,

               p_lsm303_data->mag.qd_data.qd.bit.a,
               p_lsm303_data->mag.qd_data.qd.bit.b,
               p_lsm303_data->mag.qd_data.qd_dir,
               p_lsm303_data->mag.qd_data.qd_cnt,

               p_lsm303_data->mag.axis_peak.bit.y);

#elif (DEBUG_APP_SHOW_QD_ACCEL == 1)

  NRF_LOG_INFO("angle/a/b/dir/cnt | %3d,%u,%u,%2d,%d",
               p_lsm303_data->accel.angle,

               p_lsm303_data->accel.qd_data.qd.bit.a,
               p_lsm303_data->accel.qd_data.qd.bit.b,
               p_lsm303_data->accel.qd_data.qd_dir,
               p_lsm303_data->accel.qd_data.qd_cnt);

#elif (DEBUG_APP_SHOW_QD_AXIS_ACCEL == 1)

  NRF_LOG_INFO("angle/a/b/x/z | %3d,%u,%u,%5d,%5d",
               p_lsm303_data->accel.angle,

               p_lsm303_data->accel.qd_data.qd.bit.a,
               p_lsm303_data->accel.qd_data.qd.bit.b,
               abs(p_lsm303_data->accel.axis.bit.x),
               abs(p_lsm303_data->accel.axis.bit.z));

#elif (DEBUG_APP_SHOW_AXIS_MAG == 1)

  NRF_LOG_INFO("angle/x/y/z/dir/cnt/ins | %3d,%5d,%5d,%5d,%2d,%d",
               p_lsm303_data->accel.angle,
               p_lsm303_data->mag.axis.bit.x,
               p_lsm303_data->mag.axis.bit.y,
               p_lsm303_data->mag.axis.bit.z,

               p_lsm303_data->mag.qd_data.qd_dir,
               p_lsm303_data->mag.qd_data.qd_cnt);

#elif (DEBUG_APP_SHOW_AXIS_MAG == 2)

#if 0
    NRF_LOG_INFO("area/angle/ins | %d,%d,%3d,%d",
    p_lsm303_data->accel.area, 
    app_state.locked,
    p_lsm303_data->accel.angle, 
    app_state.inserted
#endif

#if 0
  NRF_LOG_INFO("ins/insInd/Lck/LckTwc/areaId/angle| %5d,%5d,%5d,%3d",
               p_lsm303_data->mag.axis.bit.x,
               p_lsm303_data->mag.axis.bit.y,
               p_lsm303_data->mag.axis.bit.z,
               p_lsm303_data->accel.angle

#endif


#if 0
  NRF_LOG_INFO("ins/insInd/Lck/LckTwc/areaId/angle| %5d,%5d,%5d,%d,%d,%3d",
               p_lsm303_data->mag.axis.bit.x,
               p_lsm303_data->mag.axis.bit.y,
               p_lsm303_data->mag.axis.bit.z,
               app_state.inserted,
               app_state.areaId,
               p_lsm303_data->accel.angle

#endif


#if 1
  NRF_LOG_INFO("ins/Lck/areaId/angle| %d,%d,%d,%3d",
               app_state.inserted,
               app_state.locked,
               app_state.areaId,
               p_lsm303_data->accel.angle

#endif
  );


  

#elif (DEBUG_APP_SHOW_MAG_AB == 1)

  NRF_LOG_INFO("angle/x/z/a/b | %3d,%5d,%5d,%d,%d",
               p_lsm303_data->accel.angle,
               p_lsm303_data->mag.axis.bit.x,
               p_lsm303_data->mag.axis.bit.z,

               p_lsm303_data->mag.qd_data.qd.bit.a,
               p_lsm303_data->mag.qd_data.qd.bit.b);

#elif (DEBUG_APP_SHOW_AXIS_MAG_2 == 1)

  NRF_LOG_INFO("angle/x/y/z | %3d,%5d,%5d,%5d",
               p_lsm303_data->accel.angle,
               p_lsm303_data->mag.axis.bit.x,
               p_lsm303_data->mag.axis.bit.y,
               p_lsm303_data->mag.axis.bit.z);

#elif (DEBUG_APP_SHOW_ACCEL_AXIS == 1)

  NRF_LOG_INFO("A: angle/x/y/z | %3d, %5d, %5d, %5d",
               p_lsm303_data->accel.angle,

               abs(p_lsm303_data->accel.axis.bit.x),
               abs(p_lsm303_data->accel.axis.bit.y),
               abs(p_lsm303_data->accel.axis.bit.z));

#endif

  // static uint8_t who_i_am_reg_addr = LSM303_REG_ACCEL_WHO_AM_I;
  // lsm303_read_reg(&who_i_am_reg_addr, &m_who_i_am, 1, test_i2c_read_callback);
}

static bool calib_handler(void *p_context)
{
  lsm303_data_t *p_lsm303_data = lsm303_data_p_get();
  int32_t qd_cnt = p_lsm303_data->mag.qd_data.qd_cnt;

  if (abs(qd_cnt) > 4)
  {
    /* key has turned more than full circle */
    if (p_lsm303_data->mag.axis_peak.bit.y > 0)
    {
      m_app_CB.key_side = 0;
    }
    else
    {
      m_app_CB.key_side = -1;
    }
    return true;
  }
  return false;
}

static void lsm303_read_end_callback(ret_code_t result, void *p_user_data)
{

  struct _lsm303_reg_dsc_t *p_my_container = {0};

  p_my_container = gcontainer_of(p_user_data, struct _lsm303_reg_dsc_t, data);

  NRF_LOG_INFO("reg: %s, val: 0x%X", p_my_container->p_name, ((uint8_t *)p_user_data)[0]);
}

static uint8_t reg_data[1];
static uint8_t addr_reg = LSM303_REG_ACCEL_INT1_SOURCE;

static uint16_t sum = 0u;

static void calibration_timeout_handler(void *calibration_ctx_s)
{
  app_state.calibrating = false;
  calibration_active = false;
  NRF_LOG_INFO("Calibration timeout");
}

static void led_timeout_handler(void *led_state_s)
{
  led_timeout_handle_t led_state = (led_timeout_handle_t)led_state_s;
  if (led_state & GREEN_OFF)
  {
    led_off(LED_GREEN);
  }
  if (led_state & RED_OFF)
  {
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

  if (app_state.calibrating)
  {
    led_on(LED_GREEN);
    led_on(LED_RED);
    off_action = BOTH_OFF;
  }
  else if (!app_state.calibrated)
  {
    led_on(LED_GREEN);
    led_on(LED_RED);
    off_action = BOTH_OFF;
  }
  else if (app_state.locked)
  {
    led_on(LED_GREEN);
    off_action = GREEN_OFF;
    // Signal if door is locked or not
  }
  else
  {
    led_on(LED_RED);
    off_action = RED_OFF;
  }
  app_timer_start(app_tmr_led_blink_id, APP_TIMER_TICKS(LED_SHORT_BLINK_TIME_MS), (void *)off_action);
}

static void calibration_start(void)
{
  // lsm303_accel_reg_int_cfg_t cfg = {.reg = ENABLE_ALL_AXIS_INT};
  // lms303_accel_int_en(cfg);

  NRF_LOG_INFO("Calibration start");
  // read_accel(); /* @note When SD will be running, this might not work as currently these are synchronous read requests */
  // read_mag();

  app_state.calibrating = true;

  InitCalibParams();

  step = 0u;
  curr_angle = 0u;
  num_of_meas = 0u;

  ret_code_t ret = app_timer_start(app_tmr_calibration_id, APP_TIMER_TICKS(CALIBRATION_TIMEOUT_MS), NULL);
  APP_ERROR_CHECK(ret);
}

void InitCalibParams(void)
{
  /*Init counter */
    for (int i = 0u; i < NUM_AREAS; i++) 
  {
    cake_data_col.cake_data[i].numOfMeasTaken = 0u;
  }
}

static void button_long_press(void)
{
  if (!app_state.calibrating)
  {
    calibration_start();
    led_on(LED_GREEN);
    app_timer_start(app_tmr_led_blink_id, APP_TIMER_TICKS(LED_SHORT_BLINK_TIME_MS), (void *)GREEN_OFF);
  }
  else
  {
    NRF_LOG_INFO("Already calibrating");
  }
}

static void button_timeout_handler(void *unused)
{
  (void)unused;
  static uint8_t btn_hold_cnt = 0;
  static bool last_state = 0;

  if (app_button_is_pushed(0))
  {
    /*if (!short_executed)
    {*/
      /* execute only once - when button is just pressed */
      /*button_short_press();
      short_executed = 1;
    }*/

    last_state = 1;

    if (++btn_hold_cnt >= LONG_BTN_TIMEOUT_MS / BTN_POLL_MS)
    {
      btn_hold_cnt = 0;
      button_long_press();
    }
    else
    {
      app_timer_start(app_tmr_btn_long_press_id, APP_TIMER_TICKS(BTN_POLL_MS), NULL);
    }
  }
  else
  {
    if (last_state == 1)
    {
      last_state = 0;
      button_short_press();
    }

    btn_hold_cnt = 0;

    /* reset short button pressed */
    
    /*button_short_press();*/
  }
}

static void button_handler(uint8_t pin_no, uint8_t button_action)
{
  (void)pin_no;
  if (button_action == APP_BUTTON_PUSH)
  {
    app_timer_start(app_tmr_btn_long_press_id, APP_TIMER_TICKS(BTN_POLL_MS), NULL);
  }
}

static void bsp_evt_handler(bsp_event_t bsp_event) {}

static uint8_t int_counters[8];
static uint8_t compare_counters[8];

static void accel_src_handle(lsm303_accel_reg_int_src_t src)
{
  ASSERT(src.bit.I_A);
  static int cnt;
  if (src.bit.Y_H)
  {
    //   read_accel(); /* @note When SD will be running, this might not work as currently these are synchronous read requests */
    //   read_mag();
    NRF_LOG_INFO("Int cnt %d, int val %u", cnt++, src.reg);
    app_tmr_print_out_handler(NULL);
    NRF_LOG_FLUSH();
  }
  else if (src.bit.Y_L)
  {
    NRF_LOG_INFO("Y_L");
  }
  else if (src.bit.Z_H)
  {
    NRF_LOG_INFO("Z_H");
  }
  else if (src.bit.Z_L)
  {
    NRF_LOG_INFO("Z_L");
  }
  else if (src.bit.X_H)
  {
    NRF_LOG_INFO("X_H");
  }
  else if (src.bit.X_L)
  {
    NRF_LOG_INFO("X_L");
  }
}

static void accel_int_read_cb(ret_code_t result, void *p_user_data)
{
  if (result == NRF_SUCCESS)
  {
    struct _lsm303_reg_dsc_t *reg = gcontainer_of(p_user_data, struct _lsm303_reg_dsc_t, data);
    ASSERT(reg->addr == LSM303_REG_ACCEL_CLICK_SRC);
    accel_src_handle((lsm303_accel_reg_int_src_t)reg->data);
  }
  else
  {
    NRF_LOG_ERROR("Error in %s %08X", __FUNCTION__, result);
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

  uint32_t err_code = app_button_init(&btn_cfg, 1, BTN_POLL_MS);
  
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

void read_lsm303_tmr_handler(void *p_context)
{

  read_accel();
  read_mag();
  lsm303_data_t *p_lsm303_data = lsm303_data_p_get();

  NRF_LOG_FLUSH();
}

void clock_event_handler(nrfx_clock_evt_type_t event) {}

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

static void flash_storage_read_cb(void *data, int len)
{
}

APP_TIMER_DEF(reread_timer); // This is kinda hacky for now, because I don't want to change how read_accel works








static void calibration_handle_keyinserted(void)
{
lsm303_data_t *p_lsm303_data = lsm303_data_p_get();
  uint16_t tmp_angle = p_lsm303_data->accel.angle; /* can be different from the lock to lock */

  /* create a tree of data */
    for (int i = 0u; i < NUM_AREAS; i++)
    {
      cake_data_col.cake_data[i].start_angle_i = (int16_t)(tmp_angle-AREA_ANGLE_START);
      cake_data_col.cake_data[i].end_angle_i = (int16_t)(tmp_angle+AREA_ANGLE_END);

      cake_data_col.cake_data[i].start_angle = define_angle_limit(tmp_angle, true);
      cake_data_col.cake_data[i].end_angle = define_angle_limit(tmp_angle, false);
      cake_data_col.cake_data[i].area_id = i;

      tmp_angle = increment_angle_step(tmp_angle);

    }

    calibration_active = true;
}



static void calibration_handle(void)
{
  

  lsm303_data_t *p_lsm303_data = lsm303_data_p_get();
  curr_angle = p_lsm303_data->accel.angle; /* can be different from the lock to lock */

  bool allDone = true;

  /* Find actual location of the key */
  uint16_t stepLocation = 99U;
  for (int i = 0u; i < NUM_AREAS; i++)
  {
    if (cake_data_col.cake_data[i].numOfMeasTaken < NUM_MEAS_AREA)
    {
      allDone = false;
    }

    if ( ((curr_angle+AREA_ANGLE) >= (uint16_t)(cake_data_col.cake_data[i].start_angle_i + AREA_ANGLE)) && ((curr_angle+AREA_ANGLE) <= (uint16_t)(cake_data_col.cake_data[i].end_angle_i + AREA_ANGLE)) )
    {
      stepLocation = i;
    }
  }

  /* Calibration is done when taking of measurment of every 1/12 at least 15 times */
  if (stepLocation != 99U && cake_data_col.cake_data[stepLocation].numOfMeasTaken < NUM_MEAS_AREA)
  {
    cake_data_col.cake_data[stepLocation].numOfMeasTaken++;
    /*NRF_LOG_INFO("Number of meas %d", cake_data_col.cake_data[stepLocation].numOfMeasTaken);*/

    /*NRF_LOG_INFO("Step location %d", stepLocation);*/
    uint16_t nmt = cake_data_col.cake_data[stepLocation].numOfMeasTaken;

    if (cake_data_col.cake_data[stepLocation].numOfMeasTaken < NUM_MEAS_AREA)
    {
      if (cake_data_col.cake_data[stepLocation].numOfMeasTaken == 1u)
      {
        /* Init meas angle list */
        for(int i = 0u; i < NUM_MEAS_AREA; i++)
        {
          angle_meas_taken[i] = 999u;
        }

        

        cake_data_col.cake_data[stepLocation].x_avg = p_lsm303_data->mag.axis.bit.x;
        cake_data_col.cake_data[stepLocation].y_avg = p_lsm303_data->mag.axis.bit.y;
        cake_data_col.cake_data[stepLocation].z_avg = p_lsm303_data->mag.axis.bit.z;

        angle_meas_taken[nmt-1] = p_lsm303_data->accel.angle;
      }
      else
      {
        /*NRF_LOG_INFO("X %d", p_lsm303_data->mag.axis.bit.x);
        NRF_LOG_INFO("Y %d", p_lsm303_data->mag.axis.bit.y);
        NRF_LOG_INFO("Z %d", p_lsm303_data->mag.axis.bit.z);*/

        uint16_t angle = p_lsm303_data->accel.angle;

        bool angle_used = false;

        for(int i = 0u; i < NUM_MEAS_AREA; i++)
        {
          if (angle == angle_meas_taken[i])
          {
            angle_used = true;
          }
        }

        if (angle_used == false)
        {
          angle_meas_taken[nmt-1] = angle;

          cake_data_col.cake_data[stepLocation].x_avg = (cake_data_col.cake_data[stepLocation].x_avg + p_lsm303_data->mag.axis.bit.x) >> 1;
          cake_data_col.cake_data[stepLocation].y_avg = (cake_data_col.cake_data[stepLocation].y_avg + p_lsm303_data->mag.axis.bit.y) >> 1;
          cake_data_col.cake_data[stepLocation].z_avg = (cake_data_col.cake_data[stepLocation].z_avg + p_lsm303_data->mag.axis.bit.z) >> 1;

          NRF_LOG_INFO("X avg %d", cake_data_col.cake_data[stepLocation].x_avg);
        NRF_LOG_INFO("Y avg %d", cake_data_col.cake_data[stepLocation].y_avg);
        NRF_LOG_INFO("Z avg %d", cake_data_col.cake_data[stepLocation].z_avg);
        }
        else
        {
          cake_data_col.cake_data[stepLocation].numOfMeasTaken--;
        }
        
        
      }
    }
    else
    {
      cake_data_col.cake_data[stepLocation].numOfMeasTaken = 99u;
    }
  }

  if (allDone)
  {
    NRF_LOG_INFO("Post analysis");
    // post analysis
    for (int i = 0u; i < NUM_AREAS; i++)
    {
      cake_data_col.cake_data[i].x_set_threshold_min_from_avg = (int16_t) ( (int32_t)( (cake_data_col.cake_data[i].x_avg*40) )/100);
      cake_data_col.cake_data[i].y_set_threshold_min_from_avg = (int16_t) ( (int32_t)( (cake_data_col.cake_data[i].y_avg*40) )/100);
      cake_data_col.cake_data[i].z_set_threshold_min_from_avg = (int16_t) ( (int32_t)( (cake_data_col.cake_data[i].z_avg*40) )/100);

      cake_data_col.cake_data[i].x_set_threshold_max_from_avg = (int16_t)((((int32_t)cake_data_col.cake_data[i].x_avg*160))/100);
      cake_data_col.cake_data[i].y_set_threshold_max_from_avg = (int16_t)((((int32_t)cake_data_col.cake_data[i].y_avg*160))/100);
      cake_data_col.cake_data[i].z_set_threshold_max_from_avg = (int16_t)((((int32_t)cake_data_col.cake_data[i].z_avg*160))/100);  
    }

    cake_data_col.x_set_insert_threshold = (int16_t) ( (int32_t)( (cake_data_col.cake_data[0].x_avg*40) )/100);

    uint16_t keks = 0u;

    NRF_LOG_INFO("Calibration Ends");
    calibration_active = false;

    app_timer_stop(app_tmr_calibration_id);
    app_state.calibrating = false;
    app_state.calibrated = true;
    timer_running = false;
    app_state.locked = true;
    led_on(LED_GREEN);
    app_timer_start(app_tmr_led_blink_id, APP_TIMER_TICKS(LED_LONG_BLINK_TIME_MS), (void *)GREEN_OFF);
  }











  /* TODO: In area ID 0 we take first 15 degrees and not also the "last"15 degress */
  #if 0
  if (12u > step >= 0u)
  {
    if (step == cake_data_col.cake_data[step].area_id)
    {
      if ( ((curr_angle+30u) >= (uint16_t)(cake_data_col.cake_data[step].start_angle_i + 30)) && ((curr_angle+30u) <= (uint16_t)(cake_data_col.cake_data[step].end_angle_i + 30)) )
      {
        num_of_meas++;
        NRF_LOG_INFO("Number of meas %d", num_of_meas);

        if (num_of_meas == 1u)
        {
          cake_data_col.cake_data[step].x_avg = p_lsm303_data->mag.axis.bit.x;
          cake_data_col.cake_data[step].y_avg = p_lsm303_data->mag.axis.bit.y;
          cake_data_col.cake_data[step].z_avg = p_lsm303_data->mag.axis.bit.z;
        }
        else
        {
          cake_data_col.cake_data[step].x_avg = (cake_data_col.cake_data[step].x_avg + p_lsm303_data->mag.axis.bit.x) >> 1;
          cake_data_col.cake_data[step].y_avg = (cake_data_col.cake_data[step].y_avg + p_lsm303_data->mag.axis.bit.y) >> 1;
          cake_data_col.cake_data[step].z_avg = (cake_data_col.cake_data[step].z_avg + p_lsm303_data->mag.axis.bit.z) >> 1;
        }
        

        if (abs(p_lsm303_data->mag.axis.bit.x) > abs(cake_data_col.cake_data[step].x_peak))
        {
          cake_data_col.cake_data[step].x_peak = p_lsm303_data->mag.axis.bit.x;
          cake_data_col.cake_data[step].angle_meas_taken_x = curr_angle;
        }

        if (abs(p_lsm303_data->mag.axis.bit.y) > abs(cake_data_col.cake_data[step].y_peak))
        {
          cake_data_col.cake_data[step].y_peak = p_lsm303_data->mag.axis.bit.y;
          cake_data_col.cake_data[step].angle_meas_taken_y = curr_angle;
        }

        if (abs(p_lsm303_data->mag.axis.bit.z) > abs(cake_data_col.cake_data[step].z_peak))
        {
          cake_data_col.cake_data[step].z_peak = p_lsm303_data->mag.axis.bit.z;
          cake_data_col.cake_data[step].angle_meas_taken_z = curr_angle;
        }

        if ((cake_data_col.cake_data[step].x_peak != 0) && (cake_data_col.cake_data[step].y_peak != 0) && (cake_data_col.cake_data[step].z_peak != 0) && (num_of_meas > 15u))
        {
          step++;
          num_of_meas=0u;
          NRF_LOG_INFO("Step %d", step);
        } 
      }
    }
  }

  if (step == 12u)
  {
    NRF_LOG_INFO("Post analysis");
    // post analysis
    for (int i = 0u; i < 12u; i++)
    {
      cake_data_col.cake_data[i].x_set_threshold_min_from_avg = (int16_t) ( (int32_t)( (cake_data_col.cake_data[i].x_avg*40) )/100);
      cake_data_col.cake_data[i].y_set_threshold_min_from_avg = (int16_t) ( (int32_t)( (cake_data_col.cake_data[i].y_avg*40) )/100);
      cake_data_col.cake_data[i].z_set_threshold_min_from_avg = (int16_t) ( (int32_t)( (cake_data_col.cake_data[i].z_avg*40) )/100);

      cake_data_col.cake_data[i].x_set_threshold_max_from_avg = (int16_t)((((int32_t)cake_data_col.cake_data[i].x_avg*160))/100);
      cake_data_col.cake_data[i].y_set_threshold_max_from_avg = (int16_t)((((int32_t)cake_data_col.cake_data[i].y_avg*160))/100);
      cake_data_col.cake_data[i].z_set_threshold_max_from_avg = (int16_t)((((int32_t)cake_data_col.cake_data[i].z_avg*160))/100);  
    }

    cake_data_col.x_set_insert_threshold = (int16_t) ( (int32_t)( (cake_data_col.cake_data[0].x_avg*40) )/100);

    uint16_t keks = 0u;

    NRF_LOG_INFO("Calibration Ends");
    calibration_active = false;
    #endif

    #if 0
    app_timer_stop(app_tmr_calibration_id);
      app_state.calibrating = false;
      app_state.calibrated = true;
      timer_running = false;
      app_state.locked = true;
      led_on(LED_GREEN);
      app_timer_start(app_tmr_led_blink_id, APP_TIMER_TICKS(LED_LONG_BLINK_TIME_MS), (void *)GREEN_OFF);
      #endif

  
}



static uint16_t increment_angle_step(uint16_t angle)
{
  uint16_t tmp = angle + AREA_ANGLE;
  
  if (tmp >= 360u)
  {
    tmp = tmp - 360u;
  }

  return tmp;
}

static uint16_t define_angle_limit(uint16_t angle, bool start_angle)
{
  uint16_t temp = 0u;

  if (start_angle)
  {
    /* if angle is close to 0*/
    if (angle < AREA_ANGLE_END)
    {
      uint16_t cnt2angle = 0u;
      while (cnt2angle <= angle)
      {
        cnt2angle++;
      }

      temp = 360u - (AREA_ANGLE_END - cnt2angle);
    }
    else
    {
      temp = angle - AREA_ANGLE_START;
    }
  }
  else
  {
    /* if angle is close to 360 */
    if (angle > (360-AREA_ANGLE_END))
    {
      uint16_t cnt2angle = 0u;
      while (!((cnt2angle + angle) >= 360u))
      {
        cnt2angle++;
      }
      temp = 0u + (AREA_ANGLE_END - cnt2angle);
    }
    else
    {
      temp = angle + AREA_ANGLE_END;
    }
  }

  return temp;
}


void read_lsm303(void *unused)
{
  // read_accel();
  // read_mag();
  timer_running = false;


  if (m_app_CB.app_state == STATE_CALIBRATING)
  {
    calibration_handle();
  }
}



/**
 * @brief Function for main application entry.
 */
int main(void)
{
  STATIC_ASSERT_MSG(sizeof(app_state) <= 4, "Size too big");

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

  app_timer_start(read_lsm303_tmr_id, APP_TIMER_TICKS(10), NULL);
  app_timer_start(app_tmr_print_out_id, APP_TIMER_TICKS(100), NULL);

  app_timer_create(&reread_timer,
                   APP_TIMER_MODE_SINGLE_SHOT,
                   read_lsm303);

  static uint16_t cnt;

  init_areas();

  while (true)
  {
    NRF_LOG_FLUSH();
    nrf_pwr_mgmt_run();

    

    if (app_state.calibrating)
    {

    #if 0
      bool calib_result;
      lsm303_data_t initial_data;
      lsm303_data_t *p_lsm303_data = lsm303_data_p_get();
      memcpy(&initial_data, p_lsm303_data, sizeof(initial_data));
      app_tmr_print_out_handler(NULL);
      app_timer_start(reread_timer, APP_TIMER_TICKS(10), NULL);
      timer_running = true;
      while (app_state.calibrating && !(calib_result = calib_handler(NULL)))
      {
        if (!timer_running)
        {
          app_timer_start(reread_timer, APP_TIMER_TICKS(10), NULL);
          timer_running = true;
        }
        if (NRF_LOG_PROCESS() == false)
        {
          nrf_pwr_mgmt_run();
        }
      }
      if (calib_result)
      {
        NRF_LOG_INFO("Key turned full circle");
      }

      NRF_LOG_INFO("calibrating %d calib_result %d", app_state.calibrating, calib_result);

      #endif

      calibration_handle_keyinserted();

      while (calibration_active == true)
      {
        calibration_handle();
      }
      

      

#if 0
      app_timer_stop(app_tmr_calibration_id);
      app_state.calibrating = false;
      app_state.calibrated = true;
      timer_running = false;
#endif
    }

    /*if (!app_state.calibrating)*/
    if (app_state.calibrated)
    {
      
      check_key_inserted();
      update_area_id();
      check_area_changed2();
      
      #if 0
      check_area_changed();
      
      update_area();
      check_area_changed2();
      #endif

    }

    
  }
}

void check_key_inserted(void)
{
  lsm303_data_t *p_lsm303_data = lsm303_data_p_get();

  if (app_state.inserted == true)
  {
    // check if key has gone out
    if (((p_lsm303_data->accel.angle >= 350u) || (p_lsm303_data->accel.angle <= 10u)))
    {
      if (abs(p_lsm303_data->mag.axis.bit.x) < abs(cake_data_col.x_set_insert_threshold))
      {
        app_state.inserted = false;
        app_state.insertedInside = false;
      }
    }
  }
  else
  {
    // check if key was inserted
    if ( ((p_lsm303_data->accel.angle >= 350u) || (p_lsm303_data->accel.angle <= 10u)) && (app_state.areaId == 0u) 
    && (abs(p_lsm303_data->mag.axis.bit.x) > abs(cake_data_col.x_set_insert_threshold)) )
    {
      app_state.inserted = true;

      if (p_lsm303_data->mag.axis.bit.y > 0)
      {
        app_state.insertedInside = false;
      }
      else
      {
        app_state.insertedInside = true;
      }
    }
  }
}


static int16_t pass = false;
  static bool have_actually_pass = false;
  static uint16_t old_area_id = 98u;
  static uint16_t area_id = 0u;
  

void update_area_id(void)
{
  lsm303_data_t *p_lsm303_data = lsm303_data_p_get();

  bool passx = false;
  bool passy = false; 
  bool passz = false;
  bool passsum = false;
  
  for (int i = 0u; i < NUM_AREAS; i++)
  {
     passx = false;
     passy = false; 
     passz = false;

    if (p_lsm303_data->mag.axis.bit.x > 0)
    {
      if ((p_lsm303_data->mag.axis.bit.x > cake_data_col.cake_data[i].x_set_threshold_min_from_avg ))
      {
        if ((cake_data_col.cake_data[i].x_set_threshold_max_from_avg > p_lsm303_data->mag.axis.bit.x ))
        {
          passx = true;
        }
      }
    }
    else
    {
      if ((cake_data_col.cake_data[i].x_set_threshold_min_from_avg > p_lsm303_data->mag.axis.bit.x))
      {
        if ((p_lsm303_data->mag.axis.bit.x > cake_data_col.cake_data[i].x_set_threshold_max_from_avg ))
        {
          passx = true;
        }
      }
    }

    if (p_lsm303_data->mag.axis.bit.y > 0)
    {
      if ((p_lsm303_data->mag.axis.bit.y > cake_data_col.cake_data[i].y_set_threshold_min_from_avg ) )
      {
        if ((cake_data_col.cake_data[i].y_set_threshold_max_from_avg > p_lsm303_data->mag.axis.bit.y ))
        {
          passy = true;
        }
      }
    }
    else
    {
      if ((cake_data_col.cake_data[i].y_set_threshold_min_from_avg > p_lsm303_data->mag.axis.bit.y))
      {
        if ((p_lsm303_data->mag.axis.bit.y > cake_data_col.cake_data[i].y_set_threshold_max_from_avg ))
        {
          passy = true;
        }
      }
    }

    if (p_lsm303_data->mag.axis.bit.z > 0)
    {
      if ((p_lsm303_data->mag.axis.bit.z > cake_data_col.cake_data[i].z_set_threshold_min_from_avg ))
      {
        if ((cake_data_col.cake_data[i].z_set_threshold_max_from_avg > p_lsm303_data->mag.axis.bit.z))
        {
          passz = true; 
        }
      }
    }
    else
    {
      if ((cake_data_col.cake_data[i].z_set_threshold_min_from_avg > p_lsm303_data->mag.axis.bit.z))
      {
        if ((p_lsm303_data->mag.axis.bit.z > cake_data_col.cake_data[i].z_set_threshold_max_from_avg ))
        {
          passz = true;
        }
      }
    }

    if ( (passx == true) && (passy == true) && (passz == true) )
    {
      have_actually_pass = true;
      area_id = cake_data_col.cake_data[i].area_id;
      app_state.areaId = area_id;
      return;
    }
    else
    {
      area_id = 99u;
    }
  }
}


static uint8_t areaIdBuf[6];

void init_areas(void)
{
  for (uint8_t i = (6 - 1); i > 0; i--)
  {
        areaIdBuf[i] = 99u;
  }
}

bool check_init_areas(void)
{
  bool ready_to_go = true;

  for (uint8_t i = (6 - 1); i > 0; i--)
  {
    if (areaIdBuf[i] == 99u)
    {
      ready_to_go = false;
    }
  }

  return ready_to_go;
}

void check_area_changed2(void)
{
  

  if (app_state.inserted)
  {
    lsm303_data_t *p_lsm303_data = lsm303_data_p_get();

    if (areaIdBuf[0] != app_state.areaId)
    {
      /* shifting buffer values to right */
      for (uint8_t i = (6 - 1); i > 0; i--)
      {
        areaIdBuf[i] = areaIdBuf[i - 1];
      }

      /* Fresh value */
      areaIdBuf[0] = app_state.areaId;

#if 0
      if ((areaIdBuf[5] == 6u) && (areaIdBuf[4] == 5u) && (areaIdBuf[3] == 4u) && (areaIdBuf[2] == 3u) && (areaIdBuf[1] == 2u) && (areaIdBuf[0] == 1u))
#endif
#if 0
      if ((areaIdBuf[3] == (areaIdBuf[2]+1)) && (areaIdBuf[2] == (areaIdBuf[1]+1)) && (areaIdBuf[1] == (areaIdBuf[0]+1)))
#endif
  if ( (areaIdBuf[2] > areaIdBuf[1]) && (areaIdBuf[1] > areaIdBuf[0] && check_init_areas()) /*&& (areaIdBuf[1] != 11u || areaIdBuf[1] != 10u)*/ )
      {
      #if 0
        if (app_state.insertedInside)
        {
        #endif
        #if 0
          if (app_state.locked)
          {
            app_state.lockedTwice = true;
          }
          #endif

          app_state.locked = false;
          init_areas();
          #if 0
        }
        else
        {
        #endif
        #if 0
          if (app_state.lockedTwice)
          {
            app_state.lockedTwice = false;
          }
          else
          {
            app_state.locked = false;
          }
          #endif
          #if 0
        }
        #endif
      }

#if 0
      if ((areaIdBuf[5] == 1u) && (areaIdBuf[4] == 2u) && (areaIdBuf[3] == 3u) && (areaIdBuf[2] == 4u) && (areaIdBuf[1] == 5u) && (areaIdBuf[0] == 6u))
#endif
#if 0
      if ((areaIdBuf[3] == (areaIdBuf[2]-1)) && (areaIdBuf[2] == (areaIdBuf[1]-1)) && (areaIdBuf[1] == (areaIdBuf[0]-1)))
#endif
      if ( (areaIdBuf[3] < areaIdBuf[2]) && (areaIdBuf[2] < areaIdBuf[1]) && check_init_areas() /*&& (areaIdBuf[1] != 0u || areaIdBuf[1] != 1u)*/ )
      {
      #if 0
        if (app_state.insertedInside)
        {
        #endif
        #if 0
          if (app_state.lockedTwice)
          {
            app_state.lockedTwice = false;
          }
          else
          {
            app_state.locked = false;
          }
          #endif
          #if 0
        }
        else
        {
        #endif
          if (app_state.locked)
          {
            app_state.lockedTwice = true;
          }

          app_state.locked = true;
          init_areas();
        }
        #if 0
      }
      #endif
    }
  }
}

/** @} */

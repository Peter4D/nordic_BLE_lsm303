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

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

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


/* Buffer for samples read from accelerometer sensor. */

APP_TIMER_DEF(app_tmr1_id);

void app_tmr1_id_handler(void* p_context);

#define LED_RED BSP_LED_0

#ifdef BSP_BUTTON_0
    #define PIN_IN BSP_BUTTON_0
#endif
#ifndef PIN_IN
    #error "Please indicate input pin"
#endif

#ifdef LED_RED
    #define PIN_OUT LED_RED
#endif

#ifndef PIN_OUT
    #error "Please indicate output pin"
#endif

#ifndef PIN_OUT
    #error "Please indicate output pin"
#endif

#ifndef LED_RED
#define LED_RED BSP_BOARD_LED_0
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
    nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_RAW_CONFIG_IN_SENSE_HITOLO(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrfx_gpiote_in_init(PIN_IN, &in_config, in_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrfx_gpiote_in_event_enable(PIN_IN, true);
}


void app_tmr1_id_handler(void* p_context) {
    static uint32_t heart_beat = 0;
    

    //LSM303_Accel.update();
    
    read_accel();
    //read_mag();

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

    NRF_LOG_INFO("\r\nTWI sensor example started nRF52805. x1");
    NRF_LOG_FLUSH();
    

    /* Configure board. */
    bsp_board_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS);

    //twi_init();
    //lsm303_setup();
    lfclk_request();
    APP_ERROR_CHECK(app_timer_init());
    gpio_init();

    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);

    //twi_config();
    
    app_timer_create(&app_tmr1_id,
                        APP_TIMER_MODE_REPEATED,
                        app_tmr1_id_handler);
    
    volatile uint32_t periode = APP_TIMER_TICKS(1000);
    APP_ERROR_CHECK(app_timer_start(app_tmr1_id, periode, NULL));

    twi_config();
    lsm303_accel_setup();
    //LSM303_Accel.quick_setup();

    while (true)
    {
        //__WFE();
        nrf_pwr_mgmt_run();
    }
}

/** @} */

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
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

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

typedef enum {                               // DEFAULT    TYPE
    LSM303_REGISTER_ACCEL_WHO_AM_I = 0x0F,     // 00000111   rw
    LSM303_REGISTER_ACCEL_CTRL_REG1_A = 0x20,  // 00000111   rw
    LSM303_REGISTER_ACCEL_CTRL_REG2_A = 0x21,  // 00000000   rw
    LSM303_REGISTER_ACCEL_CTRL_REG3_A = 0x22,  // 00000000   rw
    LSM303_REGISTER_ACCEL_CTRL_REG4_A = 0x23,  // 00000000   rw
    LSM303_REGISTER_ACCEL_CTRL_REG5_A = 0x24,  // 00000000   rw
    LSM303_REGISTER_ACCEL_CTRL_REG6_A = 0x25,  // 00000000   rw
    LSM303_REGISTER_ACCEL_REFERENCE_A = 0x26,  // 00000000   r
    LSM303_REGISTER_ACCEL_STATUS_REG_A = 0x27, // 00000000   r
    LSM303_REGISTER_ACCEL_OUT_X_L_A = 0x28,
    LSM303_REGISTER_ACCEL_OUT_X_H_A = 0x29,
    LSM303_REGISTER_ACCEL_OUT_Y_L_A = 0x2A,
    LSM303_REGISTER_ACCEL_OUT_Y_H_A = 0x2B,
    LSM303_REGISTER_ACCEL_OUT_Z_L_A = 0x2C,
    LSM303_REGISTER_ACCEL_OUT_Z_H_A = 0x2D,
    LSM303_REGISTER_ACCEL_FIFO_CTRL_REG_A = 0x2E,
    LSM303_REGISTER_ACCEL_FIFO_SRC_REG_A = 0x2F,
    LSM303_REGISTER_ACCEL_INT1_CFG_A = 0x30,
    LSM303_REGISTER_ACCEL_INT1_SOURCE_A = 0x31,
    LSM303_REGISTER_ACCEL_INT1_THS_A = 0x32,
    LSM303_REGISTER_ACCEL_INT1_DURATION_A = 0x33,
    LSM303_REGISTER_ACCEL_INT2_CFG_A = 0x34,
    LSM303_REGISTER_ACCEL_INT2_SOURCE_A = 0x35,
    LSM303_REGISTER_ACCEL_INT2_THS_A = 0x36,
    LSM303_REGISTER_ACCEL_INT2_DURATION_A = 0x37,
    LSM303_REGISTER_ACCEL_CLICK_CFG_A = 0x38,
    LSM303_REGISTER_ACCEL_CLICK_SRC_A = 0x39,
    LSM303_REGISTER_ACCEL_CLICK_THS_A = 0x3A,
    LSM303_REGISTER_ACCEL_TIME_LIMIT_A = 0x3B,
    LSM303_REGISTER_ACCEL_TIME_LATENCY_A = 0x3C,
    LSM303_REGISTER_ACCEL_TIME_WINDOW_A = 0x3D
}lsm303_Accel_Reg_t;

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Buffer for samples read from accelerometer sensor. */
static uint8_t m_who_i_am;

/**
 * @brief Function for setting active mode on MMA7660 accelerometer.
 */
void lsm303_setup(void)
{
    ret_code_t err_code;

    /* Writing to LM75B_REG_CONF "0" set temperature sensor in NORMAL mode. */
    //uint8_t reg[2] = {LM75B_REG_CONF, NORMAL_MODE};

    /* setup lsm303 accel default */
    uint8_t reg[2] = {LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x57};

    err_code = nrf_drv_twi_tx(&m_twi, LSM303_ACCEL_ADDR, reg, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);

    /* Writing to pointer byte. */
    // reg[0] = LM75B_REG_TEMP;
    // m_xfer_done = false;
    // err_code = nrf_drv_twi_tx(&m_twi, LSM303_ACCEL_ADDR, reg, 1, false);
    // APP_ERROR_CHECK(err_code);
    // while (m_xfer_done == false);
}

/**
 * @brief Function for handling data from temperature sensor.
 *
 * @param[in] temp          Temperature in Celsius degrees read from sensor.
 */
__STATIC_INLINE void data_handler(uint8_t temp)
{
    NRF_LOG_INFO("who i am: %d.", temp);
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                data_handler(m_who_i_am);
            }
            m_xfer_done = true;
            break;
        default:
            break;
    }
}

/**
 * @brief UART initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_lsm303_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_lsm303_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

/**
 * @brief Function for reading data from temperature sensor.
 */
static void read_sensor_data()
{
    uint8_t reg[1] = {LSM303_REGISTER_ACCEL_WHO_AM_I};
    ret_code_t err_code;
    m_xfer_done = false;

    
    err_code = nrf_drv_twi_tx(&m_twi, LSM303_ACCEL_ADDR, reg, sizeof(reg), true);
    APP_ERROR_CHECK(err_code);
    while(m_xfer_done == false);
    /* Read 1 byte from the specified address - skip 3 bits dedicated for fractional part of temperature. */
    err_code = nrf_drv_twi_rx(&m_twi, LSM303_ACCEL_ADDR, &m_who_i_am, sizeof(m_who_i_am));
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for main application entry.
 */
int main(void)
{
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("\r\nTWI sensor example started.");
    NRF_LOG_FLUSH();
    twi_init();
    lsm303_setup();

    while (true)
    {
        static uint32_t heart_beat = 0;

        nrf_delay_ms(1000);
        NRF_LOG_INFO("i am alive: %d", heart_beat++);

        // if(m_xfer_done == true) {
        //     read_sensor_data();
        //     NRF_LOG_FLUSH();
        // }

        do
        {
            __WFE();
        }while (m_xfer_done == false);

        read_sensor_data();
        NRF_LOG_FLUSH();
    }
}

/** @} */

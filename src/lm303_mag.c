#include "lm303_mag.h"
//#include "lsm303_common.h"

#include <stdbool.h>
#include "boards.h"
#include "bsp.h"
#include "nrf_twi_mngr.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"




// static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND m_mag_out_reg[6];

// static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND lm303_mag_xout_reg_addr = (LSM303_REG_OUT_X_L | 0x80);


// #define LM303_READ_MAG(p_reg_addr, p_buffer, byte_cnt) \
//     NRF_TWI_MNGR_WRITE(LSM303_MAG_ADDR, p_reg_addr, 1,        NRF_TWI_MNGR_NO_STOP), \
//     NRF_TWI_MNGR_READ (LSM303_MAG_ADDR, p_buffer,   byte_cnt, 0)

// #define LM303_GET_MAG(p_buffer) \
//     LM303_READ_MAG(&lm303_mag_xout_reg_addr, p_buffer, 6)


// static void read_mag_cb(ret_code_t result, void * p_user_data) {
//     int8_t* p_axis_data = (int8_t*)p_user_data;
//     accel_data_t mag_data; 

//     if (result != NRF_SUCCESS)
//     {
//         NRF_LOG_WARNING("read_mag_cb - error: %d", (int)result);
//         return;
//     }

//     for(uint8_t i = 0; i < 6; i++) {
//         mag_data.bytes[i] = p_axis_data[i];
//     }

//     NRF_LOG_RAW_INFO(" Mag read OK x[%d] y[%d] z[%d]\r\n", 
//     mag_data.axis.x,
//     mag_data.axis.y,
//     mag_data.axis.z
//     );

// }

// void read_mag(void)
// {
//     // [these structures have to be "static" - they cannot be placed on stack
//     //  since the transaction is scheduled and these structures most likely
//     //  will be referred after this function returns]
//     static nrf_twi_mngr_transfer_t const transfers[] =
//     {
//         LM303_GET_MAG(&m_mag_out_reg[0])
//     };
//     static nrf_twi_mngr_transaction_t NRF_TWI_MNGR_BUFFER_LOC_IND transaction =
//     {
//         .callback            = read_mag_cb,
//         .p_user_data         = &m_mag_out_reg[0],
//         .p_transfers         = transfers,
//         .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
//     };

//     APP_ERROR_CHECK(nrf_twi_mngr_schedule(&m_nrf_twi_mngr, &transaction));

//     // Signal on LED that something is going on.
//     bsp_board_led_invert(BSP_BOARD_LED_1);
// }
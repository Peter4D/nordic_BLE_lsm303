#include "lsm303_drv.h"
#include "lm303_accel.h"
#include "lm303_mag.h"

#include <math.h>
#define PI 3.141592654



/* TWI instance. */
nrfx_twi_t m_twi = NRFX_TWI_INSTANCE(TWI_INSTANCE_ID);

#define TWI_INSTANCE_ID             0
#define MAX_PENDING_TRANSACTIONS    10

NRF_TWI_MNGR_DEF(m_nrf_twi_mngr, MAX_PENDING_TRANSACTIONS, TWI_INSTANCE_ID);



void twi_config(void)
{
    uint32_t err_code;

    nrf_drv_twi_config_t const config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_LOWEST,
       .clear_bus_init     = false
    };

    err_code = nrf_twi_mngr_init(&m_nrf_twi_mngr, &config);
    APP_ERROR_CHECK(err_code);
}



// Set Active mode.
static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND default_config[] = {LSM303_REG_ACCEL_CTRL_1, 0x57};

nrf_twi_mngr_transfer_t const lsm303_accel_init_transfers[LSM303_ACCEL_INIT_TRANSFER_COUNT] =
{
    NRF_TWI_MNGR_WRITE(LSM303_ACCEL_ADDR, default_config, sizeof(default_config), 0)
};

void lsm303_accel_setup(void) {
    volatile ret_code_t err_code;

    //twi_config();


    err_code = nrf_twi_mngr_perform(&m_nrf_twi_mngr, NULL, lsm303_accel_init_transfers, \
        LSM303_ACCEL_INIT_TRANSFER_COUNT, NULL);

    if(err_code != NRF_SUCCESS) {
        /* sensor initialization fail */
        NRF_LOG_RAW_INFO("\r\nTWI sensor init fail. \r\n");
        NRF_LOG_FLUSH();
        //bsp_board_led_invert(BSP_BOARD_LED_0);

        APP_ERROR_CHECK(err_code);
    }
}


static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND m_accel_out_reg[6];
static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND lm303_accel_xout_reg_addr = (LSM303_REG_ACCEL_OUT_X_L | 0x80);

#define LM303_READ(p_reg_addr, p_buffer, byte_cnt) \
    NRF_TWI_MNGR_WRITE(LSM303_ACCEL_ADDR, p_reg_addr, 1,        NRF_TWI_MNGR_NO_STOP), \
    NRF_TWI_MNGR_READ (LSM303_ACCEL_ADDR, p_buffer,   byte_cnt, 0)

#define LM303_READ_ACCEL(p_buffer) \
    LM303_READ(&lm303_accel_xout_reg_addr, p_buffer, 6)



static void read_accel_cb(ret_code_t result, void * p_user_data) {
    int8_t* p_axis_data = (int8_t*)p_user_data;
    accel_data_t accel_data; 
    float angle = 0.0;

    if (result != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("read_accle_cb - error: %d", (int)result);
        return;
    }

    for(uint8_t i = 0; i < 6; i++) {
        accel_data.bytes[i] = p_axis_data[i];
    }


 
    NRF_LOG_RAW_INFO(" accel read OK x[%d] y[%d] z[%d]\r\n", 
    accel_data.axis.x,
    accel_data.axis.y,
    accel_data.axis.z
    );

    if(accel_data.axis.x == 0) { accel_data.axis.x = 1; }
    if(accel_data.axis.y == 0) { accel_data.axis.y = 1; }
    if(accel_data.axis.z == 0) { accel_data.axis.z = 1; }

    /* calculate angle */
    angle = atan2f(accel_data.axis.z, accel_data.axis.x);
    angle = angle * 180.0/PI;
    NRF_LOG_RAW_INFO(" angle[%d]\r\n",(int16_t)angle); 
}


void read_accel(void)
{
    // [these structures have to be "static" - they cannot be placed on stack
    //  since the transaction is scheduled and these structures most likely
    //  will be referred after this function returns]
    static nrf_twi_mngr_transfer_t const transfers[] =
    {
        LM303_READ_ACCEL(&m_accel_out_reg[0])
    };
    static nrf_twi_mngr_transaction_t NRF_TWI_MNGR_BUFFER_LOC_IND transaction =
    {
        .callback            = read_accel_cb,
        .p_user_data         = &m_accel_out_reg[0],
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };

    APP_ERROR_CHECK(nrf_twi_mngr_schedule(&m_nrf_twi_mngr, &transaction));

}




/*=============================================================================*/
/*=============================================================================*/

static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND mag_default_config[] = {LSM303_REG_MAG_CFG_A, 0x18};

#define LSM303_MAG_INIT_TRANSFER_COUNT 1
nrf_twi_mngr_transfer_t const lsm303_mag_init_transfers[LSM303_MAG_INIT_TRANSFER_COUNT] =
{
    NRF_TWI_MNGR_WRITE(LSM303_MAG_ADDR, mag_default_config, sizeof(mag_default_config), 0)
};

void lsm303_mag_setup(void) {
    volatile ret_code_t err_code;

    //twi_config();


    err_code = nrf_twi_mngr_perform(&m_nrf_twi_mngr, NULL, lsm303_mag_init_transfers, \
        LSM303_MAG_INIT_TRANSFER_COUNT, NULL);

    if(err_code != NRF_SUCCESS) {
        /* sensor initialization fail */
        NRF_LOG_RAW_INFO("\r\nTWI mag sensor init fail. \r\n");
        NRF_LOG_FLUSH();
        //bsp_board_led_invert(BSP_BOARD_LED_0);

        APP_ERROR_CHECK(err_code);
    }
}



static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND m_mag_out_reg[6];

//static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND lm303_mag_xout_reg_addr = (LSM303_REG_OUT_X_L | 0x80);
static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND lm303_mag_xout_reg_addr = LSM303_REG_OUT_X_L;


#define LM303_READ_MAG(p_reg_addr, p_buffer, byte_cnt) \
    NRF_TWI_MNGR_WRITE(LSM303_MAG_ADDR, p_reg_addr, 1,        NRF_TWI_MNGR_NO_STOP), \
    NRF_TWI_MNGR_READ (LSM303_MAG_ADDR, p_buffer,   byte_cnt, 0)

#define LM303_GET_MAG(p_buffer) \
    LM303_READ_MAG(&lm303_mag_xout_reg_addr, p_buffer, 6)


static void read_mag_cb(ret_code_t result, void * p_user_data) {
    int8_t* p_axis_data = (int8_t*)p_user_data;
    accel_data_t mag_data; 

    if (result != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("read_mag_cb - error: %d", (int)result);
        return;
    }

    for(uint8_t i = 0; i < 6; i++) {
        mag_data.bytes[i] = p_axis_data[i];
    }

    NRF_LOG_RAW_INFO(" Mag read OK x[%d] y[%d] z[%d]\r\n", 
    mag_data.axis.x,
    mag_data.axis.y,
    mag_data.axis.z
    );

}

void read_mag(void)
{
    // [these structures have to be "static" - they cannot be placed on stack
    //  since the transaction is scheduled and these structures most likely
    //  will be referred after this function returns]
    static nrf_twi_mngr_transfer_t const transfers[] =
    {
        LM303_GET_MAG(&m_mag_out_reg[0])
    };
    static nrf_twi_mngr_transaction_t NRF_TWI_MNGR_BUFFER_LOC_IND transaction =
    {
        .callback            = read_mag_cb,
        .p_user_data         = &m_mag_out_reg[0],
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };

    APP_ERROR_CHECK(nrf_twi_mngr_schedule(&m_nrf_twi_mngr, &transaction));

}
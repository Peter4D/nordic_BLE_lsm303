/**
 * @file lm303_accel.c
 * @author Peter Medvesek (peter.medvesek@gorenje.com)
 * @brief  @todo THIS file is not in use .... loock lsm303_drv
 * @version 0.1
 * @date 2021-03-04
 * 
 * @copyright Copyright (c) 2021 Gorenje d.o.o
 * 
 */
#include "lm303_accel.h"
//#include "lsm303_common.h"

#include <stdbool.h>
#include "boards.h"
#include "bsp.h"
#include "nrf_twi_mngr.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include <math.h>
#define PI 3.141592654

/* Common addresses definition for temperature sensor. */
// #define LSM303_ACCEL_ADDR    (0x32 >> 1)
// //#define LSM303_MAG_ADDR      (0x3C >> 1)

// typedef enum {                               // DEFAULT    TYPE
//     LSM303_REGISTER_ACCEL_WHO_AM_I = 0x0F,     // 00000111   rw
//     LSM303_REGISTER_ACCEL_CTRL_REG1_A = 0x20,  // 00000111   rw
//     LSM303_REGISTER_ACCEL_CTRL_REG2_A = 0x21,  // 00000000   rw
//     LSM303_REGISTER_ACCEL_CTRL_REG3_A = 0x22,  // 00000000   rw
//     LSM303_REGISTER_ACCEL_CTRL_REG4_A = 0x23,  // 00000000   rw
//     LSM303_REGISTER_ACCEL_CTRL_REG5_A = 0x24,  // 00000000   rw
//     LSM303_REGISTER_ACCEL_CTRL_REG6_A = 0x25,  // 00000000   rw
//     LSM303_REGISTER_ACCEL_REFERENCE_A = 0x26,  // 00000000   r
//     LSM303_REGISTER_ACCEL_STATUS_REG_A = 0x27, // 00000000   r
//     LSM303_REG_ACCEL_OUT_X_L = 0x28,
//     LSM303_REGISTER_ACCEL_OUT_X_H_A = 0x29,
//     LSM303_REGISTER_ACCEL_OUT_Y_L_A = 0x2A,
//     LSM303_REGISTER_ACCEL_OUT_Y_H_A = 0x2B,
//     LSM303_REGISTER_ACCEL_OUT_Z_L_A = 0x2C,
//     LSM303_REGISTER_ACCEL_OUT_Z_H_A = 0x2D,
//     LSM303_REGISTER_ACCEL_FIFO_CTRL_REG_A = 0x2E,
//     LSM303_REGISTER_ACCEL_FIFO_SRC_REG_A = 0x2F,
//     LSM303_REGISTER_ACCEL_INT1_CFG_A = 0x30,
//     LSM303_REGISTER_ACCEL_INT1_SOURCE_A = 0x31,
//     LSM303_REGISTER_ACCEL_INT1_THS_A = 0x32,
//     LSM303_REGISTER_ACCEL_INT1_DURATION_A = 0x33,
//     LSM303_REGISTER_ACCEL_INT2_CFG_A = 0x34,
//     LSM303_REGISTER_ACCEL_INT2_SOURCE_A = 0x35,
//     LSM303_REGISTER_ACCEL_INT2_THS_A = 0x36,
//     LSM303_REGISTER_ACCEL_INT2_DURATION_A = 0x37,
//     LSM303_REGISTER_ACCEL_CLICK_CFG_A = 0x38,
//     LSM303_REGISTER_ACCEL_CLICK_SRC_A = 0x39,
//     LSM303_REGISTER_ACCEL_CLICK_THS_A = 0x3A,
//     LSM303_REGISTER_ACCEL_TIME_LIMIT_A = 0x3B,
//     LSM303_REGISTER_ACCEL_TIME_LATENCY_A = 0x3C,
//     LSM303_REGISTER_ACCEL_TIME_WINDOW_A = 0x3D
// }lsm303_Accel_Reg_t;

// /* TWI instance. */
// static const nrfx_twi_t m_twi = NRFX_TWI_INSTANCE(TWI_INSTANCE_ID);

// #define TWI_INSTANCE_ID             0
// #define MAX_PENDING_TRANSACTIONS    10

// NRF_TWI_MNGR_DEF(m_nrf_twi_mngr, MAX_PENDING_TRANSACTIONS, TWI_INSTANCE_ID);

static uint8_t m_who_i_am;
static volatile bool m_xfer_done = true;

typedef struct _lsm303_accel_CB_t {
    lsm303_accel_data_t accel_data;
}lsm303_accel_CB_t;

static lsm303_accel_CB_t CB;

lsm303_accel_data_t raw_data_get(void);
uint8_t who_i_am_get(void);
void quick_setup(void);
void update(void);

LSM303_Accel_methods_t LSM303_Accel = {
    .raw_data_get   = raw_data_get,
    .who_i_am_get   = who_i_am_get,
    .quick_setup    = quick_setup,
    .update         = update
};

/**
 * @brief TWI events handler.
 */
static void twi_handler(nrfx_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRFX_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRFX_TWI_XFER_RX)
            {
                
                //data_handler(m_who_i_am);
            }
            m_xfer_done = true;
            break;
        default:
            break;
    }
}


static void twi_init (void)
{
    ret_code_t err_code;

    const nrfx_twi_config_t twi_lsm303_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .hold_bus_uninit    = false
    };

    err_code = nrfx_twi_init(&m_twi, &twi_lsm303_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrfx_twi_enable(&m_twi);
}

// TWI (with transaction manager) initialization.
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


/**
 * @brief This method should be called to get new walue 
 */
void update(void) {
    uint8_t reg[1] = {LSM303_REGISTER_ACCEL_WHO_AM_I};
    volatile ret_code_t err_code = 0xFF;
    m_xfer_done = false;

    err_code = nrfx_twi_tx(&m_twi, LSM303_ACCEL_ADDR, reg, sizeof(reg), true);
    APP_ERROR_CHECK(err_code);
    while(m_xfer_done == false);
    
    err_code = nrfx_twi_rx(&m_twi, LSM303_ACCEL_ADDR, &m_who_i_am, sizeof(m_who_i_am));
    APP_ERROR_CHECK(err_code);
}

lsm303_accel_data_t raw_data_get(void) {

}


// Set Active mode.
static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND default_config[] = {LSM303_REG_ACCEL_CTRL_1, 0x57};

nrf_twi_mngr_transfer_t const mma7660_init_transfers[MMA7660_INIT_TRANSFER_COUNT] =
{
    NRF_TWI_MNGR_WRITE(MMA7660_ADDR, default_config, sizeof(default_config), 0)
};

void quick_setup(void) {
    volatile ret_code_t err_code;

    //twi_init();
    twi_config();

    /* setup lsm303 accel default */
    //static uint8_t reg[2] = {LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x57};

    // err_code = nrfx_twi_tx(&m_twi, LSM303_ACCEL_ADDR, reg, sizeof(reg), false);
    // APP_ERROR_CHECK(err_code);
    // while (m_xfer_done == false);

    // APP_ERROR_CHECK(nrf_twi_mngr_perform(&m_nrf_twi_mngr, NULL, mma7660_init_transfers, \
    //     MMA7660_INIT_TRANSFER_COUNT, NULL));

    err_code = nrf_twi_mngr_perform(&m_nrf_twi_mngr, NULL, mma7660_init_transfers, \
        MMA7660_INIT_TRANSFER_COUNT, NULL);

    if(err_code != NRF_SUCCESS) {
        /* sensor initialization fail */
        NRF_LOG_RAW_INFO("\r\nTWI sensor init fail. \r\n");
        NRF_LOG_FLUSH();
        //bsp_board_led_invert(BSP_BOARD_LED_0);

        APP_ERROR_CHECK(err_code);
    }
}

uint8_t who_i_am_get(void) {

    uint8_t reg[1] = {LSM303_REGISTER_ACCEL_WHO_AM_I};
    volatile ret_code_t err_code = 0xFF;
    //m_xfer_done = false;

    if(m_xfer_done == true) {
        err_code = nrfx_twi_tx(&m_twi, LSM303_ACCEL_ADDR, reg, sizeof(reg), true);
        APP_ERROR_CHECK(err_code);
        while(m_xfer_done == false);
        
        err_code = nrfx_twi_rx(&m_twi, LSM303_ACCEL_ADDR, &m_who_i_am, sizeof(m_who_i_am));
        APP_ERROR_CHECK(err_code);
        while(m_xfer_done == false);
    }
    return m_who_i_am;
}




static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND m_accel_out_reg[6];
static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND lm303_accel_xout_reg_addr = (LSM303_REG_ACCEL_OUT_X_L | 0x80);

#define LM303_READ(p_reg_addr, p_buffer, byte_cnt) \
    NRF_TWI_MNGR_WRITE(LSM303_ACCEL_ADDR, p_reg_addr, 1,        NRF_TWI_MNGR_NO_STOP), \
    NRF_TWI_MNGR_READ (LSM303_ACCEL_ADDR, p_buffer,   byte_cnt, 0)

#define LM303_READ_ACCEL(p_buffer) \
    LM303_READ(&lm303_accel_xout_reg_addr, p_buffer, 6)



nrf_twi_mngr_transfer_t const lsm303_accel_vib_trig_setup_transfers[] =
{
    NRF_TWI_MNGR_WRITE(LSM303_ACCEL_ADDR, default_config, sizeof(default_config), 0)
};



#define LM303_ACCEL_VIBRATION_TRIG_SETUP(p_buffer)          \
    NRF_TWI_MNGR_WRITE(LSM303_ACCEL_ADDR, p_reg_addr, 1,        NRF_TWI_MNGR_NO_STOP), \


uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND mma7660_xout_reg_addr = (LSM303_REG_ACCEL_OUT_X_L | 0x80);


/*=======================================*/

void lms303_accel_vibration_trig_setup() 
{

    /* 0b0010 1111 -> data_rate_10Hz | enable all axis;    0x57  */
    
    static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND ctrl_reg_config[] = {
        (LSM303_REG_ACCEL_CTRL_1 | 0x80), 
        0x2F, /* 0b0010 1111 -> data_rate_10Hz | enable all axis; */
        0x09, /* 0b0000 1001 -> FDS: Filtered Data Selection | HPIS1 (High Pass filter for interrupt) */
        0x40, /* 0b0100 0000 -> AOI1 interrupt on INT1 pin. */
        0x80, /* 0b1000 0000 -> output registers not updated until MSB and LSB have been read) */
        0x08, /* 0b0000 1000 -> Latch interrupt request */
        0x02  /* 0b0000 0010 -> interrupt active-low)*/
    };

    /* interrupt threshold:  x * (accel_range / 127 ) mG  @todo this formula needs to be confirmed */
    static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND int_th_config[] = {LSM303_REG_ACCEL_INT1_THS, 0x03};
    
    /* set interrupt duration 40 ms*/
    //static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND int_dur_config[] = {LSM303_REG_ACCEL_INT1_DURATION, 0x00};



    /* 0b0010 1010 ->
    b5: Enable interrupt generation on Z low event or on direction recognition.
    b3: Enable interrupt generation on Y high event or on direction recognition.
    b1: Enable interrupt generation on X high event or on direction recognition.*/
    static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND int_en_config[] = { LSM303_REG_ACCEL_INT1_CFG, 0x2A};

    nrf_twi_mngr_transfer_t const lsm303_accel_vib_trig_setup_transfers[] =
    {
        NRF_TWI_MNGR_WRITE(LSM303_ACCEL_ADDR, default_config, sizeof(default_config), 0),
        NRF_TWI_MNGR_WRITE(LSM303_ACCEL_ADDR, ctrl_reg_config, sizeof(ctrl_reg_config), 0),
        NRF_TWI_MNGR_WRITE(LSM303_ACCEL_ADDR, int_th_config, sizeof(int_th_config), 0),
        NRF_TWI_MNGR_WRITE(LSM303_ACCEL_ADDR, int_en_config, sizeof(int_en_config), 0),
    };
    /* Reading at this address clears the INT1_SRC_A (31h) -> if latched option is selected */
    //readRegister(INT1_SRC, I2C_ADDRESS);

    /* REFERENCE/DATACAPTURE_A (26h): Reference value for interrupt generation. 
    @note what is purpose of this */
    // readRegister(LSM303_REG_ACCEL_REFERENCE, I2C_ADDRESS);

    // //x,y,z
    // /* @note what is purpose of this  */

    // readRegister(LSM303_REG_ACCEL_REFERENCE, I2C_ADDRESS);
    // readRegister(INT1_SRC, I2C_ADDRESS);
    // readRegister(LSM303_REG_ACCEL_REFERENCE, I2C_ADDRESS);
}


/*=======================================*/




// Buffer for data read from sensors.
#define BUFFER_SIZE  6
static uint8_t m_buffer[BUFFER_SIZE];


typedef union _accel_data_t {
    struct {
        int16_t x;
        int16_t y;
        int16_t z;
    }axis;
    uint8_t bytes[6];
}accel_data_t;


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

    // Signal on LED that something is going on.
    bsp_board_led_invert(BSP_BOARD_LED_1);
}









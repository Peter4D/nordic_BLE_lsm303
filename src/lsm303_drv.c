#include "lsm303_drv.h"
#include "lm303_accel.h"
#include "lm303_mag.h"
#include "app_timer.h"

#include <math.h>
#define PI 3.141592654

#ifndef DEBUG_ACCEL_PRINT_OUT_EN
#define DEBUG_ACCEL_PRINT_OUT_EN 0
#endif

#ifndef DEBUG_MAG_PRINT_OUT_EN
#define DEBUG_MAG_PRINT_OUT_EN 0
#endif



/* TWI instance. */
nrfx_twi_t m_twi = NRFX_TWI_INSTANCE(TWI_INSTANCE_ID);

#define TWI_INSTANCE_ID             0
#define MAX_PENDING_TRANSACTIONS    10

NRF_TWI_MNGR_DEF(m_nrf_twi_mngr, MAX_PENDING_TRANSACTIONS, TWI_INSTANCE_ID);


static lsm303_data_t lsm303_data = {
    .accel = 0,
    .accel_rad = 0.0,
    .accel_rad_int = 0,
    .accel_angle = 0,

    .mag = 0,
    .peak_mag_x = {0},
    .peak_mag_z = {0},
    .mag_dir = 0 
};
 


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



// static float app_round(float var) 
// { 
//     float value = (int)(var * 100 + 0.5); 
//     return (float)value / 100; 
// } 

static void read_accel_cb(ret_code_t result, void * p_user_data) {
    int8_t* p_axis_data = (int8_t*)p_user_data;
    //int32_t angle_rad = 0;
    
    if (result != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("read_accle_cb - error: %d", (int)result);
        return;
    }

    for(uint8_t i = 0; i < 6; i++) {
        //accel_data.bytes[i] = p_axis_data[i];
        lsm303_data.accel.bytes[i] = p_axis_data[i];
    }

    if(lsm303_data.accel.axis.x == 0) { lsm303_data.accel.axis.x = 1; }
    if(lsm303_data.accel.axis.y == 0) { lsm303_data.accel.axis.y = 1; }
    if(lsm303_data.accel.axis.z == 0) { lsm303_data.accel.axis.z = 1; }

    /* calculate angle */
    lsm303_data.accel_rad = atan2f(lsm303_data.accel.axis.z, lsm303_data.accel.axis.x) + PI;
    lsm303_data.accel_angle = lsm303_data.accel_rad * 180.0/PI;

    #if (DEBUG_ACCEL_PRINT_OUT_EN == 1)
    lsm303_data.accel_rad_int = (int16_t)( lsm303_data.accel_rad * 100 + 0.5 ); 

    NRF_LOG_RAW_INFO("Accel x[%d] y[%d] z[%d] angle[%d] rad[%d.%d]\r\n", 
    lsm303_data.accel.axis.x,
    lsm303_data.accel.axis.y,
    lsm303_data.accel.axis.z,
    lsm303_data.accel_angle,
    lsm303_data.accel_rad_int / 100,
    lsm303_data.accel_rad_int % 100
    );
    #endif
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


void lms303_accel_vibration_trig_setup(void) 
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

    static nrf_twi_mngr_transfer_t const lsm303_accel_vib_trig_setup_transfers[] =
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

   
    static nrf_twi_mngr_transaction_t NRF_TWI_MNGR_BUFFER_LOC_IND transaction =
    {
        .callback            = NULL,
        .p_user_data         = NULL,
        .p_transfers         = lsm303_accel_vib_trig_setup_transfers,
        .number_of_transfers = sizeof(lsm303_accel_vib_trig_setup_transfers) / sizeof(lsm303_accel_vib_trig_setup_transfers[0])
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


void axis_peak_detect_process(void) {
    static uint32_t event_cnt = 0;

    if(lsm303_data.peak_mag_x.peak_detected_F == 1 && lsm303_data.peak_mag_z.peak_detected_F == 1){
        if(lsm303_data.peak_mag_z.time < lsm303_data.peak_mag_x.time) {
            lsm303_data.mag_dir = 1;
        }else {
            lsm303_data.mag_dir = 0;
        }

        NRF_LOG_RAW_INFO("\n\rcnt(%d) MAG_dir[%d]. P&tm: x[%d/%d] z[%d/%d] \r\n",
        event_cnt++, 
        lsm303_data.mag_dir,
        lsm303_data.peak_mag_x.value,
        lsm303_data.peak_mag_x.time,

        lsm303_data.peak_mag_z.value,
        lsm303_data.peak_mag_z.time
        );

        lsm303_data.peak_mag_x.peak_detected_F = 0;
        lsm303_data.peak_mag_x.value = 0;
        
        lsm303_data.peak_mag_z.peak_detected_F = 0;
        lsm303_data.peak_mag_z.value = 0;
    }
}


#define AXIS_LOW_TH     6000
#define HYSTERYSIS      2000
static void axis_peak_detect(int16_t axis_val, axis_peak_detect_t* p_axis_peak) {
    

    if(axis_val < 0) {
        p_axis_peak->neg_val_F = 1;
    }

    axis_val = abs(axis_val);
    if(axis_val > AXIS_LOW_TH) {
        if(axis_val > p_axis_peak->value) {
            p_axis_peak->peak_detected_F = 1;
            p_axis_peak->value = axis_val;
            p_axis_peak->time = app_timer_cnt_get();
        }
    }else if(axis_val < AXIS_LOW_TH - HYSTERYSIS){
        if(p_axis_peak->peak_detected_F == 1) {
            /* peak detected do procesing */
            axis_peak_detect_process();
        }
    }
}


static void read_mag_cb(ret_code_t result, void * p_user_data) {
    int8_t* p_axis_data = (int8_t*)p_user_data;
    //axis_data_t mag_data; 

    if (result != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("read_mag_cb - error: %d", (int)result);
        return;
    }

    for(uint8_t i = 0; i < 6; i++) {
        //mag_data.bytes[i] = p_axis_data[i];
        lsm303_data.mag.bytes[i] = p_axis_data[i];
    }

    /* peak detect */
    axis_peak_detect(lsm303_data.mag.axis.x, &lsm303_data.peak_mag_x);
    axis_peak_detect(lsm303_data.mag.axis.z, &lsm303_data.peak_mag_z);

    #if (DEBUG_MAG_PRINT_OUT_EN == 1)
    NRF_LOG_RAW_INFO("Mag x[%d] y[%d] z[%d]\r\n", 
    lsm303_data.mag.axis.x,
    lsm303_data.mag.axis.y,
    lsm303_data.mag.axis.z
    );
    #endif

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


/*=============================================================================*/

lsm303_data_t* lsm303_data_p_get(void) {
    return &lsm303_data;
}


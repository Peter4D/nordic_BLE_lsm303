#include "lsm303_drv.h"
#include "app_timer.h"

#include <math.h>
#define PI (float)3.141592654

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


#define LSM303_ACCEL_INIT_TRANSFER_COUNT 1
// Set Active mode.
/* enable only x and z axis for accelerometer */
static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND default_config[] = {LSM303_REG_ACCEL_CTRL_1, 0x55};

static nrf_twi_mngr_transfer_t const lsm303_accel_init_transfers[LSM303_ACCEL_INIT_TRANSFER_COUNT] =
{
    NRF_TWI_MNGR_WRITE(LSM303_ACCEL_ADDR, default_config, sizeof(default_config), 0)
};

static lsm303_data_2_t lsm303_data = {
    .peak_mag_x = AXIS_PEAK_DETECT_INIT("x"),
    .peak_mag_z = AXIS_PEAK_DETECT_INIT("z")
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



void lsm303_accel_setup(void) {
    
    volatile ret_code_t err_code;

    
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
    
    if (result != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("read_accle_cb - error: %d", (int)result);
        return;
    }

    for(uint8_t i = 0; i < 6; i++) {
        lsm303_data.accel.axis.bytes[i] = p_axis_data[i];
    }

    if(lsm303_data.accel.axis.bit.x == 0) { lsm303_data.accel.axis.bit.x = 1; }
    if(lsm303_data.accel.axis.bit.y == 0) { lsm303_data.accel.axis.bit.y = 1; }
    if(lsm303_data.accel.axis.bit.z == 0) { lsm303_data.accel.axis.bit.z = 1; }
    

    /* calculate angle */
    lsm303_data.accel.rad = atan2f(lsm303_data.accel.axis.bit.z, lsm303_data.accel.axis.bit.x) + PI;
    lsm303_data.accel.angle = lsm303_data.accel.rad * (float)180.0/PI;

    #if (DEBUG_ACCEL_PRINT_OUT_EN == 1)
    lsm303_data.accel.rad_int = (int16_t)( lsm303_data.accel.rad * 100 + 0.5 ); 

    NRF_LOG_RAW_INFO("Accel x[%d] y[%d] z[%d] angle[%d] rad[%d.%d]\r\n", 
    lsm303_data.accel.axis.bit.x,
    lsm303_data.accel.axis.bit.y,
    lsm303_data.accel.axis.bit.z,
    lsm303_data.accel.angle,
    lsm303_data.accel.rad_int / 100,
    lsm303_data.accel.rad_int % 100
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

void lsm303_read_reg(uint8_t* const p_reg_addr, uint8_t* p_data, size_t size,
                    void (*read_end_cb)(ret_code_t result, void * p_user_data)) 
{
    
    ASSERT(p_data != NULL);
    ASSERT(read_end_cb != NULL);

    static nrf_twi_mngr_transfer_t i2c_transfer_w;
    static nrf_twi_mngr_transfer_t i2c_transfer_r;

    i2c_transfer_w.operation = NRF_TWI_MNGR_WRITE_OP(LSM303_ACCEL_ADDR);
    i2c_transfer_w.p_data = p_reg_addr;
    i2c_transfer_w.length = 1;
    i2c_transfer_w.flags = NRF_TWI_MNGR_NO_STOP;

    i2c_transfer_r.operation = NRF_TWI_MNGR_READ_OP(LSM303_ACCEL_ADDR);
    i2c_transfer_r.p_data = p_data;
    i2c_transfer_r.length = size;
    i2c_transfer_r.flags = 0;

    static nrf_twi_mngr_transfer_t transfers[2];

    transfers[0] = i2c_transfer_w;
    transfers[1] = i2c_transfer_r;

    static nrf_twi_mngr_transaction_t NRF_TWI_MNGR_BUFFER_LOC_IND transaction =
    {
        .p_transfers         = transfers,
        .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])
    };

    transaction.p_user_data = (void*)p_data;
    transaction.callback = read_end_cb;

    APP_ERROR_CHECK(nrf_twi_mngr_schedule(&m_nrf_twi_mngr, &transaction));
}



void lms303_accel_vibration_trig_setup(void) 
{
    ret_code_t err_code;

    //static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND CTRL_1_cfg[] = {LSM303_REG_ACCEL_CTRL_1, 0x2F};
    static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND CTRL_1_cfg[] = {LSM303_REG_ACCEL_CTRL_1, 0x55};
    //static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND CTRL_2_cfg[] = {LSM303_REG_ACCEL_CTRL_2, 0x09};
    static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND CTRL_2_cfg[] = {LSM303_REG_ACCEL_CTRL_2, 0x01};
    static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND CTRL_3_cfg[] = {LSM303_REG_ACCEL_CTRL_3, 0x40};
    static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND CTRL_4_cfg[] = {LSM303_REG_ACCEL_CTRL_4, 0x80};
    static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND CTRL_5_cfg[] = {LSM303_REG_ACCEL_CTRL_5, 0x08};
    //static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND CTRL_5_cfg[] = {LSM303_REG_ACCEL_CTRL_5, 0x88};
    static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND CTRL_6_cfg[] = {LSM303_REG_ACCEL_CTRL_6, 0x02};

    /* interrupt threshold:  x * (accel_range / 127 ) mG  @todo this formula needs to be confirmed */
    //static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND int_th_config[] = {LSM303_REG_ACCEL_INT1_THS, 0x03};
    static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND int_th_config[] = {LSM303_REG_ACCEL_INT1_THS, 0x37};
    
    /* set interrupt duration 40 ms*/
    //static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND int_dur_config[] = {LSM303_REG_ACCEL_INT1_DURATION, 0x00};



    /* 0b0010 1010 ->
    b5: Enable interrupt generation on Z low event or on direction recognition.
    b3: Enable interrupt generation on Y high event or on direction recognition.
    b1: Enable interrupt generation on X high event or on direction recognition.*/
    //static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND int_en_config[] = { LSM303_REG_ACCEL_INT1_CFG, 0x2A};
    static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND int_en_config[] = { LSM303_REG_ACCEL_INT1_CFG, 0x20};

    static nrf_twi_mngr_transfer_t const lsm303_accel_vib_trig_setup_transfers[] =
    {
        //NRF_TWI_MNGR_WRITE(LSM303_ACCEL_ADDR, CTRL_5_cfg, sizeof(CTRL_5_cfg), 0),
        NRF_TWI_MNGR_WRITE(LSM303_ACCEL_ADDR, default_config, sizeof(default_config), 0),
        NRF_TWI_MNGR_WRITE(LSM303_ACCEL_ADDR, CTRL_1_cfg, sizeof(CTRL_1_cfg), 0),
        NRF_TWI_MNGR_WRITE(LSM303_ACCEL_ADDR, CTRL_2_cfg, sizeof(CTRL_2_cfg), 0),
        NRF_TWI_MNGR_WRITE(LSM303_ACCEL_ADDR, CTRL_3_cfg, sizeof(CTRL_3_cfg), 0),
        NRF_TWI_MNGR_WRITE(LSM303_ACCEL_ADDR, CTRL_4_cfg, sizeof(CTRL_4_cfg), 0),
        NRF_TWI_MNGR_WRITE(LSM303_ACCEL_ADDR, CTRL_5_cfg, sizeof(CTRL_5_cfg), 0),
        NRF_TWI_MNGR_WRITE(LSM303_ACCEL_ADDR, CTRL_6_cfg, sizeof(CTRL_6_cfg), 0),
        
        NRF_TWI_MNGR_WRITE(LSM303_ACCEL_ADDR, int_th_config, sizeof(int_th_config), 0),
        NRF_TWI_MNGR_WRITE(LSM303_ACCEL_ADDR, int_en_config, sizeof(int_en_config), 0),
    };


    static nrf_twi_mngr_transaction_t NRF_TWI_MNGR_BUFFER_LOC_IND transaction =
    {
        .callback            = NULL,
        .p_user_data         = NULL,
        .p_transfers         = lsm303_accel_vib_trig_setup_transfers,
        .number_of_transfers = sizeof(lsm303_accel_vib_trig_setup_transfers) / sizeof(lsm303_accel_vib_trig_setup_transfers[0])
    };

    //APP_ERROR_CHECK(nrf_twi_mngr_schedule(&m_nrf_twi_mngr, &transaction));
    //err_code = nrf_twi_mngr_schedule(&m_nrf_twi_mngr, &transaction));

    //err_code = nrf_twi_mngr_perform(&m_nrf_twi_mngr, NULL, lsm303_accel_vib_trig_setup_transfers, 9, NULL);
    err_code = nrf_twi_mngr_perform(&m_nrf_twi_mngr, NULL, lsm303_accel_vib_trig_setup_transfers, ARRAY_SIZE(lsm303_accel_vib_trig_setup_transfers), NULL);
    //err_code = nrf_twi_mngr_perform(&m_nrf_twi_mngr, NULL, lsm303_accel_read_back_transfers, 2, NULL);
    APP_ERROR_CHECK(err_code);

    lms303_accel_int_en();
}

void lms303_accel_int_en(void) {
    ret_code_t err_code;

    //static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND int_en[] = { LSM303_REG_ACCEL_INT1_CFG, 0x2A};
    static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND int_en[] = { LSM303_REG_ACCEL_INT1_CFG, 0x08};

    static nrf_twi_mngr_transfer_t const lsm303_accel_int_en[] =
    {
        NRF_TWI_MNGR_WRITE(LSM303_ACCEL_ADDR, int_en, sizeof(int_en), 0),
    };

    err_code = nrf_twi_mngr_perform(&m_nrf_twi_mngr, NULL, lsm303_accel_int_en, ARRAY_SIZE(lsm303_accel_int_en), NULL);
    APP_ERROR_CHECK(err_code);

}

void lms303_accel_int_disable(void) {
    ret_code_t err_code;

    static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND int_disable[] = { LSM303_REG_ACCEL_INT1_CFG, 0x00};

    static nrf_twi_mngr_transfer_t const lsm303_accel_int_disable[] =
    {
        NRF_TWI_MNGR_WRITE(LSM303_ACCEL_ADDR, int_disable, sizeof(int_disable), 0),
    };

    err_code = nrf_twi_mngr_perform(&m_nrf_twi_mngr, NULL, lsm303_accel_int_disable, ARRAY_SIZE(lsm303_accel_int_disable), NULL);
    APP_ERROR_CHECK(err_code);
}

static uint8_t m_read_back_reg[6];
static uint8_t NRF_TWI_MNGR_BUFFER_LOC_IND lm303_accel_ctrl_addr = (LSM303_REG_ACCEL_CTRL_1 | 0x80);

#define LM303_READ(p_reg_addr, p_buffer, byte_cnt) \
    NRF_TWI_MNGR_WRITE(LSM303_ACCEL_ADDR, p_reg_addr, 1, NRF_TWI_MNGR_NO_STOP), \
    NRF_TWI_MNGR_READ (LSM303_ACCEL_ADDR, p_buffer,   byte_cnt, 0)

#define LM303_READ_BACK(p_buffer) \
    LM303_READ(&lm303_accel_ctrl_addr, p_buffer, 6)

void lsm303_setup_read_back_check(void) {
    volatile ret_code_t err_code;

    static nrf_twi_mngr_transfer_t const lsm303_accel_read_back_transfers[] =
    {
        LM303_READ_BACK(&m_read_back_reg[0])
    };

    err_code = nrf_twi_mngr_perform(&m_nrf_twi_mngr, NULL, lsm303_accel_read_back_transfers, 2, NULL);

    APP_ERROR_CHECK(err_code);
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


/* deprecated */
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

static void axis_peak_detect_process_2(axis_peak_detect_t* p_axis_peak) {
    static uint32_t event_cnt = 0;
    

    NRF_LOG_RAW_INFO("\n\rcnt(%d) (%s) a[%d]. P&tm: [%d/%d] \r\n",
    event_cnt++,
    p_axis_peak->p_name,
    p_axis_peak->angle,
    p_axis_peak->value,
    p_axis_peak->time
    );

    p_axis_peak->value = 0;
    p_axis_peak->time = 0;
    p_axis_peak->peak_detected_F = 0;
}

#define AXIS_NOISE_FLOOR_TH     6000
#define HYSTERYSIS      2000
// static void axis_peak_detect(int16_t axis_val, axis_peak_detect_t* p_axis_peak) {
    
//     lsm303_data_2_t* p_lsm303_data = lsm303_data_p_get();

//     if(axis_val < 0) {
//         p_axis_peak->neg_val_F = 1;
//     }

//     axis_val = abs(axis_val);
//     if(axis_val > AXIS_NOISE_FLOOR_TH) {
//         if(axis_val > p_axis_peak->value) {
//             p_axis_peak->peak_detected_F = 1;
//             p_axis_peak->value = axis_val;
//             p_axis_peak->time = app_timer_cnt_get();
//             p_axis_peak->angle = p_lsm303_data->accel.angle;
//         }
//     }else if(axis_val < AXIS_NOISE_FLOOR_TH - HYSTERYSIS){
//         if(p_axis_peak->peak_detected_F == 1) {
//             /* peak detected do procesing */
//             //axis_peak_detect_process();
//             axis_peak_detect_process_2(p_axis_peak);
//         }
//     }
// }

static uint8_t number_sign_get(int32_t number) {
    if(number < 0) {
        return 1;
    }else{
        return 0;
    }
}

static void axis_peak_detect_2(int16_t cur_axis_val, int16_t* p_peak_axis_val) {

    uint16_t axis_cur_val_abs = abs(cur_axis_val);
    uint16_t axis_peak_val_abs = abs(*p_peak_axis_val);

    if(axis_cur_val_abs > AXIS_NOISE_FLOOR_TH) {

        if(number_sign_get(cur_axis_val) != number_sign_get(*p_peak_axis_val)) {
            *p_peak_axis_val = 0;
        }

        if(axis_cur_val_abs > axis_peak_val_abs) {

            *p_peak_axis_val = axis_cur_val_abs;
            if(cur_axis_val < 0) {
                *p_peak_axis_val *= -1;
            }
        }
    }
}


#define MAG_X_TH    5000
#define MAG_X_HYST  500

#define MAG_Z_TH  6000
#define MAG_Z_HYST  500

static void signal_condition_mag_x(mag_t* p_mag_data) {
    int16_t x = p_mag_data->axis.bit.x;
    //int16_t x = abs(p_mag_data->axis.bit.x);

    if(x > MAG_X_TH) {
        p_mag_data->qd.bit.a = 1;
    }else if(x < (MAG_X_TH - MAG_X_HYST) ) {
        p_mag_data->qd.bit.a = 0;
    }
}

static void signal_condition_mag_z(mag_t* p_mag_data) {
    int16_t z = p_mag_data->axis.bit.z;
    //int16_t z = abs(p_mag_data->axis.bit.z);

    if(z > MAG_Z_TH) {
        p_mag_data->qd.bit.b = 1;
    }else if(z < (MAG_Z_TH - MAG_Z_HYST) ) {
        p_mag_data->qd.bit.b = 0;
    }
}

static void quadrature_sig_decode(mag_t* p_mag_data) {
    static uint8_t qd_old = 0;
    static const int8_t qd_lut[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

    if(p_mag_data->qd.byte != qd_old) {
        /* check direction */
        p_mag_data->qd_cnt += qd_lut[p_mag_data->qd.byte];
        
        if(qd_lut[p_mag_data->qd.byte] != 0) {
            p_mag_data->qd_dir = qd_lut[p_mag_data->qd.byte];
        }

        p_mag_data->qd.bit.a_old = p_mag_data->qd.bit.a;
        p_mag_data->qd.bit.b_old = p_mag_data->qd.bit.b;
        
        qd_old = p_mag_data->qd.byte;
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
        lsm303_data.mag.axis.bytes[i] = p_axis_data[i];
    }

    /* peak detect */
    // axis_peak_detect(lsm303_data.mag.axis.bit.x, &lsm303_data.peak_mag_x);
    // axis_peak_detect(lsm303_data.mag.axis.bit.z, &lsm303_data.peak_mag_z);

    axis_peak_detect_2(lsm303_data.mag.axis.bit.x, &lsm303_data.mag.axis_peak.bit.x);
    axis_peak_detect_2(lsm303_data.mag.axis.bit.y, &lsm303_data.mag.axis_peak.bit.y);
    axis_peak_detect_2(lsm303_data.mag.axis.bit.z, &lsm303_data.mag.axis_peak.bit.z);

    signal_condition_mag_x(&lsm303_data.mag);
    signal_condition_mag_z(&lsm303_data.mag);

    quadrature_sig_decode(&lsm303_data.mag);

    #if (DEBUG_MAG_PRINT_OUT_EN == 1)
    NRF_LOG_RAW_INFO("Mag x[%d] y[%d] z[%d]\r\n", 
    lsm303_data.mag.axis.bit.x,
    lsm303_data.mag.axis.bit.y,
    lsm303_data.mag.axis.bit.z
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

lsm303_data_2_t* lsm303_data_p_get(void) {
    return &lsm303_data;
}


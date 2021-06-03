// https://www.st.com/resource/en/datasheet/lsm303agr.pdf
#ifndef LSM303ARG_REGS_H
#define LSM303ARG_REGS_H

typedef union{
  struct{
    uint8_t Xen  : 1; // X-axis enable
    uint8_t Yen  : 1; // Y-axis enable
    uint8_t Zen  : 1; // Z-axis enable
    uint8_t LPen : 1; //Low-power mode enable 
    uint8_t odr  : 4; // Data rate selection
  };
  uint8_t byte;
} ctrl_reg1_a_t;

typedef union{
  struct{
    uint8_t hpis1    : 1; // High-pass filter enable for AOI function on interrupt 1
    uint8_t hpis2    : 1; // High-pass filter enable for AOI function on interrupt 2
    uint8_t hpclick  : 1; // High-pass filter enable for CLICK function
    uint8_t fds      : 1; // Filtered data selection
    uint8_t hpcf     : 2; // High-pass filter cutoff frequency selection       
    uint8_t hpm      : 2; // High-pass filter mode selection
  };
  uint8_t byte;
} ctrl_reg2_a_t;

typedef union{
  struct{
    uint8_t reserved   :1;
    uint8_t I1_OVERRUN :1; // FIFO overrun interrupt on INT1 pin
    uint8_t I1_WTM     :1; // FIFO watermark interrupt on INT1 pin
    uint8_t I1_DRDY2   :1; // DRDY2 interrupt on INT1 pin
    uint8_t I1_DRDY1   :1; // DRDY1 interrupt on INT1 pin
    uint8_t I1_AOI2    :1; // AOI2 interrupt on INT1 pin
    uint8_t I1_AOI1    :1; // AOI1 interrupt on INT1 pin
    uint8_t I1_CLICK   :1; // CLICK interrupt on INT1 pin
  };
  uint8_t byte;
} ctrl_reg3_a_t;

typedef union{
  struct{
    uint8_t spi_en     :1; // 3-wire SPI interface enable
    uint8_t self_test  :2; // Self-test enable
    uint8_t hr         :1; // Operating mode selection
    uint8_t fs         :2; // Full-scale selection
    uint8_t ble        :1; // Big/Little Endian data selection
    uint8_t bdu        :1; // Block data update
  };
  uint8_t byte;
} ctrl_reg4_a_t;

typedef union{
  struct{
    uint8_t D4D_INT2   :1; // 4D enable: 4D detection is enabled on INT2 pin when 6D bit on INT2_CFG_A (34h) is set to 1.Table 46. CTRL_REG6_A registerI2_CLICKen   I2_INT1I2_INT2BOOT_I2P2_ACT- -H_LACTIVE-Table 47. CTRL_REG6_A descriptionI2_CLICKenClick interrupt on INT2 pin. Default value: 0(0: disabled; 1: enabled)I2_INT1Interrupt 1 function enable on INT2 pin. Default value: 0(0: function disabled; 1: function enabled)I2_INT2Interrupt 2 function enable on INT2 pin. Default value: 0(0: function disabled; 1: function enabled)BOOT_I2Boot on INT2 pin enable. Default value: 0(0: disabled; 1:enabled)P2_ACTActivity interrupt enable on INT2 pin. Default value: 0.(0: disabled; 1:enabled)H_LACTIVEinterrupt active. Default value: 0.(0: interrupt active-high; 1: interrupt active-low)Table 48. REFERENCE/DATACAPTURE_A registerRef7Ref6Ref5Ref4Ref3Ref2Ref1Ref0
    uint8_t LIR_INT2   :1; // Latch interrupt request on INT2_SRC_A (35h) register, with INT2_SRC_A (35h)register cleared by reading INT2_SRC_A (35h) itself
    uint8_t D4D_INT1   :1; // 4D enable: 4D detection is enabled on INT1 pin when 6D bit on INT1_CFG_A (30h) is set to 1
    uint8_t LIR_INT1   :1; // Latch interrupt request on INT1_SRC_A (31h), with INT1_SRC_A (31h) register cleared by reading INT1_SRC_A (31h) itself
    uint8_t reserved   :2;
    uint8_t FIFO_EN    :1; // FIFO enable
    uint8_t BOOT       :1; // Reboot accelerometer memory content
  };
  uint8_t byte;
} ctrl_reg5_a_t;

typedef union{
  struct{
    uint8_t reserved_0 :1;
    uint8_t H_LACTIVE  :1; // interrupt active
    uint8_t reserved_1 :1; 
    uint8_t P2_ACT     :1; // Activity interrupt enable on INT2 pin
    uint8_t BOOT_I2    :1; // Boot on INT2 pin enable
    uint8_t I2_INT2    :1; // Interrupt 2 function enable on INT2 pin
    uint8_t I2_INT1    :1; // Interrupt 1 function enable on INT2 pin
    uint8_t I2_CLICKen :1; // Click interrupt on INT2 pin
  };
  uint8_t byte;
} ctrl_reg6_a_t;

typedef union{
  struct{
    uint8_t xl          :1; // Enable interrupt generation on Z high event or on direction recognition
    uint8_t xh          :1; // Enable interrupt generation on Z high event or on direction recognition
    uint8_t yl          :1;
    uint8_t yh          :1;
    uint8_t zl          :1;
    uint8_t zh          :1;
    uint8_t _6d         :1; // 6-direction detection function enabled
    uint8_t AOI         :1; // And/Or combination of interrupt events
  };
  uint8_t byte;
} int1_cfg_a_t;

typedef union{
  struct{
    uint8_t xl          :1; // X low
    uint8_t xh          :1; // X high
    uint8_t yl          :1;
    uint8_t yh          :1;
    uint8_t zl          :1;
    uint8_t zh          :1;
    uint8_t active      :1; // Interrupt active
  };
  uint8_t byte;
} int1_src_a_t;

typedef union{
  struct{
    uint8_t threshold   :7;
    uint8_t always_zero :1;
  };
  uint8_t byte;
} int1_ths_a_t;

#endif /* LSM303ARG_REGS_H */


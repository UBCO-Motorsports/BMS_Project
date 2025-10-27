#ifndef BQ79600_REG_H
#define BQ79600_REG_H

// Shadow registers for BQ7961x communication - directly accessible on BQ79600
#define BQ79600_CONTROL1_SHADOW   0x0309  // This is actually the BQ79616 CONTROL1 address, accessed via BQ79600
                                          // For BQ79600's own CONTROL1, use BQ79600_CONTROL1_OWN

// BQ79600 Own Registers
#define BQ79600_CONTROL1_OWN    0x0200 // BQ79600's own Control Register 1 (example, verify from datasheet)
                                       // Datasheet bq79600-q1.pdf Table 9-2 shows "CONTROL1" at 0x0309
                                       // This seems to be the shadowed version. Let's find BQ79600 specific config.
                                       // The PDF actually refers to BQ79600 CONTROL1 at 0x0309 (for its own control like soft reset)
#define BQ79600_CONTROL1        0x0309 // From BQ79600-Q1 datasheet Table 9-2
                                       // Bit 5 SEND_WAKE: Send WAKE tone to stack
                                       // Bit 2 GOTO_SLEEP
                                       // Bit 1 SOFT_RESET

#define BQ79600_DEV_CONF1       0x2001  
#define BQ79600_DEV_CONF2       0x2002  

#define BQ79600_FAULT_SUMMARY_REG 0x2100
#define BQ79600_FAULT_COMM1     0x2104  
#define BQ79600_FAULT_COMM2     0x2105 
#define BQ79600_FAULT_SYS       0x2102 // System Fault Register
#define BQ79600_FAULT_PWR       0x2103 // Power Fault Register

#define BQ79600_FAULT_RST       0x2030  

// Bit definitions for BQ79600_CONTROL1 (0x0309)
// src/bq79600_reg.h (ensure/update)
#define BQ79600_CTRL1_DIR_SEL_BIT         (1 << 7)
#define BQ79600_CTRL1_SEND_SHUTDOWN_BIT   (1 << 6)
#define BQ79600_CTRL1_SEND_WAKE_BIT       (1 << 5)
#define BQ79600_CTRL1_SEND_SLPTOACT_BIT   (1 << 4)
#define BQ79600_CTRL1_GOTO_SHUTDOWN_BIT   (1 << 3)
#define BQ79600_CTRL1_GOTO_SLEEP_BIT      (1 << 2)
#define BQ79600_CTRL1_SOFT_RESET_BIT      (1 << 1)
#define BQ79600_CTRL1_ADDR_WR_BIT         (1 << 0) // For BQ79600 to take its own address during sequence

#define BQ79600_DEV_CONF1       0x2001 // For verification step

// Bit definitions for BQ79600_DEV_CONF1 (0x2001)
#define BQ79600_DEVCFG1_SNIFDET_EN_BIT   (1 << 7)
#define BQ79600_DEVCFG1_SNIFDET_DIS_BIT  (1 << 6)
#define BQ79600_DEVCFG1_TONE_RX_EN_BIT   (1 << 5) 
#define BQ79600_DEVCFG1_FCOMM_EN_BIT     (1 << 4) 
#define BQ79600_DEVCFG1_TWO_STOP_EN_BIT  (1 << 3) 
#define BQ79600_DEVCFG1_NFAULT_EN_BIT    (1 << 2) 
#define BQ79600_DEVCFG1_HB_TX_EN_BIT     (1 << 0) 

// Bit definitions for BQ79600_DEV_CONF2 (0x2002)
#define BQ79600_DEVCFG2_COMM_MODE_RING   (0x02 << 4) 
#define BQ79600_DEVCFG2_AUTO_COMM_SEL_BIT (1 << 1)   

// Bit definitions for BQ79600_FAULT_COMM1 (0x2104)
#define BQ79600_FLTCOMM1_FCOMM_DET_BIT   (1 << 5)
#define BQ79600_FLTCOMM1_FTONE_DET_BIT   (1 << 4)
#define BQ79600_FLTCOMM1_HB_FAIL_BIT     (1 << 3)
#define BQ79600_FLTCOMM1_HB_FAST_BIT     (1 << 2)

// Bit definitions for BQ79600_FAULT_COMM2 (0x2105)
#define BQ79600_FLTCOMM2_COML_FRAME_FLT_BIT (1 << 1)
#define BQ79600_FLTCOMM2_COMH_FRAME_FLT_BIT (1 << 0)


// Bit definitions for BQ79600_FAULT_RST (0x2030)
#define BQ79600_FLTRST_RST_ALL_FLGS_BIT  (1 << 7) // RST_ALL_FLGS
#define BQ79600_FLTRST_RST_COML_H_BIT    (1 << 6) // RST_STACK_COM_FLT
#define BQ79600_FLTRST_RST_UART_SPI_BIT  (1 << 5) // RST_HOST_COM_FLT
#define BQ79600_FLTRST_RST_FCOMM_DET_BIT (1 << 4)
#define BQ79600_FLTRST_RST_FTONE_DET_BIT (1 << 3)
#define BQ79600_FLTRST_RST_HB_BIT        (1 << 2) // RST_HB_FLT
#define BQ79600_FLTRST_RST_SYSFLT_BIT    (1 << 1) // RST_SYS_FLT
#define BQ79600_FLTRST_RST_PWRFLT_BIT    (1 << 0) // RST_PWR_FLT


#endif // BQ79600_REG_H
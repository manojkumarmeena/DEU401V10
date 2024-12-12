#ifndef VL53L0XMKM_H
#define VL53L0XMKM_H
/***********************************************************************
***********************************************************************/
#include "stm32f4xx_hal.h"
#include <stdint.h>
/***********************************************************************
***********************************************************************/
#define VL53L0X_ADDRESS					 								0x52

#define REG_IDENTIFICATION_MODEL_ID							0xc0
#define REG_IDENTIFICATION_REVISION_ID					0xc2

#define REG_SYSRANGE_START											0x00

#define REG_RESULT_INTERRUPT_STATUS 						0x13
#define RESULT_RANGE_STATUS      								0x14
#define ALGO_PHASECAL_LIM                       0x30
#define ALGO_PHASECAL_CONFIG_TIMEOUT            0x30

#define GLOBAL_CONFIG_VCSEL_WIDTH               0x32
#define FINAL_RANGE_CONFIG_VALID_PHASE_LOW      0x47
#define FINAL_RANGE_CONFIG_VALID_PHASE_HIGH     0x48

#define PRE_RANGE_CONFIG_VCSEL_PERIOD           0x50
#define PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI      0x51
#define PRE_RANGE_CONFIG_VALID_PHASE_LOW        0x56
#define PRE_RANGE_CONFIG_VALID_PHASE_HIGH       0x57

#define REG_MSRC_CONFIG_CONTROL                 0x60
#define FINAL_RANGE_CONFIG_VCSEL_PERIOD         0x70
#define FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI    0x71
#define MSRC_CONFIG_TIMEOUT_MACROP              0x46
#define FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT  0x44
#define SYSRANGE_START                          0x00
#define SYSTEM_SEQUENCE_CONFIG                  0x01
#define SYSTEM_INTERRUPT_CONFIG_GPIO            0x0A
#define RESULT_INTERRUPT_STATUS                 0x13
#define VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV       0x89
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_0        0xB0
#define GPIO_HV_MUX_ACTIVE_HIGH                 0x84
#define SYSTEM_INTERRUPT_CLEAR                  0x0B
/***********************************************************************
***********************************************************************/
void Initialize_VL53L0X_Device(void);

/** Returns 0 if the distance was measured correctly, the error code otherwise. */
uint8_t vl53l0x_measure_distance(void);

void Write_VL53L0X_Reg(uint16_t regaddress,uint8_t regvalue);
uint8_t Read_VL53L0X_Reg(uint16_t regaddress);
/***********************************************************************
***********************************************************************/
#endif /* VL53L0XMKM_H */

#ifndef VL6180XMKM_H
#define VL6180XMKM_H
/***********************************************************************
***********************************************************************/
#include "stm32f4xx_hal.h"
#include <stdint.h>
/***********************************************************************
***********************************************************************/
#define VL6180X_ADDRESS					 							0x29<<1
#define VL6180X_SYSTEM_INTERRUPT_CLEAR 				0x15
#define VL6180X_SYSRANGE_START 								0x18
#define VL6180X_RESULT_RANGE_STATUS 					0x4d
#define VL6180X_RESULT_INTERRUPT_STATUS_GPIO 	0x4f
#define VL6180X_RESULT_RANGE_VAL 							0x62
/***********************************************************************
***********************************************************************/
void Initialize_VL6180X_Device(void);

/** Returns 0 if the distance was measured correctly, the error code otherwise. */
uint8_t vl6180x_measure_distance(void);

void Write_VL6180X_Reg(uint16_t regaddress,uint8_t regvalue);
uint8_t Read_VL6180X_Reg(uint16_t regaddress);

#endif /* VL6180MKM_H */

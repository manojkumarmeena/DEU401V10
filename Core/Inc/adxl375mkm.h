#ifndef __ADXL375MKM_H
#define __ADXL375MKM_H
/******************************************************************************
******************************************************************************/
#include "stm32f4xx_hal.h"
/******************************************************************************
******************************************************************************/
#define ADXL375_DEVID_REG		0x00

#define ADXL375_BWRATE_REG		0x2C
#define ADXL375_PWRCTRL_REG		0x2D
#define ADXL375_INTENBL_REG		0x2E
#define ADXL375_INTMAP_REG		0x2F
#define ADXL375_INTSRC_REG		0x30
#define ADXL375_DATAFMT_REG		0x31
#define ADXL375_FIFOCTRL_REG	0x38
#define ADXL375_FIFOSTATUS_REG	0x39

#define ADXL375_ACCLXL_REG		0x32
#define ADXL375_ACCLXM_REG		0x33
#define ADXL375_ACCLYL_REG		0x34
#define ADXL375_ACCLYM_REG		0x35
#define ADXL375_ACCLZL_REG		0x36
#define ADXL375_ACCLZM_REG		0x37

#define ADXL375_OFFSETX_REG		0x1E
#define ADXL375_OFFSETY_REG		0x1F
#define ADXL375_OFFSETZ_REG		0x20
/******************************************************************************
******************************************************************************/
uint8_t Init_Adxl375(void);
uint8_t Read_Adxl375_Id(void);
uint8_t Read_Adxl375_Reg(uint8_t regaddr);
void Read_Adxl375_Regs(uint8_t regaddr,uint8_t *rxbuf,uint8_t bytecount);
uint8_t Write_Adxl375_Reg(uint8_t regaddr,uint8_t bytevalue);
uint8_t Update_Accel_Data(void);
uint8_t Update_Adxl375_Data(void);
void Adxl375_Offset_Calculation(void);
void Adxl375_Trigger_Wait(void);
void Adxl375_Motion_Detection(void);
/******************************************************************************
******************************************************************************/
#endif	/*__ADXL375MKM_H*/

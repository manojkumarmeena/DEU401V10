#ifndef __UTILFUNCMKM_H
#define __UTILFUNCMKM_H
/******************************************************************************
******************************************************************************/
#include "stm32f4xx_hal.h"
/******************************************************************************
******************************************************************************/
extern CRC_HandleTypeDef hcrc;
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
/******************************************************************************
******************************************************************************/
void Display_8Bit_Hex(uint8_t hexval);
void Display_Binary(uint8_t hexval);
void Display_16Bit_Hex(uint16_t hexval);
void Display_32Bit_Hex(uint32_t hexval);
uint8_t Calculate_Checksum(uint8_t *databuf);
/******************************************************************************
	Developed By Manoj Kumar Meena(MKM),MAY2018 for SNSRTG-432-V10
******************************************************************************/
#endif	/*__UTILFUNCMKM_H*/

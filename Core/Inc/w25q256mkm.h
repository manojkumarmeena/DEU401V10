#ifndef __W25Q256MKM_H
#define __W25Q256MKM_H
/******************************************************************************
******************************************************************************/
#include "stm32f4xx_hal.h"
/******************************************************************************
******************************************************************************/
#define	PAGE_PROGRAM		0x02 

#define SECTOR_ERASE_4K		0x20
#define BLOCK_ERASE_32K		0x52
#define BLOCK_ERASE_64K		0xD8
#define CHIP_ERASE				0xC7		//0x60

#define READ_DATA					0x03

#define ERASE_SEC_REG			0x44
#define READ_SEC_REG			0x48
#define READ_BLOCK_LOCK		0x3D
#define GLOBAL_BLOCK_LOCK	0x7E
#define GLOBAL_BLOCK_UNLK	0x98
#define ENTER_QPI_MODE		0x38

#define WRITE_ENABLE		0x06
#define WRITE_DISABLE		0x04
#define VOLSR_WRITE_EN	0x50

#define READ_SR1_REG		0x05
#define READ_SR2_REG		0x35
#define READ_SR3_REG		0x15

#define WRITE_SR1_REG		0x01
#define WRITE_SR2_REG		0x31
#define WRITE_SR3_REG		0x11

#define READ_MANUF_ID		0x90
#define READ_UNIQUE_ID	0x4B
#define READ_SFDP_REG		0x5A
#define READ_JEDEC_ID		0x9F

#define POWER_DOWN			0xB9
#define RELEASE_PWR_DOWN	0xAB
#define ENABLE_RESET		0x66
#define RESET_DEVICE		0x99
#define ERASE_PROG_SUS		0x75
#define ERASE_PROG_RES		0x7A

#define FLASH_INFO_SECTOR	0x001000	//Sector for Inofrmation
#define FLASH_DATA_BLOCK	0xFF0000	//Block fotr Sensor Data
/******************************************************************************
******************************************************************************/
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef hspi1;
/******************************************************************************
******************************************************************************/
extern void Display_8Bit_Hex(uint8_t);
extern void Display_16Bit_Hex(uint16_t);
extern void Display_32Bit_Hex(uint32_t);
/******************************************************************************
******************************************************************************/
uint8_t Init_Flash128(void);
void Probe_Flash(void);
void Check_Busy_Flag(void);
void test_Flash(void);
void Init_Flash(void);
uint8_t Init_Flash256(void);
void Diagnose_Flash(void);
void Check_Busy_Flag(void);
void Check_WEL_Flag(void);
void Flash_Write_Enable(void);
void Flash_Write_Disable(void);
void Flash_Chip_Erase(void);
uint8_t Flash_Read_SR1(void);
uint8_t Flash_Read_SR2(void);
uint8_t Flash_Read_SR3(void);
void Flash_Reset_Device(void);
void Flash_Power_Down(void);
void Flash_Erase_Suspend(void);
void Flash_Erase_Resume(void);
void Flash_Write_SR1(uint8_t dataval);
void Flash_Write_SR2(uint8_t dataval);
void Flash_Write_SR3(uint8_t dataval);
void Send_Command(uint8_t command);
uint8_t Send_Command_Get_Response(uint8_t command);
void Flash_Block_Unlock(void);
uint8_t Flash_Release_PowerDown(void);

uint16_t Flash_Read_ManId(void);
uint8_t Flash_Read_ElecId(void);
uint32_t Flash_Jedec_Id(void);

void Flash_Sector_Erase(uint32_t);
void Flash_Block_Erase(uint32_t address);
void Flash_Write_Data(uint32_t address,uint8_t *databuf,uint16_t txcount);
void Flash_Read_Data(uint32_t memadd,uint8_t *databuf,uint16_t rxcount);
/******************************************************************************
	Developed By Manoj Kumar Meena(MKM),MAY2018 for SNSRTG-432-V10
******************************************************************************/
#endif	/*__W25Q256MKM_H*/

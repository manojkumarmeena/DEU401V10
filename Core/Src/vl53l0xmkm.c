#include "vl53l0xmkm.h"
#include "utilfuncmkm.h"
/******************************************************************************
******************************************************************************/
extern I2C_HandleTypeDef hi2c1;
/******************************************************************************
******************************************************************************/
extern uint8_t distance;
/******************************************************************************
******************************************************************************/
void Initialize_VL53L0X_Device()
{
}
/******************************************************************************
******************************************************************************/
uint8_t Read_VL53L0X_Reg(uint16_t regaddress)
{
	uint8_t txbuffer[1],rxbuffer[3];
	
	txbuffer[0] = 0xC0;
	while(HAL_I2C_IsDeviceReady(&hi2c1,VL53L0X_ADDRESS,5,100) != HAL_OK);
	HAL_I2C_Master_Transmit(&hi2c1,VL53L0X_ADDRESS,txbuffer,1,100);
	HAL_Delay(10);

	txbuffer[0] = 0x00;
	while(HAL_I2C_IsDeviceReady(&hi2c1,VL53L0X_ADDRESS,5,100) != HAL_OK);
	HAL_I2C_Master_Transmit(&hi2c1,VL53L0X_ADDRESS,txbuffer,1,100);

	HAL_I2C_Master_Receive(&hi2c1,VL53L0X_ADDRESS,rxbuffer,3,1000);

	Display_8Bit_Hex(rxbuffer[0]);
	Display_8Bit_Hex(rxbuffer[1]);
	Display_8Bit_Hex(rxbuffer[2]);
	
	return(rxbuffer[0]);
}
/******************************************************************************
******************************************************************************/
void Write_VL53L0X_Reg(uint16_t regaddress,uint8_t regvalue)
{
	uint8_t txbuffer[2];
	txbuffer[0] = regaddress;
	txbuffer[1] = regvalue;
	while(HAL_I2C_IsDeviceReady(&hi2c1,VL53L0X_ADDRESS,5,100) != HAL_OK);
	HAL_I2C_Master_Transmit(&hi2c1,VL53L0X_ADDRESS,txbuffer,2,10);
}
/*************************************************************************************
Code Designed & Developed by MKM for STG401V11 in AUG 2024
*************************************************************************************/

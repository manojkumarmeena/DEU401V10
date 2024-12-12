#include "tlv493dmkm.h"
/******************************************************************************
******************************************************************************/
extern I2C_HandleTypeDef hi2c1;
extern uint8_t magdatabuff[10];
extern float tempvalue;
/******************************************************************************
******************************************************************************/
void Initialize_TLV493D_Device()
{
	uint8_t rxbuffer[10]={0x00,0x00,0x00};
	uint8_t txbuffer[4]={0x00,0x00,0x00};
	
	Reset_TLV493D_Device();
	HAL_Delay(100);
	
	while(HAL_I2C_IsDeviceReady(&hi2c1,TLV493D_ADDRESS,5,100) != HAL_OK);
	HAL_I2C_Master_Receive(&hi2c1,TLV493D_ADDRESS,rxbuffer,10,100);

	for(int i=0;i<10;i++)	{	Display_8Bit_Hex(rxbuffer[i]);	}

	txbuffer[0] = 0x00;
	
	txbuffer[1] = rxbuffer[7] & 0x78;
	txbuffer[1] = txbuffer[1] | 0x01;
	
	txbuffer[2] = rxbuffer[8];
	
	txbuffer[3] = rxbuffer[9] & 0x1F;
	txbuffer[3] = txbuffer[3] | 0x00;

	while(HAL_I2C_IsDeviceReady(&hi2c1,TLV493D_ADDRESS,5,100) != HAL_OK);
	HAL_I2C_Master_Transmit(&hi2c1,TLV493D_ADDRESS,txbuffer,4,100);
}
/******************************************************************************
******************************************************************************/
void Reset_TLV493D_Device()
{
	uint8_t txbuff[1] = {0x00};
	while(HAL_I2C_IsDeviceReady(&hi2c1,TLV493D_ADDRESS,5,100) != HAL_OK);
	HAL_I2C_Master_Transmit(&hi2c1,TLV493D_ADDRESS,txbuff,1,100);
	HAL_Delay(10);
} 
/******************************************************************************
******************************************************************************/
void Read_TLV493D_Device()
{
	while(HAL_I2C_IsDeviceReady(&hi2c1,TLV493D_ADDRESS,5,100) != HAL_OK);
	HAL_I2C_Master_Receive(&hi2c1, TLV493D_ADDRESS,magdatabuff,10,100);
}
/***************************************************************************
***************************************************************************/
float Get_Mag_X()
{
	uint16_t magfieldx=0x00;
	float xmag=0.0;
	
	magfieldx = magdatabuff[0];
	magfieldx = (magfieldx << 4) | ((magdatabuff[4]>>4) & 0x0F);
	
	if(magfieldx > 2048)	{	magfieldx = magfieldx - 4096;	}
	xmag = magfieldx * 0.098;
	return(xmag);
}
/***************************************************************************
***************************************************************************/
float Get_Mag_Y()
{
	uint16_t magfieldy=0x00;
	float ymag=0.0;
	
	magfieldy = magdatabuff[1];
	magfieldy = (magfieldy << 4) | (magdatabuff[4] & 0x0F);
	
	if(magfieldy > 2048)	{	magfieldy = magfieldy - 4096;	}
	ymag = magfieldy * 0.098;
	return(ymag);
}
/***************************************************************************
***************************************************************************/
float Get_Mag_Z()
{
	uint16_t magfieldz=0x00;
	float zmag=0.0;
	
	magfieldz = magdatabuff[2];
	magfieldz = (magfieldz << 4) | (magdatabuff[5] & 0x0F);
	
	if(magfieldz > 2048)	{	magfieldz = magfieldz - 4096;	}
	zmag = magfieldz * 0.098;
	return(zmag);
}
/***************************************************************************
***************************************************************************/
float Get_Mag_Temp()
{
	uint16_t magtemp=0x0000;
	float tempmag=0.0;
	
	magtemp = magdatabuff[3];
	magtemp = (magtemp << 4) & 0x0F00;
	magtemp = magtemp | magdatabuff[6];
	
	if(magtemp > 2048)	{	magtemp = magtemp - 4096;	}
	tempmag = (magtemp * 1.1) - 340;
	return(tempmag);
}
/***************************************************************************
***************************************************************************/
void Diagnose_TLV493D_Device()
{
	float tempvalue1=0.0,tempvalue2=0.0;
	uint8_t showtemp[10];

	Read_TLV493D_Device();
	while((magdatabuff[5] & 0x40) != 0x00)
	{	
		Reset_TLV493D_Device();
		HAL_Delay(10);		
		Read_TLV493D_Device();
	}

	tempvalue2 = Get_Mag_X();
	sprintf((char *)showtemp,"%0.5fmT", tempvalue2);
	HAL_UART_Transmit(&huart1,"\nXMAG=",6,10);
	HAL_UART_Transmit(&huart1,showtemp,6,10);	

	tempvalue2 = Get_Mag_Y();
	sprintf((char *)showtemp,"%0.5fmT", tempvalue2);
	HAL_UART_Transmit(&huart1,"\nYMAG=",6,10);
	HAL_UART_Transmit(&huart1,showtemp,6,10);	

	tempvalue2 = Get_Mag_Z();
	sprintf((char *)showtemp,"%0.5fmT", tempvalue2);
	HAL_UART_Transmit(&huart1,"\nZMAG=",6,10);
	HAL_UART_Transmit(&huart1,showtemp,6,10);	

	tempvalue1 = tempvalue;
	tempvalue = Get_Mag_Temp();
	tempvalue = (tempvalue1 + tempvalue)/2;
	sprintf((char *)showtemp,"%0.5fC", tempvalue);
	HAL_UART_Transmit(&huart1,"\nTEMP=",6,10);
	HAL_UART_Transmit(&huart1,showtemp,7,10);	
}
/*************************************************************************************
Code Designed & Developed by MKM for STG401V11 in AUG 2024
*************************************************************************************/


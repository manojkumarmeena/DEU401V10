#include "tfminismkm.h"
/******************************************************************
******************************************************************/
void Initialize_TFMini_Device()
{
	uint8_t txbuff[10],rxbuff[7];
	txbuff[0] = (TFMini_Version_CMD & 0xFF000000)>>24;
	txbuff[1] = (TFMini_Version_CMD & 0x00FF0000)>>16;
	txbuff[2] = (TFMini_Version_CMD & 0x0000FF00)>>8;
	txbuff[3] = (TFMini_Version_CMD & 0x000000FF)>>0;
	while(rxbuff[6] == 1)
	{
		HAL_UART_Transmit(&huart1,txbuff,4,10);
		HAL_UART_Receive(&huart1,rxbuff,7,1000);
	}
	HAL_UART_Transmit(&huart1,rxbuff,7,10);

	txbuff[0] = (TFMini_RST_CMD & 0xFF000000)>>24;
	txbuff[1] = (TFMini_RST_CMD & 0x00FF0000)>>16;
	txbuff[2] = (TFMini_RST_CMD & 0x0000FF00)>>8;
	txbuff[3] = (TFMini_RST_CMD & 0x000000FF)>>0;
	while(rxbuff[6] == 1)
	{
		HAL_UART_Transmit(&huart1,txbuff,4,10);
		HAL_UART_Receive(&huart1,rxbuff,7,1000);
	}
	HAL_UART_Transmit(&huart1,rxbuff,7,10);
}
/*************************************************************************************
Raw Data
*************************************************************************************/
void Raw_Distance_Data()
{
	uint8_t rxbuff[9];
	HAL_UART_Transmit(&huart1,"\nGet Ready to Collect Data",26,100);
	while(1)
	{
		__HAL_UART_ENABLE(&huart1);
		HAL_UART_Receive(&huart1,rxbuff,9,5);
		__HAL_UART_DISABLE(&huart1);
		if((rxbuff[0] == 0x59) & (rxbuff[1] == 0x59))	
		{	HAL_UART_Transmit(&huart1,rxbuff,9,5);	rxbuff[0]=0;	rxbuff[1]=0;	}
	}
}
/*************************************************************************************
Refined Data
*************************************************************************************/
void Fine_Distance_Data()
{
	uint8_t rxbuff[9], showtemp[8], tempdata[6];
	float distdata=0.0;
	__HAL_UART_ENABLE(&huart1);
	HAL_UART_Receive(&huart1,rxbuff,9,5);
	__HAL_UART_DISABLE(&huart1);
	if((rxbuff[0] == 0x59) & (rxbuff[1] == 0x59))
	{ 
		distdata = (rxbuff[3] << 8) + rxbuff[2];
		sprintf((char *)showtemp,"%04.0f",distdata);
		HAL_UART_Transmit(&huart1,"\nDST=",5,5);
		HAL_UART_Transmit(&huart1,showtemp,4,5);		
	}
}
/*************************************************************************************
Code Designed & Developed by MKM for STG401V11 in AUG 2024
*************************************************************************************/

#include "jdy25mkm.h"
/******************************************************************************
******************************************************************************/
extern UART_HandleTypeDef huart1;
/******************************************************************************
******************************************************************************/
void Initialize_JDY25_Device()
{
	uint8_t status=0, rxbuff[20];
	
	while(status != 1)
	{
		HAL_UART_Transmit(&huart1,"AT\r\n",8,100);
		HAL_UART_Receive(&huart1,rxbuff,4,500);
		if(Buffercmp(rxbuff,"+OK",3) == 0)
		{	
			status = 1;	
			Display_System_Message(" JDY25M OK  ");	
		}
		else	{	Display_System_Message("RST ERROR");	}
		HAL_Delay(100);
	}
	for(int i=0;i<20;i++)	{	rxbuff[i] = 0;	}
	status = 0;
	
	while(status != 1)
	{
		HAL_UART_Transmit(&huart1,"AT+RESET\r\n",10,10);
		HAL_UART_Receive(&huart1,rxbuff,5,500);
		if(Buffercmp(rxbuff,"+OK\r\n",5) == 0)
		{
			status = 1;	
			Display_System_Message(" RESET  OK  ");				
		}
		else	{	Display_System_Message("RST ERROR");	}
		HAL_Delay(100);
	}
	for(int i=0;i<20;i++)	{	rxbuff[i] = 0;	}
	status = 0;

	while(status != 1)
	{
		HAL_UART_Transmit(&huart1,"AT+NAMEJDY01\r\n",14,10);
		HAL_UART_Receive(&huart1,rxbuff,11,500);
		if(Buffercmp(rxbuff,"+NAME=JDY01",11) == 0) 
		{ status = 1;	Display_System_Message("NAME SETOK");	 }
		else	{	Display_System_Message(rxbuff);	}
		HAL_Delay(100);
	}
	for(int i=0;i<20;i++)	{	rxbuff[i] = 0;	}
	status = 0;

	while(status != 1)
	{
		HAL_UART_Transmit(&huart1,"AT+ROLE0\r\n",10,10);
		HAL_UART_Receive(&huart1,rxbuff,5,500);
		if(Buffercmp(rxbuff,"OK\r\n",4) == 0) 
		{ status = 1;	Display_System_Message("ROLE SLAVE");	 }
		else	{	Display_System_Message(rxbuff);	}
	for(int i=0;i<20;i++)	{	rxbuff[i] = 0;	}
	}
	status = 0;
}
/******************************************************************************
******************************************************************************/
void Initialize_JDY23_Device()
{
	uint8_t status=0, rxbuff[20];
	
	while(status != 1)
	{
//		HAL_UART_Transmit(&huart1,"AT+NAMEMKM\r\n",12,100);
		HAL_UART_Transmit(&huart1,"AT+BAUD0\r\n",10,100);
		HAL_UART_Receive(&huart1,rxbuff,5,1000);
		if(Buffercmp(rxbuff,"+OK",3) == 0)
		{	
			status = 1;	
			Display_System_Message(" JDY25M OK  ");	
		}
		else	{	Display_System_Message("RST ERROR");	}
		HAL_Delay(100);
	}
	for(int i=0;i<20;i++)	{	rxbuff[i] = 0;	}
	status = 0;
}
/******************************************************************************
******************************************************************************/
void Reset_JDY25_Device(void)
{
	uint8_t status=0, rxbuff[6];
	while(status != 1)
	{
		HAL_UART_Transmit(&huart1,"AT+RESET\r\n",10,10);
		HAL_UART_Receive(&huart1,rxbuff,6,500);
		if(Buffercmp(rxbuff,"+OK\r\n",5) == 0)
		{
			status = 1;	
			Display_System_Message(" RESET  OK  ");				
		}
		else	{	Display_System_Message("RST ERROR");	}
		HAL_Delay(100);
	}
	for(int i=0;i<20;i++)	{	rxbuff[i] = 0;	}
	status = 0;
}
/*************************************************************************************
Code Designed & Developed by MKM for STG401V11 in AUG 2024
*************************************************************************************/

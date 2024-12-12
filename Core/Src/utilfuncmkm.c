#include "utilfuncmkm.h"
/******************************************************************************
******************************************************************************/
extern UART_HandleTypeDef huart1;
/******************************************************************************
******************************************************************************/
void Display_8Bit_Hex(uint8_t hexval)
{
	uint8_t datatx[10];
	unsigned char1=0,char2=0;
	char1 = (hexval & 0xF0)>>4;
	if(char1 > 9)	{ char1 = char1 + 0x07;	} 
	else			{ char1 = char1 + 0x00;	} 
	char1 = char1 + 0x30;
	datatx[0] = char1;

	char2 = hexval & 0x0F;
	if(char2 > 9)	{ char2 = char2 + 0x07;	} 
	else			{ char2 = char2 + 0x00;	} 
	char2 = char2 + 0x30;
	datatx[1] = char2;
	
	HAL_UART_Transmit(&huart1,datatx,2,10);
}
/******************************************************************************
******************************************************************************/
void Display_16Bit_Hex(uint16_t hexval)
{
	Display_8Bit_Hex((hexval & 0xFF00)>>8);
	Display_8Bit_Hex((hexval & 0x00FF)>>0);
}
/******************************************************************************
******************************************************************************/
void Display_32Bit_Hex(uint32_t hexval)
{
	Display_8Bit_Hex((hexval & 0xFF000000)>>24);
	Display_8Bit_Hex((hexval & 0x00FF0000)>>16);
	Display_8Bit_Hex((hexval & 0x0000FF00)>>8);
	Display_8Bit_Hex((hexval & 0x000000FF)>>0);
}
/******************************************************************************
******************************************************************************/
void Display_Binary(uint8_t hexval)
{
	uint8_t datatx[10];
	datatx[0] = 0x30 + ((hexval & 0x80)>>7);
	datatx[1] = 0x30 + ((hexval & 0x40)>>6);
	datatx[2] = 0x30 + ((hexval & 0x20)>>5);
	datatx[3] = 0x30 + ((hexval & 0x10)>>4);

	datatx[4] = 0x30 + ((hexval & 0x08)>>3);
	datatx[5] = 0x30 + ((hexval & 0x04)>>2);
	datatx[6] = 0x30 + ((hexval & 0x02)>>1);
	datatx[7] = 0x30 + ((hexval & 0x01)>>0);
	HAL_UART_Transmit(&huart1,datatx,8,10);
}
/******************************************************************************
******************************************************************************/
uint8_t Calculate_Checksum(uint8_t *databuf)
{
	uint8_t chksum=0;
	for(uint8_t i=0;i<7;i++)
	{
		chksum += databuf[i];
	}
	return(chksum);
}
/*************************************************************************************
Code Designed & Developed by MKM for STG401V20BSPV10 in AUG 2024
*************************************************************************************/

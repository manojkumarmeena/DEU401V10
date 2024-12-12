#include "main.h"
#include "adxl375mkm.h"
/******************************************************************************
******************************************************************************/
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;
extern CRC_HandleTypeDef hcrc;
/******************************************************************************
******************************************************************************/
extern uint8_t axldata[6];
extern uint8_t compsensordata[40];
extern uint8_t gtrtmr;
/******************************************************************************
******************************************************************************/
float offsetx;
float offsety;
float offsetz;
uint16_t axl375offsetx;
uint16_t axl375offsety;
uint16_t axl375offsetz;
/******************************************************************************
******************************************************************************/
uint8_t Init_Adxl375()
{
	uint8_t temp,regaddress,bytevalue;

	if(Read_Adxl375_Id() == 0xE5)
	{		
		/*Bandwitdh & Output Datarate*/
		/*000X XXXX: Low Power(D4), Rate(D3-D0)*/
		regaddress = 0x2C;
		temp = Read_Adxl375_Reg(regaddress);
		regaddress = 0x2C;	bytevalue = (temp & 0xE0) | 0x0D;
		Write_Adxl375_Reg(regaddress,bytevalue);
		
		/*Power Control & Measutrement Mode*/
		/*Link(D5), Auto Sleep(D4), Measure(D3), Sleep(D2),Wakeup(D1-D0)*/
		/*00XX XXXX*/
		regaddress = 0x2D;	bytevalue = 0x04;
		Write_Adxl375_Reg(regaddress,bytevalue);
		HAL_Delay(100);
		regaddress = 0x2D;
		temp = Read_Adxl375_Reg(regaddress);
		regaddress = 0x2D;	bytevalue = (temp & 0xC0) | 0x08;
		Write_Adxl375_Reg(regaddress,bytevalue);
		
		/*Data Format: Self test(D7), SPI(D6), INT_Invert(D5),Justify(D2)*/
		/*XXX0 1X11*/
		
		regaddress = 0x31;
		temp = Read_Adxl375_Reg(regaddress);
//		regaddress = 0x31;	bytevalue = (temp & 0x0B) | 0x00;
		regaddress = 0x31;	bytevalue = 0x0B;
		Write_Adxl375_Reg(regaddress,bytevalue);
		
		/*Interrupt Source-XXXXXXXX*/
		/*Data Ready(D7), Overrrun(D0))*/
		regaddress = 0x30;
		temp = Read_Adxl375_Reg(regaddress);
		regaddress = 0x30;	bytevalue = (temp & 0x00) | 0x80;
		Write_Adxl375_Reg(regaddress,bytevalue);

		/*Fifo Control: Fifo Mode(D7-D6), Trigger(D5),Samples(D4-D0)*/
		/*XX XXXXXX*/
		regaddress = 0x38;
		temp = Read_Adxl375_Reg(regaddress);
		regaddress = 0x38;	bytevalue = (temp & 0x00) | 0x00;
		Write_Adxl375_Reg(regaddress,bytevalue);
		
		
		/*Interrupt Enable-XXXXX0XX*/
		/*Data Ready(D7), Overrrun(D0))*/
		regaddress = 0x2E;
		temp = Read_Adxl375_Reg(regaddress);
		regaddress = 0x2E;	bytevalue = (temp & 0x00) | 0x00;
		Write_Adxl375_Reg(regaddress,bytevalue);
		
		Adxl375_Offset_Calculation();
		
		return(1);
	}
	else	{	return(0);	}
}
/******************************************************************************
THIS FUNCTION WAITS TILL A FIXED VALUE OF g IS ACHIEVED. THIS SI BASICALLY DONE
TO RECORD ONLY DURING TIME WINDOW OF INTEREST AND HAVE ONLY MEANINGFULL DATA
******************************************************************************/
void Adxl375_Motion_Detection()
{
		uint8_t regaddress=0,bytevalue=0;

		/*Shock Axis Enable*/
		/*Suppress(D3), X-Axis(D2), Y-Axis(D1), Z-Axis(D0)*/
		regaddress = 0x2A;	bytevalue = 0x06;
		Write_Adxl375_Reg(regaddress,bytevalue);

		/*Shock Threshold: 3*780mg = 2.34g */
		regaddress = 0x1D;	bytevalue = 0x07;
		Write_Adxl375_Reg(regaddress,bytevalue);
	
		/*Shock duration: 50*625us = 30ms */
		regaddress = 0x21;	bytevalue = 100;
		Write_Adxl375_Reg(regaddress,bytevalue);

		//Interrupt Enable-X
		//Data Ready(D7), Shock Detection (D6),Overrrun(D0))
		regaddress = 0x2E;	bytevalue = 0x40;
		Write_Adxl375_Reg(regaddress,bytevalue);

		HAL_UART_Transmit(&huart1,"\nWaiting for Shock",18,10);
		while((Read_Adxl375_Reg(0x30)& 0x40) != 0x40);

		/*Interrupt Enable-X*/
		/*Data Ready(D7), Shock Detection (D6),Overrrun(D0))*/
		regaddress = 0x2E;	bytevalue = 0x00;
		Write_Adxl375_Reg(regaddress,bytevalue);
}
/******************************************************************************
THIS FUNCTION WAITS TILL A FIXED VALUE OF g IS ACHIEVED. THIS SI BASICALLY DONE
TO RECORD ONLY DURING TIME WINDOW OF INTEREST AND HAVE ONLY MEANINGFULL DATA
******************************************************************************/
void Adxl375_Trigger_Wait()
{
	uint8_t temp1[6];
	uint16_t temp2[3];
	uint8_t recordflag=0;
	uint16_t gtrgvalue=0;
	gtrgvalue = 0x0028 * (gtrtmr&0x0F);
	while(recordflag == 0)
	{
		Read_Adxl375_Regs(ADXL375_ACCLXL_REG,temp1,6);
		temp2[0] = ((temp1[1]<<8) +  temp1[0]) + axl375offsetx;
		temp2[1] = ((temp1[3]<<8) +  temp1[2]) + axl375offsety;
		temp2[2] = ((temp1[5]<<8) +  temp1[4]) + axl375offsetz;
			
		if(temp2[0] > 0x8000)	{	temp2[0] = (~temp2[0]) + 0x0001;	}		
		if(temp2[1] > 0x8000)	{	temp2[1] = (~temp2[1]) + 0x0001;	}
		if(temp2[2] > 0x8000)	{	temp2[2] = (~temp2[2]) + 0x0001;	}

		if(temp2[0] > gtrgvalue)	{	recordflag = 1;	}	//{	Display_16Bit_Hex(temp2[0]);	recordflag = 1;	}
		if(temp2[1] > gtrgvalue)	{	recordflag = 1;	}	//{	Display_16Bit_Hex(temp2[1]);	recordflag = 1;	}
		if(temp2[2] > gtrgvalue)	{	recordflag = 1;	}	//{	Display_16Bit_Hex(temp2[2]);	recordflag = 1;	}
	}
}
/******************************************************************************
THIS FUNCTION CALCULATES THE OFFSET OF ADXL375 FOR OUTPUT CORRECTION OF 3 AXES.
******************************************************************************/
void Adxl375_Offset_Calculation()
{
	uint8_t temp1[6];
	int16_t temp2[3]={0x0000,0x0000,0x0000};
	int16_t temp3[3]={0x0000,0x0000,0x0000};

	Read_Adxl375_Regs(ADXL375_ACCLXL_REG,temp1,6);
	temp2[0] = (temp1[1]<<8) +  temp1[0];
	temp2[1] = (temp1[3]<<8) +  temp1[2];
	temp2[2] = (temp1[5]<<8) +  temp1[4];
	for(int i=0;i<100;i++)
	{		
		Read_Adxl375_Regs(ADXL375_ACCLXL_REG,temp1,6);
		temp3[0] = (temp1[1]<<8) +  temp1[0];
		temp3[1] = (temp1[3]<<8) +  temp1[2];
		temp3[2] = (temp1[5]<<8) +  temp1[4];
		
		temp2[0] = (temp2[0] + temp3[0]);
		temp2[1] = (temp2[1] + temp3[1]);
		temp2[2] = (temp2[2] + temp3[2]);
		HAL_Delay(10);
	}
	temp2[0] = (temp2[0]/100);
	temp2[1] = (temp2[1]/100);
	temp2[2] = (temp2[2]/100);
	
	axl375offsetx = (~((uint16_t)temp2[0])) + 0x0001; 
	axl375offsety = (~((uint16_t)temp2[1])) + 0x0001; 
	axl375offsetz = (~(((uint16_t)temp2[2]) - 0x0014)) + 0x0001;
}
/******************************************************************************
******************************************************************************/
uint8_t Read_Adxl375_Id()
{
	uint8_t temp[1];
	temp[0]=0xC0|(0x3F & 0x00);
	
	/*Dummy Read Required for correct reading*/
	HAL_GPIO_WritePin(CS_ADXL375_GPIO_Port,CS_ADXL375_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,temp,1,10);
	temp[0] = 0x00;
	HAL_SPI_Receive(&hspi1,temp,1,100);
	HAL_GPIO_WritePin(CS_ADXL375_GPIO_Port,CS_ADXL375_Pin,GPIO_PIN_SET);
	return(temp[0]);
}
/******************************************************************************
******************************************************************************/
uint8_t Write_Adxl375_Reg(uint8_t regaddr,uint8_t bytevalue)
{
	uint8_t adxladdr=0;
	adxladdr |= regaddr;	//setting address bits (A5A4A3A2A1A0) 
	
	/*Dummy Read Required for correct reading*/
	HAL_GPIO_WritePin(CS_ADXL375_GPIO_Port,CS_ADXL375_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,&adxladdr,1,100);
	HAL_SPI_Transmit(&hspi1,&bytevalue,1,100);
	HAL_GPIO_WritePin(CS_ADXL375_GPIO_Port,CS_ADXL375_Pin,GPIO_PIN_SET);
	if(Read_Adxl375_Reg(adxladdr) != bytevalue)
	{	return(0);	}
	return(1);
}
/******************************************************************************
******************************************************************************/
uint8_t Read_Adxl375_Reg(uint8_t regaddr)
{
	uint8_t temp[1];
	temp[0]=0xC0|(0x3F & regaddr);
		
	HAL_GPIO_WritePin(CS_ADXL375_GPIO_Port,CS_ADXL375_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,temp,1,10);
	temp[0]=0;
	HAL_SPI_Receive(&hspi1,temp,1,10);
	HAL_GPIO_WritePin(CS_ADXL375_GPIO_Port,CS_ADXL375_Pin,GPIO_PIN_SET);
	return(temp[0]);	
}
/******************************************************************************
******************************************************************************/
void Read_Adxl375_Regs(uint8_t regaddr,uint8_t *rxbuf, uint8_t bytecount)
{
	uint8_t temp=0;
	temp=0xC0|(0x3F & regaddr);

	HAL_GPIO_WritePin(CS_ADXL375_GPIO_Port,CS_ADXL375_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,&temp,1,20);
	HAL_SPI_Receive(&hspi1,rxbuf,bytecount,1000);
	HAL_GPIO_WritePin(CS_ADXL375_GPIO_Port,CS_ADXL375_Pin,GPIO_PIN_SET);
}
/******************************************************************************
******************************************************************************/
uint8_t Update_Adxl375_Data()
{
	uint8_t crcval=0;
	uint8_t rxbuf[6],count=0;;
	int16_t aclxx,aclyy,aclzz;
	
	Read_Adxl375_Regs(ADXL375_ACCLXL_REG,rxbuf,6);

	compsensordata[count] = '@';	count++;

	compsensordata[count] = (rxbuf[1]);	count++;
	compsensordata[count] = (rxbuf[0]);	count++;
	compsensordata[count] = (rxbuf[3]);	count++;
	compsensordata[count] = (rxbuf[2]);	count++;
	compsensordata[count] = (rxbuf[5]);	count++;
	compsensordata[count] = (rxbuf[4]);	count++;

	
	for(uint8_t i=0;i<7;i++)	{	crcval += compsensordata[i];	}	
	compsensordata[count] = crcval;	count++;
	compsensordata[count] = 0x00; count++;
	compsensordata[count] = 0x00; count++;
	compsensordata[count] = 0x0D; count++;
	compsensordata[count] = 0x0A; count++;
	
	return(count);	
}
/******************************************************************************
******************************************************************************/
uint8_t Update_Accel_Data()
{
	int16_t aclxx,aclyy,aclzz;
	uint8_t crcval=0,temp1,temp2;

	uint8_t rxbuf[6],count=0;;
	uint8_t showtemp[10];
	uint8_t regaddress=0,bytecount=0;
	float acclx=0.0,accly=0.0,acclz=0.0;
	
	regaddress = ADXL375_ACCLXL_REG;	bytecount = 6;
	Read_Adxl375_Regs(regaddress,rxbuf,bytecount);	
	
	aclxx= (((rxbuf[1])<<8) + rxbuf[0]);
	aclyy= (((rxbuf[3])<<8) + rxbuf[2]);
	aclzz= (((rxbuf[5])<<8) + rxbuf[4]);

	acclx=(((float)aclxx) * (0.049)) + offsetx;
	accly=(((float)aclyy) * (0.049)) + offsety;
	acclz=(((float)aclzz) * (0.049)) + offsetz;
	
	compsensordata[count] = '@';	count++;
	
	sprintf((char *)showtemp,"%0.5f",acclx);
	for(int i=0;i<6;i++)
	{	compsensordata[count] = showtemp[i];	count++;	}
	compsensordata[count] = ',';	count++;
	
	sprintf((char *)showtemp,"%0.5f",accly);
	for(int i=0;i<6;i++)
	{	compsensordata[count] = showtemp[i];	count++;	}
	compsensordata[count] = ',';	count++;

	sprintf((char *)showtemp,"%0.5f",acclz);
	for(int i=0;i<6;i++)
	{	compsensordata[count] = showtemp[i];	count++;	}
	compsensordata[count] = ',';	count++;

//	crcval = HAL_CRC_Calculate(&hcrc,(uint32_t *)compsensordata,count++);
//	compsensordata[count] = (uint8_t)((crcval & 0xFF00)>>8);	count++;
//	compsensordata[count] = (uint8_t)((crcval & 0x00FF)>>0);	count++;
	
	for(uint8_t i=0;i<count;i++)	{	crcval += compsensordata[i];	}	
	temp1 = (crcval & 0xF0)>>4; 
	if(temp1 > 0x09){	temp1 = temp1 + 0x37;	} else	{	temp1 = temp1 + 0x30;	}
	temp2 = (crcval & 0x0F)>>0;
	if(temp2 > 0x09){	temp2 = temp2 + 0x37;	} else	{	temp2 = temp2 + 0x30;	}
	compsensordata[count] = temp1;	count++;
	compsensordata[count] = temp2;	count++;
	
	compsensordata[count] = 0x0D;	count++;
	compsensordata[count] = 0x0A;	count++;

	return(count);	
}
/*************************************************************************************
Code Designed & Developed by MKM for STG401V20BSPV10 in AUG 2024
*************************************************************************************/

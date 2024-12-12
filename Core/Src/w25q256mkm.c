#include "main.h"
#include "w25q256mkm.h"
/******************************************************************************
******************************************************************************/
uint8_t Init_Flash128()
{
	uint8_t status = 0;
	Flash_Reset_Device();
	Probe_Flash();
	Flash_Chip_Erase();
	if(Flash_Read_ElecId() == 0x17)		{	status= 1;	}
	else	{	status= 0;	}
	return(status);
}	
/******************************************************************************
******************************************************************************/
void Init_Flash()
{
	uint8_t status = 0;
	Flash_Reset_Device();
	Probe_Flash();
	Flash_Chip_Erase();
	if(Flash_Read_ElecId() == 0x17)		{	status= 1;	}
	else	{	status= 0;	}
//	return(status);
}
/******************************************************************************
******************************************************************************/
uint8_t Init_Flash256()
{
	Flash_Reset_Device();
	while(Flash_Read_ElecId() != 0x18)	{	HAL_Delay(100);	}
	return(1);	
}
/******************************************************************************
******************************************************************************/
void Probe_Flash()
{
	uint8_t elecid=0;
	uint16_t manid=0;
	uint32_t jdecid=0;
	
	manid = Flash_Read_ManId();
	HAL_UART_Transmit(&huart1,"\nManID:",7,10);
	Display_16Bit_Hex(manid);
	
	elecid = Flash_Read_ElecId();
	HAL_UART_Transmit(&huart1,"\nElecID:",8,10);
	Display_8Bit_Hex(elecid);
			
	jdecid = Flash_Jedec_Id();
	HAL_UART_Transmit(&huart1,"\nJdecID:",8,10);
	Display_32Bit_Hex(jdecid);
}
/******************************************************************************
******************************************************************************/
void Diagnose_Flash()
{	
	uint32_t memadd=0x121000;
	uint8_t txbuf[256];
	uint8_t rxbuf[256];

	Flash_Sector_Erase(memadd);
	
	HAL_UART_Transmit(&huart1,"Writing Data",12,10);
	for(int i=0;i<256;i++)	{	txbuf[i] = i;		}
	Flash_Write_Data(memadd,txbuf,256);
	HAL_UART_Transmit(&huart1,"Data Written",12,10);
	
	memadd=0x121000;
	Flash_Read_Data(memadd,rxbuf,256);
	HAL_UART_Transmit(&huart1,"\nReading Data\n",14,10);
	for(int i=0;i<256;i++)
	{	
		Display_8Bit_Hex(rxbuf[i]);
		if((((i+1)%32) == 0))	{	HAL_UART_Transmit(&huart1,"\n",1,10);	}		
	}
}
/******************************************************************************
******************************************************************************/
void Erase_Write_Sector(uint32_t secadd,uint8_t *databuf)
{
	Flash_Sector_Erase(secadd);
	Flash_Write_Data(secadd,databuf,200);	
}
/******************************************************************************
******************************************************************************/
void Flash_Write_Data(uint32_t address,uint8_t *databuf,uint16_t txcount)
{		
	uint8_t command[260]={0x02,0x00,0x00,0x00};	
	command[1] = (uint8_t)((address & 0xFF0000)>>16);
	command[2] = (uint8_t)((address & 0x00FF00)>>8);
	command[3] = (uint8_t)((address & 0x0000FF)>>0);
	
	while((Flash_Read_SR1() & 0x01) != 0x00);
	Flash_Write_Enable();
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,command,4,1000);
	HAL_SPI_Transmit(&hspi1,databuf,txcount,1000);
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_SET);
	Flash_Write_Disable();
//	while((Flash_Read_SR1() & 0x01) == 0x01);
}
/******************************************************************************
******************************************************************************/
void Flash_Read_Data(uint32_t address,uint8_t *databuf,uint16_t rxcount)
{
	uint8_t command[4]={0x03,0x00,0x00,0x00};
	command[1] = (uint8_t)((address & 0xFF0000)>>16);
	command[2] = (uint8_t)((address & 0x00FF00)>>8);
	command[3] = (uint8_t)((address & 0x0000FF)>>0);
	
	while((Flash_Read_SR1() & 0x01) != 0);
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,command,4,1000);
	HAL_SPI_Receive(&hspi1,databuf,3,1000);
	HAL_SPI_Receive(&hspi1,databuf,rxcount,1000);
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_SET);
}
/******************************************************************************
******************************************************************************/
void Flash_Chip_Erase()
{
	uint8_t count=0;
	HAL_UART_Transmit(&huart1,"\nERASING CHIP:",14,10);
	Display_8Bit_Hex(Flash_Read_SR1());
	while( (Flash_Read_SR1() & 0x02) != 0x02)
	{	Send_Command(WRITE_ENABLE);	}
	Display_8Bit_Hex(Flash_Read_SR1());
	while((Flash_Read_SR1() & 0x01) != 0x01)
	{	Send_Command(CHIP_ERASE);	}
	Display_8Bit_Hex(Flash_Read_SR1());
	
	while((Flash_Read_SR1() & 0x01) == 0x01)
	{	HAL_Delay(950);		HAL_UART_Transmit(&huart1,"-",1,10);	Display_8Bit_Hex(count);	count++;	}
	HAL_UART_Transmit(&huart1,"CHIP ERASED",11,10);
}
/******************************************************************************
******************************************************************************/
void Flash_Sector_Erase(uint32_t address)
{
	uint8_t command[4] ={0x20,0x00,0x00,0x00};
	command[1] = (uint8_t)((address & 0x00FF0000)>>16);
	command[2] = (uint8_t)((address & 0x0000FF00)>>8);
	command[3] = (uint8_t)((address & 0x000000FF)>>0);
	
	Send_Command(WRITE_ENABLE);
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,command,4,1000);
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_SET);
	while(( Flash_Read_SR1() & 0x01) != 0x00);
}
/******************************************************************************
******************************************************************************/
void Flash_Block_Erase(uint32_t address)
{
	uint8_t command[4] ={0xD8,0x00,0x00,0x00};
	command[1] = (uint8_t)((address & 0xFF0000)>>16);
	command[2] = (uint8_t)((address & 0x00FF00)>>8);
	command[3] = (uint8_t)((address & 0x0000FF)>>0);
	
	Send_Command(WRITE_ENABLE);
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,command,4,1000);
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_SET);
	while(( Flash_Read_SR1() & 0x01) != 0x00);
}
/******************************************************************************
******************************************************************************/
void Check_Busy_Flag()
{
	while((Flash_Read_SR1() & 0x01) == 1);
}
/******************************************************************************
******************************************************************************/
void Check_WEL_Flag()
{
	while( (Flash_Read_SR1() & 0x02) != 0x02);
}
/******************************************************************************
******************************************************************************/
uint16_t Flash_Read_ManId()
{
	uint8_t datatx1[5],command1[4]={0x90,0x00,0x00,0x00};	
	uint16_t manid=0;
	/*Dummy Read Required for correct reading*/
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,command1,4,100);
	HAL_SPI_Receive(&hspi1,datatx1,3,1000);
	HAL_SPI_Receive(&hspi1,datatx1,2,1000);
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_SET);
	
	manid = (manid | datatx1[0])<<8;
	manid = (manid | datatx1[1])<<0;
	return(manid);
}
/******************************************************************************
******************************************************************************/
uint8_t Flash_Read_ElecId()
{
	uint8_t datatx[2],command[4]={0xAB,0x00,0x00,0x00};
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,command,4,1000);
	HAL_SPI_Receive(&hspi1,datatx,1,1000);
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_SET);
	return(datatx[0]);
}
/******************************************************************************
******************************************************************************/
uint32_t Flash_Jedec_Id()
{
	uint8_t temp[3];
	temp[0]= 0x9F;
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin, GPIO_PIN_RESET);	
	HAL_SPI_Transmit(&hspi1,temp,1,1000);
	temp[0] = 0;
	HAL_SPI_Receive(&hspi1,temp,3,2000);
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin, GPIO_PIN_SET);	
	
	return(temp[0]<<16|temp[1]<<8|temp[2]);
}
/******************************************************************************
******************************************************************************/
void Flash_Write_Enable()
{
	uint8_t command =0x06;
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,&command,1,1000);
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_SET);
	//Check_WEL_Flag();
}
/******************************************************************************
******************************************************************************/
void Flash_Write_Disable()
{
	uint8_t command =0x04;
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,&command,1,1000);
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_SET);
}
/******************************************************************************
******************************************************************************/
uint8_t Flash_Read_SR1()
{
	uint8_t command = READ_SR1_REG,response;
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,&command,1,100);
	HAL_SPI_Receive(&hspi1,&response,1,500);
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_SET);
	return(response);
}
/******************************************************************************
******************************************************************************/
uint8_t Flash_Read_SR2()
{
	uint8_t command = READ_SR2_REG,response;
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,&command,1,1000);
	HAL_SPI_Receive(&hspi1,&response,1,1000);	
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_SET);
	return(response);
}
/******************************************************************************
******************************************************************************/
uint8_t Flash_Read_SR3()
{
	uint8_t command =READ_SR3_REG,response;
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,&command,1,1000);
	HAL_SPI_Receive(&hspi1,&response,1,1000);	
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_SET);
	return(response);
}
/******************************************************************************
******************************************************************************/
void Flash_Write_SR1(uint8_t dataval)
{
	uint8_t command[2];
	command[0] =0x01; command[1]=dataval;
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,command,2,1000);
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_SET);
}
/******************************************************************************
******************************************************************************/
void Flash_Write_SR2(uint8_t dataval)
{
	uint8_t command[2];
	command[0] =0x31; command[1]=dataval;
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,command,2,1000);
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_SET);
}
/******************************************************************************
******************************************************************************/
void Flash_Write_SR3(uint8_t dataval)
{
	uint8_t command[2];
	command[0] =0x11; command[1]=dataval;
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,command,2,1000);
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_SET);
}
/******************************************************************************
******************************************************************************/
void Flash_Reset_Device()
{
	uint8_t temp[2];
	temp[0] = 0x66;
	temp[1] = 0x99;

	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin, GPIO_PIN_RESET);	
	HAL_SPI_Transmit(&hspi1,temp,2,100);
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin, GPIO_PIN_SET);	
	HAL_Delay(50);
}
/******************************************************************************
******************************************************************************/
void Flash_Power_Down()
{
	uint8_t command =0xB9;
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,&command,1,1000);
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_SET);
}
/******************************************************************************
******************************************************************************/
void Flash_Erase_Suspend()
{
	uint8_t command =0x75;
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,&command,1,1000);
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_SET);
}
/******************************************************************************
******************************************************************************/
void Flash_Erase_Resume()
{
	uint8_t command =0x7A;
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,&command,1,1000);
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_SET);
}
/******************************************************************************
******************************************************************************/
void Flash_Block_Unlock()
{
	uint8_t command[4] = {0x39,0x00,0x00,0x00};
	
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,command,4,1000);
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_SET);
}
/******************************************************************************
******************************************************************************/
uint8_t Flash_Release_PowerDown()
{
	uint8_t command[4]= {0xAB,0x00,0x00,0x00};
	uint8_t response=0x00;
	
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,command,4,1000);
	HAL_SPI_Receive(&hspi1,&response,1,1000);
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_SET);
	return(response);
}
/******************************************************************************
******************************************************************************/
void Flash_Block_Lock()
{
	uint8_t command[4] = {0x36,0x00,0x00,0x00};
	
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,command,4,1000);
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_SET);
}
/******************************************************************************
******************************************************************************/
void Send_Command(uint8_t command)
{
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,&command,1,1000);
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_SET);
}
/******************************************************************************
******************************************************************************/
uint8_t Send_Command_Get_Response(uint8_t command)
{
	uint8_t response=0;
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,&command,1,1000);
	response = HAL_SPI_Receive(&hspi1,&response,1,1000);
	HAL_GPIO_WritePin(CS_W25Q256_GPIO_Port,CS_W25Q256_Pin,GPIO_PIN_SET);
	return(response);
}
/*************************************************************************************
Code Designed & Developed by MKM for STG401V11 in AUG 2024
*************************************************************************************/

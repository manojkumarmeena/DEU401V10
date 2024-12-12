#include "ms5611mkm.h"
#include "main.h"
#include "utilfuncmkm.h"
#include "stdio.h"
#include "math.h"
/******************************************************************************
******************************************************************************/
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;

extern double P; 					// compensated pressure value
extern double T; 					// compensated temperature value
extern float A;						// compensated Altitude value
/******************************************************************************
******************************************************************************/
unsigned long digitaltemperature = 0x00000000;
unsigned long digitalpressure 		= 0x00000000;
float temperature=0.0,pressure=0.0;

unsigned int c0_coefficient=0x0000;
unsigned int c1_coefficient=0x0000;
unsigned int c2_coefficient=0x0000;
unsigned int c3_coefficient=0x0000;
unsigned int c4_coefficient=0x0000;
unsigned int c5_coefficient=0x0000;
unsigned int c6_coefficient=0x0000;
unsigned int c7_coefficient=0x0000;
unsigned int prombuff[8];
/******************************************************************************
******************************************************************************/
void Initialize_MS5611_Device()
{
	Reset_MS5611_Device();
	Read_MS5611_PROM();
}
/******************************************************************************
******************************************************************************/
void Reset_MS5611_Device(void)
{
	uint8_t txbuffer[1];
	txbuffer[0] = 0x1E;
	while(HAL_I2C_IsDeviceReady(&hi2c1,MS5611_ADDRESS,5,100) != HAL_OK);
	HAL_I2C_Master_Transmit(&hi2c1,MS5611_ADDRESS,txbuffer,1,100);
	HAL_Delay(5);						//Delay to allow Reset to complete(2.8ms)
}
/******************************************************************************
This function reads prom data required for callibration.
******************************************************************************/
void Read_MS5611_PROM()
{
	uint8_t rxbuffer[2],txbuffer[1];
	txbuffer[0] = 0xA0;
	uint8_t count=0,byteno=0;

	txbuffer[0] = 0xA0;
	while(HAL_I2C_IsDeviceReady(&hi2c1,MS5611_ADDRESS,5,100) != HAL_OK);
	HAL_I2C_Master_Transmit(&hi2c1,MS5611_ADDRESS,txbuffer,1,100);
	while(HAL_I2C_IsDeviceReady(&hi2c1,MS5611_ADDRESS,5,100) != HAL_OK);
	HAL_I2C_Master_Receive(&hi2c1,MS5611_ADDRESS,rxbuffer,2,100);
	prombuff[0] = (rxbuffer[0]<<8) + rxbuffer[1];

	txbuffer[0] = 0xA2;
	while(HAL_I2C_IsDeviceReady(&hi2c1,MS5611_ADDRESS,5,100) != HAL_OK);
	HAL_I2C_Master_Transmit(&hi2c1,MS5611_ADDRESS,txbuffer,1,100);
	while(HAL_I2C_IsDeviceReady(&hi2c1,MS5611_ADDRESS,5,100) != HAL_OK);
	HAL_I2C_Master_Receive(&hi2c1,MS5611_ADDRESS,rxbuffer,2,100);
	prombuff[1] = (rxbuffer[0]<<8) + rxbuffer[1];
	
	txbuffer[0] = 0xA4;
	while(HAL_I2C_IsDeviceReady(&hi2c1,MS5611_ADDRESS,5,100) != HAL_OK);
	HAL_I2C_Master_Transmit(&hi2c1,MS5611_ADDRESS,txbuffer,1,100);
	while(HAL_I2C_IsDeviceReady(&hi2c1,MS5611_ADDRESS,5,100) != HAL_OK);
	HAL_I2C_Master_Receive(&hi2c1,MS5611_ADDRESS,rxbuffer,2,100);
	prombuff[2] = (rxbuffer[0]<<8) + rxbuffer[1];
	
	txbuffer[0] = 0xA6;
	while(HAL_I2C_IsDeviceReady(&hi2c1,MS5611_ADDRESS,5,100) != HAL_OK);
	HAL_I2C_Master_Transmit(&hi2c1,MS5611_ADDRESS,txbuffer,1,100);
	while(HAL_I2C_IsDeviceReady(&hi2c1,MS5611_ADDRESS,5,100) != HAL_OK);
	HAL_I2C_Master_Receive(&hi2c1,MS5611_ADDRESS,rxbuffer,2,100);
	prombuff[3] = (rxbuffer[0]<<8) + rxbuffer[1];
	
	txbuffer[0] = 0xA8;
	while(HAL_I2C_IsDeviceReady(&hi2c1,MS5611_ADDRESS,5,100) != HAL_OK);
	HAL_I2C_Master_Transmit(&hi2c1,MS5611_ADDRESS,txbuffer,1,100);
	while(HAL_I2C_IsDeviceReady(&hi2c1,MS5611_ADDRESS,5,100) != HAL_OK);
	HAL_I2C_Master_Receive(&hi2c1,MS5611_ADDRESS,rxbuffer,2,100);
	prombuff[4] = (rxbuffer[0]<<8) + rxbuffer[1];
	
	txbuffer[0] = 0xAA;
	while(HAL_I2C_IsDeviceReady(&hi2c1,MS5611_ADDRESS,5,100) != HAL_OK);
	HAL_I2C_Master_Transmit(&hi2c1,MS5611_ADDRESS,txbuffer,1,100);
	while(HAL_I2C_IsDeviceReady(&hi2c1,MS5611_ADDRESS,5,100) != HAL_OK);
	HAL_I2C_Master_Receive(&hi2c1,MS5611_ADDRESS,rxbuffer,2,100);
	prombuff[5] = (rxbuffer[0]<<8) + rxbuffer[1];
	
	txbuffer[0] = 0xAC;
	while(HAL_I2C_IsDeviceReady(&hi2c1,MS5611_ADDRESS,5,100) != HAL_OK);
	HAL_I2C_Master_Transmit(&hi2c1,MS5611_ADDRESS,txbuffer,1,100);
	while(HAL_I2C_IsDeviceReady(&hi2c1,MS5611_ADDRESS,5,100) != HAL_OK);
	HAL_I2C_Master_Receive(&hi2c1,MS5611_ADDRESS,rxbuffer,2,100);
	prombuff[6] = (rxbuffer[0]<<8) + rxbuffer[1];
	
	txbuffer[0] = 0xAE;
	while(HAL_I2C_IsDeviceReady(&hi2c1,MS5611_ADDRESS,5,100) != HAL_OK);
	HAL_I2C_Master_Transmit(&hi2c1,MS5611_ADDRESS,txbuffer,1,100);
	while(HAL_I2C_IsDeviceReady(&hi2c1,MS5611_ADDRESS,5,100) != HAL_OK);
	HAL_I2C_Master_Receive(&hi2c1,MS5611_ADDRESS,rxbuffer,2,100);
	prombuff[7] = (rxbuffer[0]<<8) + rxbuffer[1];
	
	c0_coefficient = prombuff[0];
	c1_coefficient = prombuff[1];
	c2_coefficient = prombuff[2];
	c3_coefficient = prombuff[3];
	c4_coefficient = prombuff[4];
	c5_coefficient = prombuff[5];
	c6_coefficient = prombuff[6];
	c7_coefficient = prombuff[7];
	
	HAL_UART_Transmit(&huart1,"\nPROM=",6,10);
	for(uint8_t i=0;i<8;i++) { Display_16Bit_Hex(prombuff[i]); HAL_UART_Transmit(&huart1,":",1,10);	}
}
/******************************************************************************
This function calculates pressure using 24bit value obtained in D1 conversion.
******************************************************************************/
void Read_MS5611_ADC()
{
	uint8_t txbuffer[1],rxbuffer[3],showtemp[10];
	digitalpressure			= 0x00000000;
	digitaltemperature 	= 0x00000000;
	
	txbuffer[0] = 0x48;					//conversion D1 (OSR=4096)
	while(HAL_I2C_IsDeviceReady(&hi2c1,MS5611_ADDRESS,5,100) != HAL_OK);
	HAL_I2C_Master_Transmit(&hi2c1,MS5611_ADDRESS,txbuffer,1,100);
	HAL_Delay(10);
	
	txbuffer[0] = 0x00;					//conversion D1 (OSR=4096)
	while(HAL_I2C_IsDeviceReady(&hi2c1,MS5611_ADDRESS,5,100) != HAL_OK);
	HAL_I2C_Master_Transmit(&hi2c1,MS5611_ADDRESS,txbuffer,1,100);

	HAL_I2C_Master_Receive(&hi2c1,MS5611_ADDRESS,rxbuffer,3,100);
	
	digitalpressure = (digitalpressure << 8) | rxbuffer[0];
	digitalpressure = (digitalpressure << 8) | rxbuffer[1];
	digitalpressure = (digitalpressure << 8) | rxbuffer[2];

	txbuffer[0] = 0x58;					//conversion D2 (OSR=4096)
	while(HAL_I2C_IsDeviceReady(&hi2c1,MS5611_ADDRESS,5,100) != HAL_OK);
	HAL_I2C_Master_Transmit(&hi2c1,MS5611_ADDRESS,txbuffer,1,100);
	HAL_Delay(10);
	
	txbuffer[0] = 0x00;					//conversion D2 (OSR=4096)
	while(HAL_I2C_IsDeviceReady(&hi2c1,MS5611_ADDRESS,5,100) != HAL_OK);
	HAL_I2C_Master_Transmit(&hi2c1,MS5611_ADDRESS,txbuffer,1,100);

	HAL_I2C_Master_Receive(&hi2c1,MS5611_ADDRESS,rxbuffer,3,100);

	digitaltemperature = (digitaltemperature << 8) | rxbuffer[0];
	digitaltemperature = (digitaltemperature << 8) | rxbuffer[1];
	digitaltemperature = (digitaltemperature << 8) | rxbuffer[2];

//	HAL_UART_Transmit(&huart1,"\nMS5611 PDATA=0x",16,10);
//	Display_32Bit_Hex(digitalpressure);

//	HAL_UART_Transmit(&huart1,"\nMS5611 PRSR=",13,10);
//	sprintf((char *)showtemp,"%8d",(int)digitalpressure);
//	HAL_UART_Transmit(&huart1,showtemp,8,10);

//	HAL_UART_Transmit(&huart1,"\nMS5611 TDATA=0x",16,10);
//	Display_32Bit_Hex(digitaltemperature);	
//	
//	HAL_UART_Transmit(&huart1,"\nMS5611 TEMP=",13,10);
//	sprintf((char *)showtemp,"%8d",(int)digitaltemperature);
//	HAL_UART_Transmit(&huart1,showtemp,8,10);
}
/******************************************************************************
This function Displays Pressure value in decimal.
******************************************************************************/
void Calculate_MS5611_Parameters()
{
	unsigned long D1; 	// ADC value of the pressure conversion
	unsigned long D2; 	// ADC value of the temperature conversion
	unsigned int C[8]; 	// calibration coefficients
	double dT; 					// difference between actual and measured temperature
	double OFF; 				// offset at actual temperature
	double SENS; 				// sensitivity at actual temperature
	int i;
	unsigned char n_crc; // crc value of the prom
	uint8_t showtemp[10];
	
	D1=0;
	D2=0;
	Reset_MS5611_Device();
	Read_MS5611_PROM();
	for(int i=0;i<8;i++)	{	C[i] = prombuff[i];	}
	n_crc = checksum(C); 			// calculate the CRC
//	for(;;) 						// loop without stopping
//	{
		Read_MS5611_ADC();
		D2= digitaltemperature;	// cmd_adc(CMD_ADC_D2+CMD_ADC_4096); // read D2
		D1= digitalpressure;		// cmd_adc(CMD_ADC_D1+CMD_ADC_4096); // read D1
		
		// calcualte 1st order pressure and temperature (MS5607 1st order algorithm)
		dT = D2-(C[5] * pow(2,8));
		OFF = (C[2] * pow(2,17)) + (dT * C[4]/pow(2,6));
		SENS = C[1] * pow(2,16) + dT * C[3]/pow(2,7);
		
		T = (2000+(dT * C[6])/pow(2,23))/100;
		
		P = (((D1 * SENS)/pow(2,21) - OFF)/(pow(2,15))/100);

		A = (1 - pow(P/(double)1013.250, 0.190295)) * 44330.0;
		//A = (1 - (pow((P / (double)sea_pressure), 0.190284))) * 145366.45;
		
		HAL_UART_Transmit(&huart1,"\nM5611 TMP=",11,10);
		sprintf((char *)showtemp,"%f",T);
		HAL_UART_Transmit(&huart1,showtemp,5,10);

		HAL_UART_Transmit(&huart1," PSR=",5,10);
		sprintf((char *)showtemp,"%f",P);
		HAL_UART_Transmit(&huart1,showtemp,5,10);
		
		HAL_UART_Transmit(&huart1," ALT=",5,10);
		sprintf((char *)showtemp,"%f",A);
		HAL_UART_Transmit(&huart1,showtemp,5,10);
		
		HAL_Delay(25);
//	}
}
/******************************************************************************
This function Displays Pressure value in decimal.
******************************************************************************/
unsigned char checksum(unsigned int n_prom[])
{
	int cnt; 								// simple counter
	unsigned int n_rem; 		// crc reminder
	unsigned int crc_read; 	// original value of the crc
	unsigned char n_bit;
	
	n_rem = 0x00;
	crc_read= prombuff[7]; 							//save read CRC
	prombuff[7] = (0xFF00 & (prombuff[7])); //CRC byte is replaced by 0
	
	for (cnt = 0; cnt < 16; cnt++) 		// operation is performed on bytes
	{																	// choose LSB or MSB
		if (cnt%2==1) n_rem ^= (unsigned short) ((prombuff[cnt>>1]) & 0x00FF);
		else n_rem ^= (unsigned short) (prombuff[cnt>>1]>>8);
		for (n_bit = 8; n_bit > 0; n_bit--)
		{
			if (n_rem & (0x8000))	{	n_rem = (n_rem << 1) ^ 0x3000;	}
			else	{	n_rem = (n_rem << 1);	}
		}
	}
	
	n_rem= (0x000F & (n_rem >> 12)); 			// final 4-bit reminder is CRC code
	prombuff[7]=crc_read; 									// restore the crc_read to its original place
	
	return (n_rem ^ 0x0);
}
///***********************************************************************************
//***********************************************************************************/
//float getAltitude() 
//{
//    A = (1 - pow(P/(double)1013.250, 0.190295)) * 44330.0;
//    //A = (1 - (pow((P / (double)sea_pressure), 0.190284))) * 145366.45;
//    return((float)A);
//}
////********************************************************
////! @brief get altitude from known sea level barometer, 
////! @      no pre-pressure calculation
////!
////! @enter float sea level barometer
////! @return float altitude in feet
////********************************************************  
//float getAltitudeFT(float sea_pressure) 
//{
//    A = (1 - (pow((P / (double)sea_pressure), 0.190284))) * 145366.45;
//    return((float)A);
//} 
////********************************************************
////! @brief get sea level pressure from known altitude(ft), 
////! @      no pre-pressure calculation
////!
////! @enter float known altitude in feet
////! @return float seal level barometer in mb
////********************************************************  

//float getSeaLevelBaroFT(float known_alt) 
//{
//    S = pow(pow((P * INHG), 0.190284) + 0.00001313 * known_alt , 5.2553026) * MB;
//    return((float)S);
//} 
////********************************************************
////! @brief get sea level pressure from known altitude(m), 
////! @      no pre-pressure calculation
////!
////! @enter float known altitude in meters
////! @return float seal level barometer in mb
////********************************************************  
//float getSeaLevelBaroM(float known_alt) 
//{
//    S = pow(pow((P * INHG), 0.190284) + 0.00001313 * known_alt * FTMETERS , 5.2553026) * MB;
//    return((float)S);
//} 
/*************************************************************************************
Code Designed & Developed by MKM for STG401BSPV11 in AUG 2024
*************************************************************************************/

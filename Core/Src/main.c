/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "mpu9250mkm.h"
#include "w25q256mkm.h"
#include "adxl375mkm.h"
#include "bme280mkm.h"
#include "ms5611mkm.h"
#include "rn4871mkm.h"
#include "tlv493dmkm.h"
#include "utilfuncmkm.h"
#include "tfminismkm.h"
#include "ssd1306mkm.h"
#include "vl6180xmkm.h"
#include "vl53l0xmkm.h"
#include "tm_stm32_ahrs_imu.h"
#include "jdy25mkm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define COMPORT &huart1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
uint8_t compsensordata[40];
uint16_t gtrtmr=0;
uint8_t magdatabuff[10];
float tempvalue=0.0;
double P; 					// compensated pressure value
double T; 					// compensated temperature value
float A;						// compensated Altitude value

uint8_t distance=0;
uint8_t comportflag='1';		//'1'-UART & '2'-BLE
uint8_t memlogstate='0';		//'0'-Fresh,'1'-Data Logged,'2'-Data Moved,'3'-Memory Erased

float offsetmx=3.09,offsetmy=0.0,offsetmz=0.166;
float offsetax=0.0,offsetay=0.12,offsetaz=-3.000;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
void Diagnose_Flash_Memory(void);
void Init_BME_280(struct bme280_dev *dev);
void user_delay_ms(uint32_t period);
int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
uint8_t Update_Sensor_Data(struct bme280_dev *dev);
void Test_SSD1306_Display(void);
void Display_System_Message(char *sysmsg);
uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);

void Display_AHSRS_Data(void);
void Display_MS56611_Data(void);
void Display_Target_Proximity(void);

uint8_t Convert_MPU_Data1(uint8_t *rxbuf,int logcountval);
uint8_t Convert_ADXL_Data1(uint8_t *rxbuf,int logcountval);
void Read_Store_ADXL3X5_Data(void);
void Read_Store_MPU9250_Data(void);
void Read_Store_MPU9250_Data1(void);
void Read_Relay_Accel_Data(void);
void Update_Comportflag(void);
void Visualize_MPU9250_Data(void);
void Init_Calibrate_MPU(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
struct bme280_dev dev;
TM_AHRSIMU_t mpuimu;

float tlv493temp=0.0;
uint16_t tlvtemperature=0x0000;
uint8_t codeinfo[20] = {"\nSTG401V20,06AUG2024"};
uint8_t screenflag[1]= {'0'};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t rxdata[10];
	uint8_t txdata[4];
	uint8_t memdata1[24];
	uint8_t memdata2[24];
	uint8_t memdata3[24];
	txdata[0] = 'X';
	txdata[1] = 'F';
	txdata[2] = '0';
	txdata[3] = '1';
	uint8_t memflag=0,countx=0,commflag='0';
	uint8_t memlogflag = '0',memreadflag='0', memeraseflag='0',logicflag='0';
	char showtmp[10];	
//	uint8_t tempdata[6];
//	struct bme280_data comp_data;
//	float tempvalue=0.0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
//	uint8_t tempbuff[10],lfcom=0x0D;
//	uint8_t showtemp1[10];

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
	Init_Calibrate_MPU();
	Init_Flash256();

	countx = 0;
	rxdata[0]='0';
	while(countx < 5)
	{
		HAL_UART_Receive(&huart1,rxdata,1,1000);
		if(rxdata[0] == '1')			
		{	
			HAL_UART_Transmit(&huart1,"\nMODE-01",8,10); 	rxdata[0] = '0';	
		}
		else if(rxdata[0] == '2')	{	HAL_UART_Transmit(&huart1,"\nMODE-02",8,10);	countx = 5;	}	
		else if(rxdata[0] == '3')	{	HAL_UART_Transmit(&huart1,"\nMODE-03",8,10);	countx = 5;	}	
		else if(rxdata[0] == '4')	{	Visualize_MPU9250_Data();	countx = 5;		rxdata[0] = '0';	}	
		else											{	HAL_UART_Transmit(&huart1,"\nMODE-XX",8,10);	rxdata[0] = '0';	}	
		countx++;
	}
	if(rxdata[0] == '0')	{	rxdata[0] = '1';	}
	
	HAL_GPIO_WritePin(GPLED3_GPIO_Port,GPLED3_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPLED4_GPIO_Port,GPLED4_Pin,GPIO_PIN_SET);
	HAL_Delay(250);
	HAL_GPIO_WritePin(GPLED3_GPIO_Port,GPLED3_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPLED4_GPIO_Port,GPLED4_Pin,GPIO_PIN_RESET);
	
	memlogstate = '0';
	memlogflag = '0'; memreadflag = '0'; memeraseflag = '0';
	
	Flash_Read_Data(0x00002000-3,memdata1,15);
	if(Buffercmp(memdata1,"[MEMORY   FULL]",16) == 0)	{	memlogflag = '1';		}
	
	Flash_Read_Data(0x00003000-3,memdata2,15);
	if(Buffercmp(memdata1,"[MEMORY   READ]",16) == 0)	{	memreadflag = '1';	}
	
	Flash_Read_Data(0x00004000-3,memdata3,15);
	if(Buffercmp(memdata1,"[MEMORY ERASED]",16) == 0)	{	memeraseflag = '1';	}

	while(1)
	{	
		while(rxdata[0] == '0')
		{
			HAL_UART_Transmit(&huart1,"\n1-STORE  2-RELAY  3-ERASE",26,30);
			HAL_UART_Receive(&huart1,rxdata,1,1000);
		}
		
		if(rxdata[0] == '1')
		{
			Flash_Read_Data(0x00002000-3,memdata1,15);
			if(Buffercmp(memdata1,"[MEMORY   FULL]",15) != 0)
			{
				Flash_Read_Data(0x00004000-3,memdata3,15);
				if(Buffercmp(memdata3,"[MEMORY ERASED]",15) == 0)
				{
//					Read_Store_MPU9250_Data();
					Read_Store_MPU9250_Data1();
					Flash_Write_Data(0x00002000,"[MEMORY   FULL]",15);
				}
				else	
				{
					HAL_UART_Transmit(&huart1,"\nERR: MEMORY NOT ERASED",23,20);	
				}
			}
			else	
			{
				HAL_UART_Transmit(&huart1,"\nERR: MEM ALREADY FULL ",23,20);	
			}
			rxdata[0] = '0';
		}
		
		else if(rxdata[0] == '2')	
		{	
			Flash_Read_Data(0x00002000-3,memdata1,15);
			if(Buffercmp(memdata1,"[MEMORY   FULL]",15) == 0)
			{
				Read_Relay_Accel_Data();
				Flash_Write_Data(0x00003000,"[MEMORY   READ]",15);
			}
			else	
			{
				HAL_UART_Transmit(&huart1,"\nERR: MEM EMPTY",15,15);		
			}
			rxdata[0] = '0';
		}
		
		else if(rxdata[0] == '3')
		{
			Flash_Read_Data(0x00003000-3,memdata2,15);
			if(Buffercmp(memdata2,"[MEMORY   READ]",15) == 0)
			{
				if(Init_Flash256() == 1)		
				{
					HAL_UART_Transmit(&huart1,"\n[Erasing Required Memory]",26,50);	
					uint32_t tempaddress=0x00000000;
					for(int i=0;i<65;i++)
					{
						Flash_Block_Erase(tempaddress);
						HAL_UART_Transmit(&huart1,"\n[",2,2);
						Display_32Bit_Hex(tempaddress);
						HAL_UART_Transmit(&huart1,"-BLOCK ERASED-]",15,10);
						tempaddress = tempaddress + 0x00010000;
					}
					Flash_Write_Data(0x00004000,"[MEMORY ERASED]",15);
					HAL_UART_Transmit(&huart1,"\n[Memory Erasing Complete]",26,50);	
				}
				else	{	HAL_UART_Transmit(&huart1,"\nFlash NOK",10,10);		}
			}
			else	{	HAL_UART_Transmit(&huart1,"\nERR: MEM NOT READ",19,20);		}
			rxdata[0] = '0';
		}
		else if(rxdata[0] == 'X')
		{
				uint32_t tempaddress=0x00000000;
				for(int i=0;i<65;i++)
				{
					Flash_Block_Erase(tempaddress);
					HAL_UART_Transmit(&huart1,"\n[",2,2);
					Display_32Bit_Hex(tempaddress);
					HAL_UART_Transmit(&huart1,"-BLOCK ERASED-]",15,10);
					tempaddress = tempaddress + 0x00010000;
				}
				Flash_Write_Data(0x00004000,"[MEMORY ERASED]",15);
				HAL_UART_Transmit(&huart1,"\n[Memory Erasing Complete]",26,50);			
		}
		else	{	;	}
		rxdata[0] = '0';
	}	
	
	//Simulate_ADXL375_Data();
	//Simulate_MPU9250_Data();
//	Visualize_MPU9250_Data();
//	Read_Store_MPU9250_Data();
	//Read_Relay_Accel_Data();
	while(1)	{	HAL_Delay(200);	}
//	{
//		HAL_Delay(200);
//		HAL_UART_Transmit(&huart1,"\nHello",6,10);
//		HAL_GPIO_TogglePin(GPLED2_GPIO_Port,GPLED2_Pin);
//	}
	SSD1306_Init();
//	SSD1306_Clear();
//	SSD1306_GotoXY(0,20);
//	SSD1306_Puts("INITALIZING",&Font_11x18,1);
//	SSD1306_UpdateScreen();
//	HAL_Delay(1000);
//	Initialize_JDY25_Device();

	SSD1306_Clear();
	SSD1306_GotoXY(0,0);
	SSD1306_Puts("1-THP 2-YPR",&Font_11x18,1);
	SSD1306_GotoXY(0,20);
	SSD1306_Puts("3-XXX 4-XXX",&Font_11x18,1);
	SSD1306_GotoXY(0,40);
	SSD1306_Puts("MODE>",&Font_11x18,1);
	SSD1306_UpdateScreen();
	
	while(1)
	{
		if(HAL_GPIO_ReadPin(TRG01_GPIO_Port,TRG01_Pin) == 0x01)	
		{	
			if(screenflag[0] < '5') 	{ screenflag[0]++;			}
			else 											{ screenflag[0] = '1';	}
			SSD1306_GotoXY(60,40);
			SSD1306_Puts((char*)screenflag,&Font_11x18,1);
			SSD1306_UpdateScreen();			
		}
		if(HAL_GPIO_ReadPin(TRG02_GPIO_Port,TRG02_Pin) == 0x01) 
		{ 
			if(screenflag[0] == '1')
			{
				SSD1306_Clear();
				SSD1306_GotoXY(0,0);
				SSD1306_Puts("T=       dC",&Font_11x18,1);
				SSD1306_GotoXY(0,20);
				SSD1306_Puts("P=       mb",&Font_11x18,1);
				SSD1306_GotoXY(0,40);
				SSD1306_Puts("A=       mt",&Font_11x18,1);
				SSD1306_UpdateScreen();
				Display_MS56611_Data();
			}
			else if(screenflag[0] == '2')
			{
				Display_AHSRS_Data();
			}
			else if(screenflag[0] == '3')
			{
				Display_Target_Proximity();
				HAL_Delay(200);
			}
			else if(screenflag[0] == '4')
			{
			}
			else if(screenflag[0] == '5')
			{
			}
			else		{		SSD1306_Clear();	}
		}
		HAL_Delay(200);
	}	
	//Initialize_RN4871_Device();
	Initialize_JDY25_Device();
	while(1)
	{
		//HAL_UART_Transmit(&huart1,codeinfo,20,10);
		HAL_GPIO_TogglePin(GPLED2_GPIO_Port,GPLED2_Pin);
		HAL_Delay(10);
	}
	while(1)
	{
		Generate_AHSRS_Data();
		HAL_Delay(100);
	}

	Initialize_VL53L0X_Device();
	while(1)
	{
		
		//HAL_UART_Transmit(&huart1,"\nVL53L0X ID=",12,10);
		Display_8Bit_Hex(Read_VL53L0X_Reg(0xC0));
		HAL_GPIO_TogglePin(GPLED2_GPIO_Port,GPLED2_Pin);
		HAL_Delay(1000);
	}

	Initialize_VL6180X_Device();
	while(1)
	{
		
		//HAL_UART_Transmit(&huart1,"\nVL6180X ID=",12,10);
		Display_8Bit_Hex(Read_VL6180X_Reg(0x000));
		Display_8Bit_Hex(Read_VL6180X_Reg(0x001));
		Display_8Bit_Hex(Read_VL6180X_Reg(0x002));
		Display_8Bit_Hex(Read_VL6180X_Reg(0x003));
		Display_8Bit_Hex(Read_VL6180X_Reg(0x004));
		Display_8Bit_Hex(Read_VL6180X_Reg(0x005));
		Display_8Bit_Hex(Read_VL6180X_Reg(0x006));
		Display_8Bit_Hex(Read_VL6180X_Reg(0x007));

		HAL_GPIO_TogglePin(GPLED2_GPIO_Port,GPLED2_Pin);
		HAL_Delay(1000);
	}
	
	Test_SSD1306_Display();
	SSD1306_GotoXY(0,0);
	SSD1306_Puts("T=       dC",&Font_11x18,1);
	SSD1306_GotoXY(0,20);
	SSD1306_Puts("P=       mb",&Font_11x18,1);
	SSD1306_GotoXY(0,40);
	SSD1306_Puts("A=       mt",&Font_11x18,1);
	SSD1306_UpdateScreen();
	while(1)
	{
		Calculate_MS5611_Parameters();
		
		sprintf((char *)showtmp,"%  0.2f",T);		
		SSD1306_GotoXY(15,0);
		SSD1306_Puts(showtmp,&Font_11x18,1);

		sprintf((char *)showtmp,"% 0.2f",P);
		SSD1306_GotoXY(15,20);
		SSD1306_Puts(showtmp,&Font_11x18,1);
		
		sprintf((char *)showtmp,"% 0.2f",A);
		SSD1306_GotoXY(15,40);
		SSD1306_Puts(showtmp,&Font_11x18,1);

		SSD1306_UpdateScreen();
		
		HAL_Delay(500);
	}
	//HAL_UART_Transmit(&huart1,"\nGet Ready to Collect Data",26,100);
	while(1)
	{
		Fine_Distance_Data();
		printf("\nHello");
		HAL_Delay(100);
	}
	
	
//	Init_BME_280(&dev);
//	while(1)
//	{
//		countx = Update_Sensor_Data(&dev);
//		HAL_UART_Transmit(&huart1,"\nBME280=",8,10);		
//		HAL_UART_Transmit(&huart1,compsensordata,countx,100);
//		HAL_Delay(1000);
//	}

//	Init_Adxl375();
//	while(1)
//	{
//		HAL_UART_Transmit(&huart1,"\nADXL=",6,10);
//		countx = Update_Accel_Data();
//		HAL_UART_Transmit(&huart1,compsensordata,countx,100);	
//		HAL_Delay(250);
//	}

//	Init_MPU9250_New();
//	while(1)
//	{
//		HAL_UART_Transmit(&huart1,"\nMPU9250=",8,10);
//		Display_MPU_Values();
//		HAL_Delay(250);
//	}

//	Initialize_TLV493D_Device();
//	while(1)
//	{
//		HAL_UART_Transmit(&huart1,"\n|----------------------|",25,15);
//		Diagnose_TLV493D_Device();
//		HAL_Delay(500);
//	}
	//Initialize_TFMini_Device();
	//Initialize_MS5611_Device();
	//Initialize_RN4871_Device();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_GPIO_TogglePin(GPLED1_GPIO_Port,GPLED1_Pin);
		HAL_GPIO_TogglePin(GPLED2_GPIO_Port,GPLED2_Pin);
		
		HAL_UART_Transmit(&huart1,"\nADXLID=",8,10);
		Display_8Bit_Hex(Read_Adxl375_Id());

		HAL_UART_Transmit(&huart1,"\nMPU ID=",8,10);
		Display_8Bit_Hex(Read_MPU_ID());

		HAL_UART_Transmit(&huart1,"\nW25QID=",8,10);
		Flash_Reset_Device();
		Display_32Bit_Hex(Flash_Jedec_Id());

		HAL_UART_Transmit(&huart1,"\nADXL=",6,10);
		countx = Update_Accel_Data();
		HAL_UART_Transmit(&huart1,compsensordata,countx,100);

		HAL_UART_Transmit(&huart1,"\nMPU9250=",8,10);
		Display_MPU_Values();

		countx = Update_Sensor_Data(&dev);
		HAL_UART_Transmit(&huart1,"\nBME280=",8,10);		
		HAL_UART_Transmit(&huart1,compsensordata,countx,100);		
		
		//Diagnose_Flash_Memory();				
		
		HAL_Delay(100);
		HAL_UART_Transmit(&huart1,"\n|---------------------------------|",36,10);		
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 230400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPLED1_Pin|GPLED2_Pin|GPLED3_Pin|GPLED4_Pin
                          |CS_MPU9250_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CS_W25Q256_Pin|CS_ADXL375_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BNRST_GPIO_Port, BNRST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : GPLED1_Pin GPLED2_Pin */
  GPIO_InitStruct.Pin = GPLED1_Pin|GPLED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : GPLED3_Pin GPLED4_Pin */
  GPIO_InitStruct.Pin = GPLED3_Pin|GPLED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : TRG01_Pin TRG02_Pin */
  GPIO_InitStruct.Pin = TRG01_Pin|TRG02_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : EXINT1_Pin EXINT2_Pin */
  GPIO_InitStruct.Pin = EXINT1_Pin|EXINT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_W25Q256_Pin CS_ADXL375_Pin */
  GPIO_InitStruct.Pin = CS_W25Q256_Pin|CS_ADXL375_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_MPU9250_Pin */
  GPIO_InitStruct.Pin = CS_MPU9250_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_MPU9250_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MODE_Pin */
  GPIO_InitStruct.Pin = MODE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MODE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BNRST_Pin */
  GPIO_InitStruct.Pin = BNRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BNRST_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/******************************************************************************
******************************************************************************/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//  if(GPIO_Pin == TRG01_Pin)							{	if(screenflag < 2) 	{ screenflag++;	}	}
//  else if(GPIO_Pin == TRG02_Pin)				{	if(screenflag < 1) 	{ screenflag--;	}	}
//	else	{	__NOP;	}
}
/******************************************************************************
******************************************************************************/
void Display_MS56611_Data()
{
	char showtemp[10];
	while(1)
	{
	Calculate_MS5611_Parameters();
		
	sprintf((char *)showtemp,"%  0.2f",T);		
	SSD1306_GotoXY(15,0);
	SSD1306_Puts(showtemp,&Font_11x18,1);

	sprintf((char *)showtemp,"% 0.2f",P);
	SSD1306_GotoXY(15,20);
	SSD1306_Puts(showtemp,&Font_11x18,1);
		
	sprintf((char *)showtemp,"% 0.2f",A);
	SSD1306_GotoXY(15,40);
	SSD1306_Puts(showtemp,&Font_11x18,1);

	SSD1306_UpdateScreen();
		
	HAL_Delay(250);
	}
}
/******************************************************************************
******************************************************************************/
void Test_SSD1306_Display()
{
		SSD1306_Init();
		//Test_SSD1306_Display();
		SSD1306_GotoXY(0,20);
		SSD1306_Puts("STG401BSPV11",&Font_11x18,1);
		SSD1306_UpdateScreen();	
		HAL_Delay(1500);

		SSD1306_Clear();	
}
/******************************************************************************
******************************************************************************/
uint8_t Update_Sensor_Data(struct bme280_dev *dev)
{
	uint8_t count=0;
	uint8_t showtemp[10];
	struct bme280_data comp_data;

	bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
	sprintf((char *)showtemp,"%0.4f",comp_data.temperature);
	for(int i=0;i<5;i++)	{ compsensordata[count] = showtemp[i];	count++;}
	compsensordata[count] = ',';	count++;
		
	sprintf((char *)showtemp,"%0.4f",comp_data.humidity);
	for(int i=0;i<5;i++)	{ compsensordata[count] = showtemp[i];	count++;}
	compsensordata[count] = ',';	count++;
			
	sprintf((char *)showtemp,"%6f",((97300 - comp_data.pressure)*7));
	for(int i=0;i<6;i++)	{ compsensordata[count] = showtemp[i];	count++;}
	compsensordata[count] = ',';	count++;		

	compsensordata[count] = 0x0D;	count++;
	compsensordata[count] = 0x0A;	count++;

	return(count);	
}
/***************************************************************************
***************************************************************************/
void Diagnose_Flash_Memory()
{
	uint8_t databuff[256],rxbuff[256];
	for(uint8_t i=0;i<255;i++)
	{
		databuff[i] = i-2;
	}
	
	Flash_Chip_Erase();
	HAL_Delay(1000);
	Flash_Write_Enable();
	Flash_Write_Data(0x00000100,databuff,255);
	HAL_Delay(1000);
	Flash_Write_Disable();
	HAL_UART_Transmit(&huart1,"\n|--Displaying Data Stored in Flash--|",38,100);
	Flash_Read_Data(0x00000100,rxbuff,255);
	for(uint8_t i=0;i<255;i++)
	{
		Display_8Bit_Hex(rxbuff[i]);
		HAL_Delay(5);
	}
	HAL_Delay(2000);
}
/***************************************************************************
***************************************************************************/
void Init_BME_280(struct bme280_dev *dev)
{
  int8_t rslt;
  uint8_t settings_sel;
//  struct bme280_data comp_data;
	
	dev->dev_id = BME280_I2C_ADDR_PRIM;
	dev->intf = BME280_I2C_INTF;
	dev->read = user_i2c_read;
	dev->write = user_i2c_write;
	dev->delay_ms = user_delay_ms;
	rslt = bme280_init(dev);

	/*Normal Mode of Operation*/
	dev->settings.osr_h = BME280_OVERSAMPLING_1X;
	dev->settings.osr_p = BME280_OVERSAMPLING_16X;
	dev->settings.osr_t = BME280_OVERSAMPLING_2X;
	dev->settings.filter = BME280_FILTER_COEFF_16;
	dev->settings.standby_time = BME280_STANDBY_TIME_10_MS;
	
	settings_sel = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_STANDBY_SEL;
	settings_sel |= BME280_FILTER_SEL;
	rslt = bme280_set_sensor_settings(settings_sel, dev);
	rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, dev);
}
/******************************************************************************
******************************************************************************/
void user_delay_ms(uint32_t period)
{
    HAL_Delay(period);
}
/******************************************************************************
******************************************************************************/
int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
  int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
	
	while(HAL_I2C_IsDeviceReady(&hi2c1,0x76<<1,5,100) != HAL_OK);
	HAL_I2C_Mem_Read(&hi2c1,0x76<<1,reg_addr,I2C_MEMADD_SIZE_8BIT,reg_data,len,100);		
  return rslt;
}
/******************************************************************************
******************************************************************************/
int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
  int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
	while(HAL_I2C_IsDeviceReady(&hi2c1,0x76<<1,5,100) != HAL_OK);
	HAL_I2C_Mem_Write(&hi2c1,0x76<<1,reg_addr,I2C_MEMADD_SIZE_8BIT,reg_data,len,100);
  return rslt;
}
/******************************************************************************
******************************************************************************/
//void Generate_DMP_Data()
//{
//	bool dmpReady = false;  // set true if DMP init was successful
//	uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
//	uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
//	uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
//	uint16_t fifoCount;     // count of all bytes currently in FIFO
//	uint8_t fifoBuffer[64]; // FIFO storage buffer

//	// orientation/motion vars
//	float euler[3];         // [psi, theta, phi]    Euler angle container
//	float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//	// packet structure for InvenSense teapot demo
//	uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

//  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
//  // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
//  // the baud timing being too misaligned with processor ticks. You must use
//  // 38400 or slower in these cases, or use some kind of external separate
//  // crystal solution for the UART timer.

//    // initialize device
//    Send_Message("\nInitializing MPU9250",19);
//    MPU6050_initialize();
//    //pinMode(INTERRUPT_PIN, INPUT);

////    // verify connection
////    Serial.println(F("Testing device connections..."));
////    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

//    // load and configure the DMP
//    Send_Message("\nInitializing DMP",17);
//    devStatus = MPU6050_dmpInitialize();

//    // supply your own gyro offsets here, scaled for min sensitivity
//    MPU6050_setXGyroOffset(220);
//    MPU6050_setYGyroOffset(76);
//    MPU6050_setZGyroOffset(-85);
//    MPU6050_setZAccelOffset(1788); // 1688 factory default for my test chip

//    // make sure it worked (returns 0 if so)
//    if (devStatus == 0) {
//        // turn on the DMP, now that it's ready
//        Send_Message("\nEnabling DMP",14);
//        MPU6050_setDMPEnabled(true);

//        // enable Arduino interrupt detection
//        mpuIntStatus = MPU6050_getIntStatus();

//        // set our DMP Ready flag so the main loop() function knows it's okay to use it
//        dmpReady = true;

//        // get expected DMP packet size for later comparison
//        packetSize = MPU6050_dmpGetFIFOPacketSize();
//    } else {
//        // ERROR!
//        // 1 = initial memory load failed
//        // 2 = DMP configuration updates failed
//        // (if it's going to break, usually the code will be 1)
//        Send_Message("\nDMP Initialization failed",26);
////        Serial.print(devStatus);
//    }
//	while(1)
//	{
//    // if programming failed, don't try to do anything
//    if (!dmpReady) return;

//    // wait for MPU interrupt or extra packet(s) available
////    while (!mpuInterrupt && fifoCount < packetSize) {
////        // other program behavior stuff here
////        // .
////        // .
////        // .
////        // if you are really paranoid you can frequently test in between other
////        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
////        // while() loop to immediately process the MPU data
////        // .
////        // .
////        // .
////    }

//    // reset interrupt flag and get INT_STATUS byte
//    //mpuInterrupt = false;
//    mpuIntStatus = MPU6050_getIntStatus();

//    // get current FIFO count
//    fifoCount = MPU6050_getFIFOCount();

//    // check for overflow (this should never happen unless our code is too inefficient)
//    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
//        // reset so we can continue cleanly
//        MPU6050_resetFIFO();
//        Send_Message("\nFIFO overflow!",15);

//    // otherwise, check for DMP data ready interrupt (this should happen frequently)
//    } else if (mpuIntStatus & 0x02) {
//        // wait for correct available data length, should be a VERY short wait
//        while (fifoCount < packetSize) fifoCount = MPU6050_getFIFOCount();

//        // read a packet from FIFO
//        MPU6050_getFIFOBytes(fifoBuffer, packetSize);
//        
//        // track FIFO count here in case there is > 1 packet available
//        // (this lets us immediately read more without waiting for an interrupt)
//        fifoCount -= packetSize;

//        // display quaternion values in InvenSense Teapot demo format:
//			teapotPacket[2] = fifoBuffer[0];
//			teapotPacket[3] = fifoBuffer[1];
//			teapotPacket[4] = fifoBuffer[4];
//			teapotPacket[5] = fifoBuffer[5];
//			teapotPacket[6] = fifoBuffer[8];
//			teapotPacket[7] = fifoBuffer[9];
//			teapotPacket[8] = fifoBuffer[12];
//			teapotPacket[9] = fifoBuffer[13];
//			Send_Message(teapotPacket,14);
//			teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
//    }
//	}		
//}
/******************************************************************************
******************************************************************************/
void Display_AHSRS_Data()
{
	uint8_t showtemp[10],tempdata[7];
	float aclx=0.0,acly=0.0,aclz=0.0;
	float gyrx=0.0,gyry=0.0,gyrz=0.0;
	float magx=0.0,magy=0.0,magz=0.0;
	int16_t dataxx,datayy,datazz;

	//Init_MPU9250_New();
	Init_MPU9250();
	TM_AHRSIMU_Init(&mpuimu,0.15,100,0.0);

	SSD1306_Clear();
	SSD1306_GotoXY(0,0);
	SSD1306_Puts("ORIENTATION",&Font_11x18,1);
	SSD1306_UpdateScreen();
	HAL_Delay(2000);

	SSD1306_Clear();
	SSD1306_GotoXY(0,0);
	SSD1306_Puts("Y=",&Font_11x18,1);
	SSD1306_GotoXY(0,20);
	SSD1306_Puts("R=",&Font_11x18,1);
	SSD1306_GotoXY(0,40);
	SSD1306_Puts("P=",&Font_11x18,1);
	SSD1306_UpdateScreen();

while(1)
	{	
		while(HAL_I2C_IsDeviceReady(&hi2c1,0x68<<1,5,100) != HAL_OK);
		HAL_I2C_Mem_Read(&hi2c1,0x68<<1,0x3B,I2C_MEMADD_SIZE_8BIT,tempdata,6,100);			
		
		dataxx= (((tempdata[0])<<8) + tempdata[1]);
		datayy= (((tempdata[2])<<8) + tempdata[3]);
		datazz= (((tempdata[4])<<8) + tempdata[5]);
			
		aclx=(float)dataxx * (16.0/32768.0);
		acly=(float)datayy * (16.0/32768.0);
		aclz=(float)datazz * (16.0/32768.0);

		while(HAL_I2C_IsDeviceReady(&hi2c1,0x68<<1,5,100) != HAL_OK);
		HAL_I2C_Mem_Read(&hi2c1,0x68<<1,0x43,I2C_MEMADD_SIZE_8BIT,tempdata,6,100);			
		
		dataxx= (((tempdata[0])<<8) + tempdata[1]);
		datayy= (((tempdata[2])<<8) + tempdata[3]);
		datazz= (((tempdata[4])<<8) + tempdata[5]);
			
		gyrx=(float)dataxx * (16.0/32768.0);
		gyry=(float)datayy * (16.0/32768.0);
		gyrz=(float)datazz * (16.0/32768.0);

		if(Read_AK8963_Reg(AK8963_ST1) & 0x01) 
		{
			uint8_t c = tempdata[6];
			while(HAL_I2C_IsDeviceReady(&hi2c1,AK8963_ADDRESS,5,100) != HAL_OK);
			HAL_I2C_Mem_Read(&hi2c1,AK8963_ADDRESS,AK8963_HXL,I2C_MEMADD_SIZE_8BIT,tempdata,6,100);			
			// Check if magnetic sensor overflow set, if not then report data
			if(!(c & 0x08)) 
			{ 
				dataxx = (int16_t)(((int16_t)tempdata[1] << 8) | tempdata[0]); 
				datayy = (int16_t)(((int16_t)tempdata[3] << 8) | tempdata[2]) ; 
				datazz = (int16_t)(((int16_t)tempdata[5] << 8) | tempdata[4]) ;
				
				magx=(float)dataxx * (0.15);
				magy=(float)datayy * (0.15);
				magz=(float)datazz * (0.15);
			}
		}
		TM_AHRSIMU_UpdateAHRS(&mpuimu,gyrx,gyry,gyrz,aclx,acly,aclz,magx,magy,magz);

		HAL_UART_Transmit(&huart1,"\ny",2,10);
		sprintf((char *)showtemp,"%3.3f",mpuimu.Yaw);
		SSD1306_GotoXY(30,0);
		SSD1306_Puts((char *)showtemp,&Font_11x18,1);
		HAL_UART_Transmit(&huart1,showtemp,6,10);
		HAL_UART_Transmit(&huart1,"yp",2,10);
		
		sprintf((char *)showtemp,"%3.3f",mpuimu.Pitch);
		SSD1306_GotoXY(30,20);
		SSD1306_Puts((char *)showtemp,&Font_11x18,1);
		HAL_UART_Transmit(&huart1,showtemp,6,10);
		HAL_UART_Transmit(&huart1,"pr",2,10);
		
		sprintf((char *)showtemp,"%3.3f",mpuimu.Roll);
		SSD1306_GotoXY(30,40);
		SSD1306_Puts((char *)showtemp,&Font_11x18,1);
		HAL_UART_Transmit(&huart1,showtemp,6,10);
		HAL_UART_Transmit(&huart1,"r",1,10);
		
		SSD1306_UpdateScreen();
	
		HAL_Delay(5);
	}
}
/***************************************************************************
***************************************************************************/
void Display_System_Message(char *sysmsg)
{
	SSD1306_Clear();
	SSD1306_GotoXY(0,0);
	SSD1306_Puts("SYS MESSAGE",&Font_11x18,1);
	SSD1306_GotoXY(0,20);
	SSD1306_Puts(sysmsg,&Font_11x18,1);
//	SSD1306_GotoXY(0,40);
//	SSD1306_Puts("           ",&Font_11x18,1);
	SSD1306_UpdateScreen();
	HAL_Delay(1000);
}
/******************************************************************************
******************************************************************************/
void Display_Target_Proximity()
{
	uint8_t rxbuff[9], showtemp[8], tempdata[6];
	float distdata=0.0;
	SSD1306_Clear();
	while(1)
	{
		__HAL_UART_ENABLE(&huart1);
		HAL_UART_Receive(&huart1,rxbuff,9,5);
		__HAL_UART_DISABLE(&huart1);
		if((rxbuff[0] == 0x59) & (rxbuff[1] == 0x59))
		{ 
			distdata = (rxbuff[3] << 8) + rxbuff[2];
			sprintf((char *)showtemp,"%0.1f",distdata);
			SSD1306_GotoXY(0,25);
			SSD1306_Puts((char *)showtemp,&Font_16x26,1);
			SSD1306_GotoXY(85,25);
			SSD1306_Puts("cm",&Font_16x26,1);
			SSD1306_UpdateScreen();
		}
		HAL_Delay(50);
	}
}
/******************************************************************************
******************************************************************************/
uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if ((*pBuffer1) != *pBuffer2)
    {
      return BufferLength;
    }
    pBuffer1++;
    pBuffer2++;
  }

  return 0;
}
/******************************************************************************
Function to Read and Store 1 Minute Acceleration data in External Flash Memory.
Data to be stored in circular buffer for last 1 minute encapsulating the event. 
No of Packets in 1minute= 1000*60 = 60000
Frame Bytes = 1(HDR)+2(CNT)+2(AXLX)+2(AXLY)+2(AXLZ)+2(CRC)+1(FTR)=1+1+5*2=12
******************************************************************************/
void Read_Store_ADXL3X5_Data()
{
	uint32_t pktcount1 = 0, maxcount=100000;
	uint32_t currentaddress1=0x00010000;

	uint8_t pktcountbuf[6], loginfobuf[23],logfilestamp[100],logdatatemp[32],rxbuf[6];
	uint8_t tempbuf[26]={"[-Frames Stored=0000000-]\n"};
	uint8_t crcval=0,status=0,count=0;
	uint16_t aclxx,aclyy,aclzz;
	
	Init_Adxl375();
	status = Init_Flash256();

	if(status == 1)	
	{	
		//Flash_Chip_Erase();
		Flash_Block_Erase(0x00000000);
		Flash_Block_Erase(0x00010000);
		Flash_Block_Erase(0x00020000);
	}
	else	{	HAL_UART_Transmit(&huart1,"\nFlash NOK",10,10);	}

	currentaddress1 = 0x00010000;
	HAL_UART_Transmit(&huart1,"\n[-Logging Data: S Stops-]",26,50);

	while(HAL_GPIO_ReadPin(TRG01_GPIO_Port,TRG01_Pin) != GPIO_PIN_SET)	{	HAL_Delay(10);	}
	HAL_GPIO_WritePin(GPLED1_GPIO_Port,GPLED1_Pin,GPIO_PIN_SET);
	while((pktcount1 < maxcount) && (HAL_GPIO_ReadPin(TRG02_GPIO_Port,TRG02_Pin) == GPIO_PIN_RESET))
	{	
		Read_Adxl375_Regs(ADXL375_ACCLXL_REG,rxbuf,6);
		count=0;
		crcval=0;
		compsensordata[count] = '$';	count++;
		compsensordata[count] = rxbuf[0];	count++;
		compsensordata[count] = rxbuf[1];	count++;
		compsensordata[count] = rxbuf[2];	count++;
		compsensordata[count] = rxbuf[3];	count++;
		compsensordata[count] = rxbuf[4];	count++;
		compsensordata[count] = rxbuf[5];	count++;

		for(uint8_t i=1;i<7;i++)	{	crcval += compsensordata[i];	}	
		compsensordata[count] = crcval;	count++;
		compsensordata[count] = 0x0D;	count++;
		compsensordata[count] = 0x0A;	count++;
		Flash_Write_Data(currentaddress1,compsensordata,count);
		currentaddress1 = currentaddress1 + 10;
		pktcount1++;
		HAL_GPIO_TogglePin(GPLED2_GPIO_Port,GPLED2_Pin);
	}
	HAL_GPIO_WritePin(GPLED1_GPIO_Port,GPLED1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPLED2_GPIO_Port,GPLED2_Pin,GPIO_PIN_RESET);

	loginfobuf[0] = (uint8_t)((pktcount1 & 0xFF000000)>>24);
	loginfobuf[1] = (uint8_t)((pktcount1 & 0x00FF0000)>>16);
	loginfobuf[2] = (uint8_t)((pktcount1 & 0x0000FF00)>>8);
	loginfobuf[3] = (uint8_t)((pktcount1 & 0x000000FF)>>0);
	
	loginfobuf[4] = (uint8_t)((currentaddress1 & 0xFF000000)>>24);
	loginfobuf[5] = (uint8_t)((currentaddress1 & 0x00FF0000)>>16);
	loginfobuf[6] = (uint8_t)((currentaddress1 & 0x0000FF00)>>8);
	loginfobuf[7] = (uint8_t)((currentaddress1 & 0x000000FF)>>0);

	loginfobuf[8] = logfilestamp[12];
	loginfobuf[9] = logfilestamp[13];
	loginfobuf[10] = logfilestamp[14];
	loginfobuf[11] = logfilestamp[15];
	loginfobuf[12] = logfilestamp[16];
	loginfobuf[13] = logfilestamp[17];

	loginfobuf[14] = logfilestamp[3];
	loginfobuf[15] = logfilestamp[4];
	loginfobuf[16] = logfilestamp[5];
	loginfobuf[17] = logfilestamp[6];
	loginfobuf[18] = logfilestamp[9];
	loginfobuf[19] = logfilestamp[10];
	
	crcval = HAL_CRC_Calculate(&hcrc,(uint32_t *)loginfobuf,8);
	loginfobuf[20] = (crcval & 0xFF00)>>8;
	loginfobuf[21] = (crcval & 0x00FF)>>0;
	loginfobuf[22] = 'A';
	Flash_Write_Data(0x00001000,loginfobuf,23);
	
	sprintf((char *)pktcountbuf,"%07d",pktcount1);
	for(uint8_t i=16;i<23;i++)	{	tempbuf[i] = pktcountbuf[i-16];	}
	HAL_UART_Transmit(&huart1,"\n",1,5);
	HAL_UART_Transmit(&huart1,tempbuf,26,5);
	HAL_UART_Transmit(&huart1,"\n",1,5);
	for(uint8_t i=0;i<23;i++)	{	Display_8Bit_Hex(loginfobuf[i]);	}
}
/******************************************************************************
Function to Read and Store 1 Minute Acceleration data in External Flash Memory.
Data to be stored in circular buffer for last 1 minute encapsulating the event. 
No of Packets in 1minute= 1000*60 = 60000
Frame Bytes = 1(HDR)+4(CNT)+2(AXLX)+2(AXLY)+2(AXLZ)+1(CRC)+1("\r"+1("\n")=1+4+6+1+2=14
******************************************************************************/
void Read_Store_MPU9250_Data()
{
	uint32_t pktcount1 = 0, maxcount=300000;			//100000pkts/1400000bytes require 24blocks of 64KB
	uint32_t currentaddress1=0x00010000;

	uint8_t pktcountbuf[6], showtemp[10],loginfobuf[23],logfilestamp[100],logdatatemp[32],rxbuf[7];
	uint8_t tempbuf[26]={"[-Frames Stored=0000000-]\n"};
	uint8_t crcval=0,status=0,count=0;
	uint16_t aclxx,aclyy,aclzz;

	uint8_t regaddress=0,offsetcount=0;
	
	Init_MPU9250_New();
	
	HAL_UART_Transmit(&huart1,"\n[-Waiting for User  Key-]",26,25);

	HAL_GPIO_WritePin(GPLED1_GPIO_Port,GPLED1_Pin,GPIO_PIN_SET);
	while(HAL_GPIO_ReadPin(TRG01_GPIO_Port,TRG01_Pin) == GPIO_PIN_RESET)	{	HAL_Delay(10);	}
	HAL_Delay(1000);
	while(HAL_GPIO_ReadPin(TRG01_GPIO_Port,TRG01_Pin) == GPIO_PIN_SET)	{	HAL_Delay(10);	}

	HAL_GPIO_WritePin(GPLED3_GPIO_Port,GPLED3_Pin,GPIO_PIN_SET );
	currentaddress1 = 0x00010000;
	HAL_UART_Transmit(&huart1,"\n[-Logging 100secs  DATA-]",26,25);
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPLED3_GPIO_Port,GPLED3_Pin,GPIO_PIN_RESET );
	
	HAL_GPIO_WritePin(GPLED1_GPIO_Port,GPLED1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPLED2_GPIO_Port,GPLED2_Pin,GPIO_PIN_SET);
	while(pktcount1 < maxcount)
	{
		
		Read_MPU_Regs(MPU9250_ACCEL_XOUT_H,rxbuf,6);

		count=0; crcval=0;
		compsensordata[count] = '$';	count++;
		compsensordata[count] = (pktcount1 & 0xFF000000)>>24;	count++;
		compsensordata[count] = (pktcount1 & 0x00FF0000)>>16;	count++;
		compsensordata[count] = (pktcount1 & 0x0000FF00)>>8;	count++;
		compsensordata[count] = (pktcount1 & 0x000000FF)>>0;	count++;
		compsensordata[count] = rxbuf[0];	count++;
		compsensordata[count] = rxbuf[1];	count++;
		compsensordata[count] = rxbuf[2];	count++;
		compsensordata[count] = rxbuf[3];	count++;
		compsensordata[count] = rxbuf[4];	count++;
		compsensordata[count] = rxbuf[5];	count++;

		for(uint8_t i=1;i<11;i++)	{	crcval += compsensordata[i];	}	
		compsensordata[count] = crcval;	count++;
		compsensordata[count] = 0x0D;	count++;
		compsensordata[count] = 0x0A;	count++;
		Flash_Write_Data(currentaddress1,compsensordata,count);
		currentaddress1 = currentaddress1 + 14;
		pktcount1++;
	}
	HAL_GPIO_WritePin(GPLED4_GPIO_Port,GPLED4_Pin,GPIO_PIN_SET );
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPLED4_GPIO_Port,GPLED4_Pin,GPIO_PIN_RESET );

	HAL_GPIO_WritePin(GPLED1_GPIO_Port,GPLED1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPLED2_GPIO_Port,GPLED2_Pin,GPIO_PIN_RESET);

	loginfobuf[0] = (uint8_t)((pktcount1 & 0xFF000000)>>24);
	loginfobuf[1] = (uint8_t)((pktcount1 & 0x00FF0000)>>16);
	loginfobuf[2] = (uint8_t)((pktcount1 & 0x0000FF00)>>8);
	loginfobuf[3] = (uint8_t)((pktcount1 & 0x000000FF)>>0);
	
	loginfobuf[4] = (uint8_t)((currentaddress1 & 0xFF000000)>>24);
	loginfobuf[5] = (uint8_t)((currentaddress1 & 0x00FF0000)>>16);
	loginfobuf[6] = (uint8_t)((currentaddress1 & 0x0000FF00)>>8);
	loginfobuf[7] = (uint8_t)((currentaddress1 & 0x000000FF)>>0);

	loginfobuf[8] = logfilestamp[12];
	loginfobuf[9] = logfilestamp[13];
	loginfobuf[10] = logfilestamp[14];
	loginfobuf[11] = logfilestamp[15];
	loginfobuf[12] = logfilestamp[16];
	loginfobuf[13] = logfilestamp[17];

	loginfobuf[14] = logfilestamp[3];
	loginfobuf[15] = logfilestamp[4];
	loginfobuf[16] = logfilestamp[5];
	loginfobuf[17] = logfilestamp[6];
	loginfobuf[18] = logfilestamp[9];
	loginfobuf[19] = logfilestamp[10];
	
	crcval = HAL_CRC_Calculate(&hcrc,(uint32_t *)loginfobuf,8);
	loginfobuf[20] = (crcval & 0xFF00)>>8;
	loginfobuf[21] = (crcval & 0x00FF)>>0;
	loginfobuf[22] = 'M';
	Flash_Write_Data(0x00001000,loginfobuf,23);
	
//	sprintf((char *)pktcountbuf,"%07d",pktcount1);
//	for(uint8_t i=16;i<23;i++)	{	tempbuf[i] = pktcountbuf[i-16];	}
//	HAL_UART_Transmit(&huart1,"\n",1,5);
//	HAL_UART_Transmit(&huart1,tempbuf,26,5);
//	HAL_UART_Transmit(&huart1,"\n",1,5);
//	for(uint8_t i=0;i<23;i++)	{	Display_8Bit_Hex(loginfobuf[i]);	}
	HAL_UART_Transmit(&huart1,"\n[-Logging Data Stopped -]",26,25);
}
/******************************************************************************
Function to Read and Store 1 Minute Acceleration data in External Flash Memory.
Data to be stored in circular buffer for last 1 minute encapsulating the event. 
No of Packets in 1minute= 1000*60 = 60000
Frame Bytes = 1(HDR)+4(CNT)+2(AXLX)+2(AXLY)+2(AXLZ)+1(CRC)+1("\r"+1("\n")=1+4+6+1+2=14
******************************************************************************/
void Read_Store_MPU9250_Data1()
{
	uint32_t pktcount1 = 0, maxcount=30000;			//100000pkts/1400000bytes require 24blocks of 64KB
	uint32_t currentaddress1=0x00010000;

	uint8_t pktcountbuf[6], showtemp[10],loginfobuf[23],logfilestamp[100],logdatatemp[32],rxbuf[7];
	uint8_t tempbuf[26]={"[-Frames Stored=0000000-]\n"};
	uint8_t crcval=0,status=0,count=0;
	uint16_t aclxx,aclyy,aclzz;

	uint8_t regaddress=0,offsetcount=0;
	
	Init_MPU9250_New();
	Init_Adxl375();
	
	HAL_UART_Transmit(&huart1,"\n[-Waiting for User  Key-]",26,25);

	HAL_GPIO_WritePin(GPLED1_GPIO_Port,GPLED1_Pin,GPIO_PIN_SET);
	while(HAL_GPIO_ReadPin(TRG01_GPIO_Port,TRG01_Pin) == GPIO_PIN_RESET)	{	HAL_Delay(10);	}
	HAL_Delay(1000);
	while(HAL_GPIO_ReadPin(TRG01_GPIO_Port,TRG01_Pin) == GPIO_PIN_SET)	{	HAL_Delay(10);	}

	HAL_GPIO_WritePin(GPLED3_GPIO_Port,GPLED3_Pin,GPIO_PIN_SET );
	currentaddress1 = 0x00010000;
	HAL_UART_Transmit(&huart1,"\n[-Logging 100secs  DATA-]",26,25);
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPLED3_GPIO_Port,GPLED3_Pin,GPIO_PIN_RESET );
	
	HAL_GPIO_WritePin(GPLED1_GPIO_Port,GPLED1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPLED2_GPIO_Port,GPLED2_Pin,GPIO_PIN_SET);
	while(pktcount1 < maxcount)
	{
		
		Read_MPU_Regs(MPU9250_ACCEL_XOUT_H,rxbuf,6);

		count=0; crcval=0;
		compsensordata[count] = '$';	count++;
		compsensordata[count] = (pktcount1 & 0xFF000000)>>24;	count++;
		compsensordata[count] = (pktcount1 & 0x00FF0000)>>16;	count++;
		compsensordata[count] = (pktcount1 & 0x0000FF00)>>8;	count++;
		compsensordata[count] = (pktcount1 & 0x000000FF)>>0;	count++;
		compsensordata[count] = rxbuf[0];	count++;
		compsensordata[count] = rxbuf[1];	count++;
		compsensordata[count] = rxbuf[2];	count++;
		compsensordata[count] = rxbuf[3];	count++;
		compsensordata[count] = rxbuf[4];	count++;
		compsensordata[count] = rxbuf[5];	count++;

		for(uint8_t i=1;i<11;i++)	{	crcval += compsensordata[i];	}	
		compsensordata[count] = crcval;	count++;
		compsensordata[count] = 0x0D;	count++;
		compsensordata[count] = 0x0A;	count++;
		Flash_Write_Data(currentaddress1,compsensordata,count);
		currentaddress1 = currentaddress1 + 14;
		//pktcount1++;

		Read_Adxl375_Regs(ADXL375_ACCLXL_REG,rxbuf,6);

		count=0; crcval=0;
		compsensordata[count] = '@';	count++;
		compsensordata[count] = (pktcount1 & 0xFF000000)>>24;	count++;
		compsensordata[count] = (pktcount1 & 0x00FF0000)>>16;	count++;
		compsensordata[count] = (pktcount1 & 0x0000FF00)>>8;	count++;
		compsensordata[count] = (pktcount1 & 0x000000FF)>>0;	count++;
		compsensordata[count] = rxbuf[0];	count++;
		compsensordata[count] = rxbuf[1];	count++;
		compsensordata[count] = rxbuf[2];	count++;
		compsensordata[count] = rxbuf[3];	count++;
		compsensordata[count] = rxbuf[4];	count++;
		compsensordata[count] = rxbuf[5];	count++;

		for(uint8_t i=1;i<11;i++)	{	crcval += compsensordata[i];	}	
		compsensordata[count] = crcval;	count++;
		compsensordata[count] = 0x0D;	count++;
		compsensordata[count] = 0x0A;	count++;
		Flash_Write_Data(currentaddress1,compsensordata,count);
		currentaddress1 = currentaddress1 + 14;

		pktcount1++;
	}
	HAL_GPIO_WritePin(GPLED4_GPIO_Port,GPLED4_Pin,GPIO_PIN_SET );
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPLED4_GPIO_Port,GPLED4_Pin,GPIO_PIN_RESET );

	HAL_GPIO_WritePin(GPLED1_GPIO_Port,GPLED1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPLED2_GPIO_Port,GPLED2_Pin,GPIO_PIN_RESET);

	loginfobuf[0] = (uint8_t)((pktcount1 & 0xFF000000)>>24);
	loginfobuf[1] = (uint8_t)((pktcount1 & 0x00FF0000)>>16);
	loginfobuf[2] = (uint8_t)((pktcount1 & 0x0000FF00)>>8);
	loginfobuf[3] = (uint8_t)((pktcount1 & 0x000000FF)>>0);
	
	loginfobuf[4] = (uint8_t)((currentaddress1 & 0xFF000000)>>24);
	loginfobuf[5] = (uint8_t)((currentaddress1 & 0x00FF0000)>>16);
	loginfobuf[6] = (uint8_t)((currentaddress1 & 0x0000FF00)>>8);
	loginfobuf[7] = (uint8_t)((currentaddress1 & 0x000000FF)>>0);

	loginfobuf[8] = logfilestamp[12];
	loginfobuf[9] = logfilestamp[13];
	loginfobuf[10] = logfilestamp[14];
	loginfobuf[11] = logfilestamp[15];
	loginfobuf[12] = logfilestamp[16];
	loginfobuf[13] = logfilestamp[17];

	loginfobuf[14] = logfilestamp[3];
	loginfobuf[15] = logfilestamp[4];
	loginfobuf[16] = logfilestamp[5];
	loginfobuf[17] = logfilestamp[6];
	loginfobuf[18] = logfilestamp[9];
	loginfobuf[19] = logfilestamp[10];
	
	crcval = HAL_CRC_Calculate(&hcrc,(uint32_t *)loginfobuf,8);
	loginfobuf[20] = (crcval & 0xFF00)>>8;
	loginfobuf[21] = (crcval & 0x00FF)>>0;
	loginfobuf[22] = 'M';
	Flash_Write_Data(0x00001000,loginfobuf,23);
	
	HAL_UART_Transmit(&huart1,"\n[-Logging Data Stopped -]",26,25);
}
/******************************************************************************
Function to Read and Relay Stored Acceleration data for analysis.
******************************************************************************/
void Read_Relay_Accel_Data()
{
	uint32_t finaladdress=0,readaddress=0x00010000;
	uint32_t pktcount=0,readaddress1=0x00010000;
	uint8_t logdatatemp[32],showtemp[10];
	uint8_t tempbuf[26]={"\n[,X,0000000,,1234561122,]"};
	int logcount=0;
	uint8_t logdata[14],rlymod='0',dataokflag=0,count=0,crcval=0;
	HAL_Delay(1000);
	

	HAL_UART_Transmit(&huart1,"\n[-Get Ready To Log Data-]",26,50);	
	Flash_Read_Data(0x00001000-3,logdatatemp,23);
	
	//for(uint8_t i=0;i<23;i++)	{	Display_8Bit_Hex(logdatatemp[i]);	}
	if((logdatatemp[22] == 'A') || (logdatatemp[22] == 'M'))
	{
		logcount = (logcount + logdatatemp[0])<<8;
		logcount = (logcount + logdatatemp[1])<<8;
		logcount = (logcount + logdatatemp[2])<<8;
		logcount = (logcount + logdatatemp[3])<<0;
			
		finaladdress = (finaladdress + logdatatemp[4])<<8;
		finaladdress = (finaladdress + logdatatemp[5])<<8;
		finaladdress = (finaladdress + logdatatemp[6])<<8;
		finaladdress = (finaladdress + logdatatemp[7])<<0;
		
		tempbuf[3] = '$';

		sprintf((char *)logdatatemp,"%07d",logcount);
		for(uint8_t i=0;i<7;i++)		{	tempbuf[i+5] = logdatatemp[i];	}

		//for(uint8_t i=13;i<23;i++)	{	tempbuf[i] = logdatatemp[i-5];	}
		
		//HAL_Delay(1000);
		//HAL_UART_Transmit(&huart1,tempbuf,26,20);
			
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPLED2_GPIO_Port,GPLED2_Pin,GPIO_PIN_SET);
		crcval=0;
		while(logcount != 0)
		{
			crcval=0;
			Flash_Read_Data(readaddress1-3,logdata,14);
			for(uint8_t i=1;i<11;i++)	{	crcval += logdata[i];	}
			if(crcval == logdata[11])
			{				
				count = Convert_MPU_Data1(logdata,logcount);
				HAL_UART_Transmit(&huart1,compsensordata,count,10);
			}
			readaddress1 = readaddress1 + 14;

			crcval=0;
			Flash_Read_Data(readaddress1-3,logdata,14);
			for(uint8_t i=1;i<11;i++)	{	crcval += logdata[i];	}
			if(crcval == logdata[11])
			{				
				count = Convert_ADXL_Data1(logdata,logcount);
				HAL_UART_Transmit(&huart1,compsensordata,count,10);
			}
			readaddress1 = readaddress1 + 14;
			logcount--;
			//HAL_Delay(500);
		}
		HAL_GPIO_WritePin(GPLED2_GPIO_Port,GPLED2_Pin,GPIO_PIN_RESET);
		HAL_UART_Transmit(&huart1,"\n[---Stored  Data Ends---]",26,50);
	}
	else 
	{	
		HAL_UART_Transmit(&huart1,"\n[-Valid Data  Not Found-]",26,50);	
	}
}
/******************************************************************************
******************************************************************************/
uint8_t Convert_ADXL_Data1(uint8_t *rxbuf,int logcountval)
{
	uint8_t count=0, showtemp[10];
	int16_t aclxx,aclyy,aclzz,crcval=0;
	uint32_t framecount=0;
	//float offsetx=0.0,offsety=0.0,offsetz=0.0;
	float acclx=0.0,accly=0.0,acclz=0.0;

	framecount = (rxbuf[1]<<24) + (rxbuf[2]<<16) + (rxbuf[3]<<8) + (rxbuf[4]<<0);
	
	aclxx= (rxbuf[5]<<8) + rxbuf[6];
	aclyy= (rxbuf[7]<<8) + rxbuf[8];
	aclzz= (rxbuf[9]<<8) + rxbuf[10];
	
	acclx=((((float)aclxx) * (0.00049)) + offsetax);
	accly=((((float)aclyy) * (0.00049)) + offsetay);
	acclz=((((float)aclzz) * (0.00049)) + offsetaz);
	
	compsensordata[count] = '@';	count++;
	
	sprintf((char *)showtemp,"%06d",framecount);
	for(int i=0;i<6;i++)
	{	compsensordata[count] = showtemp[i];	count++;	}
	compsensordata[count] = ',';	count++;
	
	sprintf((char *)showtemp,"%0.4f",acclx);
	for(int i=0;i<5;i++)
	{	compsensordata[count] = showtemp[i];	count++;	}
	compsensordata[count] = ',';	count++;
	
	sprintf((char *)showtemp,"%0.4f",accly);
	for(int i=0;i<5;i++)
	{	compsensordata[count] = showtemp[i];	count++;	}
	compsensordata[count] = ',';	count++;

	sprintf((char *)showtemp,"%0.4f",acclz);
	for(int i=0;i<5;i++)
	{	compsensordata[count] = showtemp[i];	count++;	}
//	compsensordata[count] = ',';	count++;

//	crcval = HAL_CRC_Calculate(&hcrc,(uint32_t *)compsensordata,count++);
//	compsensordata[count] = (uint8_t)((crcval & 0xFF00)>>8);	count++;
//	compsensordata[count] = (uint8_t)((crcval & 0x00FF)>>0);	count++;
	
//	compsensordata[count] = '}';	count++;
	compsensordata[count] = '\n';	count++;
	
	return(count);
}
/******************************************************************************
******************************************************************************/
uint8_t Convert_MPU_Data1(uint8_t *rxbuf,int logcountval)
{
	uint8_t count=0, showtemp[10];
	int16_t aclxx,aclyy,aclzz,crcval=0;
	uint32_t framecount=0;
//	float offsetx=0.0,offsety=0.0,offsetz=0.0;	
	float acclx=0.0,accly=0.0,acclz=0.0;
	
	framecount = (rxbuf[1]<<24) + (rxbuf[2]<<16) + (rxbuf[3]<<8) + (rxbuf[4]<<0);

	aclxx= (rxbuf[5]<<8) + rxbuf[6];
	aclyy= (rxbuf[7]<<8) + rxbuf[8];
	aclzz= (rxbuf[9]<<8) + rxbuf[10];
	
	acclx=(((float)aclxx) * (0.00049)) + offsetmx;
	accly=(((float)aclyy) * (0.00049)) + offsetmy;
	acclz=(((float)aclzz) * (0.00049)) + offsetmz;
	
	
	compsensordata[count] = '$';	count++;
	
	sprintf((char *)showtemp,"%06d",framecount);
	for(int i=0;i<6;i++)
	{	compsensordata[count] = showtemp[i];	count++;	}
	compsensordata[count] = ',';	count++;
	
	sprintf((char *)showtemp,"%0.4f",acclx);
	for(int i=0;i<5;i++)
	{	compsensordata[count] = showtemp[i];	count++;	}
	compsensordata[count] = ',';	count++;
	
	sprintf((char *)showtemp,"%0.4f",accly);
	for(int i=0;i<5;i++)
	{	compsensordata[count] = showtemp[i];	count++;	}
	compsensordata[count] = ',';	count++;

	sprintf((char *)showtemp,"%0.4f",acclz);
	for(int i=0;i<5;i++)
	{	compsensordata[count] = showtemp[i];	count++;	}
//	compsensordata[count] = ',';	count++;

//	crcval = HAL_CRC_Calculate(&hcrc,(uint32_t *)compsensordata,count++);
//	compsensordata[count] = (uint8_t)((crcval & 0xFF00)>>8);	count++;
//	compsensordata[count] = (uint8_t)((crcval & 0x00FF)>>0);	count++;
	
//	compsensordata[count] = '}';	count++;
	compsensordata[count] = '\n';	count++;
	
	return(count);
}
/***************************************************************************
***************************************************************************/
void Update_Comportflag()
{
	uint8_t count=0,connstatus='0',rxbuf[5];
	comportflag = '0';
	while(count < 5)
	{
		for(int i=0;i<5;i++)	{	rxbuf[i] = 0;	}
		HAL_UART_Transmit(&huart1,"GK",2,10);
		HAL_UART_Receive(&huart1,rxbuf,6,1000);
		HAL_UART_Transmit(&huart1,rxbuf,6,20);
//		if(Buffercmp(rxbuf,"%CONN",6) == 0)			{	COMPORT = "&huart1";	}
//		else	{	comportflag = '1';	}
		count++;
	}
	if(comportflag == '2')			{	HAL_UART_Transmit(&huart1,"\nBLE  OK",8,10);	}	
	else 												{	HAL_UART_Transmit(&huart1,"\nBLE NOK",8,10);	}
}
/***************************************************************************
***************************************************************************/
void Visualize_MPU9250_Data()
{
	uint8_t showtemp[10],rxbuf[6];
	uint8_t regaddress=0,crcval=0,offsetcount=0,count=0;
	
	int16_t aclxx,aclyy,aclzz;
	uint16_t pktcount=1;
	
	float acclx=0.0,accly=0.0,acclz=0.0;
//	float offsetx=3.18,offsety=0.0,offsetz=0.180;
	
	Init_MPU9250_New();
	
	while(offsetcount < 100)
	{
		for(int i=0;i<6;i++)	{	rxbuf[i] = 0;	}
		Read_MPU_Regs(MPU9250_ACCEL_XOUT_H,rxbuf,6);
		aclxx= (rxbuf[0]<<8) + rxbuf[1];
		aclyy= (rxbuf[2]<<8) + rxbuf[3];
		aclzz= (rxbuf[4]<<8) + rxbuf[5];

		acclx=(float)aclxx * 0.00049 + offsetmx;
		accly=(float)aclyy * 0.00049 + offsetmy;
		acclz=(float)aclzz * 0.00049 + offsetmz;

//		offsetxx += ((float)aclxx * 0.00049);
//		offsetyy += ((float)aclyy * 0.00049);
//		offsetzz += ((float)aclzz * 0.00049);

		count=0;
		compsensordata[count] = '$';	count++;

		sprintf((char *)showtemp,"%05d",pktcount);
		for(int i=0;i<5;i++)	{	compsensordata[count] = showtemp[i];	count++;	}
		compsensordata[count] = ',';	count++;
		
		sprintf((char *)showtemp,"%0.4f",acclx);
		for(int i=0;i<5;i++)	{	compsensordata[count] = showtemp[i];	count++;	}
		compsensordata[count] = ',';	count++;
		
		sprintf((char *)showtemp,"%0.4f",accly);
		for(int i=0;i<5;i++)	{	compsensordata[count] = showtemp[i];	count++;	}
		compsensordata[count] = ',';	count++;

		sprintf((char *)showtemp,"%0.4f",acclz);
		for(int i=0;i<5;i++)	{	compsensordata[count] = showtemp[i];	count++;	}		
		
		compsensordata[count] = '\n';	count++;	

		HAL_UART_Transmit(&huart1,compsensordata,count,20);
		pktcount++;
		offsetcount++;
		HAL_Delay(100);
	}
}
/******************************************************************************
******************************************************************************/
void Init_Calibrate_MPU()
{
	
}
/******************************************************************************
******************************************************************************/
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

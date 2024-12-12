#include "mpu9250mkm.h"
/******************************************************************************
******************************************************************************/
// Set initial input parameters
enum Ascale {
AFS_2G = 0,
AFS_4G,
AFS_8G,
AFS_16G
};
enum Gscale {
GFS_250DPS = 0,
GFS_500DPS,
GFS_1000DPS,
GFS_2000DPS
};
enum Mscale {
MFS_14BITS = 0, // 0.6 mG per LSB
MFS_16BITS // 0.15 mG per LSB
};
uint8_t Ascale = AFS_16G; // AFS_2G, AFS_4G, AFS_8G, AFS_16G
uint8_t Gscale = GFS_250DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
uint8_t Mscale = MFS_16BITS; // MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x06; // Either 8 Hz 0x02) or 100 Hz (0x06) magnetometer data ODR

float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}; // Bias corrections for gyro and accelerometer
int16_t tempCount;   // Stores the real internal chip temperature in degrees Celsius
float SelfTest[6];

/******************************************************************************
******************************************************************************/
extern uint16_t axl375offsetx;
extern uint16_t axl375offsety;
extern uint16_t axl375offsetz;
extern uint8_t gtrtmr;
/******************************************************************************
******************************************************************************/
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart1;
extern void Display_8Bit_Hex(uint8_t);
extern void Display_Binary(uint8_t);
extern void Display_16Bit_Hex(uint16_t hexval);
extern void Display_32Bit_Hex(uint32_t hexval);
/******************************************************************************
INITIALIZATION STEPS:
1. Reset MPU9250
2. Calibrate accel and gyro, load biases in bias registers
3. wait
4. Initialize mpu9250
4.1	Clear sleep mode bit (6), enable all sensors
4.2	
******************************************************************************/
uint8_t Init_MPU9250()
{
	uint8_t mpuid=0;
	// sleep off
	Write_MPU_Reg(MPU9250_PWR_MGMT_1, 0x00);
	HAL_Delay(100);
	// auto select clock source
	Write_MPU_Reg(MPU9250_PWR_MGMT_1, 0x01);
	HAL_Delay(200);
	// DLPF_CFG
	Write_MPU_Reg(MPU9250_CONFIG, 0x03);
	// sample rate divider
	Write_MPU_Reg(MPU9250_SMPLRT_DIV, 0x04);
	// gyro full scale select
	Write_MPU_Reg( MPU9250_GYRO_CONFIG, 0x18);
	// accel full scale select
	Write_MPU_Reg(MPU9250_ACCEL_CONFIG, 0x18);
	// A_DLPFCFG
	Write_MPU_Reg(MPU9250_ACCEL_CONFIG_2, 0x03);
	// BYPASS_EN
	Write_MPU_Reg(MPU9250_INT_PIN_CFG, 0x02);
	
	HAL_Delay(100);
	
	mpuid= Read_MPU_Reg(MPU9250_WHO_AM_I);
	if( mpuid != 0x71)	
	{	
		HAL_UART_Transmit(&huart1,"\nMPU ID=",8,10);
		Display_8Bit_Hex(mpuid);
		return(0);
	}
	return(1);
}
/******************************************************************************
******************************************************************************/
uint8_t Mpu9250_Freefall_Initialization()
{
	uint8_t mpuid=0;
	// sleep off
	Write_MPU_Reg(MPU9250_PWR_MGMT_1, 0x00);
	HAL_Delay(100);
	// auto select clock source
	Write_MPU_Reg(MPU9250_PWR_MGMT_1, 0x01);
	HAL_Delay(200);
	// DLPF_CFG
	Write_MPU_Reg(MPU9250_CONFIG, 0x03);
	// sample rate divider
	Write_MPU_Reg(MPU9250_SMPLRT_DIV, 0x04);
	// gyro full scale select
	Write_MPU_Reg( MPU9250_GYRO_CONFIG, 0x18);
	// accel full scale select
	Write_MPU_Reg(MPU9250_ACCEL_CONFIG, 0x18);
	// A_DLPFCFG
	Write_MPU_Reg(MPU9250_ACCEL_CONFIG_2, 0x03);
	// BYPASS_EN
	Write_MPU_Reg(MPU9250_INT_PIN_CFG, 0x02);
	
	HAL_Delay(100);
	
	mpuid= Read_MPU_Reg(MPU9250_WHO_AM_I);
	if( mpuid != 0x71)	
	{	
		HAL_UART_Transmit(&huart1,"\nMPU ID=",8,10);
		Display_8Bit_Hex(mpuid);
		return(0);
	}
	return(1);	
}
/******************************************************************************
******************************************************************************/
void Mpu9250_Detect_FreeFall()
{
	uint8_t temp1[6];
	uint16_t adxlvalx=0,adxlvaly=0,adxlvalz=0;
	uint16_t ffval=0x0044;
	HAL_UART_Transmit(&huart1,"\nFree Detection Started",23,10);
	while(1)
	{
		while(HAL_I2C_IsDeviceReady(&hi2c1,0x68<<1,5,100) != HAL_OK);
		HAL_I2C_Mem_Read(&hi2c1,0x68<<1,0x3B,I2C_MEMADD_SIZE_8BIT,temp1,6,100);			

		adxlvalx = ((temp1[0]<<8) +  temp1[1]);
		adxlvaly = ((temp1[2]<<8) +  temp1[3]);
		adxlvalz = ((temp1[4]<<8) +  temp1[5]);
					
		if(adxlvalx > 0x8000)	{	adxlvalx = (~adxlvalx) + 0x0001;	}
		if(adxlvaly > 0x8000)	{	adxlvaly = (~adxlvaly) + 0x0001;	}
		if(adxlvalz > 0x8000)	{	adxlvalz = (~adxlvalz) + 0x0001;	}

//		if(adxlvalx < ffval	&& adxlvaly < ffval &&adxlvalz < ffval)	{	Send_Message("\nFree Fall Detected",19);	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_13,1);	}
//		else																												{	Send_Message("\nNo Free Fall Found",19);	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_13,0);	}
		if(adxlvalz < ffval) {	if(adxlvaly < ffval)	{	if(adxlvalx < ffval)	{	HAL_UART_Transmit(&huart1,"\nFree Fall Detected",19,10);	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_13,1);	}	}	}
//		if(adxlvalz < ffval)	{	Send_Message("\nFree Fall Detected",19);	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_13,1);	}	
		HAL_Delay(10);
	}
		//{	Display_16Bit_Hex(temp2[0]);	recordflag = 1;	}
}
/******************************************************************************
******************************************************************************/
void Diagnose_Mpu9250()
{
	uint8_t mpuid=0;
	while(1)
	{
		HAL_UART_Transmit(&huart1,"\nProbing MPU9250",16,100);
		mpuid= Read_MPU_Reg(MPU9250_WHO_AM_I);
		Display_8Bit_Hex(mpuid);
		mpuid = 0;
		HAL_Delay(1000);
	}	
}
/******************************************************************************
******************************************************************************/
void calibrateMPU9250(float * dest1, float * dest2)
{  
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  
// reset device, reset all registers, clear gyro and accelerometer bias registers
  Write_MPU_Reg(MPU9250_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  HAL_Delay(100);  
   
// get stable time source
// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  Write_MPU_Reg(MPU9250_PWR_MGMT_1, 0x01);  
  Write_MPU_Reg(MPU9250_PWR_MGMT_2, 0x00); 
  HAL_Delay(200);
  
// Configure device for bias calculation
  Write_MPU_Reg(MPU9250_INT_ENABLE, 0x00);   // Disable all interrupts
  Write_MPU_Reg(MPU9250_FIFO_EN, 0x00);      // Disable FIFO
  Write_MPU_Reg(MPU9250_PWR_MGMT_1, 0x00);   // Turn on internal clock source
  Write_MPU_Reg(MPU9250_I2C_MST_CTRL, 0x00); // Disable I2C master
  Write_MPU_Reg(MPU9250_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  Write_MPU_Reg(MPU9250_USER_CTRL, 0x0C);    // Reset FIFO and DMP
  HAL_Delay(15);
  
// Configure MPU9250 gyro and accelerometer for bias calculation
  Write_MPU_Reg(MPU9250_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  Write_MPU_Reg(MPU9250_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  Write_MPU_Reg(MPU9250_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  Write_MPU_Reg(MPU9250_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
 
  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

// Configure FIFO to capture accelerometer and gyro data for bias calculation
  Write_MPU_Reg(MPU9250_USER_CTRL, 0x40);   // Enable FIFO  
  Write_MPU_Reg(MPU9250_FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-9250)
  HAL_Delay(40); // accumulate 40 samples in 80 milliseconds = 480 bytes

// At end of sample accumulation, turn off FIFO sensor read
  Write_MPU_Reg(MPU9250_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  Read_MPU_Regs(MPU9250_FIFO_COUNTH,data,2); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    Read_MPU_Regs(MPU9250_FIFO_R_W,data,12); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
    
    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
            
}
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;
    
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}
 
// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;

/// Push gyro biases to hardware registers
/*  writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);
*/
  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  Read_MPU_Regs(MPU9250_XA_OFFSET_H,data,2); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  Read_MPU_Regs(MPU9250_YA_OFFSET_H,data,2);
  accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  Read_MPU_Regs(MPU9250_ZA_OFFSET_H,data,2);
  accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  
  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
  
  for(ii = 0; ii < 3; ii++) {
    if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);
 
  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

// Apparently this is not working for the acceleration biases in the MPU-9250
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
/*  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
*/
// Output scaled accelerometer biases for manual subtraction in the main program
   dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
   dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
   dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}
/******************************************************************************
******************************************************************************/
uint8_t Init_MPU9250_New()
{
	uint8_t status=0;
	uint8_t data[12];	
	uint16_t xcount, packet_count, fifo_count;
	int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
	float gyrocorr[3] = {0, 0, 0}, accelcorr[3] = {0, 0, 0}, magcorr[3] = {0, 0, 0};
	
	if(Read_MPU_Reg(MPU9250_WHO_AM_I) == 0x71)
	{

		//Reset registers to default in preparation for device calibration
		Write_MPU_Reg(MPU9250_PWR_MGMT_1, 0x80);
		HAL_Delay(100);
		
		// Calibrate gyro and accelerometers, load biases in bias registers
		Write_MPU_Reg(MPU9250_PWR_MGMT_1, 0x80);
		HAL_Delay(100);
		
		// get stable time source
		// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
		Write_MPU_Reg(MPU9250_PWR_MGMT_1, 0x01);
		Write_MPU_Reg(MPU9250_PWR_MGMT_2, 0x00);
		HAL_Delay(200);
		
		// Configure device for bias calculation
		Write_MPU_Reg(MPU9250_INT_ENABLE, 0x00); // Disable all interrupts
		Write_MPU_Reg(MPU9250_FIFO_EN, 0x00); // Disable FIFO
		Write_MPU_Reg(MPU9250_PWR_MGMT_1, 0x00); // Turn on internal clock source
		Write_MPU_Reg(MPU9250_I2C_MST_CTRL, 0x00); // Disable I2C master
		Write_MPU_Reg(MPU9250_USER_CTRL, 0x00); // Disable FIFO and I2C master modes
		Write_MPU_Reg(MPU9250_USER_CTRL, 0x0C); // Reset FIFO and DMP
		HAL_Delay(15);
		
		// Configure MPU9250 gyro and accelerometer for bias calculation
		Write_MPU_Reg(MPU9250_CONFIG, 0x01); // Set low-pass filter to 188 Hz
		Write_MPU_Reg(MPU9250_SMPLRT_DIV, 0x00); // Set sample rate to 1 kHz
		Write_MPU_Reg(MPU9250_GYRO_CONFIG, 0x00); // Set gyro full-scale to 250 degrees per second, maximum sensitivity
		
		
		Write_MPU_Reg(MPU9250_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
		uint16_t gyrosensitivity = 131; // = 131 LSB/degrees/sec
		uint16_t accelsensitivity = 16384; // = 16384 LSB/g
		// Configure FIFO to capture accelerometer and gyro data for bias calculation
		Write_MPU_Reg(MPU9250_USER_CTRL, 0x40); // Enable FIFO
		Write_MPU_Reg(MPU9250_FIFO_EN, 0x78); // Enable gyro and accelerometer sensors	for FIFO (max size 512 bytes in MPU-9250)
		HAL_Delay(40); // accumulate 40 samples in 80 milliseconds = 480 bytes
		// At end of sample accumulation, turn off FIFO sensor read
		Write_MPU_Reg(MPU9250_FIFO_EN, 0x00); // Disable gyro and accelerometer 	sensors for FIFO

		// read FIFO sample count
		while(HAL_I2C_IsDeviceReady(&hi2c1,0x68<<1,5,100) != HAL_OK);
		HAL_I2C_Mem_Read(&hi2c1,0x68<<1,MPU9250_FIFO_COUNTH,I2C_MEMADD_SIZE_8BIT,data,2,100);

		fifo_count = ((uint16_t)data[0] << 8) | data[1];
		packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
		
		for (uint16_t xcount = 0; xcount < packet_count; xcount++) 
		{
			int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
			// read data for averaging
			while(HAL_I2C_IsDeviceReady(&hi2c1,0x68<<1,5,100) != HAL_OK);
			HAL_I2C_Mem_Read(&hi2c1,0x68<<1,MPU9250_FIFO_R_W,I2C_MEMADD_SIZE_8BIT,data,12,100);
			accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1] ) ; // Form signed 16-bit integer for each sample in FIFO
			accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3] ) ;
			accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5] ) ;
			gyro_temp[0] = (int16_t) (((int16_t)data[6] << 8) | data[7] ) ;
			gyro_temp[1] = (int16_t) (((int16_t)data[8] << 8) | data[9] ) ;
			gyro_temp[2] = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
			accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
			accel_bias[1] += (int32_t) accel_temp[1];
			accel_bias[2] += (int32_t) accel_temp[2];
			gyro_bias[0] += (int32_t) gyro_temp[0];
			gyro_bias[1] += (int32_t) gyro_temp[1];
			gyro_bias[2] += (int32_t) gyro_temp[2];
		}
		accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
		accel_bias[1] /= (int32_t) packet_count;
		accel_bias[2] /= (int32_t) packet_count;
		gyro_bias[0] /= (int32_t) packet_count;
		gyro_bias[1] /= (int32_t) packet_count;
		gyro_bias[2] /= (int32_t) packet_count;
		// Remove gravity from the z-axis accelerometer bias calculation
		if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;} 
		else {accel_bias[2] += (int32_t) accelsensitivity;}
		// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
		data[0] = (-gyro_bias[0]/4 >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
		data[1] = (-gyro_bias[0]/4) & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
		data[2] = (-gyro_bias[1]/4 >> 8) & 0xFF;
		data[3] = (-gyro_bias[1]/4) & 0xFF;
		data[4] = (-gyro_bias[2]/4 >> 8) & 0xFF;
		data[5] = (-gyro_bias[2]/4) & 0xFF;
		
		/// Push gyro biases to hardware registers
		/* writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
		writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
		writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
		
		writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
		writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
		writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);
		*/
		
		gyrocorr[0] = (float) gyro_bias[0]/(float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
		gyrocorr[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
		gyrocorr[2] = (float) gyro_bias[2]/(float) gyrosensitivity;
		
		/* Construct the accelerometer biases for push to the hardware accelerometer
		bias registers. These registers contain factory trim values which must be
		added to the calculated accelerometer biases; on boot up these registers will
		hold non-zero values. In addition, bit 0 of the lower byte must be preserved
		since it is used for temperature compensation calculations. Accelerometer 
		bias registers expect bias input as 2048 LSB per g, so that the accelerometer
		biases calculated above must be divided by 8.*/
		
		int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim 	biases
		
		// Read factory accelerometer trim values
		while(HAL_I2C_IsDeviceReady(&hi2c1,0x68<<1,5,100) != HAL_OK);
		HAL_I2C_Mem_Read(&hi2c1,0x68<<1,MPU9250_XA_OFFSET_H,I2C_MEMADD_SIZE_8BIT,data,2,100);
		accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
		
		while(HAL_I2C_IsDeviceReady(&hi2c1,0x68<<1,5,100) != HAL_OK);
		HAL_I2C_Mem_Read(&hi2c1,0x68<<1,MPU9250_YA_OFFSET_H,I2C_MEMADD_SIZE_8BIT,data,2,100);
		accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
		
		while(HAL_I2C_IsDeviceReady(&hi2c1,0x68<<1,5,100) != HAL_OK);
		HAL_I2C_Mem_Read(&hi2c1,0x68<<1,MPU9250_ZA_OFFSET_H,I2C_MEMADD_SIZE_8BIT,data,2,100);
		accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];
		
		
		uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
		uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
		for(xcount = 0; xcount < 3; xcount++) 
		{
		if(accel_bias_reg[xcount] & mask) mask_bit[xcount] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
		}
		
		// Construct total accelerometer bias, including calculated average accelerometer bias from above
		// Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
		accel_bias_reg[0] -= (accel_bias[0]/8); 
		accel_bias_reg[1] -= (accel_bias[1]/8);
		accel_bias_reg[2] -= (accel_bias[2]/8);
		data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
		// preserve temperature compensation bit when writing back to accelerometer bias registers
		data[1] = (accel_bias_reg[0]) & 0xFF;
		data[1] = data[1] | mask_bit[0]; 
		data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
		// preserve temperature compensation bit when writing back to accelerometer bias registers
		data[3] = (accel_bias_reg[1]) & 0xFF;
		data[3] = data[3] | mask_bit[1]; 
		data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
		data[5] = (accel_bias_reg[2]) & 0xFF;
		// preserve temperature compensation bit when writing back to accelerometer bias registers
		data[5] = data[5] | mask_bit[2]; 
		
		/* Apparently this is not working for the acceleration biases in the
		MPU-9250 Are we handling the temperature correction bit properly? Push
		accelerometer biases to hardware registers*/
		/* writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
		writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
		writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
		writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
		writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
		writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
		*/
		// Output scaled accelerometer biases for manual subtraction in the main program
		accelcorr[0] = (float)accel_bias[0]/(float)accelsensitivity;
		accelcorr[1] = (float)accel_bias[1]/(float)accelsensitivity;
		accelcorr[2] = (float)accel_bias[2]/(float)accelsensitivity;
		
	//	Send_Message("\ngyrocorr[0]=",13);
	//	sprintf((char *)data,"%.2f",gyrocorr[0]);
	//	Send_Message(data,5);
	//	Send_Message("\ngyrocorr[1]=",13);
	//	sprintf((char *)data,"%.2f",gyrocorr[1]);
	//	Send_Message(data,5);
	//	Send_Message("\ngyrocorr[2]=",13);
	//	sprintf((char *)data,"%.2f",gyrocorr[2]);
	//	Send_Message(data,5);
	//	
	//	Send_Message("\nacclcorr[0]=",13);
	//	sprintf((char *)data,"%.2f",accelcorr[0]);
	//	Send_Message(data,5);
	//	Send_Message("\nacclcorr[1]=",13);
	//	sprintf((char *)data,"%.2f",accelcorr[1]);
	//	Send_Message(data,5);
	//	Send_Message("\nacclcorr[2]=",13);
	//	sprintf((char *)data,"%.2f",accelcorr[2]);
	//	Send_Message(data,5);

		/* Clear sleep mode bit (6), enable all sensors
		Delay 100 ms for PLL to get established on x-axis
		gyro; should check for PLL ready interrupt*/
		Write_MPU_Reg(MPU9250_PWR_MGMT_1, 0x00);
		HAL_Delay(100);
		
		// get stable time source
		// Set clock source to be PLL with  x-axis gyroscope reference, bits 2:0 = 001
		Write_MPU_Reg(MPU9250_PWR_MGMT_1, 0x01); 
		// Configure Gyro and Accelerometer
		// Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
		// DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
		// Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
		Write_MPU_Reg(MPU9250_CONFIG, 0x03);
		// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
		Write_MPU_Reg(MPU9250_SMPLRT_DIV, 0x04); // Use a 200 Hz rate; the same rate set in CONFIG above
		// Set gyroscope full scale range
		// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
		uint8_t c = Read_MPU_Reg(MPU9250_GYRO_CONFIG); // get current GYRO_CONFIG register value
		// c = c & ~0xE0; // Clear self-test bits [7:5]
		c = c & ~0x02; // Clear Fchoice bits [1:0]
		c = c & ~0x18; // Clear AFS bits [4:3]
		c = c | Gscale << 3; // Set full scale range for the gyro
		// c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
		Write_MPU_Reg(MPU9250_GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register
		// Set accelerometer full-scale range configuration
		c = Read_MPU_Reg(MPU9250_ACCEL_CONFIG); // get current ACCEL_CONFIG register value
		// c = c & ~0xE0; // Clear self-test bits [7:5]
		c = c & ~0x18; // Clear AFS bits [4:3]
		c = c | Ascale << 3; // Set full scale range for the accelerometer
		Write_MPU_Reg(MPU9250_ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value
		// Set accelerometer sample rate configuration
		// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
		// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
		c = Read_MPU_Reg(MPU9250_ACCEL_CONFIG_2); // get current ACCEL_CONFIG2 register value
		c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
		c = c | 0x03; // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
		Write_MPU_Reg(MPU9250_ACCEL_CONFIG_2, c); // Write new ACCEL_CONFIG2 register value
		// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
		// but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting
		// Configure Interrupts and Bypass Enable
		// Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
		// can join the I2C bus and all can be controlled by the Arduino as master
		Write_MPU_Reg(MPU9250_INT_PIN_CFG, 0x22);
		Write_MPU_Reg(MPU9250_INT_ENABLE, 0x01); // Enable data ready (bit 0) interrupt	

		// First extract the factory calibration for each magnetometer axis
		uint8_t rawData[3]; // x/y/z gyro calibration data stored here
		Write_AK8963_Reg(AK8963_CNTL1, 0x00); // Power down magnetometer
		HAL_Delay(10);
		Write_AK8963_Reg(AK8963_CNTL1, 0x0F); // Enter Fuse ROM access mode
		HAL_Delay(10);
		
		// Read the x-, y-, and z-axis calibration values
		while(HAL_I2C_IsDeviceReady(&hi2c1,0x0C<<1,5,100) != HAL_OK);
		HAL_I2C_Mem_Read(&hi2c1,0x0C<<1,AK8963_ASAX,I2C_MEMADD_SIZE_8BIT,rawData,3,100);
		// Return x-axis sensitivity adjustment values, etc.
		magcorr[0] = (float)(rawData[0] - 128)/256.0f + 1.0f; 
		magcorr[1] = (float)(rawData[1] - 128)/256.0f + 1.0f;
		magcorr[2] = (float)(rawData[2] - 128)/256.0f + 1.0f;
		Write_AK8963_Reg(AK8963_CNTL1, 0x00); // Power down magnetometer
		HAL_Delay(10);
		/* Configure the magnetometer for continuous read and highest resolution
		set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
		and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz
		and 0110 for 100 Hz sample rates */
		// Set magnetometer data resolution and sample ODR
		Write_AK8963_Reg(AK8963_CNTL1, Mscale << 4 | Mmode); 
		HAL_Delay(10);
		
		//Send_Message("\nAK8963 initialized for active data mode",40);
		status = 1;
	}
	else	{	HAL_UART_Transmit(&huart1,"\nMPU Not Found",14,10);	status = 0;	}
	return(status);
}
/******************************************************************************
THIS FUNCTION WAITS TILL A FIXED VALUE OF g IS ACHIEVED. THIS SI BASICALLY DONE
TO RECORD ONLY DURING TIME WINDOW OF INTEREST AND HAVE ONLY MEANINGFULL DATA
******************************************************************************/
void Mpu9250_Trigger_Wait()
{
	uint8_t temp1[6];
	uint16_t temp2[3];
	uint8_t recordflag=0,recordflagx=0,recordflagy=0,recordflagz=0;
	uint8_t count=0;
	uint16_t gtrgvalue=0;
	gtrgvalue = 0x0400 * (gtrtmr&0x0F);
	
	while(recordflag == 0)
	{
		while(HAL_I2C_IsDeviceReady(&hi2c1,0x68<<1,5,100) != HAL_OK);
		HAL_I2C_Mem_Read(&hi2c1,0x68<<1,0x3B,I2C_MEMADD_SIZE_8BIT,temp1,6,100);			

		temp2[0] = ((temp1[0]<<8) +  temp1[1]);
		temp2[1] = ((temp1[2]<<8) +  temp1[3]);
		temp2[2] = ((temp1[4]<<8) +  temp1[5]);
				
		if(temp2[0] > 0x8000)	{	temp2[0] = (~temp2[0]) + 0x0001;	}
		if(temp2[1] > 0x8000)	{	temp2[1] = (~temp2[1]) + 0x0001;	}
		if(temp2[2] > 0x8000)	{	temp2[2] = (~temp2[2]) + 0x0001;	}

		if(temp2[0] > gtrgvalue)	{	recordflag = 1;	}	//{	Display_16Bit_Hex(temp2[0]);	recordflag = 1;	}
		if(temp2[1] > gtrgvalue)	{	recordflag = 1;	}	//{	Display_16Bit_Hex(temp2[1]);	recordflag = 1;	}
		if(temp2[2] > gtrgvalue)	{	recordflag = 1;	}	//{	Display_16Bit_Hex(temp2[2]);	recordflag = 1;	}
	}
}
/******************************************************************************
******************************************************************************/
void MPU_Burst_Read()
{
	uint8_t rxbuffer[20];
	uint8_t floatdata[10];
	while(HAL_I2C_IsDeviceReady(&hi2c1,0x68<<1,5,100) != HAL_OK);
	HAL_I2C_Mem_Read(&hi2c1,0x68<<1,0x3B,I2C_MEMADD_SIZE_8BIT,rxbuffer,6,100);			
	
	sprintf((char *)floatdata,"%.4g",((((uint16_t)rxbuffer[0])<<8) + rxbuffer[1]));
}
/******************************************************************************
******************************************************************************/
void Clear_MPU_Int()
{
	Read_MPU_Reg(MPU9250_INT_STATUS);
}
/******************************************************************************
******************************************************************************/
uint8_t Read_MPU_ID()
{
	uint8_t rxbuffer=0x00;
	while(HAL_I2C_IsDeviceReady(&hi2c1,0x68<<1,5,100) != HAL_OK);
	HAL_I2C_Mem_Read(&hi2c1,0x68<<1,0x75,I2C_MEMADD_SIZE_8BIT,&rxbuffer,1,100);		
	
	return(rxbuffer);
}
/******************************************************************************
******************************************************************************/
float Get_MPU_Temp() 
{
	int16_t tmc;
	float tempval=0.0;
	tmc = ((Read_MPU_Reg(MPU9250_TEMP_OUT_H)<<8) + (Read_MPU_Reg(MPU9250_TEMP_OUT_L)));
	tempval = ((float)tmc / 333.87) + 21.0;
	return(tempval);
}
/******************************************************************************
******************************************************************************/
float Get_MPU_AX()
{
	int16_t axc;
	float axval=0.0;
	axc = ((Read_MPU_Reg(MPU9250_ACCEL_XOUT_H)<<8) + (Read_MPU_Reg(MPU9250_ACCEL_XOUT_L)));
	axval =  (float)axc * (16.0/32768.0);
	return(axval);
}
/******************************************************************************
******************************************************************************/
float Get_MPU_AY()
{
	int16_t ayc;
	float ayval=0.0;
	ayc = ((Read_MPU_Reg(MPU9250_ACCEL_YOUT_H)<<8) + (Read_MPU_Reg(MPU9250_ACCEL_YOUT_L)));
	ayval =  (float)ayc * (16.0/32768.0);
	return(ayval);
}
/******************************************************************************
******************************************************************************/
float Get_MPU_AZ()
{
	int16_t azc;
	float azval=0.0;
	azc = ((Read_MPU_Reg(MPU9250_ACCEL_ZOUT_H)<<8) + (Read_MPU_Reg(MPU9250_ACCEL_ZOUT_L)));
	azval =  (float)azc * (16.0/32768.0);
	return(azval);
}
/******************************************************************************
******************************************************************************/
float Get_MPU_GX()
{
	int16_t grx;
	float gxval=0.0;
	grx = ((Read_MPU_Reg(MPU9250_GYRO_XOUT_H)<<8) + (Read_MPU_Reg(MPU9250_GYRO_XOUT_L)));
	gxval =  (float)grx * (2000.0/32768.0);
	return(gxval);
}
/******************************************************************************
******************************************************************************/
float Get_MPU_GY()
{
	int16_t gry;
	float gyval=0.0;
	gry = ((Read_MPU_Reg(MPU9250_GYRO_YOUT_H)<<8) + (Read_MPU_Reg(MPU9250_GYRO_YOUT_L)));
	gyval =  (float)gry* (2000.0/32768.0);
	return(gyval);
}
/******************************************************************************
******************************************************************************/
float Get_MPU_GZ()
{
	int16_t grz;
	float gzval=0.0;
	grz = ((Read_MPU_Reg(MPU9250_GYRO_ZOUT_H)<<8) + (Read_MPU_Reg(MPU9250_GYRO_ZOUT_L)));
	gzval =  (float)grz * (2000.0/32768.0);
	return(gzval);
}

/******************************************************************************
******************************************************************************/
void Display_MPU_Values()
{
	uint8_t showtemp[10];
	uint8_t temp[1];
	uint8_t c,rawdata[7];

	HAL_UART_Transmit(&huart1,"\nTEMP=",6,100);
	sprintf((char *)showtemp,"%3.2f",Get_MPU_Temp());
	HAL_UART_Transmit(&huart1,showtemp,5,100);
	
	HAL_UART_Transmit(&huart1,"  ACX=",6,100);
	sprintf((char *)showtemp,"%3.2f",Get_MPU_AX());
	HAL_UART_Transmit(&huart1,showtemp,5,100);
	
	HAL_UART_Transmit(&huart1,"  ACY=",6,100);
	sprintf((char *)showtemp,"%3.2f",Get_MPU_AY());
	HAL_UART_Transmit(&huart1,showtemp,5,100);
	
	HAL_UART_Transmit(&huart1,"  ACZ=",6,100);
	sprintf((char *)showtemp,"%.4f",Get_MPU_AZ());
	HAL_UART_Transmit(&huart1,showtemp,5,100);
	
	HAL_UART_Transmit(&huart1,"  GYX=",6,100);
	sprintf((char *)showtemp,"%.4f",Get_MPU_GX());
	HAL_UART_Transmit(&huart1,showtemp,5,100);
	
	HAL_UART_Transmit(&huart1,"  GYY=",6,100);
	sprintf((char *)showtemp,"%.4f",Get_MPU_GY());
	HAL_UART_Transmit(&huart1,showtemp,5,100);
	
	HAL_UART_Transmit(&huart1,"  GYZ=",6,100);
	sprintf((char *)showtemp,"%.4f",Get_MPU_GZ());
	HAL_UART_Transmit(&huart1,showtemp,5,100);

//	if(Read_AK8963_Reg(AK8963_ST1) & 0x01) 
//	{
//		while(HAL_I2C_IsDeviceReady(&hi2c1,AK8963_ADDRESS,5,100) != HAL_OK);
//		HAL_I2C_Mem_Read(&hi2c1,AK8963_ADDRESS,AK8963_HXL,I2C_MEMADD_SIZE_8BIT,rawdata,7,100);			
//		
//		if(!(rawdata[6] & 0x08)) 
//		{ 
//			HAL_UART_Transmit(&huart1,"  MGX=",6,100);
//			sprintf((char *)showtemp,"%.2f",(float)((rawdata[1] << 8) | rawdata[0]));
//			HAL_UART_Transmit(&huart1,showtemp,7,100);

//			HAL_UART_Transmit(&huart1,"  MGY=",6,100);
//			sprintf((char *)showtemp,"%.2f",(float)((rawdata[3] << 8) | rawdata[2]));
//			HAL_UART_Transmit(&huart1,showtemp,7,100);

//			HAL_UART_Transmit(&huart1,"  MGZ=",6,100);
//			sprintf((char *)showtemp,"%.2f",(float)((rawdata[5] << 8) | rawdata[4]));
//			HAL_UART_Transmit(&huart1,showtemp,7,100);
//		}
//	}
}
/******************************************************************************
******************************************************************************/
void Read_Magnetic_Data()
{
	uint8_t rawData[7]; 
	float magx=0.0,magy=0.0,magz=0.0;
	double magsig=0.0;
	uint8_t showtemp[10];
	int16_t magdata[3];
	uint32_t count=0;
	
	// x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
	while(1)
	{
	
	if(Read_AK8963_Reg(AK8963_ST1) & 0x01) 
	{
		while(HAL_I2C_IsDeviceReady(&hi2c1,AK8963_ADDRESS,5,100) != HAL_OK);
		HAL_I2C_Mem_Read(&hi2c1,AK8963_ADDRESS,AK8963_HXL,I2C_MEMADD_SIZE_8BIT,rawData,7,100);			
		// Read the six raw data and ST2 registers sequentially into data array
		uint8_t c = rawData[6];
		// End data read by reading ST2 register
		// Check if magnetic sensor overflow set, if not then report data
		if(!(c & 0x08)) 
		{ 
			magdata[0] = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]); 
			// Turn the MSB and LSB into a signed 16-bit value
			magdata[1] = (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]) ; 
			// Data stored as little Endian
			magdata[2] = (int16_t)(((int16_t)rawData[5] << 8) | rawData[4]) ;
		}
	}
//	Send_Message("\nMAGX(mG)=",10);
	magx=((float)magdata[0]) * 0.15;
//	sprintf((char *)showtemp,"%.2f",magx);
//	Send_Message(showtemp,6);
	
//	Send_Message("\nMAGY(mG)=",10);
	magy=((float)magdata[1]) * 0.15;
//	sprintf((char *)showtemp,"%.2f",magy);
//	Send_Message(showtemp,6);

//	Send_Message("\nMAGZ(mG)=",10);
	magz=((float)magdata[2]) * 0.15;
//	sprintf((char *)showtemp,"%.2f",magz);
//	Send_Message(showtemp,6);
	
//	magsig = sqrt((magx*magx) + (magy*magy) + (magz*magz));
	magsig = __sqrtf((magx*magx)+(magy*magy)+(magz*magz));
	if(magsig >= 200)	{	count++;	}
	
	HAL_UART_Transmit(&huart1,"\nMAGSIG=",8,10);
	sprintf((char *)showtemp,"%4.2f",magsig);
	HAL_UART_Transmit(&huart1,showtemp,7,10);

	HAL_UART_Transmit(&huart1,"\tCount=",7,10);
	sprintf((char *)showtemp,"%10d",count);
	HAL_UART_Transmit(&huart1,showtemp,10,10);
	HAL_Delay(5);
	}
}
/******************************************************************************
******************************************************************************/
uint8_t Read_MPU_Reg(uint8_t regaddress)
{
	uint8_t rxbuffer[1];
	while(HAL_I2C_IsDeviceReady(&hi2c1,MPU9250_ADDRESS,5,100) != HAL_OK);
	HAL_I2C_Mem_Read(&hi2c1,MPU9250_ADDRESS,regaddress,I2C_MEMADD_SIZE_8BIT,rxbuffer,1,100);		
	return(rxbuffer[0]);
}
/******************************************************************************
******************************************************************************/
void Read_MPU_Regs(uint8_t regaddress, uint8_t *rxbuf, uint8_t bytecount)
{
	while(HAL_I2C_IsDeviceReady(&hi2c1,MPU9250_ADDRESS,5,100) != HAL_OK);
	HAL_I2C_Mem_Read(&hi2c1,MPU9250_ADDRESS,regaddress,I2C_MEMADD_SIZE_8BIT,rxbuf,bytecount,100);		
}
/******************************************************************************
******************************************************************************/
void Write_MPU_Reg(uint8_t regaddress,uint8_t regvalue)
{
	uint8_t rxbuffer[1];
	rxbuffer[0] = regvalue;
	while(HAL_I2C_IsDeviceReady(&hi2c1,MPU9250_ADDRESS,5,100) != HAL_OK);
	HAL_I2C_Mem_Write(&hi2c1,MPU9250_ADDRESS,regaddress,I2C_MEMADD_SIZE_8BIT,rxbuffer,1,100);
}
/******************************************************************************
******************************************************************************/
uint8_t Read_AK8963_Reg(uint8_t regaddress)
{
	uint8_t rxbuffer[1];
	while(HAL_I2C_IsDeviceReady(&hi2c1,MPU9250_ADDRESS,5,100) != HAL_OK);
	HAL_I2C_Mem_Read(&hi2c1,AK8963_ADDRESS,regaddress,I2C_MEMADD_SIZE_8BIT,rxbuffer,1,100);		
	return(rxbuffer[0]);
}
/******************************************************************************
******************************************************************************/
void Write_AK8963_Reg(uint8_t regaddress,uint8_t regvalue)
{
	uint8_t rxbuffer[1];
	rxbuffer[0] = regvalue;
	while(HAL_I2C_IsDeviceReady(&hi2c1,MPU9250_ADDRESS,5,100) != HAL_OK);
	HAL_I2C_Mem_Write(&hi2c1,AK8963_ADDRESS,regaddress,I2C_MEMADD_SIZE_8BIT,rxbuffer,1,100);
}
/*************************************************************************************
Code Designed & Developed by MKM for STG401V11 in AUG 2024
*************************************************************************************/

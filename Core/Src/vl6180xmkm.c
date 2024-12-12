#include "vl6180xmkm.h"
/******************************************************************************
******************************************************************************/
extern I2C_HandleTypeDef hi2c1;
/******************************************************************************
******************************************************************************/
extern uint8_t distance;
/******************************************************************************
******************************************************************************/
void Initialize_VL6180X_Device()
{
		//while(HAL_I2C_IsDeviceReady(&hi2c1,VL6180X_ADDRESS,5,100) != HAL_OK)	{	;	}
    /* Mandatory: Private registers. */
    Write_VL6180X_Reg(0x0207, 0x01);
    Write_VL6180X_Reg(0x0208, 0x01);
    Write_VL6180X_Reg(0x0096, 0x00);
    Write_VL6180X_Reg(0x0097, 0xfd);
    Write_VL6180X_Reg(0x00e3, 0x00);
    Write_VL6180X_Reg(0x00e4, 0x04);
    Write_VL6180X_Reg(0x00e5, 0x02);
    Write_VL6180X_Reg(0x00e6, 0x01);
    Write_VL6180X_Reg(0x00e7, 0x03);
    Write_VL6180X_Reg(0x00f5, 0x02);
    Write_VL6180X_Reg(0x00d9, 0x05);
    Write_VL6180X_Reg(0x00db, 0xce);
    Write_VL6180X_Reg(0x00dc, 0x03);
    Write_VL6180X_Reg(0x00dd, 0xf8);
    Write_VL6180X_Reg(0x009f, 0x00);
    Write_VL6180X_Reg(0x00a3, 0x3c);
    Write_VL6180X_Reg(0x00b7, 0x00);
    Write_VL6180X_Reg(0x00bb, 0x3c);
    Write_VL6180X_Reg(0x00b2, 0x09);
    Write_VL6180X_Reg(0x00ca, 0x09);
    Write_VL6180X_Reg(0x0198, 0x01);
    Write_VL6180X_Reg(0x01b0, 0x17);
    Write_VL6180X_Reg(0x01ad, 0x00);
    Write_VL6180X_Reg(0x00ff, 0x05);
    Write_VL6180X_Reg(0x0100, 0x05);
    Write_VL6180X_Reg(0x0199, 0x05);
    Write_VL6180X_Reg(0x01a6, 0x1b);
    Write_VL6180X_Reg(0x01ac, 0x3e);
    Write_VL6180X_Reg(0x01a7, 0x1f);
    Write_VL6180X_Reg(0x0030, 0x00);

    /* Recommended : Public registers - See data sheet for more detail */

    /* Enables polling for New Sample ready when measurement completes */
    Write_VL6180X_Reg(0x0011, 0x10);
    /* Set the averaging sample period (compromise between lower noise and increased execution time) */
    Write_VL6180X_Reg(0x010a, 0x30);
    /* Sets the light and dark gain (upper nibble). Dark gain should not be changed.*/
    Write_VL6180X_Reg(0x003f, 0x46);
    /* sets the # of range measurements after which auto calibration of system is performed */
    Write_VL6180X_Reg(0x0031, 0xFF);
    /* Set ALS integration time to 100ms */
    Write_VL6180X_Reg(0x0040, 0x63);
    /* perform a single temperature calibration of the ranging sensor */
    Write_VL6180X_Reg(0x002e, 0x01);
}
/******************************************************************************
******************************************************************************/
uint8_t vl6180x_measure_distance()
{
    uint8_t status;

    /* Wait for device ready. */
    do {
        status = Read_VL6180X_Reg(VL6180X_RESULT_RANGE_STATUS);
    } while ((status & (1 << 0)) == 0);

    /* Start measurement. */
    Write_VL6180X_Reg(VL6180X_SYSRANGE_START, 0x01);

    /* Wait for measurement ready. */
    HAL_Delay(100);

    /* Read result. */
    distance = Read_VL6180X_Reg(VL6180X_RESULT_RANGE_VAL);

    /* Clear interrupt flags. */
    Write_VL6180X_Reg(VL6180X_SYSTEM_INTERRUPT_CLEAR, 0x07);

    /* Wait for device ready. */
    do {
        status = Read_VL6180X_Reg(VL6180X_RESULT_RANGE_STATUS);
    } while ((status & (1 << 0)) == 0);

    /* Return error code. */
    return (status >> 4);
}
/******************************************************************************
******************************************************************************/
uint8_t Read_VL6180X_Reg(uint16_t regaddress)
{
	uint8_t rxbuffer[1];
	//while(HAL_I2C_IsDeviceReady(&hi2c1,VL6180X_ADDRESS,5,100) != HAL_OK);
	HAL_I2C_Mem_Read(&hi2c1,VL6180X_ADDRESS,regaddress,I2C_MEMADD_SIZE_8BIT,rxbuffer,1,100);
	return(rxbuffer[0]);
}
/******************************************************************************
******************************************************************************/
void Write_VL6180X_Reg(uint16_t regaddress,uint8_t regvalue)
{
	uint8_t txbuffer[1];
	txbuffer[0] = regvalue;
	//while(HAL_I2C_IsDeviceReady(&hi2c1,VL6180X_ADDRESS,5,100) != HAL_OK);
	HAL_I2C_Mem_Write(&hi2c1,VL6180X_ADDRESS,regaddress,I2C_MEMADD_SIZE_8BIT,txbuffer,1,100);
}
/*************************************************************************************
Code Designed & Developed by MKM for STG401V11 in AUG 2024
*************************************************************************************/

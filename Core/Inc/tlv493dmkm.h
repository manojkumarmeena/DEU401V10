#ifndef TLV493DMKM_H
#define TLV493DMKM_H

#include "main.h"
#include "utilfuncmkm.h"
#include "stdio.h"
/******************************************************************************
******************************************************************************/
// TLV493 device address
#define TLV493D_ADDRESS	0x5E<<1 //0b1110111
/******************************************************************************
******************************************************************************/
void Initialize_TLV493D_Device(void);
void Reset_TLV493D_Device(void);
void Read_TLV493D_Device(void);
float Get_Mag_X(void);
float Get_Mag_Y(void);
float Get_Mag_Z(void);
float Get_Mag_Temp(void);
void Diagnose_TLV493D_Device(void);
/******************************************************************************
******************************************************************************/
#endif /* TLV493DMKM_H */

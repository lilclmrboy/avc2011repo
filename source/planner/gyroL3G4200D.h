/////////////////////////////////////////////////////////////////////////////
// File gyro.h
// Provides access and interface to the (I2C) gyro L3G4200D
// 

#ifndef _gyroL3G4200D_H_
#define _gyroL3G4200D_H_
#include "avc.h"

/////////////////////////////////////////////////////////////////////////////

int gyroL3G4200Dinit(acpStem *pStem);

int gyroL3G4200DgetX(acpStem *pStem, int *dddx);
int gyroL3G4200DgetY(acpStem *pStem, int *dddy);
int gyroL3G4200DgetZ(acpStem *pStem, int *dddz);


// Register map for L3G4200D
/*
 Register				RW	regAddr
 WHO_AM_I				r		0F
 CTRL_REG1			rw	20
 CTRL_REG3			rw	22
 CTRL_REG5			rw	24
 OUT_TEMP				r		26
 OUT_X_L				r		28
 OUT_Y_L				r		2A
 OUT_Z_L				r		2C
 FIFO_CTRL_REG	rw	2E
 INT1_CFG				rw	30
 INT1_TSH_XH		rw	32
 INT1_TSH_YH		rw	34
 INT1_TSH_ZH		rw	36
 INT1_DURATION	rw	38
 */


#endif // _gyroL3G4200D_H_
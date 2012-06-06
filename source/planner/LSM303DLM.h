/////////////////////////////////////////////////////////////////////////////
// Provides access and interface to the (I2C) gyro L3G4200D
// 

#ifndef _LSM303DLM_H_
#define _LSM303DLM_H_
#include "avc.h"

/////////////////////////////////////////////////////////////////////////////

int compassLSM303DLMinit(acpStem *pStem);

int compassLSM303DLMgetX(acpStem *pStem, int *x);
int compassLSM303DLMgetY(acpStem *pStem, int *y);
int compassLSM303DLMgetZ(acpStem *pStem, int *z);
int compassLSM303DLMgetHeading(acpStem *pStem, int *heading);




#endif // _LSM303DLM_H_
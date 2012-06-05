/////////////////////////////////////////////////////////////////////////////
// File gyro.h
// Provides access and interface to the (I2C) gyro L3G4200D
// 

#ifndef _gyro_H_
#define _gyro_H_
#include "avc.h"
#include "logger.h"


class avcGyro{
public:
  avcGyro(acpStem *pStem, aSettingFileRef settings);
  ~avcGyro(void);
  
  int init();
  int getAngularRateData(int *dddx, int *dddy, int *dddz);
  
private:
  acpStem *m_pStem;
  logger* m_logger;
  
  enum gyroHwEnumType {
    kGyro_L3G4200D,
    kGyro_none, // keep last in last
  };
  gyroHwEnumType m_gyroHWtype;
  
};

#endif //_gyro_H_
/////////////////////////////////////////////////////////////////////////////
// File gyro.h
// Provides access and interface to the (I2C) gyro L3G4200D
// 

#ifndef _compass_H_
#define _compass_H_
#include "avc.h"
#include "logger.h"


class avcCompass{
public:
  avcCompass(acpStem *pStem, aSettingFileRef settings);
  ~avcCompass(void);
  
  int init();
  int getHeading(int *heading);
  int getMagnetometerReadings(int *x, int *y, int *z);
  
private:
  acpStem *m_pStem;
  logger* m_logger;
  
  enum compassHwEnumType {
    kCompass_LSM303DLM,
    kCompass_none, // keep last in last
  };
  compassHwEnumType m_compassHwType;
  
};

#endif //_gyro_H_
/////////////////////////////////////////////////////////////////////////////
// File gyro.h
// Provides access and interface to all gyros
// 

#ifndef _gyro_H_
#define _gyro_H_
#include "avc.h"
#include "logger.h"

enum gyroHwEnumType {
  kGyro_L3G4200D,
  kGyro_none, // keep last in last
};

class avcGyro{
public:
  avcGyro(acpStem *pStem, aSettingFileRef settings);
  virtual ~avcGyro(void);
  
  virtual int init();
  virtual int getAngularRateData(int *dddx, int *dddy, int *dddz);
  
protected:
  acpStem *m_pStem;
  logger* m_logger;
  
  gyroHwEnumType m_gyroHWtype;
  
};

class gyroL3G4200D: public avcGyro {
public:
  gyroL3G4200D(acpStem *pStem, aSettingFileRef settings);
  virtual ~gyroL3G4200D();
  
  virtual int init();
  virtual int getAngularRateData(int *dddx, int *dddy, int *dddz);
  
private:
	int gyroL3G4200DgetX(int *dddx);
  int gyroL3G4200DgetY(int *dddy);
  int gyroL3G4200DgetZ(int *dddz);
  int gyroL3G4200DreadTwoByteTwosCompliment(unsigned int firstReg, int *reading);
  
};


#endif //_gyro_H_

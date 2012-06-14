/////////////////////////////////////////////////////////////////////////////
// File compass.h
// Provides access and interface to the (I2C) compass
// 

#ifndef _compass_H_
#define _compass_H_
#include "avc.h"
#include "logger.h"


class avcCompass{
public:
  avcCompass(acpStem *pStem, aSettingFileRef settings);
  virtual ~avcCompass(void);
  
  virtual int init();
  virtual int getHeadingDeg(float *headingDeg);
  virtual int getHeadingRad(float *headingRad);
  virtual int getMagnetometerReadings(int *x, int *y, int *z);
  virtual int resetCalToFactory();
  
protected:
  acpStem *m_pStem;
  logger* m_logger;
  
  typedef struct vector3 {
    double x, y, z;
  } vector3;
  
  // stores the magnetometer output
  vector3 magReading;
  
  // stores calibration information for the magnetometer
  vector3 minMagReading;
  vector3 maxMagReading;
  
  // a list of the different compasses
  enum compassHwEnumType {
    kCompass_LSM303DLM,
    kCompass_CMPS10,
    kCompass_none, // keep last in last
  };
  compassHwEnumType m_compassHwType;
  
};



class compassCMPS10 : public avcCompass {
public:
  compassCMPS10(acpStem *pStem, aSettingFileRef settings);
  virtual ~compassCMPS10(void);
  int init();
  int getHeadingDeg(float *heading);
  int getMagnetometerReadings(int *x, int *y, int *z);
  int resetCalToFactory();
private:
  
  
  int readTwoByteTwosCompliment(unsigned int firstReg, int *reading);
  int getMagneticX(int *x);
  int getMagneticY(int *y);
  int getMagneticZ(int *z);
};

#endif //_compas_H_

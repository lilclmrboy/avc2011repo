/////////////////////////////////////////////////////////////////////////////
// File accelerometer.h
// Provides access and interface to the accelerometer
// 

#ifndef _accelerometer_H_
#define _accelerometer_H_
#include "avc.h"
#include "logger.h"
#include "compass.h"


class avcAccelerometer{
public:
  avcAccelerometer(acpStem *pStem, aSettingFileRef settings);
  virtual ~avcAccelerometer(void);
  
  virtual int init();
  virtual int getAccelerometerReadings(float *accX, float *accY, float *accZ);
  
protected:
  acpStem *m_pStem;
  logger* m_logger;
    
  // a list of the different compasses
  enum accelerometerHwEnumType {
    kAccelerometer_LSM303DLM,
    kAccelerometer_none, // keep last in last
  };
  accelerometerHwEnumType m_accelerometerHwType;
  
};


class accelerometerLSM303DLM : public avcAccelerometer {
public:
  accelerometerLSM303DLM(acpStem *pStem, aSettingFileRef settings);
  virtual ~accelerometerLSM303DLM(void);
  friend int compassLSM303DLM::init();
  friend int compassLSM303DLM::getAccelerometerReadings(float *accX, float *accY, float *accZ); 
  
private:
  friend int compassLSM303DLM::readTwoByteTwosComplimentLittleEndian(unsigned int firstReg, int *reading);

};


class accelerometerADXL335 : public avcAccelerometer {
public:
  accelerometerADXL335(acpStem *pStem, aSettingFileRef settings);
  virtual ~accelerometerADXL335(void);
  int init();
  int getAccelerometerReadings(float *accX, float *accY, float *accZ);
  
private:
  
};

#endif //_accelerometer_H_

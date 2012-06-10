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
  ~avcCompass(void);
  
  virtual int init();
  virtual int getHeadingDeg(float *headingDeg);
  virtual int getHeadingRad(float *headingRad);
  virtual int getMagnetometerReadings(int *x, int *y, int *z);
  virtual int resetCalToFactory();
  virtual int getAccelerometerReadings(int *accX, int *accY, int *accZ);
  
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


class compassLSM303DLM : public avcCompass {
public:
  compassLSM303DLM(acpStem *pStem, aSettingFileRef settings);
  ~compassLSM303DLM(void);
  int init();
  int getHeadingDeg(float *heading);
  int getMagnetometerReadings(int *x, int *y, int *z);
  
  friend class accelerometerLSM303DLM;
  
private:
  typedef struct vector3D{
    double x, y, z;
    
    vector3D (double newx, double newy, double newz){
      x = newx;
      y = newy;
      z = newz;
    }
    
    vector3D (void){x=0; y=0; z=0;}
    
  } vector3D;
  
  int readTwoByteTwosComplimentLittleEndian(unsigned int firstReg, int *reading);
  int readTwoByteTwosComplimentBigEndian(unsigned int firstReg, int *reading);
  int getMagneticX(int *x);
  int getMagneticY(int *y);
  int getMagneticZ(int *z);
  int getAccelerometerReadings(int *accX, int *accY, int *accZ); // this should be moved to the accelerometer class
  
  int calculateHeadingDeg(vector3D magV, float *headingDeg);
    
  // store the initial accelerometer readings so they can be used in
  // heading calculation
  vector3D m_initalAccelerometerReadings;
  vector3D m_compassCalMin;
  vector3D m_compassCalMax;
  
  // vector helper functions
  void normalizeVector3D(vector3D *vec);
  void crossProductVector3D(const vector3D *a, const vector3D *b, vector3D *result);
  double dotProductVector3D(const vector3D *a, const vector3D *b);
  
};


class compassCMPS10 : public avcCompass {
public:
  compassCMPS10(acpStem *pStem, aSettingFileRef settings);
  ~compassCMPS10(void);
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
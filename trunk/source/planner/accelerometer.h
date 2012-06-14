/////////////////////////////////////////////////////////////////////////////
// File accelerometer.h
// Provides access and interface to the accelerometer
// 

#ifndef _accelerometer_H_
#define _accelerometer_H_
#include "avc.h"
#include "logger.h"
#include "compass.h"


/////////////////////////////////////////////////////////////////////////////
// Main accelerometer class
// use this class for access to accelerometers
// inherit from this class to implement accelerometer HAL classes
/////////////////////////////////////////////////////////////////////////////
class avcAccelerometer{
public:
  avcAccelerometer(acpStem *pStem, aSettingFileRef settings);
  virtual ~avcAccelerometer(void);
  
  virtual int init();
  virtual int getAccelerometerReadings(float *accX, float *accY, float *accZ) = 0;
  
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


/////////////////////////////////////////////////////////////////////////////
// Threaded accelerometer class
// use this class to continuously poll accelerometers
// a running average of the data read from the accelerometers can be read
// from this class. The running average is then reset
/////////////////////////////////////////////////////////////////////////////
class avcAccelerometerThread : public acpRunable {
  
public:
  avcAccelerometerThread(avcAccelerometer *avcAccelReference);
  ~avcAccelerometerThread(void);
  
  // Returns the resultant force vector
  aErr getAverageAcceleration(double *x, double *y, double *z);
  
  // acpRunable virtual
  //void step (const double time);
  
  // return the current cumulative average measurements
  int getAverageAccerlerometerMeasurements(double *x, double *y, double *z);
  
  // acpThread run handle
  int run(void);
  
private:
  // thread parts
  acpMutex* m_accelReadingLock;
  acpThread* m_pThread;
  
  logger *m_logger;
  
  unsigned long m_threadDelay;
  unsigned long m_readingCounter;
  
  // make a vector type for storing the data
  typedef struct vector3D{
    double x, y, z;
    
    vector3D (double newx, double newy, double newz){
      x = newx;
      y = newy;
      z = newz;
    }
    
    vector3D (void){x=0; y=0; z=0;}
    
  } vector3D;
	
  vector3D m_currentCumulativeAverage;
  
  // pointer to an accelerometer class
  avcAccelerometer *m_accerlerometer;
  
  // make a new measurement from the accelerometer
  int makeNewMeasurement();

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

/////////////////////////////////////////////////////////////////////////////
// File gyro.h
// Provides access and interface to all gyros
// 

#ifndef _gyro_H_
#define _gyro_H_
#include "avc.h"
#include "logger.h"

/////////////////////////////////////////////////////////////////////////////
// Main gyro class
// use this class for access to gyro readings
/////////////////////////////////////////////////////////////////////////////
class avcGyro{
public:
  avcGyro(acpStem *pStem, aSettingFileRef settings);
  virtual ~avcGyro(void);
  
  virtual int init();
  virtual int getAngularRateData(int *dddx, int *dddy, int *dddz) = 0;
  
protected:
  acpStem *m_pStem;
  logger* m_logger;
	
	enum gyroHwEnumType {
		kGyro_L3G4200D,
		kGyro_none, // keep last in last
	};
  
  gyroHwEnumType m_gyroHWtype;
  
};

/////////////////////////////////////////////////////////////////////////////
// Threaded gyro class
// use this class to continuously poll gyro readings
// a cumulative running average of the data read from the gyros can be read
// from this class. The running average is then reset
/////////////////////////////////////////////////////////////////////////////
class avcGyroThread : public acpRunable {
  
public:
  avcGyroThread(avcGyro *avcGyroReference);
  ~avcGyroThread(void);
  
  // Returns the resultant force vector
  aErr getAverageAngularRate(double *x, double *y, double *z);
  
  // acpRunable virtual
  //void step (const double time);
  
  // return the current cumulative average measurements
  int getAverageAngularRateMeasurements(double *x, double *y, double *z);
  
  // acpThread run handle
  int run(void);
  
private:
  // thread parts
  acpMutex* m_gyroReadingLock;
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
  avcGyro *m_gyro;
  
  // make a new measurement from the accelerometer
  int makeNewMeasurement();
	
};

/////////////////////////////////////////////////////////////////////////////

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
  int gyroL3G4200DreadTwoByteTwosCompliment(unsigned char firstReg, int *reading);
  
};


#endif //_gyro_H_

#ifndef LSM303DLM_h
#define LSM303DLM_h

#include "avc.h"
#include "logger.h"
#include "compass.h"
#include "accelerometer.h"
#include <math.h>

class LSM303DLM: public avcCompass, public avcAccelerometer {
public:
  LSM303DLM(acpStem *pStem, aSettingFileRef settings);
  ~LSM303DLM(void);
  int init();
  
  ////////////////////
  // Compass stuff
  ////////////////////
  int getHeadingDeg(float *heading);
  int getMagnetometerReadings(int *x, int *y, int *z);
  
  ////////////////////
  // Accel stuff
  ////////////////////
  int getAccelerometerReadings(float *accX, float *accY, float *accZ); 
  
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
  
  logger *m_logger;
  acpStem *m_pStem;
  
  int readTwoByteTwosComplimentLittleEndian(unsigned int firstReg, int *reading);
  int readTwoByteTwosComplimentBigEndian(unsigned int firstReg, int *reading);
  int getMagneticX(int *x);
  int getMagneticY(int *y);
  int getMagneticZ(int *z);
  
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


#endif //LSM303DLM_h

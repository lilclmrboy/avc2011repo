/////////////////////////////////////////////////////////////////////////////
// File: repulsiveForces.h
// This class allows us to collect all the sensor data into a total 
// repulsive force that can get added to the system controller.

#ifndef _repulsiveForces_H_
#define _repulsiveForces_H_

#include "avc.h"

// Sensor types that we have been using
//typedef enum SensorType 
//{
//  gp2d12 = 0, // 0
//  gp2y0a21ky, // 1
//  gp2y0a710k  // 2
//};

#define aREPULSIVE_THETA_KEY            "theta"
#define aREPULSIVE_THETA_DEFAULT        0.0f

#define aREPULSIVE_RADIUS_KEY           "repulsive_radius"
#define aREPULSIVE_RADIUS_DEFAULT       1.0f

#define aREPULSIVE_DESCRIPTION_KEY      "description"
#define aREPULSIVE_DESCRIPTION_DEFAULT  "Sensor"

#define aREPULSIVE_SENSORTYPE_KEY       "type"
#define aREPULSIVE_SENSORTYPE_DEFAULT   "gp2d12"

#define aREPULSIVE_GP2D12_PORT_KEY      "port"
#define aREPULSIVE_GP2D12_PORT_DEFAULT  0

#define aREPULSIVE_SENSOR_CONFIG_PREFIX "sensor_"

#define aREPULSIVE_DELAY_KEY            "repulsive_delay"
#define aREPULSIVE_DELAY_DEFAULT        400

#define aREPULSIVE_MAX_SENSORS          10

#define aREPULSIVE_SENSOR_NUM           2

///////////////////////////////////////////////////////////////
// Class definition for a singular repulsive force
// Generalized so that we can have different classes for 
// specific sensor types so they all play well together. 
class avcRepulsiveForce {
  
public:
  avcRepulsiveForce(void) : 
    m_ioRef(NULL),
    m_settings(NULL),
    m_theta(aREPULSIVE_THETA_DEFAULT)
  {};
  avcRepulsiveForce(acpStem *pStem, const char * settingFileName);
  virtual ~avcRepulsiveForce(void);
  
  // Update the repulsive force
  // Virtual function to map and scale the raw sensor data reading
  // into a useful value for our system.
  // How we get this reading from the Stem (or others)
  // may be different for sensors
  virtual aErr update(void) { return aErrNone; }
  
  // Operator function to add repulsive forces
  const avcRepulsiveForce& operator+=(const avcRepulsiveForce& rhs);
  
  inline double getUx(void) { return m_force.x; };
  inline double getUy(void) { return m_force.y; };
  
protected:
  avcForceVector m_force;
  
  // Singleton access to logger class
  logger *m_log;
  
  aIOLib m_ioRef;
  
  // Our controller owns this too. We'll let them delete this.
  acpStem *m_pStem;
  
  // Local setting file reference
  aSettingFileRef m_settings;
  
  // Theta position of sensor
  float m_theta;
  
  // Max force radius that we care about
  float m_radiusMax;
  
  // Human readable description of sensor
  acpString m_description;
  
  // Human readable actual name of sensor
  acpString m_typeName;
  
  // Sensor a2d port that it may be attached to
  unsigned char m_a2dport;

  
}; // end class avcRepulsiveForce

//////////////////////////////////////////////////////////////
// Model specific sensor class for a Sharp GP2D12
// as an example
class avcGP2D12 : public avcRepulsiveForce {
  
public:
  avcGP2D12(acpStem *pStem, const char * settingFileName)
  : avcRepulsiveForce(pStem, settingFileName) {};
  ~avcGP2D12(void){};
  
  aErr update(void);
  
}; // end class avcGP2D12

//////////////////////////////////////////////////////////////
// Model specific sensor class for a Sharp GP2Y0A710K
// as an example
class avcGP2Y0A710K : public avcRepulsiveForce {
  
public:
  avcGP2Y0A710K(acpStem *pStem, const char * settingFileName)
  : avcRepulsiveForce(pStem, settingFileName) {};
  ~avcGP2Y0A710K(void){};
  
  aErr update(void);
  
}; // end class avcGP2Y0A710K


/////////////////////////////////////////////////////////////////////////
// This is the base class that handles all the repulsive force
// management on the robot system.
class avcRepulsiveForces : public acpRunable {
  
public:
  avcRepulsiveForces(void);
  ~avcRepulsiveForces(void);
  
  // Must call this first before doing any updates. Failure to do so will
  // cause all calls on updateControl to return an error.
  aErr init(acpStem *pStem, aSettingFileRef settings);
  
  // Returns the resultant force vector
  aErr getForceResultant(avcForceVector *pForceVector);
  
  // acpRunable virtual
  void step (const double time);
  
  // acpThread run handle
  int run(void);
  int update(void);
  
private: 
  
  aIOLib m_ioRef;
  
  // Our controller owns this too. We'll let them delete this.
  acpStem *m_pStem;
  
  //Our controller owns this we'll let them delete.	
  aSettingFileRef m_settings;
  logger *m_log;
  
  bool m_bInit;
  int m_nForces;
  unsigned long m_threadDelay;
  
  avcForceVector m_RepulsiveResult;
  
  avcRepulsiveForce *m_pForces[aREPULSIVE_MAX_SENSORS];
  
  // Handle on thread object
  acpThread* m_pThread;
  
  acpMutex* m_repulseReadingLock;
  
};

#endif //_repulsiveForces_H_

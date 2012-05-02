/////////////////////////////////////////////////////////////////////////////
// File: repulsiveForces.h
// This class allows us to collect all the sensor data into a total 
// repulsive force that can get added to the system controller.

#ifndef _repulsiveForces_H_
#define _repulsiveForces_H_

#include "avc2011.h"

///////////////////////////////////////////////////////////////
// Class definition for a singular repulsive force
// Generalized so that we can have different classes for 
// specific sensor types so they all play well together. 
class avcRepulsiveForce {
  
public:
  avcRepulsiveForce(void){};
  avcRepulsiveForce(acpStem *pStem, const char * settingFileName);
  ~avcRepulsiveForce(void);
  
  // Update the repulsive force
  // Virtual function to map and scale the raw sensor data reading
  // into a useful value for our system.
  // How we get this reading from the Stem (or others)
  // may be different for sensors
  virtual aErr update(void) { return aErrNone; }
  
  // Operator function to add repulsive forces
  const avcRepulsiveForce& operator+=(const avcRepulsiveForce& rhs);
  
protected:
  avcForceVector m_force;
  
  // Singleton access to logger class
  logger *m_log;
  
  aIOLib m_ioRef;
  
  // Our controller owns this too. We'll let them delete this.
  acpStem *m_pStem;
  
  // Local setting file reference
  aSettingFileRef m_settings;
  
}; // end class avcRepulsiveForce

//////////////////////////////////////////////////////////////
// Model specific sensor class for a Sharp GP2D12
// as an example
class avcGP2D12 : public avcRepulsiveForce {
  
public:
  avcGP2D12(acpStem *pStem, const char * settingFileName);
  
  ~avcGP2D12(void){};
  
  aErr update(void);
  
}; // end class avcGP2D12


/////////////////////////////////////////////////////////////////////////
// This is the base class that handles all the repulsive force
// management on the robot system.
class avcRepulsiveForces {
  
public:
  avcRepulsiveForces(void);
  ~avcRepulsiveForces(void);
  
  // Must call this first before doing any updates. Failure to do so will
  // cause all calls on updateControl to return an error.
  aErr init(acpStem *pStem, aSettingFileRef settings);
  
  // Returns the resultant force vector
  aErr getForceResultant(avcForceVector *pForceVector);
  
private: 
  
  aIOLib m_ioRef;
  
  // Our controller owns this too. We'll let them delete this.
  acpStem *m_pStem;
  
  //Our controller owns this we'll let them delete.	
  aSettingFileRef m_settings;
  logger *m_log;
  
  bool m_bInit;
  int m_nForces;
  
  avcRepulsiveForce *m_pForces[10];
  
};

#endif //_repulsiveForces_H_

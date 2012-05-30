/////////////////////////////////////////////////////////////////////////////
// File: motModule.h
// The motion module takes a number of unit vectors, and outputs a
// vehicle velocity in terms of the desired velocities of the two 
// drive units.

#ifndef _motModule_H_
#define _motModule_H_

#include "avc.h"
#include "avcDefs.tea"

#define aMOTOR_NUM                          2
#define aMOTMODULE_THROTTLE_WINDOW_KEY      "speedscale"
#define aMOTMODULE_THROTTLE_WINDOW_DEFAULT  0.05f

class avcMotion {
  
public:
  avcMotion(void);
  ~avcMotion(void);
  
  // Must call this first before doing any updates. Failure to do so will
  // cause all calls on updateControl to return an error.
  aErr init(acpStem *pStem, aSettingFileRef settings);
  
  // Updates the wheel velocity setpoints with a desired
  // force vector. 
  aErr updateControl(const avcForceVector& potential);
  
private: 
  
  aIOLib m_ioRef;
  
  // Our controller owns this too. We'll let them delete this.
  acpStem *m_pStem;
  
  //Our controller owns this we'll let them delete.	
  aSettingFileRef m_settings;
  logger *m_log;
  short m_setpointMax;
  short m_setpoint[aMOTOR_NUM];
  short m_setpointLast[aMOTOR_NUM];
  
  bool m_bInit;
  float m_fThrottleWindow;
  
  
};

#endif //_motModule_H_

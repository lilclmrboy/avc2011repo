/////////////////////////////////////////////////////////////////////////////
// File: motModule.h
// The motion module takes a number of unit vectors, and outputs a
// vehicle velocity in terms of the desired velocities of the two 
// drive units.

#ifndef _motModule_H_
#define _motModule_H_

#include "avc2011.h"

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
	
	// Gives other folks the last setpoint value we calculated for the motor
	// motor should be: aMOTOR_LEFT or aMOTOR_RIGHT
	short inline getSetpointMotor(const unsigned char motor) 
		{ return m_setpoint[motor]; };
	
	
private: 
	
	aIOLib m_ioRef;
	
	// Our controller owns this too. We'll let them delete this.
	acpStem *m_pStem;
	
	//Our controller owns this we'll let them delete.	
	aSettingFileRef m_settings;
	
	short m_setpointMax;
	short m_setpoint[aMOTOR_NUM];
	short m_setpointLast[aMOTOR_NUM];

};

#endif //_motModule_H_

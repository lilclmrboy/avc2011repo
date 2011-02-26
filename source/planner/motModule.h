/////////////////////////////////////////////////////////////////////////////
// File: motModule.h
// The motion module takes a number of unit vectors, and outputs a
// vehicle velocity in terms of the desired velocities of the two 
// drive units.

#ifndef _motModule_H_
#define _motModule_H_

#include <stdio.h>
#include "avc2011Structs.h"
#include "avc2011Defs.tea"

class avcMotion {

public:
	avcMotion(void);
	~avcMotion(void){}
	
	// Must call this first before doing any updates. Failure to do so will
	// cause all calls on updateControl to return an error.
	aErr init(acpStem *pStem);
	
	// Updates the wheel velocity setpoints with a desired
	// force vector. 
	aErr updateControl(const avcForceVector& potential);
	
	// Gives other folks the last setpoint value we calculated for the motor
	// motor should be: aMOTOR_LEFT or aMOTOR_RIGHT
	short inline getSetpointMotor(const unsigned char motor) 
		{ return m_setpoint[motor]; };
	
	
private: 
	acpStem *m_pStem;
	
	short m_setpointMax;
	short m_setpoint[aMOTOR_NUM];
	short m_setpointLast[aMOTOR_NUM];

};

#endif //_motModule_H_

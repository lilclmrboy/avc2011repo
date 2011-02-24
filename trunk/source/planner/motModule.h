/////////////////////////////////////////////////////////////////////////////
// File: motModule.h
// The motion module takes a number of unit vectors, and outputs a
// vehicle velocity in terms of the desired velocities of the two 
// drive units.
#ifndef _motModule_H_
#define _motModule_H_

#include <stdio.h>
#include "avc2011Structs.h"

class avcMotion {

public:
	avcMotion(void) {}
	~avcMotion(void){}
	
	avcControlUpdate updateControl(const avcForceVector& potential);

};

#endif //_motModule_H_

/////////////////////////////////////////////////////////////////////////////
// File: motModule.h
// The motion module takes a number of unit vectors, and outputs a
// vehicle velocity in terms of the desired velocities of the two 
// drive units.
#include <stdarg.h> 
#include <stdio.h>
class avcMotion {

public:
	avcMotion(void) {}
	~avcMotion(void){}
	
	// The primary. This will take a variable number of input vectores,
	// sum them, and output setpoints for the vehicle's drive untis.
	// IMPORTANT: The terminal variable of this function must be a NULL.
	void getVelocity(int& lV, int& rV, ... );

};


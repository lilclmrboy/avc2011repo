/////////////////////////////////////////////////////////////////////////////
// File locModule.h
// The localization module is responsible for determining the vehicle's 
// current location given the previous control input and position, and the
// available sensor data such as encoder readings, GPS, and compass heading.

#ifdef _avcPosition_H_
#include "avc2011Defs.h"

class avcPosition 
{
public: 
	avcPosition(void);
	~avcPosition(void);

	avcStateVector getPosition(const avcStateVector& prevPos,
				   const avcForceVector& prevCont,
				   acpStem& stem);
	
};

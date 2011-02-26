/////////////////////////////////////////////////////////////////////////////
// File locModule.h
// The localization module is responsible for determining the vehicle's 
// current location given the previous control input and position, and the
// available sensor data such as encoder readings, GPS, and compass heading.

#ifndef _locModule_H_
#define _locModule_H_

#include "avc2011Structs.h"
#include "avc2011Defs.tea"

class avcPosition 
{
public: 
	avcPosition(void) : 
	  m_pStem(NULL),
	  m_curGPSTimeSec(0) {};
	~avcPosition(void) {};

	aErr init(acpStem* Stem);

	avcStateVector getPosition(const avcForceVector& Control);

private:
	acpStem* m_pStem;
	int m_curGPSTimeSec;
	avcStateVector m_curPos;	
	int getGPSTimeSec(void);
	
};

#endif //_locModule_H_

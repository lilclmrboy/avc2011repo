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
	
	//We need a valid link to the stem network here. 
	//We'll ask for current GPS coordinates, and a timestamp.
	aErr init(acpStem* Stem);
	
	//does an EKF state update.
	void updateState(const avcControlVector& control);
	
	//return the current robot position.
	avcStateVector getPosition(void) { return m_curPos; }

private:
	acpStem* m_pStem;
	int m_curGPSTimeSec;
	avcStateVector m_curPos;	
	int getGPSTimeSec(void);
	bool getGPSQuality(void);
	//longitude and latitude are in degrees...
	//Boulder is pretty close to -105 degrees longitude, and 40 degrees latitude.
	//We're using double precision number here so we'll compact minutes into 
	//fractional degrees.
	double getGPSLongitude(void);
	double getGPSLatitude(void);
	
};

#endif //_locModule_H_

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
		m_ioRef(NULL),
		m_curGPSTimeSec(0), 
		m_settings(NULL)
	{	
		aErr e;

		if(aIO_GetLibRef(&m_ioRef, &e)) 
      		throw acpException(e, "Getting aIOLib reference");						
	};

	~avcPosition(void) 
	{
		aErr e;
		if (aIO_ReleaseLibRef(m_ioRef, &e))
 	     		throw acpException(e, "unable to destroy IO lib");	
	};
	
	//We need a valid link to the stem network here. 
	//We'll ask for current GPS coordinates, and a timestamp.
	aErr init(acpStem* Stem, aSettingFileRef setting);
	
	//does an EKF state update.
	void updateState(const avcControlVector& control);
	
	//return the current robot position.
	avcStateVector getPosition(void) { return m_curPos; }

private:
	acpStem* m_pStem;
	aIOLib m_ioRef;
	int m_curGPSTimeSec;
	avcStateVector m_curPos;
	
	//Our controller owns this we'll let them delete.	
	aSettingFileRef m_settings;
	
	int getGPSTimeSec(void);
	bool getGPSQuality(void);
	//longitude and latitude are in degrees...
	//Boulder is pretty close to -105 degrees longitude, and 40 degrees latitude.
	//We're using double precision number here so we'll compact minutes into 
	//fractional degrees.
	double getGPSLongitude(void);
	double getGPSLatitude(void);
	double getCMPSHeading(void);
	
};

#endif //_locModule_H_

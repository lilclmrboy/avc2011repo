/////////////////////////////////////////////////////////////////////////////
// File locModule.h
// The localization module is responsible for determining the vehicle's 
// current location given the previous control input and position, and the
// available sensor data such as encoder readings, GPS, and compass heading.

#ifndef _locModule_H_
#define _locModule_H_

#include "avc.h"
#include "mat_math.h"
#include "compass.h"
#include "motModule.h"
#include "gps.h"
#include "accelerometer.h"

#define aUSE_GPS

class avcPosition 
{
public: 
	avcPosition(void) : 
	  m_pStem(NULL),
		m_ioRef(NULL),
		m_curGPSTimeSec(0),
		m_curClock(0),		
		m_P(3,3),
		m_Q(3,3),
		m_W(3,3),
		m_Encoder(0),
		m_metersPerTick(METER_PER_TICK), 
		m_settings(NULL)
	{	
		aErr e;

		if(aIO_GetLibRef(&m_ioRef, &e)) 
      		throw acpException(e, "Getting aIOLib reference");						
	};

	avcPosition(avcStateVector vec) :
		m_pStem(NULL),
		m_ioRef(NULL),
		m_curGPSTimeSec(0),
		m_curClock(0),
		m_gpsClock(0),
		m_curPos(vec),
		m_P(3,3),
		m_Q(3,3),
		m_W(3,3),
		m_Encoder(0),
		m_metersPerTick(METER_PER_TICK),
		m_settings(NULL)
	{
		aErr e;

		if(aIO_GetLibRef(&m_ioRef, &e)) 
      		throw acpException(e, "Getting aIOLib reference");	
	}
		
	~avcPosition(void) 
	{
		fputs ("</trkseg></trk></gpx>", gps_track);
		fclose(gps_track);
		
		aErr e;
		if (aIO_ReleaseLibRef(m_ioRef, &e))
 	     		throw acpException(e, "unable to destroy IO lib");	
    
    free(m_compass);
	};
	
	//We need a valid link to the stem network here. 
	//We'll ask for current GPS coordinates, and a timestamp.
	aErr init(acpStem* Stem, aSettingFileRef setting, avcWaypointVector firstMapPoint);
	
	//does an EKF state update.
	void updateState(short lastThrottleSetPoint);
	
	bool getGPSQuality(void);
	
	void recordGPSPoint(void);
	
	//return the current robot position.
	avcStateVector getPosition(void) { return m_curPos; }
	
	//Set the current robot position.
  void setPosition(avcWaypointVector newPos) { 
    m_curPos.x = newPos.state.x;
    m_curPos.y = newPos.state.y;
    m_curPos.h = newPos.state.h; 
  }
  
  // gps member class
  gps* m_gps;
	

#ifdef aDEBUG_FLOC
	friend int main(int, const char**);
#endif

private:
	acpStem* m_pStem;
	aIOLib m_ioRef;
	logger* m_logger;
	FILE* gps_track;
	//We'll use this time reading to get GPS updates.	
	int m_curGPSTimeSec;
	//Millisecond timing for motion update.
	long int m_curClock;
	long int m_gpsClock;	
	avcStateVector m_curPos;
	// probability matrix for state vector.	
	Matrix m_P;
	Matrix m_Q;
	Matrix m_W;	
	int m_Encoder;
	double m_metersPerTick; //encoder ticks per revolution.
  double m_wheelBase;
  
  avcCompass *m_compass;
  avcAccelerometer *m_accel;
  avcAccelerometerThread *m_accelThread;

	//Our controller owns this we'll let them delete.	
	aSettingFileRef m_settings;
	
	int getGPSTimeSec(void);
	
	//longitude and latitude are in degrees...
	//Boulder is pretty close to -105 degrees longitude, and 40 degrees latitude.
	//We're using double precision number here so we'll compact minutes into 
	//fractional degrees.
	double getGPSLongitude(void);
	double getGPSLatitude(void);
	double getHeading(void);
	int getEncoderValue(void);
	double getSteeringAngleRad(void);
  int getMotorSetPoint(void);
  int getAccelerometerReadings (float *ddx, float *ddy, float *ddz);
	
};

#endif //_locModule_H_

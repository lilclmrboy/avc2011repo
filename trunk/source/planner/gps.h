/////////////////////////////////////////////////////////////////////////////
// File gps.h
// Provides access and decoding of serial (I2C) GPS packets
// Blatantly copied from Jeremiah's tea code
// 

#ifndef _gps_H_
#define _gps_H_
#include "avc.h"
#include "TinyGPS.h"
#include "aIO.h"

/////////////////////////////////////////////////////////////////////////
// This is the base class that handles all the repulsive force
// management on the robot system.
class gps : public acpRunable {
  
public:
  
	// General destructor.
  ~gps(void) {
        aIO_ReleaseLibRef(m_ioRef, NULL);
    };
  
	static gps* getInstance();
	
  // Must call this first before doing any updates. Failure to do so will
  // cause all calls on updateControl to return an error.
  aErr init(const acpString& port, const int baud);
  
  // Returns the resultant force vector
  aErr getPosition(float *lon, float *lat, float* heading);
  
  // acpThread run handle
  int run(void);
  
private: 
	
	static gps* m_pInstance;
	
    gps(void) :
        m_bInit(false),
        m_bRunning(true),
        m_threadDelay(200)
    {
        aIO_GetLibRef(&m_ioRef, NULL);
    };
	gps(gps const&){};
	gps& operator=(gps const&);
	
    TinyGPS m_gps;
	aIOLib m_ioRef;
    aStreamRef m_serialStream;
  
  //Our controller owns this we'll let them delete.	
  logger *m_log;

  
  bool m_bInit;
  bool m_bRunning;
  unsigned long m_threadDelay;
  
  // Handle on thread object
  acpThread* m_pThread;
  
};

#define GPS_IIC_ADDR 0xD0

int aGPM_Get2DigitInt(acpStem* pStem, char addr, char reg);
int aGPM_Get3DigitInt(acpStem* pStem, char addr, char reg);
int aGPM_Get4DigitInt(acpStem* pStem, char addr, char reg);
int aGPM_GetAltitude(acpStem* pStem, char addr);
int aGPM_GetAnalogInput(acpStem* pStem, char addr, int n);

int aGPM_GetHours(acpStem* pStem);
int aGPM_GetMinutes(acpStem* pStem);
int aGPM_GetSeconds(acpStem* pStem);
int aGPM_GetDay(acpStem* pStem);
int aGPM_GetMonth(acpStem* pStem);
int aGPM_GetYear(acpStem* pStem);

int aGPM_GetLatitudeDegrees(acpStem* pStem);
int aGPM_GetLatitudeMinutes(acpStem* pStem);
int aGPM_GetLatitudeFrac(acpStem* pStem);
int aGPM_GetLatitudeDirChar(acpStem* pStem); 

int aGPM_GetLongitudeDegrees(acpStem* pStem);
int aGPM_GetLongitudeMinutes(acpStem* pStem);
int aGPM_GetLongitudeFrac(acpStem* pStem);
int aGPM_GetLongitudeDirChar(acpStem* pStem);

int aGPM_GetGPSQuality(acpStem* pStem);
int aGPM_GetSatellitesInUse(acpStem* pStem);
int aGPM_GetHDOP(acpStem* pStem);

int aGPM_GetTrueHeading(acpStem* pStem);
int aGPM_GetTrueHeadingFrac(acpStem* pStem);
int aGPM_GetMagHeading(acpStem* pStem);
int aGPM_GetMagHeadingFrac(acpStem* pStem);

int aGPM_GetSpeed(acpStem* pStem);
int aGPM_GetSpeedFrac(acpStem* pStem);

int aGPM_GetGPSModeChar(acpStem* pStem);

int aGPM_GetSatelliteID(acpStem* pStem, int n);
int aGPM_GetSatelliteSignalLevel(acpStem* pStem, int n);

int aGPM_GetDigitalIO(acpStem* pStem);

int aGPM_GetPosFoundFlag(acpStem* pStem);
int aGPM_GetMovingFlag(acpStem* pStem);
int aGPM_GetStatus(acpStem* pStem);

int aGPM_SetPortDir(acpStem* pStem, char dir);
int aGPM_SetPortBits(acpStem* pStem, char bits);

int aGPM_GetGPSQuality(acpStem* pStem);


#endif //_gps_H_

#include "locModule.h"

//We'll wait up to 30 seconds for a GPS lock on initialization.
#define aGPS_LOCK_STEPS 30
////////////////////////////////////////////////////////////////////////////
aErr
avcPosition::init(acpStem* pStem, aSettingFileRef settings) {

	m_pStem = pStem;
	m_settings = settings;

	//first we'll grab some settings from the settings file.
	

	if (m_pStem && m_pStem->isConnected()) {
		/*lets do some initialization. First we need to find out
		* whether we have a good GPS signal. If we do, we'll init
		* our starting position from the GPS position information
		* else we'll assume we're at the 0,0,0 position.	
	  */
		int timeout = 0;
		bool haveGPS = false;
		while (!(haveGPS = getGPSQuality()) && timeout < aGPS_LOCK_STEPS) {
			aIO_MSSleep(m_ioRef, 1000, NULL);
			++timeout;
		}

		if (haveGPS) {
			//Lets get a lat, lon, and heading... from compass. We shouldn't
			//be moving yet.
			
			m_curPos.x = getGPSLongitude();						
			m_curPos.y = getGPSLatitude();
			//We shouldn't have moved yet, so we get the heading from the 
			//compass instead of the GPS, we only want to get the compass
			//while outside to reduce the potential confound of a prevailing
			//non-earth magnetic field.
			m_curPos.h = getCMPSHeading();

			//We grabbed a potentially noisy set of sensor readings here,
			//so lets initialize our Probability matrices to the initial 
			//variance parameters for x, y and headings.
			
	
    } //else the default initilization of the state vector
			//and probability matrix is zero'd.

		//We should always be able to get a timestamp from the GPS unit.
		//even without the required number of satellites.		
		m_curGPSTimeSec = getGPSTimeSec();

		return aErrNone;
	} else {
		return aErrConnection;
	}
}

/////////////////////////////////////////////////////////////////////////////

void 
avcPosition::updateState(const avcControlVector& control) {
	
	//First we must do an estimation step, given the previous position
  //and the current control information.
	
	//We really only want to use GPS information if enough time has passed.
	int curSec = getGPSTimeSec();
	if (curSec != m_curGPSTimeSec) {
		//Grab current GPS and compass settings, otherwise we'll rely on 
		//a Kalman update with encoder values only.
	}

	printf("Current GPS seconds in the day: %d\n", curSec);

}

/////////////////////////////////////////////////////////////////////////////

int 
avcPosition::getGPSTimeSec(void) {
	int secs = 0;
	int tmp = 0;
	tmp = m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_MIN) << 8; 
  tmp |= m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_MIN+1);
	secs = tmp * 60;
	tmp = m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_SEC) << 8; 
  tmp |= m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_SEC+1);
	secs += tmp;
		
	return secs;	
}

/////////////////////////////////////////////////////////////////////////////

bool 
avcPosition::getGPSQuality(void) {
	bool qualityGood = false;
	if(m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_ISACTIVE))
		qualityGood = true;
  
	return qualityGood;
}

/////////////////////////////////////////////////////////////////////////////

double 
avcPosition::getGPSLongitude(void)
{
	double retVal = 0.0;
	short tmp = 0;
	tmp = m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_LON) << 8; 
  tmp |= m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_LON+1);
	retVal = (double) tmp;
	tmp = m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_LON+2) << 8; 
  tmp |= m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_LON+3);
	retVal += ((double) tmp)/60.0;
	tmp = m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_LON+4) << 8; 
  tmp |= m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_LON+5);
	retVal += ((double) tmp)/10000.0;

	return retVal;
}

/////////////////////////////////////////////////////////////////////////////

double 
avcPosition::getGPSLatitude(void)
{
	double retVal = 0.0;
	short tmp = 0;
	tmp = m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_LAT) << 8; 
  tmp |= m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_LAT+1);
	retVal = (double) tmp;
	tmp = m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_LAT+2) << 8; 
  tmp |= m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_LAT+3);
	retVal += ((double) tmp)/60.0;
	tmp = m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_LAT+4) << 8; 
  tmp |= m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_LAT+5);
	retVal += ((double) tmp)/10000.0;

	return retVal;
}

/////////////////////////////////////////////////////////////////////////////

double
avcPosition::getCMPSHeading(void) {
	double retVal = 0.0;
	short tmp = 0;
	tmp = m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_CMPS_HD) << 8; 
  tmp |= m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_CMPS_HD+1);
	retVal = ((double) tmp)/10.0;
	return retVal;
}



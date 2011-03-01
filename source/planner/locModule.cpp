#include "locModule.h"

////////////////////////////////////////////////////////////////////////////
aErr
avcPosition::init(acpStem* pStem) {

	m_pStem = pStem;

	if (m_pStem && m_pStem->isConnected()) {
		/*lets do some initialization. First we need to find out
		* whether we have a good GPS signal. If we do, we'll init
		* our starting position from the GPS position information
		* else we'll assume we're at the 0,0,0 position.	
	  */
		if (getGPSQuality()) {
			//Lets get a lat, lon, and heading... from compass. We shouldn't
			//be moving yet.
									



    } //else the default initilization of the state vector is zero'd.
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
avcPosition::getLongitude(void)
{
	double retVal = 0.0;
	int tmp = 0;
	tmp = m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_MIN) << 8; 
  tmp |= m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_MIN+1);

	return retVal;
}

/////////////////////////////////////////////////////////////////////////////

double 
avcPosition::getLatitude(void)
{
	double retVal = 0.0;

	tmp = m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_MIN) << 8; 
  tmp |= m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_MIN+1);

	return retVal;
}



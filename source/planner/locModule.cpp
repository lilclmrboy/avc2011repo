#include "locModule.h"

////////////////////////////////////////////////////////////////////////////
aErr
avcPosition::init(acpStem* pStem) {

	m_pStem = pStem;
	return aErrNone;
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
avcPosition::getGPSTimeSec() {
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






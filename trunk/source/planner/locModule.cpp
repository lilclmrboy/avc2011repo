#include "locModule.h"

////////////////////////////////////////////////////////////////////////////
aErr
avcPosition::init(acpStem* pStem) {

	m_pStem = pStem;
	return aErrNone;
}

/////////////////////////////////////////////////////////////////////////////

avcStateVector 
avcPosition::getPosition(const avcForceVector& Control) {
	
	int curSec = getGPSTimeSec();
	printf("Current GPS seconds in the day: %d\n", curSec);
	fflush(stdout);
	return m_curPos;
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






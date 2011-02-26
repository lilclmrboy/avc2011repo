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
	printf("Current GPS seconds in the day: %d", curSec);
	return m_curPos;
}

/////////////////////////////////////////////////////////////////////////////

int 
avcPosition::getGPSTimeSec() {
	int secs = 0;
	unsigned char pData[6] = {0};
	unsigned char index = 0x00;
	m_pStem->VAL_SET(aGP2_MODULE, IIC_SPD_VAL_INDEX, IIC_400Khz);
	m_pStem->IIC_XMIT(GPS_MODULE, &index, 1);
	m_pStem->IIC_RD( aGP2_MODULE, GPS_MODULE, 0x06, pData);
	secs = (pData[0] *10 + pData[1])* 3600 
	       + (pData[2] * 10 + pData[3]) * 60 + (pData[4] * 10 + pData[5]);

	return secs;	
}






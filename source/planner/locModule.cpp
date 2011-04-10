#include "locModule.h"
#include <sys/time.h>

//We'll wait up to 30 seconds for a GPS lock on initialization.
#define aGPS_LOCK_STEPS 30
#define DEG_TO_RAD (aPI/180)
#define RAD_TO_DEG (180/aPI)
////////////////////////////////////////////////////////////////////////////
aErr
avcPosition::init(acpStem* pStem, aSettingFileRef settings) {
	m_pStem = pStem;
	m_settings = settings;
	m_logger = logger::getInstance();	
	aErr e;

	char buffer[100];
	time_t rawtime;
	struct tm* timeinfo;
	time(&rawtime);
        timeinfo = localtime( &rawtime );
	strftime(buffer,100, "GPS_Track_%d_%m_%H_%M.gpx", timeinfo);
 
	gps_track = fopen(buffer, "w");
	if (!gps_track)
		return aErrIO;
	
	fputs("<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\" ?>\n",
				gps_track);
	fputs("<gpx xmlns=\"http://www.topografix.com/GPX/1/1\"" 
        " xmlns:gpxx=\"http://www.garmin.com/xmlschemas/GpxExtensions/v3\"" 
        " xmlns:gpxtpx=\"http://www.garmin.com/xmlschemas/"
        " TrackPointExtension/v1\" creator=\"Oregon 400t\" version=\"1.1\"" 
        " xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" "
        " xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 "
        " http://www.topografix.com/GPX/1/1/gpx.xsd "
        " http://www.garmin.com/xmlschemas/GpxExtensions/v3" 
        " http://www.garmin.com/xmlschemas/GpxExtensionsv3.xsd" 
        " http://www.garmin.com/xmlschemas/TrackPointExtension/v1" 
        " http://www.garmin.com/xmlschemas/TrackPointExtensionv1.xsd\">",
				gps_track);

	fputs("<trk><name>Chicken GPS track</name><trkseg>",gps_track);
	
	//first we'll grab some settings from the settings file.
	float fSetVar;
	if(aSettingFile_GetFloat(m_ioRef, m_settings, aKEY_WHEEL_RADIUS,  
			&fSetVar, aWHEEL_RADIUS, &e)) 
		throw acpException(e, "getting wheel radius from settings");
	m_wheelRd = fSetVar;

	if(aSettingFile_GetFloat(m_ioRef, m_settings, aKEY_WHEEL_TRACK,  
			&fSetVar, aWHEEL_TRACK, &e)) 
		throw acpException(e, "getting wheel track from settings");
	m_wheelTrk = fSetVar;

	if(aSettingFile_GetFloat(m_ioRef, m_settings, aKEY_ENCTICK_PER_REV,  
			&fSetVar, aENCTICKS, &e)) 
		throw acpException(e, "getting encoder ticks per rev from settings");	
	m_ticksPerRev = fSetVar;

	if (m_pStem && m_pStem->isConnected()) {
		/*lets do some initialization. First we need to find out
		* whether we have a good GPS signal. If we do, we'll init
		* our starting position from the GPS position information
		* else we'll assume we're at the 0,0,0 position.	
	  */
		//int timeout = 0;
		bool haveGPS = false;
		//while (!(haveGPS = getGPSQuality()) && timeout < aGPS_LOCK_STEPS) {
		//	aIO_MSSleep(m_ioRef, 1000, NULL);
		//	++timeout;
			
		//}

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
	
    } //else the default initilization of the state vector
			//and probability matrix is zero'd.

		//We should always be able to get a timestamp from the GPS unit.
		//even without the required number of satellites.		
		m_curGPSTimeSec = getGPSTimeSec();

    //Lets initialize the encoders to zero.
		m_pStem->MO_ENC32(aMOTO_MODULE, aMOTOR_RIGHT, 0);
		m_pStem->MO_ENC32(aMOTO_MODULE, aMOTOR_LEFT, 0);

		return aErrNone;
	} else {
		return aErrConnection;
	}
}

/////////////////////////////////////////////////////////////////////////////

void 
avcPosition::updateState() {
	
	//Grab the clock.
	long int curClock = clock();
	long tmElapsed = (curClock - m_curClock) * 1000 / CLOCKS_PER_SEC;

	//First we must do an estimation step, given the previous position
  //and the current control information. Lets do this in meters, and then
	//convert to lat, lon.
	// We should be doing this step quickly enough that we can treat the movement
	// as linear.
	/*
	* x(k+1) = { cos(theta)fVel + x(k) } + vx(k)
	*	y(k+1) = { sin(theta)fVel + y(k) } + vy(k)
	* theta(k+1) = { dTheta + theta } + vtheta(k)
	* vx(k+1) = { cos(theta)fdist/T } + vVx(k)
	* vy(k+1) = { sin(theta)fdist/T } + vVy(k)
	* vtheta(k+1) = { dTheta/T }
	*/
	
	//Get the new encoder readings.
	int curEncR = getEncoderValue(aMOTOR_RIGHT);
	int curEncL = getEncoderValue(aMOTOR_LEFT);
  
  	//convert those into distance traveled each wheel.
	double dWheelR = (curEncR - m_rEncoder)/ m_ticksPerRev;
	double dWheelL = (curEncL - m_rEncoder)/ m_ticksPerRev; 

	//The forward moving distance
	double fDist = dWheelR*m_wheelRd/2 + dWheelL*m_wheelRd/2; 	

	//The rotational difference
	double fRot = dWheelR*m_wheelRd/m_wheelTrk - dWheelL*m_wheelRd/m_wheelTrk;
	
	Vector state(6);
	state(1) = cos(m_curPos.h * DEG_TO_RAD)* fDist * aLON_PER_METER + m_curPos.x;
	state(2) = sin(m_curPos.h * DEG_TO_RAD)* fDist * aLAT_PER_METER + m_curPos.y;
	state(3) = m_curPos.h + (fRot * RAD_TO_DEG);
	state(4) = cos(m_curPos.h * DEG_TO_RAD)* fDist * aLON_PER_METER / tmElapsed;
	state(5) = sin(m_curPos.h * DEG_TO_RAD)* fDist * aLAT_PER_METER / tmElapsed;
	state(6) = fRot * RAD_TO_DEG;
  
	//EKF from here on out.
	//We really only want to use GPS information if enough time has passed.
	int curSec = getGPSTimeSec();
  
	double curLat = 0.0;
	double curLon = 0.0;
	if (curSec != m_curGPSTimeSec && getGPSQuality()) {
		//Grab current GPS and compass settings, otherwise we'll rely on 
		//a Kalman update with encoder values only.
		curLat = getGPSLatitude();
		curLon = getGPSLongitude();
	
		fprintf(gps_track, "<trkpt lat=\"%3.12f\" lon=\"%2.12f\">\n" 
									"\t<time>2011-3-17T%d:%d:%dZ</time>\n</trkpt>\n", 
		       curLat, curLon, curSec/3600, (curSec%3600)/60, curSec%60);
		m_curGPSTimeSec = curSec;
	}

	//Localization is done. Update the current robot state.
	m_curPos.x = state(1);
	m_curPos.y = state(2);
	m_curPos.h = state(3);
	m_curPos.vx = state(4);
	m_curPos.vy = state(5);
	m_curPos.vw = state(6);

	m_curClock = clock();
	
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

	//We're in the western hemisphere, so we'll have a negative longitude.
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
	retVal += ((double) tmp)/600000.0;
	
	return retVal * -1.0;
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
	retVal += ((double) tmp)/600000.0;

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

/////////////////////////////////////////////////////////////////////////////
int 
avcPosition::getEncoderValue(unsigned char motor) {
  //We could do more here to check for encoder wrap.
  return m_pStem->MO_ENC32(aMOTO_MODULE, motor);
}


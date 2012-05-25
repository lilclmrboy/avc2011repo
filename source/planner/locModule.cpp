#include "locModule.h"
#include <sys/time.h>

//We'll wait up to 30 seconds for a GPS lock on initialization.
#define aGPS_LOCK_STEPS 3
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

	fputs("Chicken GPS track",gps_track);
	
	//first we'll grab some settings from the settings file.
	float fSetVar;
	//if(aSettingFile_GetFloat(m_ioRef, m_settings, aKEY_WHEEL_RADIUS,  
	//		&fSetVar, aWHEEL_RADIUS, &e)) 
	//	throw acpException(e, "getting wheel radius from settings");
	//m_wheelRd = fSetVar;

	//if(aSettingFile_GetFloat(m_ioRef, m_settings, aKEY_WHEEL_TRACK,  
	//		&fSetVar, aWHEEL_TRACK, &e)) 
	//	throw acpException(e, "getting wheel track from settings");
	//m_wheelTrk = fSetVar;

	if(aSettingFile_GetFloat(m_ioRef, m_settings, KEY_METER_PER_TICK,  
			&fSetVar, METER_PER_TICK, &e)) 
		throw acpException(e, "getting meter per tick");	
		m_metersPerTick = fSetVar;
  
  if(aSettingFile_GetFloat(m_ioRef, m_settings, aKEY_WHEEL_BASE,  
			&fSetVar, WHEEL_BASE, &e)) 
		throw acpException(e, "getting wheel track from settings");
	m_wheelBase = fSetVar;

#ifdef aUSE_GPS	
	if (m_pStem && m_pStem->isConnected()) {
		
		/*lets do some initialization. First we need to find out
		* whether we have a good GPS signal. If we do, we'll init
		* our starting position from the GPS position information
		* else we'll assume we're at the 0,0,0 position.	
	  */
		int timeout = 0;
		bool haveGPS = false;
		while (!(haveGPS = getGPSQuality()) && timeout < aGPS_LOCK_STEPS) {
			m_logger->log(INFO, "Setting up GPS subsystem %d", timeout);
			aIO_MSSleep(m_ioRef, 1000, NULL);
			++timeout;
			
		}
//	  
//		if (haveGPS) {
//			//Lets get a lat, lon, and heading... from compass. We shouldn't
//			//be moving yet.
//		
//			m_curPos.x = getGPSLongitude();						
//			m_curPos.y = getGPSLatitude();
//			//We shouldn't have moved yet, so we get the heading from the 
//			//compass instead of the GPS, we only want to get the compass
//			//while outside to reduce the potential confound of a prevailing
//			//non-earth magnetic field.
//			m_curPos.h = getCMPSHeading(); 	
//	
//    } //else the default initilization of the state vector
//			//and probability matrix is zero'd.
//
//		//We should always be able to get a timestamp from the GPS unit.
//		//even without the required number of satellites.		
//		m_curGPSTimeSec = getGPSTimeSec();
//
//    //Lets initialize the encoders to zero.
//		//m_pStem->MO_ENC32(aMOTO_MODULE, aMOTOR_RIGHT, 0);
//		m_pStem->MO_ENC32(aMOTO_MODULE, aMOTOR_LEFT, 0);
//
//    //Initialize the Variance matrix Q.
//    //1 meter x and y, and 1 degree heading.
//    // ~ .31 meters per reading. * 10% is .0312 
//    m_Q(1,1) = aLAT_PER_METER * .0312;
//    m_Q(2,2) = aLON_PER_METER * .0312;
//    // heading variance 
//    m_Q(3,3) = DEG_TO_RAD * .25;
//
//    //The GPS is accurate to a meter, but I don't really trust that. 
//    m_W(1,1) = aLAT_PER_METER * 1.25;
//    m_W(2,2) = aLON_PER_METER * 1.25;
//    //Compass is accurate to 4 degrees.
//    m_W(3,3) = DEG_TO_RAD * 2.5;
		return aErrNone;
	} 
	
	else {
		return aErrConnection;
	}
#endif
	
	return aErrNone;
	
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
	int curEnc = getEncoderValue();
  
  	//convert those into distance traveled each wheel.
	//double dWheel = ((double) (curEnc - m_Encoder))/ m_ticksPerRev;
	//The forward moving distance
	//double fDist = dWheel * m_wheelRd;

	double fVelocity = m_metersPerTick * (curEnc - m_Encoder) / tmElapsed;
	m_logger->log(INFO, "Current Speed: %lf", fVelocity);
	
	// estimate change in x and y and heading
	// TODO - we should use previous state vector to calculate position here.
	// previous velocity.
	double dx = cos(m_curPos.h * DEG_TO_RAD)* fVelocity * tmElapsed * aLAT_PER_METER;
	double dy = sin(m_curPos.h * DEG_TO_RAD)* fVelocity * tmElapsed * aLON_PER_METER;
  
	// Change in heading due to the previous steering angle
  double fRot = fVelocity/m_wheelBase * tan(m_steerAngle);
  
	// Store current readings (for the next predict phase)
  double steerAngle = getSteeringAngle();
	m_steerAngle = steerAngle;
  m_Encoder= curEnc;
	
	Matrix state(3,1);
	//JLG flipped cos/sin - world cordinate system aligns latitude values with,
	//values along the x axis, and longitude values along the y axis.
	state(1,1) = dx + m_curPos.x;
	state(2,1) = dy + m_curPos.y;
	state(3,1) = m_curPos.h + (fRot * RAD_TO_DEG);
  m_logger->log(INFO, "Set State: ");
	//EKF from here on out.
	//We need to calculate the probability matrix for the motion each time.
	Matrix F(3,3), G(3,3);
	
	//Setting diags for F to 1, and G 33 to 1.
	F(1,1) = F(2,2) = F(3,3) = 1;
	//Calculating the Linearization Matrix points for F.
	F(1,3) = -dx*sin(m_curPos.h) - dy*cos(m_curPos.h);
	F(2,3) = dx*cos(m_curPos.h) - dy*cos(m_curPos.h);
	
	//Calculate the Linearization Matrix points for G.
	G(1,1) = cos(m_curPos.h); G(1,2) = -sin(m_curPos.h);
  G(2,1) = sin(m_curPos.h); G(2,2) = cos(m_curPos.h);
  G(3,3) = 1;
  m_logger->log(INFO, "Calculated F and G: ");
	
	//Now calculate the probabilit matrix for the position.
	m_P = F * m_P * F.transpose() + G * m_Q * G.transpose();
	
	//We really only want to use GPS information if enough time has passed.
	int curSec = getGPSTimeSec();
  
	double curLat = m_curPos.y;
	double curLon = m_curPos.x;
	double curHed = m_curPos.h;
	if (tmElapsed > 1200 && curSec != m_curGPSTimeSec && getGPSQuality()) {
	
		//Grab current GPS and compass settings, otherwise we'll rely on 
		//a Kalman update with encoder values only.
		curLat = getGPSLatitude();
		curLon = getGPSLongitude();
		curHed = getCMPSHeading();
		
		//Lets to this KF thing.
		Matrix V(3, 1);
		V(1,1) = curLat - state(1,1);
		V(2,1) = curLon - state(2,1);
		V(3,1) = curHed - state(3,1);
	  
	  Matrix S(3, 3), R(3, 3);
	  S = m_P + m_W;
	  R = m_P * S.invert();
	  state = state + R * V;
	  
	  m_P = m_P - R * m_P;
	
		fprintf(gps_track, "%3.12f, %2.12f, %3.1f\n", 
		       curLon, curLat, curHed);
		m_curGPSTimeSec = curSec;
		
		m_curClock = clock();
	}

	//Localization is done. Update the current robot state.
	m_curPos.x = state(2, 1);
	m_curPos.y = state(1, 1);
	m_curPos.h = state(3, 1);
	
}

/////////////////////////////////////////////////////////////////////////////

int 
avcPosition::getGPSTimeSec(void) {
	int secs = 0;
//	int tmp = 0;
//	tmp = m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_MIN) << 8; 
//  tmp |= m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_MIN+1);
//	secs = tmp * 60;
//	tmp = m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_SEC) << 8; 
//  tmp |= m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_SEC+1);
//	secs += tmp;
		
	return secs;	
}

/////////////////////////////////////////////////////////////////////////////

bool 
avcPosition::getGPSQuality(void) {

  unsigned char value = 0;
  m_pStem->IIC_RD(GPS_MODULE, 33, 1, &value);
  m_logger->log(INFO, "Quality value: %d", value);
	return !!(value);
}

/////////////////////////////////////////////////////////////////////////////

double 
avcPosition::getGPSLongitude(void)
{

	//We're in the western hemisphere, so we'll have a negative longitude.
	double retVal = 0.0;
//	short tmp = 0;
//	tmp = m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_LON) << 8; 
//  tmp |= m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_LON+1);
//	retVal = (double) tmp;
//	tmp = m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_LON+2) << 8; 
//  tmp |= m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_LON+3);
//	retVal += ((double) tmp)/60.0;
//	tmp = m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_LON+4) << 8; 
//  tmp |= m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_LON+5);
//	retVal += ((double) tmp)/600000.0;
	
	return retVal * -1.0;
}

/////////////////////////////////////////////////////////////////////////////

double 
avcPosition::getGPSLatitude(void)
{
	double retVal = 0.0;
//	short tmp = 0;
//	tmp = m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_LAT) << 8; 
//  tmp |= m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_LAT+1);
//	retVal = (double) tmp;
//	tmp = m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_LAT+2) << 8; 
//  tmp |= m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_LAT+3);
//	retVal += ((double) tmp)/60.0;
//	tmp = m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_LAT+4) << 8; 
//  tmp |= m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_LAT+5);
//	retVal += ((double) tmp)/600000.0;

	return retVal;
}


/////////////////////////////////////////////////////////////////////////////
// Take the compass bearing from the CMP10 board
// See:
// http://www.robot-electronics.co.uk/htm/cmps10i2c.htm
// 
// Reading from registers 2 and 3 gives the bearing with a 0.1
// increment resolution
// Result should be in radians 

double
avcPosition::getCMPSHeading(void) {
	
  double retVal = 0.0;
  unsigned short tmp = 0;
	unsigned char data[3] = { 0,0,0 };
	
	// Set the I2C read pointer
	data[0] = 1;
	aPacketRef p = m_pStem->createPacket(CMPS_MODULE, 1, data);
	m_pStem->sendPacket(p);
  
	// Read from the I2C module
	// Compass bearing as a word is in register 1
	// so we should read 2 bytes out of it
	m_pStem->IIC_RD(aUSBSTEM_MODULE, CMPS_MODULE, 1, data);
	
	// Convert readings into a word
	tmp = 255 - data[0];
	
	// Convert into the reading we need in degrees to radians
	retVal = 2 * 0.0174532925 * ((double) tmp * (255.0 / 360.0));
	
	//printf("0x%X (%d) [%f]\n", data[0], tmp, retVal);
	
	return retVal;
}


/////////////////////////////////////////////////////////////////////////////
int 
avcPosition::getEncoderValue(void) {
	
  //We could do more here to check for encoder wrap.
  return m_pStem->MO_ENC32(aMOTO_MODULE, ENCODER_IDX);
}


/////////////////////////////////////////////////////////////////////////////
double
avcPosition::getSteeringAngle(void) {

	//We could do more here to check for encoder wrap.
  unsigned char setpoint = m_pStem->PAD_IO(aSERVO_MODULE, AUTPAD_STEER);
	return (setpoint - SERVO_NEUT) * MAX_TURNANGLE / SERVO_NEUT;
	
}

/////////////////////////////////////////////////////////////////////////////
int
avcPosition::getAccelerometerReadings (float *ddx, float *ddy, float *ddz) {
  if(!ddx || !ddy || !ddz){
    m_logger->log(ERROR, "Null pointer passed to getAccelerometerReadings");
    return -1;
  }

  *ddx = m_pStem->A2D(aUSBSTEM_MODULE, aACCEL_X_CHAN);
  *ddy = m_pStem->A2D(aUSBSTEM_MODULE, aACCEL_Y_CHAN);
  *ddz = m_pStem->A2D(aUSBSTEM_MODULE, aACCEL_Z_CHAN);
  
  return 0;
  
}


///////////////////////////////////////////////////////////////////////////
// This section is for isolating and debugging this module. 
// You will need to set up a stem object and work from that.
// Use the makefile to build this in isolation.
// > make repulsiveForceTest
#ifdef aDEBUG_FLOC

////////////////////////////////////////
// main testing routine for motModule 
int 
main(int argc, 
     const char* argv[]) 
{
  
  acpStem stem;
  aSettingFileRef settings;
  aErr e = aErrNone;
  aIOLib ioRef;
  
  // Grab an aIO reference object to gain the setting to talk to the stem.
  aIO_GetLibRef(&ioRef, &e);
  
  // Read from a settings file if it exists.
  if (aSettingFile_Create(ioRef, 
                          "chicken.config",
                          &settings,
                          &e))
    throw acpException(e, "creating settings");
  
  // or, maybe command line arguements
  aArguments_Separate(ioRef, settings, NULL, argc, argv);
  
  avcPosition position; 
  
  printf("connecting to stem\n");
  
  // This starts up the stem link processing and is called once to spawn
  // the link management thread... based on the settings.
  stem.startLinkThread(settings);
  
  // Wait until we have a solid heartbeat connection so we know there is 
  // someone to talk to.
  int timeout = 0;
  
  printf("awaiting heartbeat");
  
  do { 
    printf(".");
    fflush(stdout);
    aIO_MSSleep(ioRef, 500, NULL);
    ++timeout;
  } while (!stem.isConnected() && timeout < 30);
  
  printf("\n");
  
  // Bail if no stem. What's the point little man?
  if (timeout == 10) { return 1; }
	
	position.init(&stem, settings);
	
	for (int i = 0; i < 500; i++) {
		
		position.getCMPSHeadingTest();
		
		//printf("Compass reading: %f\n", position.getCMPSHeadingTest());
		
		aIO_MSSleep(ioRef, 50, NULL);
	}
  
  //printf("GPS Quality: %s\n", (position.getGPSQuality())? "good": "bad");
  
  aIO_MSSleep(ioRef, 1000, NULL);
  
  //////////////////////////////////
  // Clean up and get outta here. Run Forrest, RUN!!!
  aSettingFile_Destroy(ioRef, settings, NULL);
  aIO_ReleaseLibRef(ioRef, NULL);
  
  return 0;
  
}


#endif

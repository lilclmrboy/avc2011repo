#include "locModule.h"
#include <sys/time.h>

//We'll wait up to 30 seconds for a GPS lock on initialization.
#define aGPS_LOCK_STEPS 3
#define DEG_TO_RAD (aPI/180)
#define RAD_TO_DEG (180/aPI)
////////////////////////////////////////////////////////////////////////////
aErr
avcPosition::init(acpStem* pStem, 
                  aSettingFileRef settings, 
                  avcWaypointVector firstMapPoint) {
	
	m_pStem = pStem;
	m_settings = settings;
	m_logger = logger::getInstance();	
  m_pCompass = new LSM303DLM(m_pStem, m_settings);
  m_pCompass->init();
	aErr e = aErrNone;
  
  // make an accelerometer class and start a thread for the EKF
  m_pAccel = new accelerometerADXL335(m_pStem, m_settings);
  m_pAccel->init();
  m_pAccelThread = new avcAccelerometerThread(m_pAccel);

	char buffer[100];
	time_t rawtime;
	struct tm* timeinfo;
	time(&rawtime);
        timeinfo = localtime( &rawtime );
	strftime(buffer,100, "GPS_Track_%d_%m_%H_%M.gpx", timeinfo);
  
  m_logger->log(DEBUG, "Writing GPS track: %s", buffer);
 
	gps_track = fopen(buffer, "w");
	if (!gps_track) {
    m_logger->log(ERROR, "Not able to write GPS track. e = %d", e);
		return aErrIO;
  }

	fprintf(gps_track, "Chicken GPS track\n");
	fflush(gps_track);
	
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
  
  // Set the first position waypoint that we passed in
  setPosition(firstMapPoint);


	if (m_pStem && m_pStem->isConnected()) {
		
		//Set current time.	
		aIO_GetMSTicks(m_ioRef, (unsigned long int*)&m_gpsClock, NULL);
		
    //Lets initialize the encoders to zero.
		m_pStem->MO_ENC32(aMOTO_MODULE, ENCODER_IDX, 0);
		m_Encoder = 0;
		
//#ifdef aUSE_GPS
#if 0
		/*lets do some initialization. First we need to find out
		* whether we have a good GPS signal. If we do, we'll init
		* our starting position from the GPS position information
		* else we'll assume we're at the 0,0,0 position.	
	  */
    
    //m_gps = gps::getInstance();
    //m_gps->init("ttyUSB1", 57600);
		int timeout = 0;
		bool haveGPS = false;
		while (!(haveGPS = getGPSQuality()) && timeout < aGPS_LOCK_STEPS) {
			m_logger->log(INFO, "Setting up GPS subsystem %d", timeout);
			aIO_MSSleep(m_ioRef, 1000, NULL);
			++timeout;
			
		}
	  
		if (0){//(haveGPS) {
			//Lets get a lat, lon, and heading. We shouldn't
		  //be moving yet.
      setPosition(avcWaypointVector(getGPSLongitude(), 
                                    getGPSLatitude(), 
                                    getHeading()));
	
    } //else the default initilization of the state vector
			//and probability matrix is zero'd.
#endif

    //Initialize the Variance matrix Q.
    //1 meter x and y, and 1 degree heading.
    // ~ .31 meters per reading. * 10% is .0312 
    m_Q(1,1) = aLAT_PER_METER * .0312;
    m_Q(2,2) = aLON_PER_METER * .0312;
    // heading variance 
    m_Q(3,3) = DEG_TO_RAD * .25;

    //The GPS is accurate to a meter, but I don't really trust that. 
    m_W(1,1) = aLAT_PER_METER * 1.25;
    m_W(2,2) = aLON_PER_METER * 1.25;
    //Compass is accurate to 4 degrees.
    m_W(3,3) = DEG_TO_RAD * 2.5;
		return aErrNone;
	} 
	
	else {
		return aErrConnection;
	}
	
	return aErrNone;
	
}

/////////////////////////////////////////////////////////////////////////////

void 
avcPosition::updateState(short lastThrottleSetPoint) {
	
	// Grab the clock. Use the aIO verison, clock is not accurate when 
	// we have stem processes running
	long unsigned int curClock;
	aIO_GetMSTicks(m_ioRef, &curClock, NULL);
	long unsigned int tmElapsed = (curClock - m_curClock);
	
	// First we must do an estimation step, given the previous position
  // and the current control information. Lets do this in meters, and then
	// convert to lat, lon.
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
  
	// Calculate the rear wheel velociy in meters / second.
  if(tmElapsed <= 0)
    m_logger->log(ERROR, "%s: Elapsed time is invalid", __FUNCTION__);
  
	// Get the steering angle.
	//double steerAngleRad = getSteeringAngleRad();
  //int motorSetPoint = getMotorSetPoint();
  int motorSetPoint = lastThrottleSetPoint;
	
  double fVelocity = (motorSetPoint < SERVO_NEUT ? -1 : 1) * m_metersPerTick * (double)(curEnc - m_Encoder) / ((double)tmElapsed / 1000.0);
  double fDistRolled = (motorSetPoint < SERVO_NEUT ? -1 : 1) * m_metersPerTick * (double)(curEnc - m_Encoder);
	m_logger->log(INFO, "Current Speed (m/s): %lf (tick/time=%d/%d)", fVelocity, (curEnc - m_Encoder), tmElapsed);
  
  m_curPos.h = getHeading();
	
  // These calculations are predictions of where we think we need to be
	// estimate change in x and y and heading
	// TODO - we should use previous state vector to calculate position here.
	// previous velocity.
	double dx = sin(m_curPos.h * DEG_TO_RAD)* fDistRolled * aLON_PER_METER;
	double dy = cos(m_curPos.h * DEG_TO_RAD)* fDistRolled * aLAT_PER_METER;
    
	// Change in heading due to the previous steering angle
  //double fRot = fDistRolled/m_wheelBase * tan(steerAngleRad) * RAD_TO_DEG;
  
  if(1){//(curEnc - m_Encoder) {
    //m_logger->log(INFO, "%s: --- SteerAng(Rad): %f", __FUNCTION__, steerAngleRad);
    m_logger->log(INFO, "%s: --- Throt: %d", __FUNCTION__, motorSetPoint);
  	m_logger->log(INFO, "%s: --- fDist: %f", __FUNCTION__, fDistRolled);
    m_logger->log(INFO, "%s: --- dx(m): %f", __FUNCTION__, dx/aLON_PER_METER);
    m_logger->log(INFO, "%s: --- dy(m): %f", __FUNCTION__, dy/aLAT_PER_METER);
    //m_logger->log(INFO, "%s: --- fRot(deg): %f", __FUNCTION__, fRot);
    m_logger->log(INFO, "%s: --- hed: %f", __FUNCTION__, m_curPos.h);
  }
	// Store current readings (for the next predict phase)
  m_Encoder= curEnc;
  m_curClock = curClock;
	
  // Update the current control state vector
  // These vector values should be in latitude, longitude and degrees
	m_curPos.x += dx;
	m_curPos.y += dy;
	//m_curPos.h += fRot; 
	
	/*
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
		curHed = getHeading();
		
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
	*/
	 
}


/////////////////////////////////////////////////////////////////////////////

void
avcPosition::recordGPSPoint(void) {

	long unsigned int curTime, timeElapsed;
	aIO_GetMSTicks(m_ioRef, &curTime, NULL);
	
	timeElapsed = curTime - m_gpsClock;
	
	if(timeElapsed > 2000 ) {//&& getGPSQuality()) {
	
        float curLat= 0.0f, curLon= 0.0f, curHed = 0.0f;
		
        m_gps->getPosition(&curLon, &curLat, &curHed);
		
		fprintf(gps_track, "%3.12f, %2.12f, %3.1f\n", 
						curLon, curLat, curHed);
		fflush(gps_track);
		
    // Store the reading for the next time around
		m_gpsClock = curTime;
		
	}
}


bool 
avcPosition::getGPSQuality(void) {

  unsigned long value = 0;
  //value = aGPM_GetGPSQuality(m_pStem);
  value = m_gps->hdop();

  m_logger->log(INFO, "GPS HDOP value: %ld", value);
  
	return ( value > 5000 ? false : true);
}

/////////////////////////////////////////////////////////////////////////////

double 
avcPosition::getGPSLongitude(void)
{

	//We're in the western hemisphere, so we'll have a negative longitude.
	float longitude=0.0, latitude=0.0, heading=0.0;
  //retVal = (double)(aGPM_GetLongitudeDegrees(m_pStem));
  //r/etVal += (double)(aGPM_GetLongitudeMinutes(m_pStem)) / 60.0;
  //retVal += (double)(aGPM_GetLongitudeFrac(m_pStem)) / 600000.0;
  
  m_gps->getPosition(&longitude, &latitude, &heading);
	
	return (double)longitude * -1.0;
}

/////////////////////////////////////////////////////////////////////////////

double 
avcPosition::getGPSLatitude(void)
{
	float longitude=0.0, latitude=0.0, heading=0.0;
  //retVal = aGPM_GetLatitudeDegrees(m_pStem);
  //retVal += (double)(aGPM_GetLatitudeMinutes(m_pStem)) / 60.0;
  //retVal += (double)(aGPM_GetLatitudeFrac(m_pStem)) / 600000.0;
  m_gps->getPosition(&longitude, &latitude, &heading);
	return (double) latitude;
}


/////////////////////////////////////////////////////////////////////////////
// Compass wrapper class for whatever widget we are using.

double
avcPosition::getHeading(void) {
	
  double retVal=0.0;
	float headingDeg=0.0f;
  if(0 != m_pCompass->getHeadingDeg(&headingDeg)){
    m_logger->log(ERROR, "%s: Error getting current heading", __FUNCTION__);
  	return 0.0;
  }
  
  retVal = (double) headingDeg;
	return retVal;
}


/////////////////////////////////////////////////////////////////////////////
int 
avcPosition::getEncoderValue(void) {
	
  //We could do more here to check for encoder wrap.
  return m_pStem->MO_ENC32(aMOTO_MODULE, ENCODER_IDX);
  
}


/////////////////////////////////////////////////////////////////////////////
// Returns the steering angle in radians
double
avcPosition::getSteeringAngleRad(void) {

	//We could do more here to check for encoder wrap.
  // Cubic method.
	
	
	// Linear method.
  // Need to read the second byte, since the PAD_IO writes 2 bytes at a time
	unsigned char setpoint = m_pStem->PAD_IO(aSERVO_MODULE, AUTPAD_STEER+1);
	return (setpoint - SERVO_NEUT) * MAX_TURNANGLE / SERVO_NEUT;
	
}

/////////////////////////////////////////////////////////////////////////////
// Returns the motor set point (to be used to determine forward/backward driving
int
avcPosition::getMotorSetPoint(void) {
  
	// Linear method.
  // Need to read the second byte, since the PAD_IO writes 2 bytes at a time
	unsigned char setpoint = m_pStem->PAD_IO(aSERVO_MODULE, AUTPAD_THROT+1);
	return setpoint;
	
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
	
	position.init(&stem, settings, avcWaypointVector(0,0,0));

//	for (int i = 0; i < 500; i++) {
  while (1){
    //float acc_x = 0, acc_y = 0, acc_z = 0;
		//printf("Compass reading: %f\n", position.getCHeading() * RAD_TO_DEG);
    printf("GPS Quality: %s\n", (position.getGPSQuality())? "good": "bad");
    printf("GPS sat count: %d\n", aGPM_GetSatellitesInUse(&stem));
    printf("GPS time: %d:%d:%d\n", aGPM_GetHours(&stem), aGPM_GetMinutes(&stem), aGPM_GetSeconds(&stem));
    printf("GPS date: %d-%d-%d\n", aGPM_GetYear(&stem), aGPM_GetMonth(&stem), aGPM_GetDay(&stem));
		
		aIO_MSSleep(ioRef, 200, NULL);
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

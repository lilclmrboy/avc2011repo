#include "locModule.h"
#include <sys/time.h>
#include "gps.h"

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
	aErr e;

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

#ifdef aUSE_GPS	
	if (m_pStem && m_pStem->isConnected()) {
		
		//Set current time.	
		aIO_GetMSTicks(m_ioRef, (unsigned long int*)&m_gpsClock, NULL);
		
    //Lets initialize the encoders to zero.
		m_pStem->MO_ENC32(aMOTO_MODULE, ENCODER_IDX, 0);
		m_Encoder = 0;
		
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
	  
		if (haveGPS) {
			//Lets get a lat, lon, and heading. We shouldn't
		  //be moving yet.
      setPosition(avcWaypointVector(getGPSLongitude(), 
                                    getGPSLatitude(), 
                                    getHeading()));
	
    } //else the default initilization of the state vector
			//and probability matrix is zero'd.


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
#endif
	
	return aErrNone;
	
}

/////////////////////////////////////////////////////////////////////////////

void 
avcPosition::updateState() {
	
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
  
  m_logger->log(INFO, "%s: Elapsed time: %dms", __FUNCTION__, tmElapsed);
	double fVelocity = m_metersPerTick * (curEnc - m_Encoder) / ((double)tmElapsed / 1000.0);
	m_logger->log(INFO, "Current Speed (m/s): %lf", fVelocity);
  
	// Get the steering angle.
	double steerAngle = getSteeringAngle();
	
  // These calculations are predictions of where we think we need to be
	// estimate change in x and y and heading
	// TODO - we should use previous state vector to calculate position here.
	// previous velocity.
	double dx = cos(m_curPos.h * DEG_TO_RAD)* fVelocity * tmElapsed * aLON_PER_METER;
	double dy = sin(m_curPos.h * DEG_TO_RAD)* fVelocity * tmElapsed * aLAT_PER_METER;
    
	// Change in heading due to the previous steering angle
  double fRot = fVelocity/m_wheelBase * tan(steerAngle);
  
  m_logger->log(INFO, "%s: dx(m), dy(m), fRot(deg): %f, %f, %f", __FUNCTION__, dx/aLON_PER_METER, dy/aLAT_PER_METER, fRot*RAD_TO_DEG);
  
	// Store current readings (for the next predict phase)
  m_Encoder= curEnc;
  m_curClock = curClock;
	
  // Update the current control state vector
  // These vector values should be in latitude, longitude and degrees
	m_curPos.x += dx;
	m_curPos.y += dy;
	m_curPos.h += fRot; 
	
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

//int 
//avcPosition::getGPSTimeSec(void) {
//	int secs = 0;
////	int tmp = 0;
////	tmp = m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_MIN) << 8; 
////  tmp |= m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_MIN+1);
////	secs = tmp * 60;
////	tmp = m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_SEC) << 8; 
////  tmp |= m_pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_GPS_SEC+1);
////	secs += tmp;
//		
//	return secs;	
//}

/////////////////////////////////////////////////////////////////////////////

void
avcPosition::recordGPSPoint(void) {

	long unsigned int curTime, timeElapsed;
	aIO_GetMSTicks(m_ioRef, &curTime, NULL);
	
	timeElapsed = curTime - m_gpsClock;
	
	if(timeElapsed > 2000 ) {//&& getGPSQuality()) {
	
		double curLat= 0.0, curLon= 0.0, curHed = 0.0;
		
		curLat = getGPSLatitude();
		curLon = getGPSLongitude();
		curHed = getHeading();
		
		fprintf(gps_track, "%3.12f, %2.12f, %3.1f\n", 
						curLon, curLat, curHed);
		fflush(gps_track);
		
    // Store the reading for the next time around
		m_gpsClock = curTime;
		
	}
}

/////////////////////////////////////////////////////////////////////////////

bool 
avcPosition::getGPSQuality(void) {

  unsigned char value = 0;
  value = aGPM_GetGPSQuality(m_pStem);

  m_logger->log(INFO, "Quality value: %d", value);
  
	return !!(value);
}

/////////////////////////////////////////////////////////////////////////////

double 
avcPosition::getGPSLongitude(void)
{

	//We're in the western hemisphere, so we'll have a negative longitude.
	double retVal = 0.0;
  retVal = (double)(aGPM_GetLongitudeDegrees(m_pStem));
  retVal += (double)(aGPM_GetLongitudeMinutes(m_pStem)) / 60.0;
  retVal += (double)(aGPM_GetLongitudeFrac(m_pStem)) / 600000.0;
	
	return retVal * -1.0;
}

/////////////////////////////////////////////////////////////////////////////

double 
avcPosition::getGPSLatitude(void)
{
	double retVal = 0.0;
  retVal = aGPM_GetLatitudeDegrees(m_pStem);
  retVal += (double)(aGPM_GetLatitudeMinutes(m_pStem)) / 60.0;
  retVal += (double)(aGPM_GetLatitudeFrac(m_pStem)) / 600000.0;
	return retVal;
}


/////////////////////////////////////////////////////////////////////////////
// Compass wrapper class for whatever widget we are using.

double
avcPosition::getHeading(void) {
	
  double retVal = 0.0;
	
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
  // Cubic method.
	
	
	// Linear method.
	unsigned char setpoint = m_pStem->PAD_IO(aSERVO_MODULE, AUTPAD_STEER);
	return (setpoint - SERVO_NEUT) * MAX_TURNANGLE / SERVO_NEUT;
	
}

/////////////////////////////////////////////////////////////////////////////
// Read the accelerometer sensor data
// See the datasheet for more information
// http://www.analog.com/static/imported-files/data_sheets/ADXL335.pdf

int
avcPosition::getAccelerometerReadings (float *ddx, float *ddy, float *ddz) {
  
  // Scaling is radiometric
  /* From datasheet:
     VCC(3.6V) = 360 mV/g
     VCC(2.0V) = 195 mV/g
     thus
     VCC(3.3V) = 329.0625 mV/g
     our readings are in normalized 0 - 1.0 values across a 3.3V range
     
  */
  const float a2d_ref = 3.3f; // V
  float scale_factor = a2d_ref * 1.0f / 0.3290625f; // g
   
    
  if(!ddx || !ddy || !ddz){
    m_logger->log(ERROR, "%s: Null pointer passed to getAccelerometerReadings", __FUNCTION__);
    return -1;
  }
 
#ifdef aACCEL_BULK_CAPTURE
  
  aUInt8 slot = 9;
  unsigned int samples = 50;
  unsigned short pace = 10;
  aStreamRef s = NULL;
  aErr e = aErrNone;
  aMemSize len = 0;
  char *pData;
  unsigned int index = 0;
  
  m_logger->log(INFO, "Starting bulk capture\n");
  
  // Tell the stem to fire off the readings
  m_pStem->A2D(aUSBSTEM_MODULE, aACCEL_X_CHAN, slot, pace, samples);
  
  // Read the data back from the Stem
  // First create a memory buffer to write the capture to
  if (aStreamBuffer_Create(m_ioRef, 4092, &s, &e))
      throw acpException(e, "creating bulk read buffer");
  
  // Unload the slot data capture into a buffer
  m_pStem->unloadSlot(aUSBSTEM_MODULE, slot, s);
  
  // Put the data into the storage array
  if (aStreamBuffer_Get(m_ioRef, s, &len, &pData, &e))
      throw acpException(e, "getting buffer data");
  
  // Debug what we have read from the stem
  m_logger->log(INFO, "Retreived %d data points from bulk capture\n", len);
  
  // Once we get data back, then we can chip through it
  // The bulk raw data is stored in 2 byte pairs creating a 16 bit word
  if (len > 1) {
    
    for (unsigned int i = 0; i < samples; i++) {
      m_logger->log(DEBUG, "[%d]: %d\n", 
                    i, 
                    pData[index] << 8 + pData[index + 1]);
      index+=2;
    }
    
  }
  
  // Clean up the stream reference
  if (aStream_Destroy(m_ioRef, s, &e))
    throw acpException(e, "destroying bulk stream");
  
  // let us know we are finished
  m_logger->log(INFO, "Ending bulk capture\n");
  
#endif // end of buik capture
	
  // Take the reading values and average them
  *ddx = scale_factor * m_pStem->A2D(aUSBSTEM_MODULE, aACCEL_X_CHAN);
  *ddy = scale_factor * m_pStem->A2D(aUSBSTEM_MODULE, aACCEL_Y_CHAN);
  *ddz = scale_factor * m_pStem->A2D(aUSBSTEM_MODULE, aACCEL_Z_CHAN);
  
  if (-1 == *ddx || -1 == *ddy || -1 == *ddz){
    m_logger->log(ERROR, "%s: A2D timeout while reading accelerometer", __FUNCTION__);
    return -1;
  }
  
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

//	for (int i = 0; i < 500; i++) {
  while (1){
    //float acc_x = 0, acc_y = 0, acc_z = 0;
		//printf("Compass reading: %f\n", position.getCHeading() * RAD_TO_DEG);
    printf("GPS Quality: %s\n", (position.getGPSQuality())? "good": "bad");
    printf("GPS sat count: %d\n", aGPM_GetSatellitesInUse(&stem));
    printf("GPS time: %d:%d:%d\n", aGPM_GetHours(&stem), aGPM_GetMinutes(&stem), aGPM_GetSeconds(&stem));
    //position.getAccelerometerReadings(&acc_x, &acc_y, &acc_z);
    //printf("Accel: %3.2f, %3.2f, %3.2f\n", acc_x, acc_y, acc_z);
		
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

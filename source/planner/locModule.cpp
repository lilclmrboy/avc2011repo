#include "locModule.h"
#include <sys/time.h>
#include "gps.h"

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
	
	//Grab the clock. Use the aIO verison, clock is not accurate when 
	// we have stem processes running
	long unsigned int curClock;
	aIO_GetMSTicks(m_ioRef, &curClock, NULL);
	long tmElapsed = (curClock - m_curClock);
	
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
  
	// Calculate the rear wheel velociy in meters / second.
	double fVelocity = m_metersPerTick * (curEnc - m_Encoder) / (tmElapsed / 1000);
	m_logger->log(INFO, "Current Speed: %lf", fVelocity);
	// Get the steering angle.
	double steerAngle = getSteeringAngle();
	
	// estimate change in x and y and heading
	// TODO - we should use previous state vector to calculate position here.
	// previous velocity.
	double dx = cos(m_curPos.h * DEG_TO_RAD)* fVelocity * tmElapsed * aLAT_PER_METER;
	double dy = sin(m_curPos.h * DEG_TO_RAD)* fVelocity * tmElapsed * aLON_PER_METER;
  
  m_logger->log(INFO, "%s: dx,dy: %f, %f", __FUNCTION__, dx, dy);
  
	// Change in heading due to the previous steering angle
  double fRot = fVelocity/m_wheelBase * tan(steerAngle);
  
	// Store current readings (for the next predict phase)
  m_Encoder= curEnc;
	
	m_curPos.x += dx * aLON_PER_METER;
	m_curPos.y += dy * aLAT_PER_METER;
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
	*/
	 
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

void
avcPosition::recordGPSPoint(void) {

	long unsigned int curTime, timeElapsed;
	aIO_GetMSTicks(m_ioRef, &curTime, NULL);
	
	timeElapsed = curTime - m_gpsClock;
	
	if(timeElapsed > 2000 ) {//&& getGPSQuality()) {
	
		double curLat= 0.0, curLon= 0.0, curHed = 0.0;
		
		curLat = getGPSLatitude();
		curLon = getGPSLongitude();
		curHed = getCMPSHeading();
		
		fprintf(gps_track, "%3.12f, %2.12f, %3.1f\n", 
						curLon, curLat, curHed);
		fflush(gps_track);
		
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
  aPacketRef p;
	  
  // test out the roll and pitch
#ifdef aDEBUG_FLOC
  p = m_pStem->createPacket(CMPS_MODULE, 4, data);
	m_pStem->sendPacket(p);
  m_pStem->IIC_RD(aUSBSTEM_MODULE, CMPS_MODULE, 1, data);
  m_logger->log(INFO, "%s: raw pitch %d",__FUNCTION__, data[0]);	
  
  p = m_pStem->createPacket(CMPS_MODULE, 4, data);
	m_pStem->sendPacket(p);
  m_pStem->IIC_RD(aUSBSTEM_MODULE, CMPS_MODULE, 1, data);
  m_logger->log(INFO, "%s: raw roll %d",__FUNCTION__, data[0]);	
#endif
  
  // Read from the I2C module
	// Compass bearing as a word is in register 1
	// so we should read 2 bytes out of it
  p = m_pStem->createPacket(CMPS_MODULE, 1, data);
	m_pStem->sendPacket(p);
	m_pStem->IIC_RD(aUSBSTEM_MODULE, CMPS_MODULE, 1, data);
  m_logger->log(INFO, "%s: raw compass %d",__FUNCTION__, data[0]);	
	
	// Convert readings into a word
	tmp = 255 - data[0];
	
	// Convert into the reading we need in degrees to radians
	retVal = DEG_TO_RAD * ((double) tmp * (255.0 / 360.0));
	
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
  // Cubic method.
	
	
	// Linear method.
	unsigned char setpoint = m_pStem->PAD_IO(aSERVO_MODULE, AUTPAD_STEER);
	return (setpoint - SERVO_NEUT) * MAX_TURNANGLE / SERVO_NEUT;
	
}

/////////////////////////////////////////////////////////////////////////////
int
avcPosition::getAccelerometerReadings (float *ddx, float *ddy, float *ddz) {
  if(!ddx || !ddy || !ddz){
    m_logger->log(ERROR, "__FUNCTION__: Null pointer passed to getAccelerometerReadings");
    return -1;
  }
	
  *ddx = m_pStem->A2D(aUSBSTEM_MODULE, aACCEL_X_CHAN);
  *ddy = m_pStem->A2D(aUSBSTEM_MODULE, aACCEL_Y_CHAN);
  *ddz = m_pStem->A2D(aUSBSTEM_MODULE, aACCEL_Z_CHAN);
  
  if (-1 == *ddx || -1 == *ddy || -1 == *ddz){
    m_logger->log(ERROR, "__FUNCTION__: A2D timeout while reading accelerometer");
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
		//printf("Compass reading: %f\n", position.getCMPSHeading() * RAD_TO_DEG);
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

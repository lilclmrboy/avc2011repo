///////////////////////////////////////////////////////////////////////////
// Filename: motModule.cpp

#include "motModule.h"

#ifdef aDEBUG_MOTMODULE
bool bDebugHeader = true;
#endif

///////////////////////////////////////////////////////////////////////////
// Constructor for the function
avcMotion::avcMotion() :
  m_pStem(NULL),
  m_settings(NULL),
  m_setpointMax(aMOTOR_SETPOINT_MAX),
  m_bInit(false),
  m_fThrottleWindow(aMOTMODULE_THROTTLE_WINDOW_DEFAULT)
{
  
  aErr e = aErrNone;
  
  // set up all the setpoint place holders
  for (int m = 0; m < aMOTOR_NUM; m++) {
    
    m_setpoint[m] = 0;
    m_setpointLast[m] = 0;
    
  }
  
  // Set the setpoint history to all neutral values
  for (int j = 0; j < aMOTMODULE_HISTORY_WINDOW; j++)
    m_setpointHistory[j] = SERVO_NEUT;
  
  // Create a aIO reference to manipulate settings file reference
  if(aIO_GetLibRef(&m_ioRef, &e)) 
    throw acpException(e, "Getting aIOLib reference");
  
}

///////////////////////////////////////////////////////////////////////////
// Destructor for the function
avcMotion::~avcMotion() 
{
  
  aErr e = aErrNone;
  
  if (aIO_ReleaseLibRef(m_ioRef, &e))
    throw acpException(e, "unable to destroy settings");
  
}

///////////////////////////////////////////////////////////////////////////
// Get the Stem object and initialize any thing else that we need to.

aErr
avcMotion::init(acpStem *pStem, aSettingFileRef settings) {
  
  // Get access to the logger class
  m_log = logger::getInstance();
  m_log->log(INFO, "Motion Module initialized");
  
  aErr e = aErrNone;
  
  // Grab the pointer to the Stem. 
  m_pStem = pStem;
  
  // Grab the pointer to the settings
  m_settings = settings;
  
  // Read motion module specific settings from the settings file
  // reference. Remember, this can get created from a configuration 
  // file, OR as command line input arguments. 
  int setpoint;
  aSettingFile_GetInt(m_ioRef, m_settings, 
		      aKEY_VELOCITY_SETPOINT_MAX,
		      &setpoint,
		      aMOTOR_SETPOINT_MAX,
		      &e);
  
  // Get the throttle max threshold as a percentage
  aSettingFile_GetFloat(m_ioRef, m_settings, 
                        aMOTMODULE_THROTTLE_WINDOW_KEY, &m_fThrottleWindow, 
                        aMOTMODULE_THROTTLE_WINDOW_DEFAULT, &e);
  
  // Boundary check the throttle window
  m_fThrottleWindow = m_fThrottleWindow < 0.0f ? 0.0f : m_fThrottleWindow;
  m_fThrottleWindow = m_fThrottleWindow > 1.0f ? 1.0f : m_fThrottleWindow;
  
  // Copy this into our member variable. Our setpoint is bounded by a 
  // 2 byte value. Short is enough. Likely even transition to an 
  // unsigned char since we are not likely to go past 200. 
  m_setpointMax = (short) setpoint;
  
  // We store and send the host side setpoint values into a scratchpad
  // A TEA process running on the servo module reads from this 
  // pad location and determines if it should use this or
  // do the manual override RC value.
  m_pStem->PAD_IO(aSERVO_MODULE, AUTPAD_THROT, SERVO_NEUT);
  m_pStem->PAD_IO(aSERVO_MODULE, AUTPAD_STEER, SERVO_NEUT);

  //m_log->log(INFO, "Read back steer pad %d",   m_pStem->PAD_IO(aSERVO_MODULE, AUTPAD_STEER+1))
  
  // Setting ramp configuration for the GP servo throttle setting
  // Where the second
  try {
    m_pStem->SRV_CFG(aSERVO_MODULE, THROT_SERVO, 
                     bitCMD_DEST_HOST | 0x81);
    m_pStem->SRV_CFG(aSERVO_MODULE, STEER_SERVO, 
                     bitCMD_DEST_HOST | 0x00);
  } catch (acpException &err) {
    m_log->log(ERROR, "%s: error writing ramp to GP", __FUNCTION__, err.msg());
  }
  
  // Read back the servo configuration from each servo motor
  unsigned char rampCheckThrottle = m_pStem->SRV_CFG(aSERVO_MODULE, THROT_SERVO);
  unsigned char rampCheckSteering = m_pStem->SRV_CFG(aSERVO_MODULE, STEER_SERVO);
  
  m_log->log(INFO, "Steering Servo Ramp: 0x%X", rampCheckSteering);
  m_log->log(INFO, "Throttle Servo Ramp: 0x%X", rampCheckThrottle);
  
  // Set the flag to inidicate that we have been properly initialized
  m_bInit = true;
  
  return e;
  
}

///////////////////////////////////////////////////////////////////////////
// Given a new goal Force vector, update the each motor setpoint

aErr
avcMotion::updateControl(const avcForceVector& potential) 
{
  double magnitude = 0.0;
  double delta = 0.0;
  aErr e = aErrNone;
  
  // Make sure you other hackers initialized this first
  if (!m_pStem) {
    m_log->log(ERROR,"avcMotion::updateControl failed. acpStem is NULL\n"
               " \tCall avcMotion::init(acpStem *)");
    return aErrInitialization;
  }
  
  if (!m_bInit) {
    m_log->log(ERROR,"avcMotion::updateControl failed since uninitialized.\n"
               " \tCall avcMotion::init(acpStem *)");
    return aErrInitialization;
  }
  
  // Boundary check the input force vectors
  if (potential.x > 1.0 || potential.x < -1.0) {
    m_log->log(ERROR,"avcMotion::updateControl failed.\n"
               "\tavcForceVector.x is out of range");
    return aErrRange;
  }
  if (potential.y > 1.0 || potential.y < -1.0) {
    m_log->log(ERROR,"avcMotion::updateControl failed.\n"
               "\tavcForceVector.y is out of range");
    return aErrRange;
  }
  
  // calculate the magnitude of the resultant input vector
  magnitude = sqrt(potential.x * potential.x + potential.y * potential.y);
  delta = atan2(potential.y, potential.x);
  
  
  // don't go backwards!
  if (potential.x < 0)
  	magnitude = magnitude * -1.0;
  
  if (delta < 0) {
    //printf("delta is negative\n  delta: %f deltad: %f", delta, aPI*2 + delta);
    delta += aPI*2;
  }
  
  // The magnitude value directly translates to the gas pedal for the 
  // rear drive motor.
  // Scale the change window by the throttle window setpoint value
  unsigned char servoDrive = (unsigned char)(SERVO_NEUT 
                                             + ((SERVO_NEUT-1) * m_fThrottleWindow * magnitude));
  unsigned char servoSteer = SERVO_NEUT;
  
  // slow the bot down if we're going backwards since the steering is more
  // sensitive in reverse (it's easier to flip it)
  if (magnitude < 0) servoDrive = (unsigned char)(SERVO_NEUT + ((SERVO_NEUT-1) * m_fThrottleWindow * magnitude*0.8));
  
  // Update the servo values
  // The magnitude value directly translates to the gas pedal for the 
  // rear drive motor.
#if 0
  unsigned char servoOffset = (unsigned char) (delta/(aPI/2.0)* (float)SERVO_MAX);
  servoOffset = servoOffset > (unsigned char)SERVO_MAX-1 ? (unsigned char)SERVO_MAX-1 : servoOffset;
  servoOffset = servoOffset < (unsigned char)SERVO_MIN+1 ? (unsigned char)SERVO_MIN+1 : servoOffset;
  
  servoSteer = SERVO_NEUT + servoOffset;
#endif  
#if 1  
  unsigned char steerdelta = 0;
  // See how things should unfold
  if ((delta >= 0.0f) && (delta < MAX_TURNANGLE)) {
    steerdelta = (unsigned char)(delta/MAX_TURNANGLE * SERVO_NEUT);
    servoSteer = SERVO_NEUT + steerdelta;
  }
  else if ((delta >= MAX_TURNANGLE) && (delta < (aPI - MAX_TURNANGLE))) {
    //steerdelta = SERVO_MIN;
    servoSteer = SERVO_MAX;
  }
  else if ((delta >= (aPI - MAX_TURNANGLE)) && (delta < aPI)) {
    steerdelta = (unsigned char)((aPI - delta)/(MAX_TURNANGLE) * SERVO_NEUT);
    servoSteer = SERVO_NEUT + steerdelta;
  }
  else if ((delta >= aPI) && (delta < (aPI + MAX_TURNANGLE))) {
    steerdelta = (unsigned char)((delta - aPI)/(MAX_TURNANGLE) * SERVO_NEUT);
    servoSteer = SERVO_NEUT - steerdelta;
  }
  else if ((delta >= (aPI + MAX_TURNANGLE)) && (delta < (2.0*aPI - MAX_TURNANGLE))) {
    //steerdelta = SERVO_MIN;
    servoSteer = SERVO_MIN;
  }	
  else if ((delta >= (aPI*2.0 - MAX_TURNANGLE)) && (delta < aPI*2.0)) {
    steerdelta = (unsigned char)((2.0*aPI - delta)/(MAX_TURNANGLE) * SERVO_NEUT);
    servoSteer = SERVO_NEUT - steerdelta;
  }
#endif
  
#ifdef aDEBUG_MOTMODULE	
  // Show us what we got
  if (bDebugHeader) {
    m_log->log(DEBUG,"MotionModule: "
               "Ux\tUy\tmag\tdelta\tsrvD\tsrvS\tsDel\trad");
    bDebugHeader = false;
  }
  
  m_log->log(DEBUG,"MotionModule: "
             "%2.2f\t%2.2f\t%2.2f\t%2.2f\t%d\t%d\t%d\t%f",
             potential.x, potential.y, 
             magnitude, delta,
             servoDrive, servoSteer, steerdelta, delta);
  
#endif	    
  
  // Send the values to the stem
  // The controller has a braking mechanism so when the throttle crosses SERVO_NEUT
  // we need to set the value, set neut, set the value
  //    Instead of doing this, making a servo ramp happen in the GP seems to work better
    	if((m_setpointLast[0] > SERVO_NEUT && servoDrive < SERVO_NEUT) ||
         (m_setpointLast[0] < SERVO_NEUT && servoDrive > SERVO_NEUT)){
        
        m_log->log(INFO, "MotionModule: Detected rapid reversal in throttle (%d to %d)", m_setpointLast[0], servoDrive);
        m_pStem->PAD_IO(aSERVO_MODULE, AUTPAD_THROT, (aUInt8) servoDrive);
        m_pStem->sleep(1000); // short sleep for it to set
        m_pStem->PAD_IO(aSERVO_MODULE, AUTPAD_THROT, (aUInt8) SERVO_NEUT);
        m_pStem->sleep(1000); // short sleep for it to set      
      }
  
  // Do a running average on the steering servo setpoint. Use this value to 
  // send to the controller.

#if 1
  for (int i=aMOTMODULE_HISTORY_WINDOW-1; i>0; i--){
    m_setpointHistory[i] = m_setpointHistory[i-1];
  }
  m_setpointHistory[0] = servoSteer;

  // Calculate the running average of the setpoint values
  // This is through all the accululated values
  int setpointCumulative = 0;
  for (int n = 0; n < aMOTMODULE_HISTORY_WINDOW; n++) {
    setpointCumulative += m_setpointHistory[n];
    
    // Debug the cumulative setpoint total
    //m_log->log(DEBUG, "%s: setpoint %d: %d", 
    //           __FUNCTION__, n, m_setpointHistory[n]);
  }

  // Divide the addition of the values by the history window
  // Cast it and roundn it.
  servoSteer = (aUInt8) ((float)setpointCumulative / (float) aMOTMODULE_HISTORY_WINDOW);
#endif
  
  m_log->log(INFO, "MotionModule: Throttle, Steer: %d, %d", servoDrive, servoSteer);
  
  aUInt16 temp = servoDrive << 8 | servoDrive;
  m_pStem->PAD_IO(aSERVO_MODULE, AUTPAD_THROT, temp);
  temp = servoSteer << 8 | servoSteer;
  m_pStem->PAD_IO(aSERVO_MODULE, AUTPAD_STEER, temp);
  
  // Make sure the reading took
  if ((m_pStem->PAD_IO(aSERVO_MODULE, AUTPAD_THROT+1) != servoDrive) || 
      (m_pStem->PAD_IO(aSERVO_MODULE, AUTPAD_STEER+1) != servoSteer)){
    m_log->log(ERROR, "MotionModule: Throttle or steer were not set: %d, %d", servoDrive, servoSteer);
    e = aErrNotReady;
  }
  
  // store the current settings for next loop
  m_setpointLast[0] = servoDrive;
  m_setpointLast[1] = servoSteer;
  
#if aDEBUG_MOTMODULE_SWEEP    
  
  // Delay so we can see things happen
  m_pStem->sleep(aSLICE);
  
} // end for loop
#endif

return e;

}


short avcMotion::getLastThrottle(){
  return m_setpointLast[0];
}
short avcMotion::getLastSteer(){
  return m_setpointLast[1];
}


///////////////////////////////////////////////////////////////////////////
// This section is for isolating and debugging this module. 
// You will need to set up a stem object and work from that.
// Use the makefile to build this in isolation.
// > make motModule
#ifdef aDEBUG_MOTMODULE

////////////////////////////////////////
int doTests(acpStem *pStem, aSettingFileRef settings);

int doTests(acpStem *pStem, aSettingFileRef settings) {
  
  avcMotion motion;
  avcForceVector Uresult;
  aErr e = aErrNone;
  
  printf("---------------------------------------------------------\n");
  printf("Performing tests on motModule\n");
  
  ///////////////////////////////////////////////////
  
  printf("Failure to initialize test...\n");
  
  e = motion.updateControl(Uresult);
  
  if (e != aErrInitialization) {
    printf("failed\n");
    return 1;
  }
  
  ///////////////////////////////////////////////////
  // Initialize the stem object for the rest of the tests
  e = motion.init(pStem, settings);
  
  // Check the upper x range
  printf("Force X component to large test...\n");
  
  Uresult.x = 1.5;
  e = motion.updateControl(Uresult);
  
  if (e != aErrRange) {
    printf("failed\n");
    return 2;
  }
  
  ///////////////////////////////////////////////////
  // Check the Ux range
#if 1
  printf("Ux component within range test...\n");
  
  Uresult.x = -1.0;
  Uresult.y = 0.0;
  
  while (Uresult.x <= 1.0) {
    e = motion.updateControl(Uresult);
    pStem->sleep(1000);
    Uresult.x += 0.1;
  } // end while loop
  
  pStem->sleep(1000);
  
  ///////////////////////////////////////////////////
  // Check the Uy range
  printf("Uy component within range test...\n");
  
  Uresult.x = 0.0;
  Uresult.y = -1.0;
  
  while (Uresult.y <= 1.0) {
    e = motion.updateControl(Uresult);
    pStem->sleep(1000);
    Uresult.y += 0.1;
  } // end while loo
  
  pStem->sleep(1000);
  
#endif
  
  ///////////////////////////////////////////////////
  // Check with the IR rangers using the Stem interaction. 
  // Requires that the gGPMonitor VM is running
#if 0	
  
  Uresult.x = 0.0;
  Uresult.y = 0.0;
  
  // Bascially, set the motors to off. Just using the code we wrote.
  e = motion.updateControl(Uresult);
  
  // Read the IR Repulsive force
  int i = 0;
  short x = 0;
  short y = 0;
  
  // Sure, 50 times seems like enough tests...
  while (i < 50) {
    
    // Any reason to NOT add a 2 byte read from Scratchpad into the 
    // the C++ acpStem?
    x = (pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_REPULSIVE_UX)) << 8;
    x |= pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_REPULSIVE_UX + 1);
    
    y = (pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_REPULSIVE_UY)) << 8;
    y |= pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_REPULSIVE_UY + 1);
    
    Uresult.x = (double) x / 32768.0;
    Uresult.y = (double) y / 32768.0;
    
    e = motion.updateControl(Uresult);
    
    pStem->sleep(500);
    
    i++;
  }
#endif 
  
  // Everything is all good from here. let's go home.
  return 0;
  
}

////////////////////////////////////////

aErr driveDebug(acpStem *pStem,
		avcMotion *pMotion, 
		avcForceVector *pVectors, 
		const int nVectors);


////////////////////////////////////////
aErr driveDebug(acpStem *pStem,
		avcMotion *pMotion, 
		avcForceVector *pVectors, 
		const int nVectors)
{
  avcForceVector Ufinal; // always defaul to 0,0
  
  // Chomp through the vectors
  for (int i = 0; i < nVectors; i++) {
    
    // Grab the vectors that are passed in
    pMotion->updateControl(pVectors[i]);
    
    pStem->sleep(500);
  }
  
//  // Stop the motors
//  pMotion->updateControl(Ufinal);
//  
//  // Let the motors settle before getting out of here
//  pStem->sleep(2000);
  
  return aErrNone;
}


////////////////////////////////////////
int driveTests(acpStem *pStem, aSettingFileRef settings);

////////////////////////////////////////

int driveTests(acpStem *pStem, aSettingFileRef settings)
{
  
  avcMotion motion;
  avcForceVector Uresult[10];
  aErr e = aErrNone;
//  float range = 0.1f;
  
  printf("---------------------------------------------------------\n");
  printf("Performing actual motion tests on motModule\n");
  
  ///////////////////////////////////////////////////
  
  // initialize the motion class
  e = motion.init(pStem, settings);
  
  // Do a steering sweep
//  for (float rad = 0; rad < aPI*2; rad += 0.314159265f ) {
  for (int i = 0; i < 1; i++ ) {
      
//    Uresult[0].x = range * cos(rad);
//    Uresult[0].y = range * sin(rad);
    
    Uresult[0].x = 1.0;
    Uresult[0].y = 0.0;
    
    Uresult[1].x = -1.0;
    Uresult[1].y = 0.0;
    
    e = driveDebug(pStem, &motion, Uresult, 2);
    
  }
  
  avcForceVector Ufinal;
  motion.updateControl(Ufinal);
  

  
//  // Drive stop
//  Uresult[1].x = 0;
//  Uresult[1].y = 0;
//  
//  // Rotate at slight angle test 1
//  Uresult[2].x = range;
//  Uresult[2].y = range;
//  
//  // Rotate at slight angle test 2
//  Uresult[3].x = 0;
//  Uresult[3].y = 0;
//  
//  // Rotate at slight angle test 3
//  Uresult[4].x = range;
//  Uresult[4].y = -1*range;
//  
//  // Rotate at slight angle test 4
//  Uresult[5].x = 0;
//  Uresult[5].y = 0;
//  
//  // Rotate in place
//  Uresult[6].x = -1*range;
//  Uresult[6].y = 0;
//  
//  // Rotate in place
//  Uresult[7].x = 0;
//  Uresult[7].y = 0;
  
  
  
  return e;
  
}



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
    
  avcMotion motion; 

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
  
  motion.init(&stem, settings);  
  
  //////////////////////////////////
  // Begin the real actual testing
  // We are connected to the stem now, we can beat on the motion control 
  // module. Ya!!!
  //doTests(&stem, settings);
  
  // Drive forward for a bit
  //driveTests(&stem, settings);
  
  // Set the encoder to zero
  stem.MO_ENC32(4, 0, 0);
  aInt32 prevEncoder  = 0;
  
  // Encoder read tests
	bool bStart = false;
	double distance = 0.0;
	long unsigned int prevClock;
	aIO_GetMSTicks(ioRef, &prevClock, NULL);
  
	int trial = 15;
	while (trial > 0) {
    // Need to read the second byte, since the PAD_IO writes 2 bytes at a time		
		if(!bStart && (bStart = !(stem.PAD_IO(aSERVO_MODULE, RCPAD_ENABLE)))) {
			printf("Starting record\n");
			distance = 0.0;
			// set steering.
			stem.PAD_IO(aSERVO_MODULE, AUTPAD_STEER, trial * 17);
		}
    // Need to read the second byte, since the PAD_IO writes 2 bytes at a time			 
		if(bStart && !(bStart = !(stem.PAD_IO(aSERVO_MODULE, RCPAD_ENABLE)))) {
			printf("Ending record: steering setpoint %d, distance: %e\n", trial * 17, distance);
			--trial;
		}
			
		long unsigned int curClock;
		aIO_GetMSTicks(ioRef, &curClock, NULL);
		long tmElapsed = (curClock - prevClock);
		prevClock = curClock;
		
		aInt32 curEncoder = stem.MO_ENC32(4, 0);
		
    aInt32 encTicks = curEncoder - prevEncoder;
		distance += encTicks * METER_PER_TICK;
		prevEncoder = curEncoder;
		
    printf("enabled %s distance: %e Time Elapsed: %ld\n",
					 bStart? "true": "false", distance, tmElapsed);
    
    aIO_MSSleep(ioRef, 200, NULL);
		
		
  }
  stem.PAD_IO(aSERVO_MODULE, AUTPAD_STEER, SERVO_NEUT);
	
  aIO_MSSleep(ioRef, 1000, NULL);
  
  //////////////////////////////////
  // Clean up and get outta here. Run Forrest, RUN!!!
  aSettingFile_Destroy(ioRef, settings, NULL);
  aIO_ReleaseLibRef(ioRef, NULL);
  
  return 0;
  
}


#endif

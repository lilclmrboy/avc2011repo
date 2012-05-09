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
m_bInit(false)

{
  
  aErr e = aErrNone;
  
  // set up all the setpoint place holders
  for (int m = 0; m < aMOTOR_NUM; m++) {
    
    m_setpoint[m] = 0;
    m_setpointLast[m] = 0;
    
  }
  
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
  
  // Copy this into our member variable. Our setpoint is bounded by a 
  // 2 byte value. Short is enough. Likely even transition to an 
  // unsigned char since we are not likely to go past 200. 
  m_setpointMax = (short) setpoint;
  
  // Get access to the logger class
  m_log = logger::getInstance();
  m_log->log(INFO, "Motion Module initialized");
  
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
  
  if (cos(delta) < 0)
  	magnitude = magnitude * -1.0;
  
  if (delta < 0) {
    //printf("delta is negative\n  delta: %f deltad: %f", delta, aPI*2 + delta);
    delta += aPI*2 + delta;
  }
  	  
    // The magnitude value directly translates to the gas pedal for the 
    // rear drive motor.
    unsigned char servoDrive = (unsigned char)(SERVO_NEUT + (127 * magnitude));
    unsigned char servoSteer = SERVO_NEUT;

    // Update the servo values
  // The magnitude value directly translates to the gas pedal for the 
  // rear drive motor.
  unsigned char steerdelta = 0;
  
  // See how things should unfold
  if ((delta >= 0.0f) && (delta < MAX_TURNANGLE)) {
    steerdelta = (unsigned char)(delta/MAX_TURNANGLE * SERVO_NEUT);
    servoSteer = SERVO_NEUT + steerdelta;
  }
  else if ((delta >= MAX_TURNANGLE) && (delta < (aPI - MAX_TURNANGLE))) {
    steerdelta = SERVO_MIN;
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
  else if ((delta >= (aPI + MAX_TURNANGLE)) && (delta < (aPI*2 - MAX_TURNANGLE))) {
    steerdelta = SERVO_MIN;
    servoSteer = SERVO_MIN;
  }	
  else if ((delta >= (aPI*2 - MAX_TURNANGLE)) && (delta < aPI*2)) {
    steerdelta = (unsigned char)((2*aPI - delta)/(MAX_TURNANGLE) * SERVO_NEUT);
    servoSteer = SERVO_NEUT - steerdelta;
  }
      
  #ifdef aDEBUG_MOTMODULEZ	
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
        
    
    m_pStem->PAD_IO(aSERVO_MODULE, AUTPAD_THROT, (aUInt16) servoDrive);
    m_pStem->PAD_IO(aSERVO_MODULE, AUTPAD_STEER, (aUInt16) servoSteer);
    
    // Make sure the reading took
    if ((m_pStem->PAD_IO(aSERVO_MODULE, AUTPAD_THROT + 1) != servoDrive) || 
      (m_pStem->PAD_IO(aSERVO_MODULE, AUTPAD_STEER + 1) != servoSteer))
      e = aErrNotReady;
    
#if aDEBUG_MOTMODULE_SWEEP    
    
    // Delay so we can see things happen
    m_pStem->sleep(aSLICE);
    
  } // end for loop
#endif

  return e;
  
}

///////////////////////////////////////////////////////////////////////////
// Update servo positioning information
//bool
//avcMotion::updateServoValues(const double magnitude, 
//			     const double delta, 
//			     aUInt16 *pServoDrive, 
//			     aUInt16 *pServoSteer)
//{
//
//  bool bFinished = true;
//  unsigned char servoSteer = SERVO_NEUT;
//  unsigned char steerdelta = 0;
//  
//  // The magnitude value directly translates to the gas pedal for the 
//  // rear drive motor.
//  unsigned char servoDrive = (unsigned char)(SERVO_NEUT + (127 * magnitude));
//  
//  // See how things should unfold
//  if ((delta >= 0.0f) && (delta < MAX_TURNANGLE)) {
//    steerdelta = (unsigned char)(delta/MAX_TURNANGLE * SERVO_NEUT);
//    servoSteer = SERVO_NEUT + steerdelta;
//  }
//  else if ((delta >= MAX_TURNANGLE) && (delta < (aPI - MAX_TURNANGLE))) {
//    steerdelta = SERVO_MIN;
//    servoSteer = SERVO_MAX;
//  }
//  else if ((delta >= (aPI - MAX_TURNANGLE)) && (delta < aPI)) {
//    steerdelta = (unsigned char)((aPI - delta)/(MAX_TURNANGLE) * SERVO_NEUT);
//    servoSteer = SERVO_NEUT + steerdelta;
//  }
//  else if ((delta >= aPI) && (delta < (aPI + MAX_TURNANGLE))) {
//    steerdelta = (unsigned char)((delta - aPI)/(MAX_TURNANGLE) * SERVO_NEUT);
//    servoSteer = SERVO_NEUT - steerdelta;
//  }
//  else if ((delta >= (aPI + MAX_TURNANGLE)) && (delta < (aPI*2 - MAX_TURNANGLE))) {
//    steerdelta = SERVO_MIN;
//    servoSteer = SERVO_MIN;
//  }	
//  else if ((delta >= (aPI*2 - MAX_TURNANGLE)) && (delta < aPI*2)) {
//    steerdelta = (unsigned char)((2*aPI - delta)/(MAX_TURNANGLE) * SERVO_NEUT);
//    servoSteer = SERVO_NEUT - steerdelta;
//  }
//  
//  // Copy the values into our results
//  *pServoDrive = (aUInt16) servoDrive;
//  *pServoSteer = (aUInt16) servoSteer;
//  
//  return bFinished;
//  
//}


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
  float range = 0.1f;
  
  printf("---------------------------------------------------------\n");
  printf("Performing actual motion tests on motModule\n");
  
  ///////////////////////////////////////////////////
  
  // initialize the motion class
  e = motion.init(pStem, settings);
  
  // Do a steering sweep
  for (float rad = 0; rad < aPI*2; rad += 0.314159265f ) {
    
    printf("rad: %f\n", rad);
    
    Uresult[0].x = range * cos(rad);
    Uresult[0].y = range * sin(rad);
    
    //e = driveDebug(pStem, &motion, Uresult, 1);
    
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
  driveTests(&stem, settings);
  
  aIO_MSSleep(ioRef, 1000, NULL);
  
  //////////////////////////////////
  // Clean up and get outta here. Run Forrest, RUN!!!
  aSettingFile_Destroy(ioRef, settings, NULL);
  aIO_ReleaseLibRef(ioRef, NULL);
  
  return 0;
  
}

#endif

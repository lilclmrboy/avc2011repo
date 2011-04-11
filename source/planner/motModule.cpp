///////////////////////////////////////////////////////////////////////////
// Filename: motModule.cpp

#include "motModule.h"

#ifdef aDEBUG_MOTMODULE
bool bDebugHeader = true;
#endif

// Not sure why this doesn't work. But, it doesn't.
//#define sgn(x) (x > 0.0001) ? 1.0 : ((x < 0.0001) ? -1.0 : 0.0)

///////////////////////////////////////////////////////////////////////////
// Sigmoid function
// Added a tolerance around 0 since it seems that passing the result of 
// atan2 was giving us some trouble. Is this a hack? Am I clueless?
// Probably. 
double sgn(double x);

double sgn(double x) 
{

	if (x > 0.0001)
		return 1.0;
	
	if (x < 0.0001)
		return -1.0;
	
	return 0.0;
	
}

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
	double t = 0.0;
	double r = 0.0;
	double v[aMOTOR_NUM] = {0.0, 0.0};
	double magnitude = 0.0;
	double delta = 0.0;
	bool bRCOverride = false;
	
	// Make sure you other hackers initialized this first
	if (!m_pStem) {
		m_log->log(ERROR,"avcMotion::updateControl failed. acpStem is NULL\n"
								 " \tCall avcMotion::init(acpStem *)\n");
		return aErrInitialization;
	}
	
	if (!m_bInit) {
		m_log->log(ERROR,"avcMotion::updateControl failed since uninitialized.\n"
								 " \tCall avcMotion::init(acpStem *)\n");
		return aErrInitialization;
	}

	// Boundary check the input force vectors
	if (potential.x > 1.0 || potential.x < -1.0) {
		m_log->log(ERROR,"avcMotion::updateControl failed.\n"
					 "\tavcForceVector.x is out of range\n");
		return aErrRange;
	}
	if (potential.y > 1.0 || potential.y < -1.0) {
		m_log->log(ERROR,"avcMotion::updateControl failed.\n"
					 "\tavcForceVector.y is out of range\n");
		return aErrRange;
	}
	
	// Check to see if we are manually over riding the motion channel
	// with the RC receiver.
	bRCOverride = m_pStem->PAD_IO(aMOTO_MODULE, aSPAD_MO_MOTION_RCENABLE);
	
	// If we need to over ride, we should do it.
	if (bRCOverride) {
		
		m_log->log(NOTICE, "Motion module disabled. RC Override\n");
		
		// Set the desired setpoint to zero
		for (int m = 0; m < aMOTOR_NUM; m++) {
			m_setpoint[m] = 0;
			
			// Check to see if the last setpoint is the same as the new one.
			// If everything is the same, then we won't bother the Stem.
			if (m_setpoint[m] != m_setpointLast[m]) {
				
				// Write the values to the scratchpad
				m_pStem->PAD_IO(aMOTO_MODULE, 
												(m == aMOTOR_LEFT) ? aSPAD_MO_MOTION_SETPOINT_LEFT : aSPAD_MO_MOTION_SETPOINT_RIGHT, 
												m_setpoint[m]);
				
				m_setpointLast[m] = m_setpoint[m];
				
			}
			
		} // end of for loop for RC over ride
		
		// The motion module on the Stem is busy. Leave us alone. Leave 
		// Brittney alone!
		return aErrBusy;
	}

	// calculate the magnitude of the resultant input vector
	magnitude = sqrt(potential.x * potential.x + potential.y * potential.y);
	delta = atan2(potential.y, potential.x);
	
	// Straight from the research paper. With some minor adjustments
	t = (cos(delta) * cos(delta)) * sgn(cos(delta));
	r = (sin(delta) * sin(delta)) * sgn(sin(delta));
	
	// Use the magnitude to scale the velocity
	v[aMOTOR_LEFT] = magnitude * (t - r);
	v[aMOTOR_RIGHT] = magnitude * (t + r);
	
	// Calculate the setpoints for the Moto to go chase
	// Boundary check the setpoints
	for (int m = 0; m < aMOTOR_NUM; m++) {
		
		// calc setpoint bounded by setpoint
		m_setpoint[m] = v[m] * m_setpointMax;
		
		// Let's only send updates if the new velocity is different then the last
		// one we sent. No point in doing EXTRA work, right?
		if (m_setpoint[m] != m_setpointLast[m]) {
			
			// Write the values to the scratchpad
			m_pStem->PAD_IO(aMOTO_MODULE, 
											(m == aMOTOR_LEFT) ? aSPAD_MO_MOTION_SETPOINT_LEFT : aSPAD_MO_MOTION_SETPOINT_RIGHT, 
											m_setpoint[m]);
			
			
			m_log->log(DEBUG,"avcMotion::updateControl channel: %d setpoint: %d",
								 m,
								 m_setpoint[m]);
			
		}
		
		// Store the last setpoint reading for next time with hopes of 
		// reducing the number of times we send commands to the lower 
		// control system.
		m_setpointLast[m] = m_setpoint[m];
		
		
	} // end of for loop
	
#ifdef aDEBUG_MOTMODULE	
	// Show us what we got
	if (bDebugHeader) {
		m_log->log(DEBUG,"Ux\tUy\t"
					 "t\tr\tmag\t"
					 "delta\t"
					 "vL\tvR\tsetL\tsetR\n");
		bDebugHeader = false;
	}
	
	m_log->log(DEBUG,"MotionModule: %2.2f\t%2.2f\t"
				 "%2.2f\t%2.2f\t%2.2f\t"
				 "%2.2f\t"
				 "%2.2f\t%2.2f\t"
				 "%d\t%d\n",
				 potential.x, potential.y, 
				 t, r, magnitude,
				 delta,
				 v[aMOTOR_LEFT], v[aMOTOR_RIGHT],
				 m_setpoint[aMOTOR_LEFT], m_setpoint[aMOTOR_RIGHT]);
	
#endif	
	
	return aErrNone;
	
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
	
	// Turn off the motors.
	pStem->PAD_IO(aMOTO_MODULE, 
								aSPAD_MO_MOTION_SETPOINT_RIGHT, 
								0);
	
	pStem->PAD_IO(aMOTO_MODULE, 
								aSPAD_MO_MOTION_SETPOINT_LEFT, 
								0);
	
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
	} // end while loop
	
	// Turn off the motors.
	pStem->PAD_IO(aMOTO_MODULE, 
								aSPAD_MO_MOTION_SETPOINT_RIGHT, 
								0);
	
	pStem->PAD_IO(aMOTO_MODULE, 
								aSPAD_MO_MOTION_SETPOINT_LEFT, 
								0);
	
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
		
		pStem->sleep(2000);
	}
	
	// Stop the motors
	pMotion->updateControl(Ufinal);
	
	// Let the motors settle before getting out of here
	pStem->sleep(2000);
	
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
	float range = 0.4f;
	
	printf("---------------------------------------------------------\n");
	printf("Performing actual motion tests on motModule\n");
	
	///////////////////////////////////////////////////
	
	// initialize the motion class
	e = motion.init(pStem, settings);
	
	// Now do a test or 2, or 3
	Uresult[0].x = range;
	Uresult[0].y = 0;
	
	// Drive backward test
	Uresult[1].x = -1*range;
	Uresult[1].y = 0;

	// Rotate at slight angle test 1
	Uresult[2].x = range;
	Uresult[2].y = range;
	
	// Rotate at slight angle test 2
	Uresult[3].x = -1*range;
	Uresult[3].y = -1*range;
	
	// Rotate at slight angle test 3
	Uresult[4].x = -1*range;
	Uresult[4].y = range;
	
	// Rotate at slight angle test 4
	Uresult[5].x = range;
	Uresult[5].y = -1*range;
	
	// Rotate in place
	Uresult[6].x = 0;
	Uresult[6].y = -1*range;
	
	// Rotate in place
	Uresult[7].x = 0;
	Uresult[7].y = range;
	
	e = driveDebug(pStem, &motion, Uresult, 8);
	
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
													128,
													"console.config",
													&settings,
													&e))
		throw acpException(e, "creating settings");
	
	// or, maybe command line arguements
	aArguments_Separate(ioRef, settings, NULL, argc, argv);
	
	printf("kitty\n");
	
	avcMotion motion; 
	
	motion.init(NULL, settings);
	
	return 0;
	
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
	
	//////////////////////////////////
	// Begin the real actual testing
	// We are connected to the stem now, we can beat on the motion control 
	// module. Ya!!!
	doTests(&stem, settings);
	
	// Drive forward for a bit
	//driveTests(&stem, settings);
	
	aIO_MSSleep(ioRef, 1000, NULL);
	
	//////////////////////////////////
	// Clean up and get outta here. Run Forrest, RUN!!!
	aSettingFile_Destroy(ioRef, settings, NULL);
	aIO_ReleaseLibRef(ioRef, NULL);
	
	return 0;
	
}

#endif

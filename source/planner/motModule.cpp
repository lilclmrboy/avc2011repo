///////////////////////////////////////////////////////////////////////////
// Filename: motModule.cpp

#include "motModule.h"
#include <math.h>

#ifdef aDEBUG_MOTMODULE

bool bDebugHeader = true;

#define aDEBUG_PRINT(arg) printf(arg);fflush(stdout)
#else 
#define aDEBUG_PRINT(arg)
#endif

#define sgn(x) (x > 0.0) ? 1.0 : ((x < 0.0) ? -1.0 : 0.0)

///////////////////////////////////////////////////////////////////////////
double sigmoid(double x);

double sigmoid (double x) {
	
	if (x > 0.0001)
		return 1.0;
		
	if (x < 0.0001)
		return -1.0;
	
	return 0;

}

///////////////////////////////////////////////////////////////////////////
avcMotion::avcMotion() :
  m_pStem(NULL),
  m_setpointMax(aMOTOR_SETPOINT_MAX)
{
	
	// set up all the setpoint holders
	for (int m = 0; m < 2; m++) {
		
		m_setpoint[m] = 0;
		m_setpointLast[m] = 0;
		
	}
	
}

///////////////////////////////////////////////////////////////////////////
// Get the Stem object and initialize any thing else that we need to.

aErr
avcMotion::init(acpStem *pStem) {
	
	m_pStem = pStem;
	
	return aErrNone;
	
}

///////////////////////////////////////////////////////////////////////////
// Given a new goal vector, update the each motor setpoint

aErr
avcMotion::updateControl(const avcForceVector& potential) 
{
	double t = 0.0;
	double r = 0.0;
	double v[aMOTOR_NUM] = {0.0, 0.0};
	double magnitude = 0.0;
	double delta = 0.0;
	
	// Make sure you other hackers initialized this first
	if (!m_pStem) {
		aDEBUG_PRINT("avcMotion::updateControl failed. acpStem is NULL\n"
								 " \tCall avcMotion::init(acpStem *)\n");
		return aErrInitialization;
	}

	// Boundary check the input force vectors
	if (potential.x > 1.0 || potential.x < -1.0) {
		aDEBUG_PRINT("avcMotion::updateControl failed.\n"
					 "\tavcForceVector.x is out of range\n");
		return aErrRange;
	}
	if (potential.y > 1.0 || potential.y < -1.0) {
		aDEBUG_PRINT("avcMotion::updateControl failed.\n"
					 "\tavcForceVector.y is out of range\n");
		return aErrRange;
	}

	// calculate the magnitude of the resultant input vector
	magnitude = sqrt(potential.x * potential.x + potential.y * potential.y);
	delta = atan2(potential.y, potential.x);
	
	// Straight from the research paper
	t = (cos(delta) * cos(delta)) * sigmoid(cos(delta));
	r = (sin(delta) * sin(delta)) * sigmoid(sin(delta));
	
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
			
		}
		
		// Store the last setpoint reading for next time
		m_setpointLast[m] = m_setpoint[m];
		
		
	} // end of for loop
	
#ifdef aDEBUG_MOTMODULE	
	// Show us what we got
	if (bDebugHeader) {
		printf("DEBUG: Ux\tUy\t"
					 "t\tr\tmag\t"
					 "delta\t"
					 "vL\tvR\tsetL\tsetR\n");
		bDebugHeader = false;
	}
	
	printf("DEBUG: %2.2f\t%2.2f\t"
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
// Use the makefile_motModule to build this in isolation.
#ifdef aDEBUG_MOTMODULE

#define aTESTWITHSTEM 1

////////////////////////////////////////
int doTests(acpStem *pStem);

int doTests(acpStem *pStem) {
	
	avcMotion motion;
	avcForceVector Uresult = {0, 0};
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
	e = motion.init(pStem);
	
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
#if 0	
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
	// Check with the IR rangers. 
	
	Uresult.x = 0.0;
	Uresult.y = 0.0;
	e = motion.updateControl(Uresult);
	
	// Read the IR Repulsive force
	int i = 0;
	short x = 0;
	short y = 0;
	while (i < 50) {
		
		x = (pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_REPULSIVE_UX)) << 8;
		x |= pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_REPULSIVE_UX + 1);
		
		y = (pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_REPULSIVE_UY)) << 8;
		y |= pStem->PAD_IO(aGP2_MODULE, aSPAD_GP2_REPULSIVE_UY + 1);
		
		printf("x = %d \t y = %d\n", x, y);
		
		pStem->sleep(250);
		
		i++;
	}
	
	// Everything is all good from here. let's go home.
	return 0;
	
}

////////////////////////////////////////
// main testing routine
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
		aIO_MSSleep(ioRef, 500, NULL);
		++timeout;
	} while (!stem.isConnected() && timeout < 10);
	
	printf("\n");

#if aTESTWITHSTEM	
	// Bail if no stem. What's the point little man?
	if (timeout == 10) { return 1; }
#endif	
	
	//////////////////////////////////
	// Begin the real actual testing
	// We are connected to the stem now, we can beat on the motion control 
	// module. Ya!!!
	doTests(&stem);

	aIO_MSSleep(ioRef, 1000, NULL);
	
	//////////////////////////////////
	// Clean up and get outta here
	aSettingFile_Destroy(ioRef, settings, NULL);
	aIO_ReleaseLibRef(ioRef, NULL);
	
	return 0;
	
}

#endif

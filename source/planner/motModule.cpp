///////////////////////////////////////////////////////////////////////////
// Filename: motModule.cpp

#include "motModule.h"
#include "avc2011Defs.tea"
#include <math.h>

#ifdef aDEBUG_MOTMODULE
#ifdef aDEBUG_H
#define aDEBUG_PRINT(arg) printf(arg);fflush(stdout)
#else 
#define aDEBUG_PRINT(arg)
#endif
#endif

#define sgn(x) (x > 0.0) ? 1.0 : ((x < 0.0) ? -1.0 : 0.0)

///////////////////////////////////////////////////////////////////////////
avcMotion::avcMotion() :
  m_pStem(NULL)
{
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
	double v[2] = {0.0, 0.0};
	
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

	// Show us what we got
	printf("\n  DEBUG: <x,y,rad>: <%2.2f, %2.2f>\n",
		potential.x, potential.y);
	
	// calculate the magnitude of the resultant input vector
	double magnitude = sqrt(potential.x * potential.x + potential.y * potential.y);
	double delta = atan2(potential.x, potential.y);
	
	// Straight from the research paper
	t = cos(delta) * cos(delta) * sgn(cos(delta));
	r = sin(delta) * sin(delta) * sgn(sin(delta));
	
	printf("\n  DEBUG: <t,r,magnitude>: <%2.2f, %2.2f, %2.2f>\n",
				 t, r, magnitude);
	
	// Use the magnitude to scale the velocity
	v[aMOTOR_LEFT] = magnitude * (t -r);
	v[aMOTOR_RIGHT] = magnitude * (t + r);
	
	// Apply the maximum setpoint threshold to the computed normalized velocities
	// Write this to the Stem's scratchpads
	
	printf("v[%d] = %2.2f \tv[%d] = %2.2f\n", 
				 aMOTOR_LEFT, v[aMOTOR_LEFT], 
				 aMOTOR_RIGHT, v[aMOTOR_RIGHT]);
	
	return aErrNone;
	
}

///////////////////////////////////////////////////////////////////////////
// This section is for isolating and debugging this module. 
// You will need to set up a stem object and work from that.
// Use the makefile_motModule to build this in isolation.
#ifdef aDEBUG_MOTMODULE

#define aTESTWITHSTEM 0

////////////////////////////////////////
int doTests(acpStem *pStem);

int doTests(acpStem *pStem) {
	
	avcMotion motion;
	avcForceVector Uresult = {0, 0};
	aErr e = aErrNone;
	
	printf("---------------------------------------------------------\n");
	printf("Performing tests on motModule\n");
	
	///////////////////////////////////////////////////
	
	printf("Failure to initialize test...");
	
	e = motion.updateControl(Uresult);
	
	if (e != aErrInitialization) {
		printf("failed\n");
		return 1;
	}
	printf("passed\n");
	
	///////////////////////////////////////////////////
	// Initialize the stem object for the rest of the tests
	e = motion.init(pStem);
	
	// Check the upper x range
	printf("Force X component to large test...");
	
	Uresult.x = 1.5;
	e = motion.updateControl(Uresult);
	
	if (e != aErrRange) {
		printf("failed\n");
		return 2;
	}
	printf("passed\n");
	
	// Check the upper x range
	printf("Force X component within range test...");
	
	Uresult.x = 0.0;
	Uresult.y = 0.5;
	e = motion.updateControl(Uresult);
	
	printf("passed\n");
	
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
													"motmodule.config",
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
	} while (!stem.isConnected() && timeout < 3);
	
	printf("\n");

#if aTESTWITHSTEM	
	// Bail if no stem. What's the point little man?
	if (timeout == 10) { return 1; }
#endif	
	
	//////////////////////////////////
	// Begin the real actual testing
	// We are connected to the stem now, we can beat on the motion control 
	// module. Ya!!!
	
	int result = doTests(&stem);
	
	if (result) {
		printf("Failed on the %d test.\n", result);
	} else {
		printf("All tests passed\n");
	}

	
	//////////////////////////////////
	
	
	// Clean up and get outta here
	aSettingFile_Destroy(ioRef, settings, NULL);
	aIO_ReleaseLibRef(ioRef, NULL);
	
	return 0;
	
}

#endif

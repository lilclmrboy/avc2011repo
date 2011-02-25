///////////////////////////////////////////////////////////////////////////
// Filename: motModule.cpp

#include "motModule.h"

#ifdef aDEBUG_MOTMODULE
#ifdef aDEBUG_H
#define aDEBUG_PRINT(arg) printf(arg);fflush(stdout)
#else 
#define aDEBUG_PRINT(arg)
#endif
#endif

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
avcMotion::updateControl(const avcForceVector& potential,
												 const double radian) 
{
	aErr e = aErrNone;
	
#ifdef aDEBUG_H	
	// Make sure you other hackers initialized this first
	if (!m_pStem) {
		printf("avcMotion::updateControl failed. acpStem is NULL\n"
								 "  Call avcMotion::init(acpStem *)\n");
		return aErrInitialization;
	}
	
	// Boundary check the input force vectors
	if (potential.x > 1.0 || potential.x < -1.0) {
		printf("avcMotion::updateControl failed. "
					 "avcForceVector.x is out of range\n");
		return aErrRange;
	}
	if (potential.y > 1.0 || potential.y < -1.0) {
		printf("avcMotion::updateControl failed. "
					 "avcForceVector.y is out of range\n");
		return aErrRange;
	}

	// Show us what we got
	printf("  DEBUG: <x,y,rad>: <%2.2f, %2.2f, %2.2f>\n",
		potential.x, potential.y, radian);
#endif
	
	return e;
	
}

///////////////////////////////////////////////////////////////////////////
// This section is for isolating and debugging this module. 
// You will need to set up a stem object and work from that.
// Use the makefile_motModule to build this in isolation.
#ifdef aDEBUG_MOTMODULE

////////////////////////////////////////
int doTests(acpStem *pStem);

int doTests(acpStem *pStem) {
	
	avcMotion motion;
	avcForceVector Uresult = {0, 0};
	aErr e = aErrNone;
	
	///////////////////////////////////////////////////
	printf("No initialization test...\n");
	e = motion.updateControl(Uresult, 0);
	if (e != aErrInitialization) {
		printf("FAIL: No initialization test\n");
		return 1;
	}
	
	///////////////////////////////////////////////////
	// Initialize the stem object
	e = motion.init(pStem);
	
	// Check the upper x range
	Uresult.x = 1.5;
	if (!e)
		e = motion.updateControl(Uresult, 0);
	
	
	
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
	} while (!stem.isConnected() && timeout < 10);
	
	printf("\n");
	
	// Bail if no stem. What's the point little man?
	if (timeout == 10) { return 1; }
	
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

///////////////////////////////////////////////////////////////////////////
// Filename: motModule.cpp

#include "motModule.h"

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
												 const double radian) {

	printf("Received Force Vector (x,y): %2.2f, %2.2f\n",
		potential.x, potential.y);
	
	return aErrNone;
	
}

///////////////////////////////////////////////////////////////////////////
// This section is for isolating and debugging this module. 
// You will need to set up a stem object and work from that.
// Use the makefile_motModule to build this in isolation.
#ifdef aDEBUG_MOTMODULE

int main()
{
	
	printf("Hello motion module\n");
	
	return 0;
	
}

#endif

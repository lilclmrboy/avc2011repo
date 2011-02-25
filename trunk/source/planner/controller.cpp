#include "controller.h"
#define aSTEM_CONN_TIMEOUT 10

avcController::avcController(void) {
	aErr e;

	if(aIO_GetLibRef(&m_ioRef, &e)) 
      		throw acpException(e, "Getting aIOLib reference");
}

/////////////////////////////////////////////////////////////////////////////

avcController::~avcController(void) {
	aErr e;
	if (aSettingFile_Destroy(m_ioRef, m_settings, &e))
      		throw acpException(e, "unable to destroy settings");
}

/////////////////////////////////////////////////////////////////////////////

bool
avcController::init(const int argc, const char* argv[]) {
	// Create a setting file to hold our link settings so we can change 
	// the settings without recompiling.
	aErr e;
    	if (aSettingFile_Create(m_ioRef, 
			        128,
			        "planner.config",
			        &m_settings,
			        &e))
      		throw acpException(e, "creating settings");
    
    	aArguments_Separate(m_ioRef, m_settings, NULL, argc, argv);
	
	
	// Don't forget to init the modules :)
	m_mot.init(&m_stem);
	m_pos.init(&m_stem);

}

/////////////////////////////////////////////////////////////////////////////

int
avcController::run(void) {
	// This starts up the stem link processing and is called once to spawn
    	// the link management thread... based on the settings.
    	m_stem.startLinkThread(m_settings);

    	// Wait until we have a solid heartbeat connection so we know there is 
    	// someone to talk to.
    	int timeout = 0;

    	aDEBUG_PRINT("awaiting heartbeat");

    	do { 
     		aDEBUG_PRINT(".");
      		aIO_MSSleep(m_ioRef, 500, NULL);
      		++timeout;
    	} while (!m_stem.isConnected() && timeout < aSTEM_CONN_TIMEOUT);
    	
	//If we timed out we should report an error condition.
	//The error was a timeout, so not so critical return 1.
	if (timeout == aSTEM_CONN_TIMEOUT) {
		aDEBUG_PRINT("\n");
		return 1;
	}

	//We have a valid stem connection.
	aDEBUG_PRINT("done\n");
	while (1 && timeout < aSTEM_CONN_TIMEOUT) {
   		aErr e = aErrNone;
		//First do the localization step. Lets get relevant GPS
   		//info from the unit, and compass heading. Along with 
   		//the previous state and control vector. 
			        


		// motion planning step	
      		avcForceVector ir;
		e = m_mot.updateControl(ir, 0);
      		
		avcStateVector pos;
		pos = m_pos.getPosition(ir);

	   	 // sleep a bit so we don't wail on the processor
	    	aIO_MSSleep(m_ioRef, 2000, NULL);
    	} // while
}


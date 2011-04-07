#include "controller.h"
#define aSTEM_CONN_TIMEOUT 10

avcController::avcController(void) : 
	m_settings(NULL),
	m_ioRef(NULL) 
{
	aErr e;

	if(aIO_GetLibRef(&m_ioRef, &e)) 
      		throw acpException(e, "Getting aIOLib reference");
}

/////////////////////////////////////////////////////////////////////////////

avcController::~avcController(void) {
	aErr e;
	if (m_settings && aSettingFile_Destroy(m_ioRef, m_settings, &e))
      		throw acpException(e, "unable to destroy settings");
	if (m_ioRef && aIO_ReleaseLibRef(m_ioRef, &e))
      		throw acpException(e, "unable to destroy ioLib");
}

/////////////////////////////////////////////////////////////////////////////

int
avcController::init(const int argc, const char* argv[]) {
	// Create a setting file to hold our link settings so we can change 
	// the settings without recompiling.
	aErr e = aErrNone;
	if (aSettingFile_Create(m_ioRef, 
			        						128,
			        						"planner.config",
			        						&m_settings,
			        						&e))
  	throw acpException(e, "creating settings");
    
  aArguments_Separate(m_ioRef, m_settings, NULL, argc, argv);
	
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
		e = aErrTimeout;
	} else {
		//We have a valid stem connection.
		aDEBUG_PRINT("done\n");
	
		// Don't forget to init the modules :)
		e = m_mot.init(&m_stem, m_settings);
		e = m_pos.init(&m_stem, m_settings);
		e = m_planner.init(m_ioRef, m_settings);
	}
	return e;
}

/////////////////////////////////////////////////////////////////////////////

int
avcController::run(void) {
	
	//Lets allow a condition to gracefully end the application. 
	bool running = true;

	while (running) {
		aErr e = aErrNone;
	
		//First do the localization step. Lets get relevant GPS
		//info from the unit, and compass heading. Along with 
		//the previous state and control vector.
		m_pos.updateState();			        
		
		// get sensor readings
		avcForceVector ir;
		
		// motion planning step
		avcForceVector motivation = m_planner.getMotivation(m_pos.getPosition(), ir);
		
		e = m_mot.updateControl(motivation);

		//avcStateVector pos;
		//pos = m_pos.getPosition(ir);

	   	 // sleep a bit so we don't wail on the processor
	    	//aIO_MSSleep(m_ioRef, 2000, NULL);
    	} // end while
	
	return 0;
}

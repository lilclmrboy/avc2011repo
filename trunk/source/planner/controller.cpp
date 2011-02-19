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
    	printf("awaiting heartbeat");
    	do {
      		printf(".");
      		fflush(stdout);
      		aIO_MSSleep(m_ioRef, 500, NULL);
      		++timeout;
    	} while (!m_stem.isConnected() && timeout < aSTEM_CONN_TIMEOUT);
    	
	//If we timed out we should report an error condition.
	//The error was a timeout, so not so critical return 1.
	if (timeout = aSTEM_CONN_TIMEOUT)
		return 1;
	
	

	printf("done\n");
    	// Now, just loop getting values from the scratchpad.
    	printf("Starting Pad Read\n");
    	while (1 && timeout < aSTEM_CONN_TIMEOUT) {
    
      		aShort x, y = 0;
      
      		x = m_stem.PAD_IO(aGP2_MODULE, aSPAD_REPULSIVE_UX);
      		x = x << 8;
      		x += m_stem.PAD_IO(aGP2_MODULE, aSPAD_REPULSIVE_UX+1);
      
      		y = m_stem.PAD_IO(aGP2_MODULE, aSPAD_REPULSIVE_UY);
      		y = y << 8;
      		y += m_stem.PAD_IO(aGP2_MODULE, aSPAD_REPULSIVE_UY+1);
      
      		printf("X,Y IR repulsive components (%d, %d)\n", x, y);
       
	   	 // sleep a bit so we don't wail on the processor
	    	aIO_MSSleep(m_ioRef, 100, NULL);
    	} // while
}


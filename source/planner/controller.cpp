#include "controller.h"
#define aSTEM_CONN_TIMEOUT 10

/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// We may want to move this out somewhere or into it's own seperate class
// so other modules can use it

#define SOUND_DIRECTORY "../../sounds/"

int PlaySound(const char * file);

/////////////////////////////////////////////////////////////////////////////
// We may want to move this out somewhere or into it's own seperate class
// so other modules can use it
int PlaySound(const char * file)
{
	
	acpString command;
	int retVal = 0;
	
	// Build up the commands to play a sound file
	// Of course, mac doesn't have the built in ones that Linux does.
	// Or vise-versa.
#ifdef aMACX
	command = "afplay ";
#else
	command = "play ";
#endif 
	
	command += SOUND_DIRECTORY;
	command += file;
	command += " &";
	
	// Tell system to do the built up command
	retVal = system((const char *) command);
	
	return retVal;
	
}


avcController::avcController(void) : 
m_settings(NULL),
m_ioRef(NULL),
m_loopdelay(aCONTROLLER_LOOP_DELAY_DEFAULT)
{
	aErr e;
	
	if(aIO_GetLibRef(&m_ioRef, &e)) 
		throw acpException(e, "Getting aIOLib reference");
  
  m_log = logger::getInstance();
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
// Main initialization for the whole shooting match

int
avcController::init(const int argc, const char* argv[]) {
  
	// Create a setting file to hold our link settings so we can change 
	// the settings without recompiling.
	aErr e = aErrNone;
	
	PlaySound("okay.wav");
  
	if (aSettingFile_Create(m_ioRef,
                          "chicken.config",
                          &m_settings,
                          &e))
		throw acpException(e, "creating settings");
	
	aArguments_Separate(m_ioRef, m_settings, NULL, argc, argv);

	// This starts up the stem link processing and is called once to spawn
	// the link management thread... based on the settings.
	m_stem.startLinkThread(m_settings);
  
  // Get the loop cycle delay from the settings
  aSettingFile_GetULong(m_ioRef, m_settings, 
                        aCONTROLLER_LOOP_DELAY_KEY, 
                        &m_loopdelay, 
                        aCONTROLLER_LOOP_DELAY_DEFAULT, &e);
	
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
    // Do the motion module
    if (aErrNone != m_mot.init(&m_stem, m_settings)) 
      m_log->log(ERROR, "%s: Failed init m_mot", __PRETTY_FUNCTION__);
    
    // Do the repulsive force class
    if (aErrNone != m_repulse.init(&m_stem, m_settings))
      m_log->log(ERROR, "%s: Failed init m_repulse", __PRETTY_FUNCTION__);
    
    // Do the planner
    if (aErrNone != m_planner.init(m_ioRef, m_settings))
      m_log->log(ERROR, "%s: Failed init m_planner", __PRETTY_FUNCTION__);
    
    // Do the position module now, but pass in the first map point from 
    // the planner
    if (aErrNone != m_pos.init(&m_stem, m_settings, m_planner.getFirstMapPoint()))    
      m_log->log(ERROR, "%s: Failed init m_pos", __PRETTY_FUNCTION__);
    
	} // end of else
  
	return e;
}

/////////////////////////////////////////////////////////////////////////////

int
avcController::run(void) {
	
	//Lets allow a condition to gracefully end the application. 
	//bool running = true;
	avcStateVector pos;
	avcForceVector rv;
  avcRepulsiveForces frepulsive; 
  
	//////////////////////////////////////////////
	m_log->log(INFO, "Checking for the go signal\n");
	
	while (m_stem.isConnected(STEMCONNECTED_WAIT)) {
		
		// Read the scratchpad for the RC enable bit
		int rcswitch = m_stem.PAD_IO(aSERVO_MODULE, DIG_RCENABLE + 1);
	  
		// When we get set automatic mode, we expect a value of 0
		if (rcswitch) {
			printf("."); fflush(stdout);
		  
		  int temp = (int) random() % 5;
		  
		  switch (temp) {               
		    case 1:
		      PlaySound("cluck1.wav");
		      break;
		    case 2:
          PlaySound("cluck7.wav");
		      break;
		    case 3:     
		      PlaySound("cluck4.wav");
		      break;       
		    case 4:     
		      PlaySound("cluck8.wav");
		      break;  
        case 5:     
		      PlaySound("cluck9.wav");
		      break;  
		      
		    default:
		      PlaySound("clucking.wav");    
		      break;                                    
		      
		  }
      
			//Record a gps point
			m_pos.recordGPSPoint();
      
		}
		else {
			PlaySound("playwithmyself.wav");
			break;
		}
		
		int extradelay = (int) random() % 1000;
	  
		// Wait for 1600 msec (long enough for clucking to finish
    aIO_MSSleep(m_ioRef, 500 + extradelay, NULL);

	}
	
	
	///////////////////////////////////////////////////////
	// go time!
	
	bool bManualOverride = false;
	
	// All this junk is about the RC stuff getting enabled. It is
  // a little more confusing since we want to trigger a sound
  // That is different depending on if we want to go to manual 
  // override mode, or autonomous control mode. 
	while (aErrNone == checkAndWaitForStem())  {
		
		// When we get set automatic mode, we expect a value of 0
		if (m_stem.PAD_IO(aSERVO_MODULE, RCPAD_ENABLE + 1)) {
			
      // First time through, it defaults to false. Then we 
      // rely on the scratchpad for the current state.
			if (!bManualOverride) {
				PlaySound("retarded.wav");
				bManualOverride = true;
			}
			
			// Wait for 1000 msec
			m_stem.sleep(2000);
			
			continue;
			
		}
		else {
			
      // We are off on autonomous mode now. Let's let Stan tell us since
      // he is frigging code.
			if (bManualOverride) {
				PlaySound("ohgodherewego.wav");
        
				//Lets set the motion module to holding still.
        m_mot.updateControl(avcForceVector(0,0));
        
			}
			
      // Set the boolean flag to tell the system we are running blind...
			bManualOverride = false;
		}
		
		//First do the localization step. Let's find out what our system is 
    // at.
		m_pos.updateState();			        
		
		// get repulsive forces
		//m_repulse.getForceResultant(&rv);
		m_log->log(INFO, "Repulsive Force: %f,%f", rv.x, rv.y);
		
		// motion planning step
    avcForceVector motivation;
		m_planner.getMotivation(&motivation, m_pos.getPosition(), rv);
		m_log->log(INFO, "Motivation: %f,%f", motivation.x, motivation.y);
		
    // Update the control system
		m_mot.updateControl(motivation);
		
		// sleep a bit so we don't wail on the processor
    aIO_MSSleep(m_ioRef, m_loopdelay, NULL);
		
	} // end while
	
  // We lost the stem, so might as well be drama queens.
	m_log->log(INFO, "BrainStem Network is no longer connected!\n");
	PlaySound("r2d2die.wav");
	
	return 0;
}


/////////////////////////////////////////////////////////////////////////////
// Check to see if the stem is connected. If it isn't, try to connect
//
// This makes sure we haven't lost control of the stem for some 
// reason. In theory, we won't lose the connection once up and going. 

aErr
avcController::checkAndWaitForStem()
{
  
  aErr e = aErrNone;
	
	if (m_stem.isConnected(STEMCONNECTED_WAIT)) {
		e = aErrNone;
	}
	else {
		aDEBUG_PRINT("\nlost stem connection\t");
		PlaySound("dang.wav");
		
		// Wait until we have a solid heartbeat connection so we know there is 
		// someone to talk to.
		int timeout = 0;
		
		aDEBUG_PRINT("awaiting heartbeat");
		
    // Give the pump a few times to prime and get running.
		do { 
			aDEBUG_PRINT(".");
			aIO_MSSleep(m_ioRef, 500, NULL);
			++timeout;
		} while (!m_stem.isConnected() && timeout < aSTEM_CONN_TIMEOUT*2);
		
		//If we timed out we should report an error condition.
		//The error was a timeout, so not so critical return 1.
		if (timeout == aSTEM_CONN_TIMEOUT*2) {
			aDEBUG_PRINT("\n");
			PlaySound("ahcrap.wav");
			e = aErrTimeout;
		} else {
			//We have a valid stem connection.
			aDEBUG_PRINT("done\n");
			PlaySound("ohgodherewego.wav");
		} // end of else
		
	}
  
  // Return the error state
  return e;
  
}




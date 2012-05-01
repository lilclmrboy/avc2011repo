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
m_lithium(0.2) 
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
	
	PlaySound("okay.wav");

	if (aSettingFile_Create(m_ioRef,
				"planner.config",
				&m_settings,
				&e))
		throw acpException(e, "creating settings");
	
	aArguments_Separate(m_ioRef, m_settings, NULL, argc, argv);

	
	aSettingFile_GetFloat (m_ioRef,m_settings,
						 "speedscale",
						 &m_lithium,
						 .2,
						 &e);

	
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
		if (!m_pos.getGPSQuality())
			m_pos.setPosition(m_planner.getFirstMapPoint());
	}
	return e;
}

/////////////////////////////////////////////////////////////////////////////

void 
avcController::getRepulsiveVector(avcForceVector& r) {
	
	short temp = 0;
	avcForceVector sonar;
	avcForceVector ir;
	
		
	temp = m_stem.PAD_IO(aUSBSTEM_MODULE, aSPAD_USB_REPULSIVE_UX) << 8; 
	m_stem.sleep(10);
	temp |= m_stem.PAD_IO(aUSBSTEM_MODULE, aSPAD_USB_REPULSIVE_UX+1);
	m_stem.sleep(10);
	ir.x = (double) temp / 32767.0;
	
	temp = m_stem.PAD_IO(aUSBSTEM_MODULE, aSPAD_USB_REPULSIVE_UY) << 8; 
	m_stem.sleep(10);
	temp |= m_stem.PAD_IO(aUSBSTEM_MODULE, aSPAD_USB_REPULSIVE_UY+1);
	m_stem.sleep(10);
	ir.y = (double) temp / 32767.0;
	
	
	// Sonar sensors repulsive
	
	//temp = m_stem.PAD_IO(aUSBSTEM_MODULE, aSPAD_GP2_SONAR_REPULSIVE_UX) << 8; 
	//temp |= m_stem.PAD_IO(aUSBSTEM_MODULE, aSPAD_GP2_SONAR_REPULSIVE_UX+1);
	//sonar.x = (double) temp / 32767.0;
	
	//temp = m_stem.PAD_IO(aUSBSTEM_MODULE, aSPAD_GP2_SONAR_REPULSIVE_UY) << 8; 
	//temp |= m_stem.PAD_IO(aUSBSTEM_MODULE, aSPAD_GP2_SONAR_REPULSIVE_UY+1);
	//sonar.y = (double) temp / 32767.0;
		
	
	// Combine all the repulsive forces
	r.x = ir.x + sonar.x;
	r.y = ir.y + sonar.y;
	
	//Boundary check the repulsive force
	r.x = r.x > 1.0 ? 1.0 : r.x;
	r.x = r.x < -1.0 ? -1.0 : r.x;
	
	r.y = r.y > 1.0 ? 1.0 : r.y;
	r.y = r.y < -1.0 ? -1.0 : r.y;

}

/////////////////////////////////////////////////////////////////////////////

int
avcController::run(void) {
	
	//Lets allow a condition to gracefully end the application. 
	//bool running = true;
	logger *m_log = logger::getInstance();
	avcStateVector pos;
	avcForceVector rv;
	aErr e = aErrNone;
	
	//////////////////////////////////////////////
	m_log->log(INFO, "Checking for the go signal\n");
	
	while (m_stem.isConnected(STEMCONNECTED_WAIT)) {
		
		// Read the scratchpad for the RC enable bit
		int rcswitch = m_stem.PAD_IO(aSERVO_MODULE, DIG_RCENABLE + 1);
	  
		// When we get set automatic mode, we expect a value of 0
		if (rcswitch) {
			printf("."); fflush(stdout);
		  
		  int temp = random() % 5;
		  
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
		  
	
		}
		else {
			PlaySound("playwithmyself.wav");
			break;
		}
		
		int extradelay = random() % 1000;
	  
		// Wait for 1600 msec (long enough for clucking to finish
		m_stem.sleep(500 + extradelay);
	}
	
	
	///////////////////////////////////////////////////////
	// go time!
	
	bool bManualOverride = false;
	
	//while (m_stem.isConnected(STEMCONNECTED_WAIT)) {
	while (aErrNone ==checkAndWaitForStem())  {
		
		// When we get set automatic mode, we expect a value of 0
		if (m_stem.PAD_IO(aSERVO_MODULE, DIG_RCENABLE + 1)) {
			
			if (!bManualOverride) {
				PlaySound("retarded.wav");
				bManualOverride = true;
			}
			
			
			// Wait for 1000 msec
			m_stem.sleep(2500);
			
			continue;
			
		}
		else {
			
			if (bManualOverride) {
				PlaySound("ohgodherewego.wav");
			
				//Lets clear the encoders to zero.
				m_stem.MO_ENC32(aSERVO_MODULE, AUTPAD_THROT, SERVO_NEUT);
				m_stem.MO_ENC32(aSERVO_MODULE, AUTPAD_STEER, SERVO_NEUT);
			}
			
			bManualOverride = false;
		}
		
		//First do the localization step. Lets get relevant GPS
		//info from the unit, and compass heading. Along with 
		//the previous state and control vector.
		m_pos.updateState();			        
		
		// get sensor readings
		// We need to fill out ir force vector. 
		// What we need to do is:
		//  1.) Read the scratchpad x and y value
		//  2.) Convert them into a normalized float value
		getRepulsiveVector(rv);
		m_log->log(INFO, "Repulsive Force: %f,%f", rv.x, rv.y);
		
		//e = m_mot.updateControl(rv);
		
		//rv.x *=4;
		//rv.y *=4;
		// motion planning step
		avcForceVector motivation = m_planner.getMotivation(m_pos.getPosition(), rv);
		
		pos = m_pos.getPosition();
		//m_log->log(INFO, "Current position:%e,%e", pos.x, pos.y);
		m_log->log(INFO, "Motivation: %f,%f", motivation.x, motivation.y);
		
		motivation.x *= m_lithium;
		motivation.y *= m_lithium;
		e = m_mot.updateControl(motivation);
		
		// sleep a bit so we don't wail on the processor
		aIO_MSSleep(m_ioRef, 1, NULL);
		
	} // end while
	
	m_log->log(INFO, "BrainStem Network is no longer connected!\n");
	PlaySound("r2d2die.wav");
	
	return 0;
}


/////////////////////////////////////////////////////////////////////////////
// Check to see if the stem is connected. If it isn't, try to connect
//

aErr
avcController::checkAndWaitForStem()
{
	
	if (m_stem.isConnected(STEMCONNECTED_WAIT)) {
		return aErrNone;
	}
	else {
		aDEBUG_PRINT("\nlost stem connection\t");
		
		aErr e = aErrNone;
		
		PlaySound("dang.wav");
		
		// Wait until we have a solid heartbeat connection so we know there is 
		// someone to talk to.
		int timeout = 0;
		
		aDEBUG_PRINT("awaiting heartbeat");
		
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
		}
		
		return e;
		
		
	}
}
	
	
	

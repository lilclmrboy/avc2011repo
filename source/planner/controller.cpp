#include "controller.h"
#define aSTEM_CONN_TIMEOUT 10

#define DEG_TO_RAD (aPI/180)
#define RAD_TO_DEG (180/aPI)

/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
// We may want to move this out somewhere or into it's own seperate class
// so other modules can use it

#define SOUND_DIRECTORY "../../sounds/"

int PlaySound(const char * file);

/////////////////////////////////////////////////////////////////////////////
// We may want to move this out somewhere or into it's own seperate class
// so other modules can use it

// Turn up the system volume
int gSetSystemVolume = 0;

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
	
	// Set the volume all the way up
	if (gSetSystemVolume == 0) {
			gSetSystemVolume = 1;
		system("aumix -v100 &");
	}
	
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
m_fInputVoltageContollerMin(12.0f),
m_loopdelay(aCONTROLLER_LOOP_DELAY_DEFAULT)
{
  aErr e;
	
  if(aIO_GetLibRef(&m_ioRef, &e))
    throw acpException(e, "Getting aIOLib reference");
  
  m_log = logger::getInstance();

}

/////////////////////////////////////////////////////////////////////////////

avcController::~avcController(void) {

  if (m_settings)
    aSettingFile_Destroy(m_ioRef, m_settings, NULL);

  if (m_ioRef)
    aIO_ReleaseLibRef(m_ioRef, NULL);
}

/////////////////////////////////////////////////////////////////////////////
// Main initialization for the whole shooting match

int
avcController::init(const int argc, const char* argv[]) {
  
  // Create a setting file to hold our link settings so we can change
  // the settings without recompiling.
  aErr e = aErrNone;
  
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
	
	aSettingFile_GetFloat(m_ioRef, 
												m_settings, 
												aCONTROLLER_INPUTV_CONTROLLER_KEY, 
												&m_fInputVoltageContollerMin, 
												aCONTROLLER_INPUTV_CONTROLLER_DEFAULT, &e);

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
  aErr e = aErrNone;
  avcStateVector pos;
  avcForceVector rv;
  avcForceVector motivation;
	
	//Need to start taking repulsive forces.
	//m_repulse.run();

  bool bManualOverride = false;
  bool bNotStarted = true;
  bool bSuccess = false;
  int extraDelay = 0;
  unsigned long int prevTime = 0;
  unsigned long int nIterations = 0;
  //Initialize the previous time to something reasonable
  aIO_GetMSTicks(m_ioRef, &prevTime, NULL);
  
  aBool doRecord;
  if(aSettingFile_GetBool(m_ioRef, m_settings, aKEY_MAKE_RECORD,
                           &doRecord, MAKE_RECORD, &e))
    throw acpException(e, "makelogs");

  FILE* locPlanTrackFile;
#ifdef aUSE_GPS
  FILE* gps_track;
#endif

  if(doRecord) {
      // open a file for logging of loc/plan data
      char buffer[100];
      time_t rawtime;
      struct tm* timeinfo;
      time(&rawtime);
      timeinfo = localtime( &rawtime );
      strftime(buffer,100, "locPlanTrack_%d_%m_%H_%M.data", timeinfo);

      locPlanTrackFile = fopen(buffer, "w");
      if (!locPlanTrackFile) {
          m_log->log(ERROR, "Not able to write locPlanTrack.");
          return aErrIO;
      }
      m_log->log(INFO, "starting log: %s", buffer);

      fprintf(locPlanTrackFile, "Long\tLat\tDistRolled\tHeading\tTargetLong\tTargetLat\tDistToNextPoint\tHeadingToNextPointRad\n");
      fflush(locPlanTrackFile);

#ifdef aUSE_GPS
      // Set up GPS track record.
      time(&rawtime);
      timeinfo = localtime( &rawtime );
      strftime(buffer, 100, "GPS_Track_%d_%m_%H_%M.gpx", timeinfo);

      m_log->log(DEBUG, "Writing GPS track: %s", buffer);

      gps_track = fopen(buffer, "w");
      if (!gps_track) {
          m_log->log(ERROR, "Not able to write GPS track. e = %d", e);
          return aErrIO;
      }

      fprintf(gps_track, "Chicken GPS track\n");
      fflush(gps_track);
#endif
  } // end if doRecord

  while (aErrNone == checkAndWaitForStem() && !bSuccess) {

    // Read the scratchpad for the RC enable bit
    // Need to read the second byte, since the PAD_IO writes 2 bytes at a time
    int rcswitch = m_stem.PAD_IO(aSERVO_MODULE, RCPAD_ENABLE);
    // All this junk is about the RC stuff getting enabled. It is
    // a little more confusing since we want to trigger a sound
    // That is different depending on if we want to go to manual
    // override mode, or autonomous control mode.
	

    if(rcswitch && !bManualOverride) {
      if(bNotStarted)
        PlaySound("okay.wav");
      else
        PlaySound("retarded.wav");

      bManualOverride = true;
    } else if(!rcswitch && bManualOverride) {

      if(bNotStarted)
        PlaySound("playwithmyself.wav");
      else
        PlaySound("ohgodherewego.wav");

      //Lets set the motion module to holding still.
      m_mot.updateControl(avcForceVector(0,0));
      bManualOverride = false;
      bNotStarted = false;

    }

    if(bManualOverride) {

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

#ifdef aUSE_GPS
      if(bNotStarted && doRecord) {

        //Record a gps point
        float curLat= 0.0f, curLon= 0.0f, curHed = 0.0f;
        gps* tmpGPS = gps::getInstance();

        if(tmpGPS->getPosition(&curLon, &curLat, &curHed) == aErrNone) {
          fprintf(gps_track, "%3.12f, %2.12f, %3.1f\n",
                  curLon, curLat, m_pos.getCompassHeading());
          fflush(gps_track);
        }

      }
#endif

      extraDelay = (int) random() % 1000;
      // Wait for 1600 msec (long enough for clucking to finish
      aIO_MSSleep(m_ioRef, 500 + extraDelay, NULL);
      m_pos.updateState();

    } else {
      
      int wayPointWasPassed=0;
      
      ///////////////////////////////////////////////////////
      // go time!
      //First do the localization step. Let's find out what our system is at.
      m_pos.updateState();
      
      // get repulsive forces
      //m_repulse.getForceResultant(&rv);
      //m_log->log(INFO, "Repulsive Force: %f,%f", rv.x, rv.y);
      
      // motion planning step
      wayPointWasPassed=0; //reset before getting update from planner
      m_planner.getMotivation(&motivation, m_pos.getPosition(), rv, &wayPointWasPassed);
      m_log->log(INFO, "Motivation: %f,%f", motivation.x, motivation.y);
      
      if(0){//(0 != wayPointWasPassed){
        avcForceVector tempMotivation(0.0,0.0);
        m_mot.updateControl(tempMotivation);
        char inputStr[256];
        gets(inputStr);
      }
      
      //make plotable log entry
      if (doRecord) {
        avcStateVector tempPos = m_pos.getPosition();
        double lastDistanceRolled = m_pos.getLastDistanceTraveled();
        avcWaypointVector tempTarget = m_planner.getNextMapPoint();
        // distance to point and heading to point
        double tempHeadingToNextPointRad = m_planner.getHeadingToNextPointRad() ; 
        double tempDistanceToNextPoint = m_planner.getDistanceToNextPoint();
        
        m_log->log(DEBUG, "Plot: %f,%f,%f,%f,%f,%f,%f", tempPos.x, tempPos.y, tempPos.h, tempTarget.state.x, tempTarget.state.y, tempDistanceToNextPoint, tempHeadingToNextPointRad*RAD_TO_DEG);
        fprintf(locPlanTrackFile, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",
                tempPos.x/aLON_PER_METER, 
                tempPos.y/aLAT_PER_METER,
                lastDistanceRolled,
                (-1.0*tempPos.h*DEG_TO_RAD+(aPI/2.0)), //shift bearing to account for plotting frame difference 0=N and CW vs CCW
                tempTarget.state.x/aLON_PER_METER,
                tempTarget.state.y/aLAT_PER_METER,
                tempDistanceToNextPoint,
                (-1.0*tempHeadingToNextPointRad) + (aPI/2.0));
        fflush(locPlanTrackFile); 
      }
    
      // Update the control system
      m_mot.updateControl(motivation);

      // We're attempting so run the loop for a consistent frequency.
      // So we get the elapsed time, and sleep if not enough time
      // has passed.
      unsigned long int curTime, elapsedTime = 0;
      // Get the current time.
      aIO_GetMSTicks(m_ioRef, &curTime, NULL);
      elapsedTime = curTime - prevTime;
      m_log->log(INFO, "Elapsed Time: %d", elapsedTime);
      if (elapsedTime < m_loopdelay)
        //Now we sleep... if we have any time left.
        aIO_MSSleep(m_ioRef, (m_loopdelay - elapsedTime), NULL);

      // We get this here to maintain correct elapsed time in the loop.
      aIO_GetMSTicks(m_ioRef, &prevTime, NULL);
      

    } // end if not Manual Override
		
		// Measure the input voltage for the controller logic
		// Just do it once in a while
		if ((nIterations++ % 50) == 0) {
			// Measure the voltage from the USBStem
			float inputVoltage = a40PINSTEM_VPWR_VOLTS(m_stem.A2D(aUSBSTEM_MODULE, a40PINSTEM_VPWR));
      
      m_log->log(INFO, "Input system voltage: %fV", 
                 inputVoltage);
      
			// If below our desired voltage
			if (m_fInputVoltageContollerMin > inputVoltage ) {
        m_log->log(INFO, "Input system voltage low: %3.2fV (limit %3.2fV)", 
                   a40PINSTEM_VPWR_VOLTS(inputVoltage), m_fInputVoltageContollerMin);
        // We have a low system voltage
				PlaySound("sonofabitch.wav");
			} // end of if for checking input voltage
		} // end if for modulus of iterations that check input voltage

  } // end while
	



  // We lost the stem, so might as well be drama queens.
  if(!bSuccess) {
    m_log->log(INFO, "BrainStem Network is no longer connected!\n");
    PlaySound("r2d2die.wav");
    return -1;
  } else {
    m_log->log(INFO, "We Made it !!!\n");
    return 0;
  }

  fclose(locPlanTrackFile);
#ifdef aUSE_GPS
  fclose(gps_track);
#endif
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




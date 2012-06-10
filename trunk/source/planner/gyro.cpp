#include "gyro.h"

/////////////////////////////////////////////////////////////////////////////
// avcGyro interface constructor
// initializes the class and the gyro
avcGyro::avcGyro(acpStem *pStem, aSettingFileRef settings){
	//setup the logger
  m_logger = logger::getInstance();
  
  // check that stem reference is OK
  if(!pStem){
    m_logger->log(ERROR, "%s: stem pointer is null; setting gyro HW to none", __FUNCTION__);
    m_gyroHWtype = kGyro_none;
    return;
  }

  // store the stem reference for later use
  m_pStem = pStem;
  m_gyroHWtype = kGyro_none;

  
  return;
}

/////////////////////////////////////////////////////////////////////////////
// Default deconstructor
avcGyro::~avcGyro(void) {}



/////////////////////////////////////////////////////////////////////////////
int avcGyro::init(){
  // call the HW init
  // Default or none
  m_logger->log(ERROR, "%s: Not implemented by gyro type %d", __FUNCTION__, (int)m_gyroHWtype);

  return -1;
}

/////////////////////////////////////////////////////////////////////////////
// Return the angular rate of acceleration from the gyro (single reading)
int avcGyro::getAngularRateData(int *dddx, int *dddy, int *dddz){
    
  // Check the pointers
  if(!dddx || !dddy || !dddz){
    m_logger->log(ERROR, "%s: Null pointer passed in", __PRETTY_FUNCTION__);
    return -1;
  }

  // Default or none
  m_logger->log(ERROR, "%s: Not implemented by gyro type %d", __FUNCTION__, (int)m_gyroHWtype);
  
  return -1;
}



/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
#ifdef aDEBUG_GYRO
int main(int argc, const char* argv[]) {

	logger* log = logger::getInstance();
  
  acpStem stem;
  aSettingFileRef settings;
  aErr e = aErrNone;
  aIOLib ioRef;
  
  // Grab an aIO reference object to gain the setting to talk to the stem.
  aIO_GetLibRef(&ioRef, &e);
  
  // Read from a settings file if it exists.
  if (aSettingFile_Create(ioRef, "chicken.config",&settings,&e))
    throw acpException(e, "creating settings");
  
  // or, maybe command line arguements
  aArguments_Separate(ioRef, settings, NULL, argc, argv);
  
	// the gyro classes are using polymorphism, so should be accessed via references
  // to ensure the child-class virtual functions are used
  avcGyro *gyro = new gyroL3G4200D(&stem, &settings);
  
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
    fflush(stdout);
    aIO_MSSleep(ioRef, 500, NULL);
    ++timeout;
  } while (!stem.isConnected() && timeout < 30);
  
  printf("\nDone; ");
  
  // Bail if no stem. What's the point little man?
  if (timeout == 10) { printf("Failed.\n"); return 1; }
  printf("Got stem.\n");
  
  gyro->init();

  for (int i=0; i<100; i++){
    int x=0, y=0, z=0;
    gyro->getAngularRateData(&x, &y, &z);
    log->log(INFO, "gyro x,y,z: %d\t%d\t%d", x, y, z);
    stem.sleep(100);
  }
  
  free(gyro);
  
}

#endif

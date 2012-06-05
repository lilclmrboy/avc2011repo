#include "gyro.h"

/////////////////////////////////////////////////////////////////////////////
// Pull in the HW implementation files
// These could be masked out based on what's being used to save program space
// Such masking doesn't seem necessary now
#include "gyroL3G4200D.h"


/////////////////////////////////////////////////////////////////////////////
// avcGyro interface constructor
// initializes the class and the gyro
avcGyro::avcGyro(acpStem *pStem, aSettingFileRef settings){
  if(!pStem){
    m_logger->log(ERROR, "%s: stem pointer is null; setting gyro HW to none", __PRETTY_FUNCTION__);
    m_gyroHWtype = kGyro_none;
  }
  
  else{
	  // Define which HW to use
  	m_gyroHWtype = kGyro_L3G4200D;
    
    m_pStem = pStem;
  }
  
  return;
}

/////////////////////////////////////////////////////////////////////////////
// Default deconstructor
avcGyro::~avcGyro(void) {}



/////////////////////////////////////////////////////////////////////////////
int avcGyro::init(){
  // call the HW init
  switch (m_gyroHWtype) {
    // L3G4200D
    case kGyro_L3G4200D:
      if(0 != gyroL3G4200Dinit(m_pStem)){
        m_logger->log(ERROR, "%s: Error while initializing L3G4200D", __PRETTY_FUNCTION__);
      }
      break;
      
    // Default or none
    case kGyro_none:
    default:
      m_logger->log(INFO, "%s: Gyro type %d not supported", __PRETTY_FUNCTION__, (int)m_gyroHWtype);
      break;
  }
  
  return 0;
}

/////////////////////////////////////////////////////////////////////////////
// Return the angular rate of acceleration from the gyro (single reading)
int avcGyro::getAngularRateData(int *dddx, int *dddy, int *dddz){
  
  // Check the pointers
  if(!dddx || !dddy || !dddz){
    m_logger->log(ERROR, "%s: Null pointer passed in", __PRETTY_FUNCTION__);
    return 0;
  }
  
  // Set up some temp variables
  int tempDddx=0, tempDddy=0, tempDddz=0;
  
  switch (m_gyroHWtype) {
    // L3G4200D
    case kGyro_L3G4200D:
      gyroL3G4200DgetX(m_pStem, &tempDddx);
      gyroL3G4200DgetY(m_pStem, &tempDddy);
      gyroL3G4200DgetZ(m_pStem, &tempDddz);
      break;
    
    // Default or none
    case kGyro_none:
    default:
      m_logger->log(INFO, "%s: Gyro type %d not supported", __FUNCTION__, (int)m_gyroHWtype);
      break;
  }
  
  // Move the temp readings to the passed locations
  *dddx = tempDddx;
  *dddy = tempDddy;
  *dddz = tempDddz;
  
  return 0;
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
  
  avcGyro gyro = avcGyro(&stem, &settings);
  
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
  
  printf("\n");
  
  gyro.init();
  
  // Bail if no stem. What's the point little man?
  if (timeout == 10) { return 1; }

  for (int i=0; i<100; i++){
    int x=0, y=0, z=0;
    gyro.getAngularRateData(&x, &y, &z);
    log->log(INFO, "gyro x,y,z: %d\t%d\t%d", x, y, z);
    stem.sleep(100);
  }
  
}

#endif
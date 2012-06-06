#include "compass.h"
#include "LSM303DLM.h"

/////////////////////////////////////////////////////////////////////////////
// Constructor
avcCompass::avcCompass(acpStem *pStem, aSettingFileRef settings){
  m_compassHwType = kCompass_LSM303DLM;
  m_pStem = pStem;
}

/////////////////////////////////////////////////////////////////////////////
// Default deconstructor
avcCompass::~avcCompass(void) {}

/////////////////////////////////////////////////////////////////////////////
// initialize the HW
int avcCompass::init(){
  switch (m_compassHwType){
      // LSM303DLM compass
    case kCompass_LSM303DLM:
      compassLSM303DLMinit(m_pStem);
      break;
      
      // default and undefined compass
    case kCompass_none:
    default:
      m_logger->log(INFO, "%s: not supported by compass type %d", __PRETTY_FUNCTION__, m_compassHwType);
      break;
  }
  return 0;
}

/////////////////////////////////////////////////////////////////////////////
// read the current heading
int avcCompass::getHeading(int *heading){
  // Check the pointers
  if(!heading){
    m_logger->log(ERROR, "%s: Null pointer passed in", __PRETTY_FUNCTION__);
    return -1;
  }
  
  switch (m_compassHwType){
    // LSM303DLM compass
    case kCompass_LSM303DLM:
      compassLSM303DLMgetHeading(m_pStem, heading);
      break;
      
    // default and undefined compass
    case kCompass_none:
    default:
      m_logger->log(INFO, "%s: not supported by compass type %d", __PRETTY_FUNCTION__, m_compassHwType);
      break;
  }
  return 0;
}

/////////////////////////////////////////////////////////////////////////////
// get all three axes reading
int avcCompass::getMagnetometerReadings(int *x, int *y, int *z){
  // Check the pointers
  if(!x || !y || !z){
    m_logger->log(ERROR, "%s: Null pointers passed in", __PRETTY_FUNCTION__);
    return -1;
  }
  
  switch (m_compassHwType){
      // LSM303DLM compass
    case kCompass_LSM303DLM:
      compassLSM303DLMgetX(m_pStem, x);
      compassLSM303DLMgetY(m_pStem, y);
      compassLSM303DLMgetZ(m_pStem, z);
      break;
      
      // default and undefined compass
    case kCompass_none:
    default:
      m_logger->log(INFO, "%s: not supported by compass type %d", __PRETTY_FUNCTION__, m_compassHwType);
      break;
  }
  
  return 0;
}



/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
#ifdef aDEBUG_COMPASS
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
  
  avcCompass compass = avcCompass(&stem, &settings);
  
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
  
  // Bail if no stem. What's the point little man?
  if (timeout == 10) { return 1; }
  
  compass.init();
  
  //for (int i=0; i<100; i++){
  while(1){
    int x=0, y=0, z=0;
    compass.getMagnetometerReadings(&x, &y, &z);
    log->log(INFO, "compass x,y,z: %d\t%d\t%d", x, y, z);
    stem.sleep(100);
  }
  
}

#endif
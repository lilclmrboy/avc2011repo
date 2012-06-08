#include "compass.h"

#define DEG_TO_RAD (aPI/180)
#define RAD_TO_DEG (180/aPI)

/////////////////////////////////////////////////////////////////////////////
// Constructor
avcCompass::avcCompass(acpStem *pStem, aSettingFileRef settings){
	m_logger = logger::getInstance();
  m_compassHwType = kCompass_none;
  m_pStem = pStem;
}

/////////////////////////////////////////////////////////////////////////////
// Default deconstructor
avcCompass::~avcCompass(void) {}

/////////////////////////////////////////////////////////////////////////////
// initialize the HW
int avcCompass::init(){
  // default and undefined compass
  m_logger->log(ERROR, "%s: not supported by compass type %d", __FUNCTION__, m_compassHwType);
  return -1;
}

/////////////////////////////////////////////////////////////////////////////
// read the current heading in degress
int avcCompass::getHeadingDeg(float *headingDeg){
  // Check the pointers
  if(!headingDeg){
    m_logger->log(ERROR, "%s: Null pointer passed in", __FUNCTION__);
    return -1;
  }
  
  m_logger->log(INFO, "%s: not supported by compass type %d", __FUNCTION__, m_compassHwType);
  return -1;
}

/////////////////////////////////////////////////////////////////////////////
// read the current heading in radians
int avcCompass::getHeadingRad(float *headingRad){
  // check pointer
  if(!headingRad)
		return -1;
  
  float headingDeg=0.0;
  if(0 != getHeadingDeg(&headingDeg))
    return -1;
  
  *headingRad = headingDeg * DEG_TO_RAD;
  
  return 0;
}

/////////////////////////////////////////////////////////////////////////////
// get all three axes reading
int avcCompass::getMagnetometerReadings(int *x, int *y, int *z){
  // Check the pointers
  if(!x || !y || !z){
    m_logger->log(ERROR, "%s: Null pointers passed in", __FUNCTION__);
    return -1;
  }
  
  m_logger->log(INFO, "%s: not supported by compass type %d", __FUNCTION__, m_compassHwType);
  return -1;
}

/////////////////////////////////////////////////////////////////////////////
// get all three axes reading
int avcCompass::resetCalToFactory(){
	m_logger->log(INFO, "%s: not supported by compass type %d", __FUNCTION__, m_compassHwType);
  return -1;
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
  
  avcCompass *compass = new compassLSM303DLM(&stem, &settings);
  compass->init();
  
  printf("\n\n Running the LSM303DLM compass\n");
  
  for (int i=101; i<100; i++){
  //while(1){
    int x=0, y=0, z=0;
    compass->getMagnetometerReadings(&x, &y, &z);
    log->log(INFO, "compass x,y,z: %d\t%d\t%d", x, y, z);
    stem.sleep(100);
  }
  
  free(compass);
  
  compass = new compassCMPS10(&stem, &settings);
  compass->init();
  
  printf("\n\n Now the CMPS10\n");
  
  //compass->resetCalToFactory();
  
  //for (int i=0; i<100; i++){
	while(1){
    int x=0, y=0, z=0;
    compass->getMagnetometerReadings(&x, &y, &z);
    //log->log(INFO, "compass x,y,z: %d\t%d\t%d", x, y, z);
    
    float headingDeg=0.0;
    compass->getHeadingDeg(&headingDeg);
    log->log(INFO, "compass heading: %f", headingDeg);
    stem.sleep(100);
  }
  
}

#endif
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
// temporary for testing
int avcCompass::getAccelerometerReadings(int *accX, int *accY, int *accZ){
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
  
  printf("\n\nRunning the LSM303DLM compass\n");
  
  int minx=0, miny=0, minz=0, maxx=0, maxy=0, maxz=0;
  
  for (int i=0; i<100; i++){
  //while(1){
    int magx=0, magy=0, magz=0;
    compass->getMagnetometerReadings(&magx, &magy, &magz);
    //log->log(INFO, "mag x,y,z: %6d\t%6d\t%6d", magx, magy, magz);
    
    int accx=0, accy=0, accz=0;
    compass->getAccelerometerReadings(&accx, &accy, &accz);
    //log->log(INFO, "accel x,y,z: %6d\t%6d\t%6d", accx, accy, accz);
    
    float headingDeg=0;
    compass->getHeadingDeg(&headingDeg);
    log->log(INFO, "compass heading: %f", headingDeg);
    //log->log(INFO, "compass m, a, h: %d,%d,%d; %d,%d,%d; %f", magx, magy, magz, accx, accy, accz, headingDeg);
    
    if(magx < minx) minx = magx;
    if(magy < miny) miny = magy;
    if(magz < minz) minz = magz;
    
    if(magx > maxx) maxx = magx;
    if(magy > maxy) maxy = magy;
    if(magz > maxz) maxz = magz;
    
    stem.sleep(100);
  }
  
  log->log(INFO, "compass min: %d %d %d", minx, miny, minz);
  log->log(INFO, "compass max: %d %d %d", maxx, maxy, maxz);
  
  free(compass);
  
//  printf("\n\n Now the CMPS10\n");  
//  compass = new compassCMPS10(&stem, &settings);
//  compass->init();
//  
//  for (int i=0; i<100; i++){
//	while(1){
//    int x=0, y=0, z=0;
//    compass->getMagnetometerReadings(&x, &y, &z);
//    //log->log(INFO, "compass x,y,z: %d\t%d\t%d", x, y, z);
//    
//    float headingDeg=0.0;
//    compass->getHeadingDeg(&headingDeg);
//    log->log(INFO, "compass heading: %f", headingDeg);
//    stem.sleep(100);
//  }
//  
//  //compass->resetCalToFactory();
//  
//  free(compass);
}

#endif
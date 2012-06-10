#include "accelerometer.h"

/////////////////////////////////////////////////////////////////////////////
// Constructor
avcAccelerometer::avcAccelerometer(acpStem *pStem, aSettingFileRef settings){
	m_logger = logger::getInstance();
  m_accelerometerHwType = kAccelerometer_none;
  m_pStem = pStem;
}

/////////////////////////////////////////////////////////////////////////////
// Default deconstructor
avcAccelerometer::~avcAccelerometer(void) {}

/////////////////////////////////////////////////////////////////////////////
// initialize the HW
int avcAccelerometer::init(){
  // default and undefined compass
  m_logger->log(ERROR, "%s: not supported by accelerometer type %d", __FUNCTION__, kAccelerometer_none);
  return -1;
}


/////////////////////////////////////////////////////////////////////////////
// get all three axes reading
int avcAccelerometer::getAccelerometerReadings(float *x, float *y, float *z){
  // Check the pointers
  if(!x || !y || !z){
    m_logger->log(ERROR, "%s: Null pointers passed in", __FUNCTION__);
    return -1;
  }
  
  m_logger->log(INFO, "%s: not supported by compass type %d", __FUNCTION__, kAccelerometer_none);
  return -1;
}


/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
#ifdef aDEBUG_ACCELEROMETER
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
  
  avcAccelerometer *accelerometer;
  
  ///////////////////
  printf("\n\nRunning the LSM303DLM accelerometer test\n");
 
  accelerometer = new accelerometerLSM303DLM(&stem, &settings);
  accelerometer->init();
  
  for (int i=0; i<100; i++){
  //while(1){
    float accx=0, accy=0, accz=0;
    accelerometer->getAccelerometerReadings(&accx, &accy, &accz);
    log->log(INFO, "accel x,y,z: %3.2f\t%3.2f\t%3.2f", accx, accy, accz);
    
    stem.sleep(100);
  }

  free(accelerometer);
 
  ////////////////////
  printf("\n\nRunning the AXDL335 accelerometer test\n");
  
  accelerometer = new accelerometerADXL335(&stem, &settings);
  accelerometer->init();
  
  for (int i=0; i<100; i++){
    //while(1){
    float accx=0, accy=0, accz=0;
    accelerometer->getAccelerometerReadings(&accx, &accy, &accz);
    log->log(INFO, "accel x,y,z: %3.2f\t%3.2f\t%3.2f", accx, accy, accz);
    
    stem.sleep(100);
  }
  
  free(accelerometer);
  
  
}

#endif

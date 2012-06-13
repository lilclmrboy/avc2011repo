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
// Accelerometer thread
/////////////////////////////////////////////////////////////////////////////
avcAccelerometerThread::avcAccelerometerThread(avcAccelerometer *avcAccelReference){
  // get a logger reference
  m_logger = logger::getInstance();
  
  // check the pointer
  if(!avcAccelReference)
    m_logger->log(ERROR, "Null reference to avcAccelerometer passed into avcAccelerometerThread");

  // store the accelerometer reference
  m_accerlerometer = avcAccelReference;
    
  // set the cumulative average and counters to 0
  m_currentCumulativeAverage.x=0.0;
  m_currentCumulativeAverage.y=0.0;
  m_currentCumulativeAverage.z=0.0;
  m_readingCounter = 0;
  
  // the accelerometer update rate is 50Hz
  // set the thread delay to be half that (25Hz = 40ms delay)
	m_threadDelay = 40;
  
  m_logger->log(INFO, "Starting acceleromter thread process");
  
  // Fire off the thread
  m_pThread = acpOSFactory::thread("acclerometerAveraging");
  m_accelReadingLock = acpOSFactory::mutex("accelerometerLock");
  m_pThread->start(this);
  
}

avcAccelerometerThread::~avcAccelerometerThread(void){
  
}


/////////////////////////////////////////////////////////////////////////////
// read the average accelerometer measurements since the last reading
int avcAccelerometerThread::getAverageAccerlerometerMeasurements(double *x, double *y, double *z){
  // check the pointers
  if(!x || !y || !z){
    m_logger->log(ERROR, "%s: Null pointer passed in", __FUNCTION__);
    return -1;
  }
  
  // grab the mutex lock
  m_accelReadingLock->lock();
  
  // stuff the reading into the pointers
  *x = m_currentCumulativeAverage.x;
  *y = m_currentCumulativeAverage.y;
  *z = m_currentCumulativeAverage.z;
  
  // reset the readings
  //m_currentCumulativeAverage.x = 0.0;
  //m_currentCumulativeAverage.y = 0.0;
  //m_currentCumulativeAverage.z = 0.0;
  // since we're computing the cumulative average directly, we just have to reset the counter
  m_readingCounter=0;
  
  // release the loc
  m_accelReadingLock->unlock();
  
  return 0;
}

/////////////////////////////////////////////////////////////////////////////
// unused virtual function from acpThread
//void avcAccelerometerThread::step (const double time){
//  
//}

/////////////////////////////////////////////////////////////////////////////
// main thread function to update cumulative average of accelerometer readings
int avcAccelerometerThread::run(void){
  
  while (!m_pThread->isDone()) {
    bool bIdle = true;
    
    // make a new measurement
    makeNewMeasurement();
    
    // handle any messages first
    if (m_pThread->handleMessage())
      bIdle = false;
    
    // yield if nothing is happening
    if (bIdle)
      m_pThread->yield(m_threadDelay);
  }

  return 0;
}

/////////////////////////////////////////////////////////////////////////////
// go make an accelerometer reading and push it into the cumulative average
int avcAccelerometerThread::makeNewMeasurement(void){
  
  // get new readings from the accelerometer
  float newX=0, newY=0, newZ=0;
  if(0 != m_accerlerometer->getAccelerometerReadings(&newX, &newY, &newZ)){
    m_logger->log(ERROR, "%s: not able to get accelerometer readings", __FUNCTION__);
  }
  
  // get the lock
  m_accelReadingLock->lock();
  
  //update the cumulative average
  //m_currentCumulativeAverage.x += ((double)newX - m_currentCumulativeAverage.x) / (double)(m_readingCounter+1);
  //m_currentCumulativeAverage.y += ((double)newY - m_currentCumulativeAverage.y) / (double)(m_readingCounter+1);
  //m_currentCumulativeAverage.z += ((double)newZ - m_currentCumulativeAverage.z) / (double)(m_readingCounter+1);
  
  //do the cumulative average with a multiply so we can simply reset the counter to clear the averaging
  m_currentCumulativeAverage.x += ((double)newX + (double)m_readingCounter * m_currentCumulativeAverage.x) / (double)(m_readingCounter+1);
  m_currentCumulativeAverage.y += ((double)newY + (double)m_readingCounter * m_currentCumulativeAverage.y) / (double)(m_readingCounter+1);
  m_currentCumulativeAverage.z += ((double)newZ + (double)m_readingCounter * m_currentCumulativeAverage.z) / (double)(m_readingCounter+1);
  m_readingCounter++;
  
  // release the lock
  m_accelReadingLock->unlock();
  
  return 0;
  
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
  avcAccelerometerThread *accelThread;
  
#if 1
  
  ///////////////////
  printf("\n\nRunning the LSM303DLM accelerometer test\n");
 
  accelerometer = new accelerometerLSM303DLM(&stem, &settings);
  accelerometer->init();
  accelThread = new avcAccelerometerThread(accelerometer);
  
  for (int i=0; i<100; i++){
  //while(1){
    float accx=0, accy=0, accz=0;
    accelerometer->getAccelerometerReadings(&accx, &accy, &accz);
    log->log(INFO, "accel x,y,z: %3.2f\t%3.2f\t%3.2f", accx, accy, accz);
    
    double avgAccx=0, avgAccy=0, avgAccz=0;
    accelThread->getAverageAccerlerometerMeasurements(&avgAccx, &avgAccy, &avgAccz);
    log->log(INFO, "avg accel x,y,z: %3.2f\t%3.2f\t%3.2f", avgAccx, avgAccy, avgAccz);
    
    stem.sleep(100);
  }

  free(accelerometer);
  
#else
 
  ////////////////////
  printf("\n\nRunning the AXDL335 accelerometer test\n");
  
  accelerometer = new accelerometerADXL335(&stem, &settings);
  accelerometer->init();
  
  for (int i=0; i<100; i++){
    //while(1){
    float accx=0, accy=0, accz=0;
    accelerometer->getAccelerometerReadings(&accx, &accy, &accz);
    log->log(INFO, "accel x,y,z: %3.2f\t%3.2f\t%3.2f", accx, accy, accz);
    
    stem.sleep(5);
  }
  
  free(accelerometer);
#endif

  
}

#endif

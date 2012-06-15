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
// Gyro thread
/////////////////////////////////////////////////////////////////////////////
avcGyroThread::avcGyroThread(avcGyro *avcGyroReference){
  // get a logger reference
  m_logger = logger::getInstance();
  
  // check the pointer
  if(!avcGyroReference)
    m_logger->log(ERROR, "Null reference to avcGyro passed into avcGyroThread");
	
  // store the gyro reference
  m_gyro = avcGyroReference;
	
  // set the cumulative average and counters to 0
  m_currentCumulativeAverage.x=0.0;
  m_currentCumulativeAverage.y=0.0;
  m_currentCumulativeAverage.z=0.0;
  m_readingCounter = 0;
  
  // the gyro update rate is 50Hz
  // set the thread delay to be half that (25Hz = 40ms delay)
	m_threadDelay = 100;
  
  m_logger->log(INFO, "Starting gyro thread process");
  
  // Fire off the thread
  m_pThread = acpOSFactory::thread("gyroAveraging");
  m_gyroReadingLock = acpOSFactory::mutex("gyroLock");
  m_pThread->start(this);
  
}

avcGyroThread::~avcGyroThread(void){
  
}

/////////////////////////////////////////////////////////////////////////////
// read the average gyro measurements since the last reading
int avcGyroThread::getAverageAngularRateMeasurements(double *x, double *y, double *z){
  // check the pointers
  if(!x || !y || !z){
    m_logger->log(ERROR, "%s: Null pointer passed in", __FUNCTION__);
    return -1;
  }
  
  // grab the mutex lock
  m_gyroReadingLock->lock();
  
  // stuff the reading into the pointers
  *x = m_currentCumulativeAverage.x;
  *y = m_currentCumulativeAverage.y;
  *z = m_currentCumulativeAverage.z;
  
  //m_logger->log(DEBUG,"clearing counter at %d", m_readingCounter);
  // reset the readings
  m_currentCumulativeAverage.x = 0.0;
  m_currentCumulativeAverage.y = 0.0;
  m_currentCumulativeAverage.z = 0.0;
  m_readingCounter=0;
  
  // release the loc
  m_gyroReadingLock->unlock();
  
  return 0;
}


/////////////////////////////////////////////////////////////////////////////
// main thread function to update cumulative average of gyro readings
int avcGyroThread::run(void){
  
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
// go make an gyro reading and push it into the cumulative average
int avcGyroThread::makeNewMeasurement(void){
  
  // get new readings from the gyro
  int newX=0, newY=0, newZ=0;
  if(0 != m_gyro->getAngularRateData(&newX, &newY, &newZ)){
    m_logger->log(ERROR, "%s: not able to get gyro readings", __FUNCTION__);
  }
  
  // get the lock
  m_gyroReadingLock->lock();
  
  //update the cumulative average
  m_currentCumulativeAverage.x += ((double)newX - m_currentCumulativeAverage.x) / (double)(m_readingCounter+1);
  m_currentCumulativeAverage.y += ((double)newY - m_currentCumulativeAverage.y) / (double)(m_readingCounter+1);
  m_currentCumulativeAverage.z += ((double)newZ - m_currentCumulativeAverage.z) / (double)(m_readingCounter+1);
  
  m_readingCounter++;
  
  // release the lock
  m_gyroReadingLock->unlock();
  
  return 0;
  
}

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
#ifdef aDEBUG_GYRO

#include <iostream>
#include <fstream>
using namespace std;

int main(int argc, const char* argv[]) {
  
  logger* log = logger::getInstance();
  
  acpStem stem;
  aSettingFileRef settings;
  aErr e = aErrNone;
  aIOLib ioRef;
	ofstream logfile;
	acpString data;	
  
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
  
  avcGyro *gyro;
  avcGyroThread *gyroThread;
  
  printf("\n\nRunning the L3G4200D gyro test\n");
  gyro = new gyroL3G4200D(&stem, &settings);
	logfile.open("../aUser/L3G4200D.csv");
	
  gyro->init();
  gyroThread = new avcGyroThread(gyro);
  
  for (int i=0; i<50; i++){
    //while(1){
    int vx=0, vy=0, vz=0;
    gyro->getAngularRateData(&vx, &vy, &vz);
    log->log(INFO, "gyro x,y,z:     %d\t%d\t%d", vx, vy, vz);
    
    double avgVx=0, avgVy=0, avgVz=0;
    gyroThread->getAverageAngularRateMeasurements(&avgVx, &avgVy, &avgVz);
    log->log(INFO, "avg gyro x,y,z: %3.2f\t%3.2f\t%3.2f", avgVx, avgVy, avgVz);
		
		// Log the data to a file
		if (!(vx == 0 && vy == 0 && vz == 0))
			logfile << vx << ", " << vy << ", " << vz << endl;
    
    stem.sleep(100);
  }
	
	logfile.close();
  
  free(gyro);
	
}

#endif

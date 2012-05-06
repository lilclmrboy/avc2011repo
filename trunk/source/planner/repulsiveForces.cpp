///////////////////////////////////////////////////////////////////////////
// Filename: repulsiveForces.cpp

#include "repulsiveForces.h"

#ifdef aDEBUG_FREPULSIVE
bool bDebugHeader = true;
#endif


///////////////////////////////////////////////////////////////////////////
// Operator function to add repulsive vector forces together
// Boundary check the force addition
const avcRepulsiveForce& avcRepulsiveForce::operator+=(const avcRepulsiveForce& rhs)
{
  m_force.x += rhs.m_force.x;
  
  m_force.x = m_force.x > 1.0 ? 1.0 : m_force.x;
  m_force.x = m_force.x < -1.0 ? -1.0 : m_force.x;
  
  m_force.y += rhs.m_force.y;
  
  m_force.y = m_force.y > 1.0 ? 1.0 : m_force.y;
  m_force.y = m_force.y < -1.0 ? -1.0 : m_force.y;
  
  return *this;
}

///////////////////////////////////////////////////////////////////////////
// generalized repulisive force constructor
avcRepulsiveForce::avcRepulsiveForce(acpStem *pStem, 
                                     const char * settingFileName) :
  m_theta(aREPULSIVE_THETA_DEFAULT),
  m_radiusMax(aREPULSIVE_RADIUS_DEFAULT)
{
  aErr e = aErrNone;
  
  // Create aIO library reference
  aIO_GetLibRef(&m_ioRef, &e);
  
  // Grab the pointer to the Stem
  m_pStem = pStem;
  
  // Create setting file reference based on a specific config file
  // name
  aSettingFile_Create(m_ioRef, settingFileName, &m_settings, &e);
  
  // Get the theta value from settings
  aSettingFile_GetFloat(m_ioRef, m_settings, 
                        aREPULSIVE_THETA_KEY, &m_theta, 
                        aREPULSIVE_THETA_DEFAULT, &e);
  
  // Get the human readable description
  char *pDescription;
  aSettingFile_GetString(m_ioRef, m_settings, 
                         aREPULSIVE_DESCRIPTION_KEY, &pDescription, 
                         aREPULSIVE_DESCRIPTION_DEFAULT, &e);
  m_description = pDescription;
  
  // Get the force radius that we care about
  aSettingFile_GetFloat(m_ioRef, m_settings, 
                        aREPULSIVE_RADIUS_KEY, &m_radiusMax, 
                        aREPULSIVE_RADIUS_DEFAULT, &e);
  
}

///////////////////////////////////////////////////////////////////////////
// generalized repulisive force destructor
avcRepulsiveForce::~avcRepulsiveForce(void)
{
  
  aErr e = aErrNone;
  
  // Clean up the setting file reference
  if (m_settings)
    aSettingFile_Destroy(m_ioRef, m_settings, &e);
  
  // Clean up the library reference
  if (m_ioRef)
    aIO_ReleaseLibRef(m_ioRef, &e);
  
}

///////////////////////////////////////////////////////////////////////////
// GP2D12 constructor
avcGP2D12::avcGP2D12(acpStem *pStem, const char * settingFileName) :
  avcRepulsiveForce(pStem, settingFileName),
  m_a2dport(aREPULSIVE_GP2D12_PORT_DEFAULT)
{
  
  aIOLib ioRef;
  aSettingFileRef settings;
  aErr e = aErrNone;
  
  // Create aIO library reference
  aIO_GetLibRef(&ioRef, &e);
  
  // Get a handle on the logger instance
  m_log = logger::getInstance();
  
  // Create setting file reference based on a specific config file
  // name
  aSettingFile_Create(ioRef, settingFileName, &settings, &e);
  
  // Get the port that the sensor is attached to
  int a2dport = 0;
  aSettingFile_GetInt(m_ioRef, m_settings, 
                      aREPULSIVE_GP2D12_PORT_KEY, &a2dport, 
                      aREPULSIVE_GP2D12_PORT_DEFAULT, &e);
  m_a2dport = (unsigned char) a2dport;
  
  // Clean up the setting file and ioRef
  aSettingFile_Destroy(ioRef, settings, &e);
  aIO_ReleaseLibRef(ioRef, &e);
  
}

///////////////////////////////////////////////////////////////////////////
// GP2D12 sensor update
aErr
avcGP2D12::update(void) {
  
  aErr e = aErrNone;
  float reading = 0.0f;
  float distance = 0.0f;
  float force_distance = 0.0f;
  float a1 = -36.8421f;
  float k = 94.7368f;
  
  // Read from the Stem. This is normalized from 0.0 to 1.0
  // The a2d on the BrainStem GP is 0 to 5V
  // 
  
#ifdef aDEBUG_FREPULSIVE // Using Symonym, right???
  reading = m_pStem->A2D(6, m_a2dport) * 5.0f;
#else
  reading = m_pStem->A2D_RD(aSERVO_MODULE, m_a2dport) * 5.0f;
#endif
  
  // Calculate the distance
  // Function derived from datasheet figure 5
  if (reading < 0.001f) reading = 0.001f;
  
  // Calculate a rough linear fit in meters
  distance = (a1 * reading + k) / 100.0f;
  
  // We can't get a negative distance
  if (distance < 0.0f) distance = 0.0f;
  
  // Get the force distance and normalize it
  // when things are detected very close, we want a very large force
  // when they are far away, we want small force effects 
  force_distance = (m_radiusMax - distance) / m_radiusMax;
    
  // Get the x and y components
  // This is repulsive, so we want to go backwards
  m_force.x = cos(m_theta + aPI) * force_distance;
  m_force.y = sin(m_theta + aPI) * force_distance;

#ifdef aDEBUG_FREPULSIVE
  m_log->log(INFO, "%s CH%d: %f (%f cm) [%f] Rx: %f Ry: %f", 
             (const char *) m_description,
             m_a2dport,
             reading,
             distance,
             force_distance,
             m_force.x, m_force.y);
#endif
  
  return e;
  
}

///////////////////////////////////////////////////////////////////////////
// Constructor for the function
avcRepulsiveForces::avcRepulsiveForces(void) :
  m_pStem(NULL),
  m_settings(NULL),
  m_bInit(false),
  m_threadDelay(aREPULSIVE_DELAY_DEFAULT),
  m_pThread(NULL)
{
  
  aErr e = aErrNone;
  
  // Get access to the logger class
  m_log = logger::getInstance();
  
  // Create a aIO reference to manipulate settings file reference
  if(aIO_GetLibRef(&m_ioRef, &e)) 
    throw acpException(e, "Getting aIOLib reference");
  
}

///////////////////////////////////////////////////////////////////////////
// Destructor for the function
avcRepulsiveForces::~avcRepulsiveForces(void) 
{
  
  aErr e = aErrNone;
  
  // Destroy the thread object if created
  if (m_pThread)
    delete m_pThread;
  
  if (m_ioRef) {
    if (aIO_ReleaseLibRef(m_ioRef, &e))
      throw acpException(e, "unable to destroy settings");
  }
  
}

///////////////////////////////////////////////////////////////////////////
// Get the Stem object and initialize any thing else that we need to.

aErr
avcRepulsiveForces::init(acpStem *pStem, aSettingFileRef settings) {
  
  aErr e = aErrNone;
  
  // Grab the pointer to the Stem. 
  m_pStem = pStem;
  
  // Grab the pointer to the settings
  m_settings = settings;
  
  // Get the delay value for the thread polling
  aSettingFile_GetULong(m_ioRef, m_settings, 
                        aREPULSIVE_DELAY_KEY, &m_threadDelay, 
                        aREPULSIVE_DELAY_DEFAULT, &e);
  
  // Get access to the logger class
  m_log = logger::getInstance();
  m_log->log(INFO, "Repulsive Force Module initialized");
  
  // Number of forces (aka, sensors)
  m_nForces = 2;
  
  // Set all the sensors to zero
  printf("setting all forces to NULL\n");
  for (int i = 0; i < aREPULSIVE_MAX_SENSORS; i++)
    m_pForces[i] = NULL;
  
  // Set up the actual sensors that we are working with
  acpString sensor_cfg;
  
  sensor_cfg.format("%s0.config", aREPULSIVE_SENSOR_CONFIG_PREFIX);  
  m_pForces[0] = new avcGP2D12(pStem, sensor_cfg);
  
  sensor_cfg.format("%s1.config", aREPULSIVE_SENSOR_CONFIG_PREFIX);
  m_pForces[1] = new avcGP2D12(pStem, sensor_cfg);
  
  printf("created forces\n");
  
  // Set the flag to inidicate that we have been properly initialized
  m_bInit = true;
  
  m_log->log(INFO, "Starting thread process");
  
  // Fire off the thread
  m_pThread = acpOSFactory::thread("repForce");
  m_pThread->start(this);
  
  return e;
  
}

///////////////////////////////////////////////////////////////////////////
// Given a new goal Force vector, update the each motor setpoint

aErr
avcRepulsiveForces::getForceResultant(avcForceVector *pU) 
{
  
  aErr e = aErrNone;

  // Copy information we care about into the passed in 
  // force vector
  pU->x = m_RepulsiveResult.x;
  pU->y = m_RepulsiveResult.y;

  m_log->log(INFO, "[%s] %s: URx=%f URy=%f", 
	     __FILE__, __PRETTY_FUNCTION__,
	     pU->x, pU->y);

  return e;
  
}

///////////////////////////////////////////////////////////////////////////
// Time step for thread?

void
avcRepulsiveForces::step (const double time) 
{
}

///////////////////////////////////////////////////////////////////////////
// Thread run handle

int
avcRepulsiveForces::run(void) 
{
  
  while (!m_pThread->isDone()) {
    bool bIdle = true;
    
    // Update all the sensors since we are not busy
#ifdef aDEBUG_FREPULSIVE    
    m_log->log(INFO, "Updating rforce run thread");
#endif
    
    // Make sure you other hackers initialized this first
    if (!m_pStem || !m_bInit) {
      m_log->log(ERROR,"%s::%s failed. acpStem is NULL"
                 " or class not intialized\n"
                 " \tCall %s::%s(acpStem *)", 
                 __FILE__, __PRETTY_FUNCTION__,
                 __FILE__, __PRETTY_FUNCTION__);
      return aErrInitialization;
    }
    
    // Update the sensor readings for all the sensors we care about
    avcRepulsiveForce Uresult;
    int i = 0;
    do {
      m_pForces[i]->update();
      
      // Add the new force
      Uresult += *m_pForces[i]; 
      
    } while (m_pForces[++i] != NULL);
    
    m_RepulsiveResult.x = Uresult.getUx();
    m_RepulsiveResult.y = Uresult.getUy();
    
    // handle any messages first
    if (m_pThread->handleMessage())
      bIdle = false;
    
    // yield if nothing is happening
    if (bIdle)
      m_pThread->yield(m_threadDelay);
  }
  
  return 0;
  
}

///////////////////////////////////////////////////////////////////////////
// This section is for isolating and debugging this module. 
// You will need to set up a stem object and work from that.
// Use the makefile to build this in isolation.
// > make repulsiveForceTest
#ifdef aDEBUG_FREPULSIVE

////////////////////////////////////////
// main testing routine for motModule 
int 
main(int argc, 
     const char* argv[]) 
{
  
  acpStem stem;
  aSettingFileRef settings;
  aErr e = aErrNone;
  aIOLib ioRef;
  
  // Grab an aIO reference object to gain the setting to talk to the stem.
  aIO_GetLibRef(&ioRef, &e);
  
  // Read from a settings file if it exists.
  if (aSettingFile_Create(ioRef, 
			  "chicken.config",
			  &settings,
			  &e))
    throw acpException(e, "creating settings");
  
  // or, maybe command line arguements
  aArguments_Separate(ioRef, settings, NULL, argc, argv);
    
  avcRepulsiveForces frepulsive; 

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
  
  frepulsive.init(&stem, settings);
  avcForceVector Urepulsive;
  
  for (int i = 0; i < 100; i++) {
    frepulsive.getForceResultant(&Urepulsive);
    stem.sleep(50);
  }
  
  aIO_MSSleep(ioRef, 100, NULL);
  
  //////////////////////////////////
  // Clean up and get outta here. Run Forrest, RUN!!!
  aSettingFile_Destroy(ioRef, settings, NULL);
  aIO_ReleaseLibRef(ioRef, NULL);
  
  return 0;
  
}

#endif

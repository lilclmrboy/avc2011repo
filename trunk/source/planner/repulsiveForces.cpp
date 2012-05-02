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
  m_theta(0.0f)
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
                         "description", &pDescription, 
                         "Sensor", &e);
  m_description = pDescription;
  
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
  m_a2dport(0)
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
  aSettingFile_GetInt(m_ioRef, m_settings, "port", &a2dport, 0, &e);
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
  
  // Read from the Stem
  reading = m_pStem->A2D_RD(aSERVO_MODULE, m_a2dport);
  
  // Get the x and y components
  // This is repulsive, so we want to go backwards
  m_force.x = cos(m_theta + aPI) * reading;
  m_force.y = sin(m_theta + aPI) * reading;
  
  m_log->log(INFO, "%s on CH%d: %f Rx: %f Ry: %f", 
             (const char *) m_description,
             m_a2dport,
             reading, m_force.x, m_force.y);
  
  return e;
  
}

///////////////////////////////////////////////////////////////////////////
// Constructor for the function
avcRepulsiveForces::avcRepulsiveForces(void) :
  m_pStem(NULL),
  m_settings(NULL),
  m_bInit(false)
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
  
  // Get access to the logger class
  m_log = logger::getInstance();
  m_log->log(INFO, "Repulsive Force Module initialized");
  
  // Number of forces (aka, sensors)
  m_nForces = 2;
  
  // Set all the sensors to zero
  printf("setting all forces to NULL\n");
  for (int i = 0; i < m_nForces; i++)
    m_pForces[i] = NULL;
  
  // Set up the actual sensors that we are working with
  m_pForces[0] = new avcGP2D12(pStem, "gp2d12a.config");
  m_pForces[1] = new avcGP2D12(pStem, "gp2d12b.config");
  
  printf("created forces\n");
  
  // Set the flag to inidicate that we have been properly initialized
  m_bInit = true;
  
  return e;
  
}

///////////////////////////////////////////////////////////////////////////
// Given a new goal Force vector, update the each motor setpoint

aErr
avcRepulsiveForces::getForceResultant(avcForceVector *pU) 
{
  
  aErr e = aErrNone;
  avcRepulsiveForce Uresult;
  
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
  int i = 0;
  do {
    m_pForces[i]->update();
    
    // Add the new force
    Uresult += *m_pForces[i]; 
    
  } while (m_pForces[++i] != NULL);
  

  // Copy information we care about into the passed in 
  // force vector
  pU->x = Uresult.getUx();
  pU->y = Uresult.getUy();

  m_log->log(INFO, "[%s] %s: URx=%f URy=%f", 
	     __FILE__, __PRETTY_FUNCTION__,
	     pU->x, pU->y);
  
  return e;
  
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
  
  for (int i = 0; i < 10; i++) {
    frepulsive.getForceResultant(&Urepulsive);
    stem.sleep(500);
  }
  
  aIO_MSSleep(ioRef, 1000, NULL);
  
  //////////////////////////////////
  // Clean up and get outta here. Run Forrest, RUN!!!
  aSettingFile_Destroy(ioRef, settings, NULL);
  aIO_ReleaseLibRef(ioRef, NULL);
  
  return 0;
  
}

#endif

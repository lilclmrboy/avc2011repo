///////////////////////////////////////////////////////////////////////////
// Filename: repulsiveForces.cpp

#include "repulsiveForces.h"

#ifdef aDEBUG_FREPULSIVE
bool bDebugHeader = true;
#endif


///////////////////////////////////////////////////////////////////////////
// Operator function to add repulsive vector forces together
//avcRepulsiveForce avcRepulsiveForce::operator+=(const avcRepulsiveForce& rhs)
//{
//  m_force.x += rhs.m_force.x;
//  m_force.y += rhs.m_force.y;
//  return *this;
//}

///////////////////////////////////////////////////////////////////////////
// GP2D12 sensor update
aErr
avcGP2D12::update(void) {
  
  aErr e = aErrNone;
  
  // Take a reading from the stem. 
  
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
  
  // Create a aIO reference to manipulate settings file reference
  if(aIO_GetLibRef(&m_ioRef, &e)) 
    throw acpException(e, "Getting aIOLib reference");
  
}

///////////////////////////////////////////////////////////////////////////
// Destructor for the function
avcRepulsiveForces::~avcRepulsiveForces(void) 
{
  
  aErr e = aErrNone;
  
  if (aIO_ReleaseLibRef(m_ioRef, &e))
    throw acpException(e, "unable to destroy settings");
  
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
  
  for (int i = 0; i < 10; i++)
    m_pForces[i] = NULL;
  
  // Set up the actual sensors that we are working with
  m_pForces[0] = new avcGP2D12(pStem, "gp2d12a.config");
  
  // Set the flag to inidicate that we have been properly initialized
  m_bInit = true;
  
  return e;
  
}

///////////////////////////////////////////////////////////////////////////
// Given a new goal Force vector, update the each motor setpoint

aErr
avcRepulsiveForces::getForceResultant(avcForceVector *pForceVector) 
{
  aErr e = aErrNone;
  
  // Make sure you other hackers initialized this first
  if (!m_pStem || !m_bInit) {
    m_log->log(ERROR,"%s::%s failed. acpStem is NULL"
         " or class not intialized\n"
	       " \tCall %s::%s(acpStem *)", 
               __FILE__, __PRETTY_FUNCTION__,
               __FILE__, __PRETTY_FUNCTION__);
    return aErrInitialization;
  }
  
  m_log->log(INFO, "[%s] %s: running", __FILE__, __PRETTY_FUNCTION__);
  
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
  
  frepulsive.getForceResultant(&Urepulsive);
  
  aIO_MSSleep(ioRef, 1000, NULL);
  
  //////////////////////////////////
  // Clean up and get outta here. Run Forrest, RUN!!!
  aSettingFile_Destroy(ioRef, settings, NULL);
  aIO_ReleaseLibRef(ioRef, NULL);
  
  return 0;
  
}

#endif

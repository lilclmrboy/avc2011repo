/////////////////////////////////////////////////////////////////////////////
// File: logger.cpp
// The general purpose logging class

#include "logger.h"

///////////////////////////////////////////////////////////////////////////
// Class constructor
logger::logger(void) :
  m_pConsole(NULL),
  m_pLogTxt(NULL)
{
	// Let's write to STDOUT. We could make this just about anywhere if we want
	m_pConsole = stdout;
	
	// Open and create a plain log file to write to
	char outputfile[32];
	sprintf(outputfile, "logger.log");
	m_pLogTxt = fopen(outputfile,"a");
	
	if (m_pLogTxt == NULL)
		aDEBUG_PRINT("Could not create %s file\n", outputfile);
	
}

///////////////////////////////////////////////////////////////////////////
// Class constructor
logger::~logger(void)
{
	
	/* Exit and close up our documents */
  if (m_pConsole)
    fclose(m_pConsole); // Close the console  
	
	if (m_pLogTxt)
    fclose(m_pLogTxt); // Close the general log file 
	
}

///////////////////////////////////////////////////////////////////////////
// Grabs the system time and formats as a string
const char *
logger::getTime(void) {
	
	char buffer[80];
	
	// cheater. this gets rid of some memory issues
	acpString info;

	// Standard textbook get the time stuff
	time(&m_rawtime);
  m_pTimeinfo = localtime (&m_rawtime);
  strftime (buffer, 80, "%x-%X", m_pTimeinfo);
	
	// Safe and friendly copy
	info = buffer;
	
	return info;
}

///////////////////////////////////////////////////////////////////////////
// Append a general comment to the log output
void
logger::append(const char * info, aLogType type /* = LogAll */)
{	
	
	int i = type;
	bool bLogAll = (type == LogAll) ? true : false;
	
	do {
		
		// Depending on whom we write to, we might do different stuff
		switch (i) {

			case LogConsole:
				
				// Write the string out. We might want a different format 
				fprintf(m_pConsole, "LOG (%s): %s\n", getTime(), info);
				
				break;
				
			case LogText:
				
				// Check to make sure file is not NOT created
				if (m_pLogTxt)
					fprintf(m_pLogTxt, "LOG (%s): %s\n", getTime(), info);
				
				break;
				
			default:
				break;
		}
		
		// Decrement the counter
		i--;
		
	} while (bLogAll && (i > 0));
	
}

///////////////////////////////////////////////////////////////////////////
// Append a vector to the log output
void
logger::append(const avcStateVector& statevector, aLogType type /* = LogAll */)
{
	
	int i = type;
	bool bLogAll = (type == LogAll) ? true : false;
	
	do {
				
		// Depending on whom we write to, we might do different stuff
		switch (i) {
			case LogConsole:
				
				// Write the string out. We might want a different format 
				fprintf(m_pConsole, "LOG (%s): "
								"Longitude: %2.2f Latitude: %2.2f "
								"Heading: %2.2f "
								"Vx: %2.2f Vy: %2.2f Vw: %2.2f"
								"\n", 
								getTime(), 
								statevector.x,
								statevector.y,
								statevector.h,
								statevector.vx,
								statevector.vy,
								statevector.vw);
				
				break;
				
			case LogText:
				
				// Check to make sure file is not NOT created
				if (m_pLogTxt)
					// Write the string out. We might want a different format 
					fprintf(m_pLogTxt, "LOG (%s): "
									"Longitude: %2.2f Latitude: %2.2f "
									"Heading: %2.2f "
									"Vx: %2.2f Vy: %2.2f Vw: %2.2f"
									"\n", 
									getTime(), 
									statevector.x,
									statevector.y,
									statevector.h,
									statevector.vx,
									statevector.vy,
									statevector.vw);
				break;
				
			default:
				break;
		}
		
		// Decrement the counter
		i--;
		
	} while (bLogAll && (i > 0));

}

///////////////////////////////////////////////////////////////////////////
// Append a state vector to the log output
void
logger::append(const avcForceVector& potential, aLogType type /* = LogAll */)
{
	
	int i = type;
	bool bLogAll = (type == LogAll) ? true : false;
	
	do {
		
		// Depending on whom we write to, we might do different stuff
		switch (i) {
			case LogConsole:
				
				// Write the string out. We might want a different format 
				fprintf(m_pConsole, "LOG (%s): "
								"Ux: %2.2f Uy: %2.2f"
								"\n", 
								getTime(), 
								potential.x,
								potential.y);
				
				break;
				
			case LogText:
				
				// Check to make sure file is not NOT created
				if (m_pLogTxt)
					// Write the string out. We might want a different format 
					fprintf(m_pLogTxt, "LOG (%s): "
									"Ux: %2.2f Uy: %2.2f"
									"\n", 
									getTime(), 
									potential.x,
									potential.y);
				break;
				
			default:
				break;
		}
		
		// Decrement the counter
		i--;
		
	} while (bLogAll && (i > 0));
	
}

///////////////////////////////////////////////////////////////////////////
// This section is for isolating and debugging this module. 
#ifdef aDEBUG_LOGGER

////////////////////////////////////////
// main testing routine for logger 
int 
main(int argc, 
     const char* argv[]) 
{
	logger log;
	avcForceVector Uresult;
	avcStateVector State;
	
	log.append("New logging session started");
	log.append("hello little kitty", LogConsole);
	log.append("This a bit of garble", LogText);
	log.append(State);
	
	for (int i = 0; i < 10; i++) {
		
		Uresult.x = i;
		Uresult.y = -1*i;
		
		log.append(Uresult);

	}

	return 0;
	
}

#endif

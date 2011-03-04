/////////////////////////////////////////////////////////////////////////////
// File: logger.cpp
// The general purpose logging class

#include "logger.h"

///////////////////////////////////////////////////////////////////////////
// Class constructor
logger::logger(void) :
  m_pConsole(NULL),
  m_pLogTxt(NULL), 
  m_ioRef(NULL),
  m_settings(NULL)
{
	aErr e = aErrNone;
	char outputfile[32];
	
	if (m_pLogTxt == NULL)
		aDEBUG_PRINT("Could not create %s file\n", outputfile);
	
	// Create a aIO reference to manipulate settings file reference
	if(aIO_GetLibRef(&m_ioRef, &e)) 
		throw acpException(e, "Getting aIOLib reference");
	
	// Read from the SettingsFileRef to see if we should change things for our
	// logger. 
	// We need to first call the create settings file reference function
	// Then grab what we want out of it
	// OR, we should pass it in. I like that a bit more. 
	
	// Let's write to STDOUT. We could make this just about anywhere if we want
	// Like STDERR, etc
	m_pConsole = stdout;
	
	// Open and create a plain log file to write to
	// String should be grabbed from the settings file reference
	aSettingFile_Create(m_ioRef,
											32,
											aLOGGER_CONFIG,
											&m_settings,
											&e);
	
	// Grab the log file name from settings
	char* pLogFile;
	aSettingFile_GetString(m_ioRef,
												 m_settings,
												 aKEY_LOGGER_FILENAME, 
												 &pLogFile,
												 aLOGGER_FILENAME,
												 &e);
	
	m_LogTextFileName = pLogFile;
	m_pLogTxt = fopen((const char *) m_LogTextFileName,"a");
	
}

///////////////////////////////////////////////////////////////////////////
// Class constructor
logger::logger(aSettingFileRef settings) :
  m_pConsole(NULL),
  m_pLogTxt(NULL), 
  m_ioRef(NULL),
  m_settings(settings)
{
	aErr e = aErrNone;
	char outputfile[32];
	
	if (m_pLogTxt == NULL)
		aDEBUG_PRINT("Could not create %s file\n", outputfile);
	
	// Create a aIO reference to manipulate settings file reference
	if(aIO_GetLibRef(&m_ioRef, &e)) 
		throw acpException(e, "Getting aIOLib reference");
	
	// Let's write to STDOUT. We could make this just about anywhere if we want
	// Like STDERR, etc
	m_pConsole = stdout;
	
	// Open and create a plain log file to write to
	m_LogTextFileName = "logger.log";
	m_pLogTxt = fopen((const char *) m_LogTextFileName,"a");
	
}

///////////////////////////////////////////////////////////////////////////
// Class constructor
logger::~logger(void)
{
	aErr e = aErrNone;
	
	/* Exit and close up our documents */
  if (m_pConsole)
    fclose(m_pConsole); // Close the console  
	
	if (m_pLogTxt)
    fclose(m_pLogTxt); // Close the general log file 
	
	/* Clean up input settings */
	if (m_settings && aSettingFile_Destroy(m_ioRef, m_settings, &e))
		throw acpException(e, "unable to destroy settings");
	
	if (aIO_ReleaseLibRef(m_ioRef, &e))
		throw acpException(e, "unable to destroy settings");
	
}

///////////////////////////////////////////////////////////////////////////
// Grabs the system time and formats as a string
acpString
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
logger::appendString(const char * info, aLogType type /* = LogAll */)
{	
	
	int i = type;
	bool bLogAll = (type == LogAll) ? true : false;
	
	do {
		
		// Depending on whom we write to, we might do different stuff
		// We could so some fancy things, or not here. 
		switch (i) {
				
			case LogConsole:
				
				// Write the string out. We might want a different format 
				fprintf(m_pConsole, "LOG (%s): %s\n", (const char*) getTime(), info);
				
				break;
				
			case LogText:
				
				// Check to make sure file is not NOT created
				if (m_pLogTxt)
					fprintf(m_pLogTxt, "LOG (%s): %s\n", (const char*) getTime(), info);
				
				break;
				
			default:
				break;
		}
		
		// Decrement the counter
		i--;
		
	} while (bLogAll && (i > 0));
	
}


///////////////////////////////////////////////////////////////////////////
// Append a general comment to the log output
void
logger::append(const char * info, aLogType type /* = LogAll */)
{	
	
	// Write the data out
	appendString(info, type);
	
}

///////////////////////////////////////////////////////////////////////////
// Append a vector to the log output
// We build up the information and pass it to our private function 
// that handles the writing. 
void
logger::append(const avcStateVector& statevector, aLogType type /* = LogAll */)
{
	
	char buffer[100];
	
	switch (type) {
		case LogAll:
		case LogConsole:
		case LogText:
			sprintf(buffer,
							"Longitude: %2.2f Latitude: %2.2f Heading: %2.2f "
							"Vx: %2.2f Vy: %2.2f Vw: %2.2f",
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
	
	// Write the data out
	appendString(buffer, type);
	
}

///////////////////////////////////////////////////////////////////////////
// Append a state vector to the log output
void
logger::append(const avcForceVector& potential, aLogType type /* = LogAll */)
{
	
	char buffer[100];
	
	switch (type) {
		case LogAll:
		case LogConsole:
		case LogText:
			sprintf(buffer,
							"Ux: %2.2f Uy: %2.2f",
							potential.x,
							potential.y);
			
			break;
		default:
			break;
	}
	
	// Write the data out
	appendString(buffer, type);
	
}


///////////////////////////////////////////////////////////////////////////
// Clears the current log file out
aErr
logger::emptyLog(void)
{
	
	aDEBUG_PRINT("Clearing out \"%s\" log file.\n", (const char *) m_LogTextFileName);
	
	if (m_pLogTxt)
    fclose(m_pLogTxt); // Close the general log file 
	
	// Re-open the log file with the flag to create an empty file
		m_pLogTxt = fopen((const char *) m_LogTextFileName,"w");
	
	return aErrNone;
	
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
	
	log.emptyLog();
	
	log.append("Second logging session started");
	
	return 0;
	
}

#endif

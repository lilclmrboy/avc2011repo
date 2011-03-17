/////////////////////////////////////////////////////////////////////////////
// File: logger.cpp
// The general purpose logging class

#include "logger.h"

////////////////////////////////////////////////////////////////////////////
// This is a private static member of the logger class and cannot be 
// accessed directly. To access the logger, use logger::getInstance();
logger* logger::m_pInstance = NULL;

///////////////////////////////////////////////////////////////////////////
// This is simple, In a multithreaded environment we'd need a handle on a 
// lock before entering this function.
logger*
logger::getInstance() {

	if (!m_pInstance) //only one instance is ever created.
		m_pInstance = new logger;

	return m_pInstance;

}

///////////////////////////////////////////////////////////////////////////
// Class Destructor
logger::~logger(void)
{
	
	/* Exit and close up our documents */
  if (m_pInfo) {
    fclose(m_pInfo); // Close the info log 
		m_pInfo = NULL;	
	}

	if (m_pError)
    fclose(m_pError); // Close the error log 
	
}

/////////////////////////////////////////////////////////////////////////////
//Opens a logfile "filename" and sets it for both info and error.
void 
logger::openLogFile(const char* filename) {
	// we reopen... ensuring we close any active stream.
	m_pError = freopen(filename, "w", m_pInfo);
}	
	
/////////////////////////////////////////////////////////////////////////////
void 
logger::openInfoLogFile(const char* filename) {
	// we reopen ... 	
	freopen(filename, "w", m_pInfo);
}

/////////////////////////////////////////////////////////////////////////////
	
void 
logger::openErrorLogFile(const char* filename) {
	//again lets reopen ...
	freopen(filename, "w", m_pError);

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
				
			case LogInfo:
				
				// Write the string out. We might want a different format 
				fprintf(m_pInfo, "LOG (%s): %s\n", (const char*) getTime(), info);
				
				break;
				
			case LogError:
				
				fprintf(m_pError, "LOG (%s): %s\n", (const char*) getTime(), info);
				
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
logger::append(const char * msg, aLogType type /* = LogInfo */)
{	
	
	// Write the data out
	appendString(msg, type);
	
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
		case LogInfo:
		case LogError:
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
		case LogInfo:
		case LogError:
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
// printf style logging interface for errors
// errors will always use "LogAll" style
void
logger::logError(const char *fmt, ...)
{
	
	char *buff = NULL;

	//char *insert_buff = NULL;
	size_t buff_size;
	va_list args;
		
	va_start(args, fmt);
	buff_size = strlen(fmt) + 50;
	buff = (char*) calloc (buff_size, sizeof(char));
	//insert_buff = (char*) calloc (buff_size, sizeof(char));
	sprintf(buff, "ERROR - %s\n", fmt);
	
	//appendString(insert_buff, LogAll);
	vfprintf(m_pInfo, buff, args);
	va_end(args);
	free (buff);
	//free (insert_buff);
	
}

///////////////////////////////////////////////////////////////////////////
// printf style logging interface for info
// uses "LogAll" style
void
logger::logInfo(const char *fmt, ...)
{
	
	char *Buf = NULL;//[4096];
	char *insert_buff = NULL;
	int buff_size;
	va_list args;
	
	va_start(args, fmt);
	buff_size = 10000;//(strlen (args) + 100 < 4096) ? 4096 : strlen(args) + 100;
	Buf = (char*) calloc (buff_size, sizeof(char));
	insert_buff = (char*) calloc (buff_size, sizeof(char));
	vsprintf(Buf, fmt, args);
	va_end(args);
	sprintf(insert_buff, "INFO - %s", Buf);
	
	appendString(insert_buff, LogAll);
	
	free (Buf);
	free (insert_buff);
	
}


///////////////////////////////////////////////////////////////////////////
// This section is for isolating and debugging this module. 
#ifdef aDEBUG_LOGGER

int testMacros(int state);

////////////////////////////////////////
// main testing routine for logger 
int 
main(int argc, 
     const char* argv[]) 
{
	logger* log = logger::getInstance();
	avcForceVector Uresult;
	avcStateVector State;
	
	log->append("New logging session started");
	log->append("hello little kitty", LogInfo);
	log->append("This a bit of garble", LogError);
	log->append(State);
	
	for (int i = 0; i < 10; i++) {
		
		Uresult.x = i;
		Uresult.y = -1*i;
		
		log->append(Uresult);
		
	}
	
	log->append("Second logging session started");
	
	log->logError("%s:%s: Testing logError()", __FILE__, __PRETTY_FUNCTION__);
	log->logInfo("%s:%s: Testing logInfo()", __FILE__, __PRETTY_FUNCTION__);
	
	testMacros(-1);

	log->append("\n\nTesting G_CHK_ERR_RETURN(aErrNone)");
	if(testMacros(0) == -1){
		log->append("Pass: G_CHK_ERR_RETURN(aErrNone)");
	}
	else {
		log->append("Fail: G_CHK_ERR_RETURN(aErrNone)");
	}
	
	log->append("\n\nTesting G_CHK_ERR_RETURN(aErrUnknown)");
	if(testMacros(1) == aErrUnknown){
		log->append("Pass: G_CHK_ERR_RETURN(aErrUnknown)");
	}
	else {
		log->append("Fail: G_CHK_ERR_RETURN(aErrUnknown) didn't return aErrUnknown");
	}

	
	log->append("\n\nTesting G_CHK_ERR(aErrNone)");
	if(testMacros(2) == -1)
		log->append("Pass: G_CHK_ERR(aErrNone)");
	else {
		log->append("Fail: G_CHK_ERR(aErrNone) returned (it shouldn't)");
	}


	log->append("\n\nTesting G_CHK_ERR(aErrUnknown)");
	if(testMacros(3) == -1)
		log->append("Pass: G_CHK_ERR(aErrUnknown)");
	else 
		log->append("Fail: G_CHK_ERR(aErrUnknown) returned (it shouldn't)");
		
	log->append("\n\nTesting CHECK_ARG()");
	if(testMacros(4) == aErrParam)
		log->append("Pass: Testing CHECK_ARG()");
	else
		log->append("Fail: Testing CHECK_ARG()");
	
	log->append("\n\nTesting CHECK_ARG_RETURN()");
	if(testMacros(5) == -1)
		log->append("Fail: CHECK_ARG_RETURN()");
	else
		log->append("Pass: CHECK_ARG_RETURN()");
	
	log->append("\n\nTesting RETURN_ERROR()");
	if(testMacros(6) == 1)
		log->append("Pass RETURN_ERROR()");
	else
		log->append("Fail RETURN_ERROR()");
	
	
	return 0;
	
}

int testMacros(int state){
	int *p=NULL;
	aErr status = aErrNone;

	logger* m_logger = logger::getInstance();
	
	switch (state) {
		case 0:
			G_CHK_ERR_RETURN(aErrNone);
			return -1;
			break;
		case 1:

			G_CHK_ERR_RETURN(aErrUnknown);
			m_logger->append("Fail: CHECK_ARG_RETURN(aErrUnknown) didn't return");
			return 0;
			break;
		case 2:
			G_CHK_ERR(aErrNone, "Fail: G_CHK_ERR(aErrNone,...)");
			return -1;
			break;
		case 3:
			G_CHK_ERR(aErrUnknown, "Pass: G_CHK_ERR(aErrUnknown,...)");
			return -1;
			break;
		case 4:
			CHECK_ARG(1);
			CHECK_ARG(p);
			return status;
		case 5:
			
			CHECK_ARG_RETURN(1);
			CHECK_ARG_RETURN(p);
			return -1;
		case 6:
			RETURN_ERROR(1, "Testing RETURN_ERROR()");
			return -1;
			
		default:
			break;
	}

	return 0;
}
	

#endif

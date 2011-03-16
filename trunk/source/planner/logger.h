/////////////////////////////////////////////////////////////////////////////
// File: logger.h

#ifndef _logger_H_
#define _logger_H_

#include "avc2011.h"
#include <time.h>

// We can add more to this as we see needed
// Last enum shoud be LogAll
// First enum should have a "1" as the starting number.
enum aLogType {
	LogConsole = 1,
	LogText,
	LogAll
};

///////////////////////////////////////////////////////////////////////////
// This is a singleton class.

class logger {
public:

	// General destructor. 
	~logger(void);
	
	//We use this method to get a pointer to the logger.	
	static logger* getInstance();	
	void setLogLevel(int level);
	int getLogLevel(void);	

	// Append to an output log file. Depending on what and where, this
	// might end up a bit different.
	// Default is to output to STDIO
	void append(const char * info, aLogType type = LogAll);
	void append(const avcForceVector& potential, aLogType type = LogAll);
	void append(const avcStateVector& statevector, aLogType type = LogAll);

	// printf style logging interface for errors
	// goes to LogAll
	void logError(const char *fmt, ...);

	// printf style logging interface for info
	// goes to LogAll
	void logInfo(const char *fmt, ...);

	
	// Clears the contents of the current log file. Only valid for text files 
	// that get used by this class. 
	aErr emptyLog(void);
	
private:
	
	// Base class constructor. We're going to rely on a log level,
	// and check to see if a logfile named AVClog.log exists in the aUser
	// folder. If it does then we'll log to the file.
	logger(void);
	logger(logger& const) {};
	logger& operator=(logger& const) {};
	

	FILE *m_pConsole;
	FILE *m_pLogTxt;
	
	time_t m_rawtime;
	struct tm *m_pTimeinfo;
	
	aIOLib m_ioRef;
	
	//It is likely controller owns this we'll let them delete it.
	aSettingFileRef m_settings;
	
	// Holding pen for output log file name
	acpString m_LogTextFileName;
	
	// Gets a string that is based on the system time.
	acpString getTime(void);
	
	// Writes a formatted string to somewhere
	void appendString(const char * info, aLogType type = LogAll);
	
};

///////////////////////////////////////////////////////////////////////////
// All macros assume aUtil.h is included for error values and that
// local logger class is m_logger
#define LOG_INFO(desc)\
	{\
	m_logger.logInfo ((char*)"%s:%s", __FILE__, __PRETTY_FUNCTION__);\
	m_logger.logInfo ((char*)desc);\
	}

#define LOG_ERROR(desc)\
	{\
	m_logger.logError ((char*)"%s:%s", __FILE__, __PRETTY_FUNCTION__);\
	m_logger.logError ((char*)desc);\
	}

#define RETURN_ERROR(a, desc)\
	{\
	LOG_ERROR(desc);\
	return a;\
	}

#define G_CHK_ERR_RETURN(a)\
	if (a != aErrNone)\
	{\
		RETURN_ERROR (a, #a);\
	}else

#define G_CHK_ERR(a, desc)\
	if (a != aErrNone)\
	{\
		LOG_ERROR(desc);\
		m_logger.logError ((char*)#a);\
	}else

#define CHECK_ARG_RETURN(a)\
	if (!a)\
	{\
		RETURN_ERROR (aErrParam, "argument " #a " is NULL pointer");\
	}else

// Needs to have local aErr status
#define CHECK_ARG(p)\
	if (!p)\
	{\
		LOG_ERROR("argument " #p " is NULL pointer");\
		status = aErrParam;\
	}


#endif //_logger_H_

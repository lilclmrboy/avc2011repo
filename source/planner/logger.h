/////////////////////////////////////////////////////////////////////////////
// File: logger.h

#ifndef _logger_H_
#define _logger_H_

#include "avc.h"
#include <time.h>

// We can add more to this as we see needed
// First enum should have a "1" as the starting number.
enum aLogType {
	RAW = 1,
	DEBUG,
	INFO,
	NOTICE,	
	ERROR
};

///////////////////////////////////////////////////////////////////////////
// This is a singleton class.

class logger {
public:

	// General destructor. 
	~logger(void);
	
	//We use this method to get a pointer to the logger.	
	static logger* getInstance();
	//Opens a logfile "filename" and sets it for both info and error.
	void openLogFile(const char* filename);	
	//Opens a logfile for a specific log stream.
	void openInfoLogFile(const char* filename);
	void openErrorLogFile(const char* filename);	

	// Append to an output log file. Depending on what and where, this
	// might end up a bit different.
	// Default is to output to STDOUT, STDERR, and the default format
	// is raw... meaning nothing is appended.

	void log(aLogType type = RAW, const char* fmt = "", ...);
  void append(const char * msg, aLogType type = INFO);
	void append(const avcForceVector& potential, aLogType type = INFO);
	void append(const avcStateVector& statevector, aLogType type = INFO);

private:
	//static member variable.
	static logger* m_pInstance;
	// Base class constructor. We're going to rely on a log level,
	// and check to see if a logfile named AVClog.log exists in the aUser
	// folder. If it does then we'll log to the file.
	logger(void): m_pInfo(stdout), m_pError(stderr) {};
	logger(logger const&){};
	logger& operator=(logger const&);
	

	FILE *m_pInfo;
	FILE *m_pError;
	
	time_t m_rawtime;
	struct tm *m_pTimeinfo;
	
	// Holding pen for output log file name
	acpString m_LogTextFileName;
	
	// Gets a string that is based on the system time.
	acpString getTime(void);
};

///////////////////////////////////////////////////////////////////////////
// All macros assume aUtil.h is included for error values and that
// local logger class is m_logger
#define LOG_INFO(logger, desc)\
	{\
	logger->log (INFO, (char*)"%s:%s", __FILE__, __PRETTY_FUNCTION__);\
	logger->log (INFO, (char*)desc);\
	}

#define LOG_ERROR(logger, desc)\
	{\
	logger->log (ERROR, (char*)"%s:%s", __FILE__, __PRETTY_FUNCTION__);\
	logger->log (ERROR, (char*)desc);\
	}

#define RETURN_ERROR(logger, a, desc)\
	{\
	LOG_ERROR(logger, desc);\
	return a;\
	}

#define G_CHK_ERR_RETURN(logger, a)\
	if (a != aErrNone)\
	{\
		RETURN_ERROR (logger, a, #a);\
	}else

#define G_CHK_ERR(logger, a, desc)\
	if (a != aErrNone)\
	{\
		LOG_ERROR(logger, desc);\
		logger->log (ERROR, (char*)#a);\
	}else

#define CHECK_ARG_RETURN(logger, a)\
	if (!a)\
	{\
		RETURN_ERROR (logger, aErrParam, "argument " #a " is NULL pointer");\
	}else

// Needs to have local aErr status
#define CHECK_ARG(logger, p)\
	if (!p)\
	{\
		LOG_ERROR(logger, "argument " #p " is NULL pointer");\
		status = aErrParam;\
	}


#endif //_logger_H_

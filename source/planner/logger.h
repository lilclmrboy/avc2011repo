/////////////////////////////////////////////////////////////////////////////
// File: logger.h

#ifndef _logger_H_
#define _logger_H_

#include "avc2011.h"

// We can add more to this as we see needed
enum aLogType {
	LogConsole = 1,
	LogText,
	LogAll
};

///////////////////////////////////////////////////////////////////////////
class logger {
public:
	logger(void);
	~logger(void);
	
	// Append to an output log file. Depending on what and where, this
	// might end up a bit different.
	// Default is to output to STDIO
	void append(const char * info, aLogType type = LogAll);
	void append(const avcForceVector& potential, aLogType type = LogAll);
	void append(const avcStateVector& statevector, aLogType type = LogAll);
	
private:
	
	FILE *m_pConsole;
	FILE *m_pLogTxt;
	
	time_t m_rawtime;
	struct tm *m_pTimeinfo;
	
	// Gets a string that is based on the system time.
	const char * getTime(void);
	
};

#endif //_logger_H_

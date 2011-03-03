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
class logger {
public:
	
	// Base class constructor. A local settings file is created and used 
	// based on a "logger.conf" file that it default looks for. 
	logger(void);
	
	// This allows most of the adjustable settings to be over ridden with
	// a settings file reference. These can come from command line arguments, 
	// another process, or a configuration file. More likely useful when 
	// being integrated into the big fancy system. 
	logger(aSettingFileRef settings);
	
	// General destructor. 
	~logger(void);
	
	// Append to an output log file. Depending on what and where, this
	// might end up a bit different.
	// Default is to output to STDIO
	void append(const char * info, aLogType type = LogAll);
	void append(const avcForceVector& potential, aLogType type = LogAll);
	void append(const avcStateVector& statevector, aLogType type = LogAll);
	
	// Clears the contents of the current log file. Only valid for text files 
	// that get used by this class. 
	aErr emptyLog(void);
	
private:
	
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

#endif //_logger_H_

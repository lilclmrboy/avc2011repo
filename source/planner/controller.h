/////////////////////////////////////////////////////////////////////////////
// File controller.h
// The controller drives the main loop, and will include the motion module,
// the localization module, and the logger. 
#ifndef _controller_h_
#define _controller_h_

#include "motModule.h"
#include "locModule.h"
#include "logger.h"
#include "aCmd.tea"
#include "aStem.h"
#include "avc2011Defs.tea"

class avcController
{
public:
	avcController(void);
	~avcController(void);
	
	bool init(const int argc, const char* argv[]);
	
	//return of non-zero is error condition.
	int run(void);

private:
	aSettingFileRef m_settings;
	//avcMotion m_mot;
	//avcPosition m_pos;
	//avcLogger m_logger; 
	acpStem m_stem;
	aIOLib m_ioRef;

};

#endif //_controller_h_


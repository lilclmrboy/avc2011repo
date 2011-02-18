/////////////////////////////////////////////////////////////////////////////
// File Planner.h
// The planner drives the main loop, and will include the motion module,
// the localization module, and the logger. 
#ifndef _planner_h_
#define _planner_h_

#include "motModule.h"
#include "locModule.h"
#include "logger.h"
#include "aCmd.tea"
#include "aStem.h"
#include "avc2011Defs.tea"

class avcPlanner
{
public:
	avcPlanner(void);
	~avcPlanner(void);
	
	bool init(const int argc, const char* argv[]);
	bool run(void);

private:
	avcMotion m_mot;
	avcPosition m_pos;
	avcLogger m_logger; 
	acpStem m_stem;
	aIORef m_ioRef;

};


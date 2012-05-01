/////////////////////////////////////////////////////////////////////////////
// File controller.h
// The controller drives the main loop. For now the control loop executes
// a series of steps. 
// 1. Localize
// 2. plan
// 3. Motion control update
// 4. log
// 
// State we need to keep.
// 1. The previous control update, be that force vector or something else.
// 2. The previous robot position and heading. For use with 1 to generate
//    new position estimate for localization. 
// 3. Current localized heading.
// 4. ...
//
// Inputs and outputs. 
//
// Inputs to localization 
// 	previous control output
// 	previous position
// 	GPS position
// 	Compass heading
// Outputs from localization
// 	Vehicle x, y coordinate.
// 	Vehicle heading.
// 
// Inputs to Motion planning.
// 	Current Vehicle position and heading.
// 	Repulsive force vector/s.
// 	Goal input as next waypoint, or camera... arch sighting.
// Outputs to Motion planning.
// 	Vector, (magnitude and direction) corresponding to the desired velocity,
// 	and orientation. According to the paper, the magnitude should be some
// 	value between 0 and the maximum velocity.
// 		
// Inputs to Motion Control.
// 	Vector, (magnitude and direction) of desired velocity and direction.
// Outputs
// 	Setpoints for our left and right motors. According to the equations in 
// 	the motion control paper.

#ifndef _controller_H_
#define _controller_H_

#include "motModule.h"
#include "locModule.h"
#include "planner.h"
#include "logger.h"
#include "aCmd.tea"
#include "aStem.h"
#include "avc2012Defs.tea"
#include "avc2011Structs.h"

/////////////////////////////////////////////////////////////////////////////
// We may want to move this out somewhere or into it's own seperate class
// so other modules can use it
int PlaySound(const char * file);

/////////////////////////////////////////////////////////////////////////////

class avcController
{
public:
	avcController(void);
	~avcController(void);
	
	int init(const int argc, const char* argv[]);
	
	//return of non-zero is error condition.
	int run(void);

private:
	
	void getRepulsiveVector(avcForceVector& r);
	aErr checkAndWaitForStem();
	
	//The controller owns the settings, no other module should destroy them.
	aSettingFileRef m_settings;
	avcMotion m_mot;
	avcPlanner m_planner;
	avcPosition m_pos;
	//avcLogger m_logger; 
	acpStem m_stem;
	aIOLib m_ioRef;
	avcStateVector m_curState;
	//This contains the current setpoints for the left
	//and right motors.
	avcControlVector m_curControl;	
	float m_lithium;	
};

#endif //_controller_H_


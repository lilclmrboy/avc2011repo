/////////////////////////////////////////////////////////////////////////////
// File: Planner.h
// The planner is the high level goal seeking module. It is primarily
// responsible for taking position hypotheses from the localizer and 
// then making decisions about the next goal state.
//
// Its inputs are the current robot position, repulsive vector, 
// and the overall goal. It outputs the atractive force vector based 
//on these inputs.
#include "avc2011Structs.h"

class avcPlanner {

public:
	avcPlanner(void);
	~avcPlanner(void);

	//The main operation, we'll take some inputs, and product a unit
	//vector output.
	avcForceVector getMotivation(const avcStateVector pos, 	
				     const avcForceVector repulse);

};

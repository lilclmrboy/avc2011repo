/////////////////////////////////////////////////////////////////////////////
// File: Planner.h
// The planner is the high level goal seeking module. It is primarily
// responsible for taking position hypotheses from the localizer and 
// then making decisions about the next goal state.
//
// Its inputs are the current robot position, repulsive vector, 
// and the overall goal. It outputs the atractive force vector based 
//on these inputs.
#include "avc.h"
#include "logger.h"

// ubuntu seems to be missing PATH_MAX
#ifndef PATH_MAX
# define PATH_MAX 4096
#endif

class avcPlanner {

public:
	avcPlanner(void);
	~avcPlanner(void);
	
	// initializes the planner with settings file
	aErr init(aIOLib ioRef, aSettingFileRef settings);

	//The main operation, we'll take some inputs, and product a unit
	//vector output.
	aErr getMotivation(avcForceVector *pForceResult, const avcStateVector& pos, 	
				     const avcForceVector& repulse, int *wayPointWasPassed);
	

	// inserts waypoints into the map of waypoints
	// new points are inserted as the next unpassed waypoint
	// should be used by camera/laser system to add "bonus" waypoints
	aErr insertMapPoint(const avcStateVector newPosition);
	const avcWaypointVector& getFirstMapPoint(void) { return m_waypoints.front(); }
  avcWaypointVector getNextMapPoint(void);
  
#ifdef aDEBUG_PLANNER
	friend int main(int, const char**);
#endif
	
private:
	std::vector<avcWaypointVector> m_waypoints; //map of waypoint to navigate
	float m_repulseVectorWeight; // weighting factor between sensors and goal
	unsigned int m_normalizeMotivationVector; // flag to normalize motivation

	// used to define a dount and slice for passing waypoints
	float m_unpassedZetaSliceDeg;
	float m_minUnPassedDistanceToWaypoint;
	float m_maxUnPassedDistanceToWaypoint;

	logger* m_logger;
	
	// used by getMotivation to check for and update passed waypoints
	aErr checkForPassedWayPoints(const avcStateVector& pos, int *wayPointWasPassed);
	
	// used by planner functions to get the index of the first unpassed waypoint
	int getFirstUnpassedWayPoint(void);
	
	// computes a unitless force vector between two state vectors
	aErr calcForceVectorBetweenStates(const avcStateVector& state1,
							 const avcStateVector& state2,
							 avcForceVector *pGoalForceVec);
	// computes a dimensional vector between two states in absolute plane
	aErr calcPolarVectorBetweenStates (const avcStateVector& state1,
								  const avcStateVector& state2,
								  double *r, double *theta);
	
	// normalized a force vector to unit length
	aErr normalizeForceVector ( avcForceVector *pForceVector);
	
	// convenience function to unwrap angles to 0-360
	double unwrapAngleDeg(double phi);
	
	// Attempts to load map from file. If error, inits the map with one
	// point (0.0,0.0,0.0).
	void loadMap(const char* mapfile);
};

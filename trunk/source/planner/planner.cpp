#include "planner.h"

avcPlanner::avcPlanner(void) {
	// setup some example waypoints to play with
	// this might be better to read from a file (or website) later
	waypoints.push_back(avcWaypointVector(-105.2104311212637,40.06449357185146,0.0));
	waypoints.push_back(avcWaypointVector(-105.2104381888533,40.06476958373474,0.0));
	waypoints.push_back(avcWaypointVector(-105.2104340684065,40.06498350881099,0.0));
	waypoints.push_back(avcWaypointVector(-105.2104302362354,40.06514141501155,0.0));
	waypoints.push_back(avcWaypointVector(-105.2104177692964,40.06519889301632,90.0));
	waypoints.push_back(avcWaypointVector(-105.2100671219435,40.06520567299177,90.0));
	waypoints.push_back(avcWaypointVector(-105.2097545474683,40.06519010908657,180.0));
	waypoints.push_back(avcWaypointVector(-105.2097391471105,40.06506188953844,180.0));
	waypoints.push_back(avcWaypointVector(-105.2097412598431,40.06494888446523,180.0));
	waypoints.push_back(avcWaypointVector(-105.2097427813108,40.0647942160145,180.0));
	waypoints.push_back(avcWaypointVector(-105.2097402029708,40.0646459956635,180.0));
	waypoints.push_back(avcWaypointVector(-105.2097427890488,40.06448581625421,270.0));
	waypoints.push_back(avcWaypointVector(-105.2100969789239,40.06449047916893,270.0));
	waypoints.push_back(avcWaypointVector(-105.2104311212637,40.06449357185146,0.0));
	
	
}

/////////////////////////////////////////////////////////////////////////////

avcPlanner::~avcPlanner(void) {}

/////////////////////////////////////////////////////////////////////////////

avcForceVector 
avcPlanner::getMotivation(const avcStateVector pos,
			  const avcForceVector repulse) {
	
	avcForceVector result;
	
	return result;
}

/////////////////////////////////////////////////////////////////////////////


aErr
avcPlanner::insertMapPoint(const avcStateVector newPosition) {
	// inserts waypoints into the map of waypoints
	// new points are inserted as the next unpassed waypoint

	//create a new waypoint
	avcWaypointVector newWaypoint = avcWaypointVector(newPosition);
	
	//copy state vector
	newWaypoint.state = newPosition;
	
	// get the first way point not marked as passed
	int currentWaypointIndex = getFirstUnpassedWayPoint();
	
	try {
		//insert the new waypoint into the waypoint vector
		waypoints.insert(waypoints.begin()+currentWaypointIndex, 1, newWaypoint);
	}
	catch (int &e) {
		// TODO: updated once logger is roughed out
		//logger("__class__::__function__ : Insering new waypoint failed. "
		//		"Inserting new waypoint at end of map.");
		waypoints.push_back(newWaypoint);
	}

	return aErrNone;
	
}

/////////////////////////////////////////////////////////////////////////////

aErr
avcPlanner::checkForPassedWayPoints(void) {
	// compare current position to unpassed waypoints to determine if
	// points should be marked as passed
	
	// options:
	// a) has bot passed line perpendicular to waypoint heading
	//		(corners should be 45˚ headings)
	// b) ^ or V towards waypoint heading within some distance
	// c) simple distance from waypoint (circle)
	
	return aErrNone;

}

/////////////////////////////////////////////////////////////////////////////

int
avcPlanner::getFirstUnpassedWayPoint(void) {
	for (int i = 0; i < (int) waypoints.size(); i++) {
		if(0 == waypoints[i].waypointPassed)
			return (int) i;
	}
	
	return -1;
}

/////////////////////////////////////////////////////////////////////////////

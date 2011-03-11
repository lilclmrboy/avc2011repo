#include "planner.h"

#define DEG_TO_RAD (aPI/180)
#define RAD_TO_DEG (180/aPI)

avcPlanner::avcPlanner(void) {
		
	// setup some example waypoints to play with
	// this might be better to read from a file (or website) later
	m_waypoints.push_back(avcWaypointVector(-105.2104311212637,40.06449357185146,0.0));
	m_waypoints.push_back(avcWaypointVector(-105.2104381888533,40.06476958373474,0.0));
	m_waypoints.push_back(avcWaypointVector(-105.2104340684065,40.06498350881099,0.0));
	m_waypoints.push_back(avcWaypointVector(-105.2104302362354,40.06514141501155,0.0));
	m_waypoints.push_back(avcWaypointVector(-105.2104177692964,40.06519889301632,90.0));
	m_waypoints.push_back(avcWaypointVector(-105.2100671219435,40.06520567299177,90.0));
	m_waypoints.push_back(avcWaypointVector(-105.2097545474683,40.06519010908657,180.0));
	m_waypoints.push_back(avcWaypointVector(-105.2097391471105,40.06506188953844,180.0));
	m_waypoints.push_back(avcWaypointVector(-105.2097412598431,40.06494888446523,180.0));
	m_waypoints.push_back(avcWaypointVector(-105.2097427813108,40.0647942160145,180.0));
	m_waypoints.push_back(avcWaypointVector(-105.2097402029708,40.0646459956635,180.0));
	m_waypoints.push_back(avcWaypointVector(-105.2097427890488,40.06448581625421,270.0));
	m_waypoints.push_back(avcWaypointVector(-105.2100969789239,40.06449047916893,270.0));
	m_waypoints.push_back(avcWaypointVector(-105.2104311212637,40.06449357185146,0.0));
	
	
}

/////////////////////////////////////////////////////////////////////////////

avcPlanner::~avcPlanner(void) {}

/////////////////////////////////////////////////////////////////////////////

aErr
avcPlanner::init(aIOLib ioRef, aSettingFileRef settings) {
	
	//initialize the logger
	m_logger = logger::logger(settings);
	
	aErr e = aErrNone;
	
	// get the repulse vector weighting from the settings config
	float repulse_weight;
	aSettingFile_GetFloat (ioRef, settings,
						   aKEY_REPULSE_WEIGHT,
						   &repulse_weight,
						   1.0,
						   &e);
	// Copy repulse weight to member variable.
	m_repulseVectorWeight = repulse_weight;
	

	int normalizeMotivationVector = 0;
	aSettingFile_GetInt (ioRef, settings,
						   aKEY_REPULSE_WEIGHT,
						   &normalizeMotivationVector,
						   0,
						   &e);
	// Copy flag to member variable.
	m_normalizeMotivationVector = normalizeMotivationVector;

	return e;
}

/////////////////////////////////////////////////////////////////////////////

avcForceVector 
avcPlanner::getMotivation(const avcStateVector pos,
			  const avcForceVector repulse) {
	
	avcForceVector motivationVector, goal;
	avcWaypointVector nextUnpassedWaypoint;
	
	// check to see if we've passed the current waypoint
	// this could pontentially mark an added bonus waypoint as passed before
	// we attempt to drive to it. This seems to be the right behavior, since we
	// don't want to drive far off course to get to a bonus waypoint
	checkForPassedWayPoints();
	
	
	
	
	try {
		//get the next unpassed waypoint
		nextUnpassedWaypoint = m_waypoints[getFirstUnpassedWayPoint()];
		// calculate a vector between the current position to the next waypoint
		vectorBetweenStates (pos,
							 m_waypoints[getFirstUnpassedWayPoint()].state,
							 &goal);
	}
	catch (int &e) {
		// probably couldn't find any unpassed waypoints. We might have
		// completed the map.
		// set the goal vector to 0,0 so we just look for sensor inputs
		m_logger.append("__class__::__function__ : Error while looking for \
						unpassed waypoints\n\t\t\
						Goal vector will be 0,0.", LogAll);
		goal.x = 0.0;
		goal.y = 0.0;
	}
	
	// here the goal vector should either be a unit vector or 0 length
	
	// sum the goal vector with sensor repluse vector
	// get summing weight from config settings
	motivationVector.x = goal.x + repulse.x;
	motivationVector.y = goal.y + repulse.y;
	
	//may want to re-normalize here (with zero magnitude option)
	if (1 == m_normalizeMotivationVector) {
		normalizeForceVector(&motivationVector);
	}
	
	return motivationVector;
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
		m_waypoints.insert(m_waypoints.begin()+currentWaypointIndex, 1, newWaypoint);
	}
	catch (int &e) {
		// log error for inserting new point
		m_logger.append("__class__::__function__ : Insering new waypoint\
						failed.\n\t\t\
						Adding new waypoint at end of map.", LogAll);
		m_waypoints.push_back(newWaypoint);
		return aErrUnknown;
	}

	return aErrNone;
}

/////////////////////////////////////////////////////////////////////////////

aErr
avcPlanner::checkForPassedWayPoints(void) {
	// compare current position to unpassed waypoints to determine if
	// points should be marked as passed
	
	// TODO: options:
	// a) has bot passed line perpendicular to waypoint heading
	//		(corners should be 45˚ headings)
	// b) ^ or V towards waypoint heading within some distance
	// c) simple distance from waypoint (circle)
	
	return aErrNone;

}

/////////////////////////////////////////////////////////////////////////////

int
avcPlanner::getFirstUnpassedWayPoint(void) {
	for (int i = 0; i < (int) m_waypoints.size(); i++) {
		if(0 == m_waypoints[i].waypointPassed)
			return (int) i;
	}
	
	return -1;
}

/////////////////////////////////////////////////////////////////////////////

aErr
avcPlanner::vectorBetweenStates(const avcStateVector& state1, const avcStateVector& state2, 
					avcForceVector *pGoalForceVec) {
	// compute a goal heading from state1 to state2 in absolute coordinates
	// normally state1 is the current state; state2 is to desired next state
	// resultant vector will be unit length
	// using haversine great circle method
	/* http://www.movable-type.co.uk/scripts/latlong.html
		var y = Math.sin(dLon) * Math.cos(lat2);
		var x = Math.cos(lat1)*Math.sin(lat2) -
		Math.sin(lat1)*Math.cos(lat2)*Math.cos(dLon);
		var brng = Math.atan2(y, x).toDeg();
	 */
	double x=0.0, y=0.0, headingToNextStateRad=0.0, goalHeading=0.0;
	
	// check the force vector pointer validity
	if (!pGoalForceVec) {
		m_logger.append("__class__::__function__ : goal vector is NULL.", LogAll);
		return aErrParam;
	}
	
	y = sin( (state2.x - state1.x) * DEG_TO_RAD ) * cos(state2.y * DEG_TO_RAD);
	x = cos(state1.y * DEG_TO_RAD) * sin(state2.y * DEG_TO_RAD) - 
		sin(state1.y * DEG_TO_RAD) * cos(state2.y * DEG_TO_RAD) *
		cos( (state2.x - state1.x) * DEG_TO_RAD );
	
	headingToNextStateRad = atan2(y, x);

	// the vector difference between the goal heading
	// and the current heading is the direction the "force" should applied
	// in order to reach the next waypoint goal; this goal heading is a
	// a heading relative to the bot's current heading
	goalHeading = headingToNextStateRad - (state1.h * DEG_TO_RAD);
	
	//convert bearing to an x-y force unit vector
	// this conversion to cartesian will eliminate >|pi| headings
	pGoalForceVec->x = cos(goalHeading);
	pGoalForceVec->y = sin(goalHeading);
	
	//normalize the force vector
	normalizeForceVector(pGoalForceVec);
	
	return aErrNone;
	
}

/////////////////////////////////////////////////////////////////////////////

void
avcPlanner::normalizeForceVector(avcForceVector *pForceVector) {
	// normalizes force vector to unit length
	
	//check input parameter
	if (!pForceVector) {
		
		acpString info;
		info.format("%s::%s : force vector is NULL.", __func__, __func__);
		
		m_logger.append((const char *) info,
						LogAll);
		return;
	}
	
	//calculate the vector magnitude
	double mag = sqrt( pow(pForceVector->x, 2) + pow(pForceVector->y, 2) );
	
	//check for near zero vectors and return a 0 length vector
	if (mag < 10e-8) {
		
		acpString info;
		info.format("%s::%s : truncting near zero\
								magnitude vector.", __func__, __func__);
		
		m_logger.append((const char *) info, LogAll);
		pForceVector->x = 0.0;
		pForceVector->y = 0.0;
	}
	else {
		pForceVector->x = pForceVector->x/mag;
		pForceVector->y = pForceVector->y/mag;
	}
}


///////////////////////////////////////////////////////////////////////////
// This section is for isolating and debugging this module. 
#ifdef aDEBUG_PLANNER

////////////////////////////////////////
// main testing routine for logger 
int 
main(int argc, 
     const char* argv[]) 
{
	avcPlanner planner;
	
	planner.normalizeForceVector(NULL);
	
	return 0;
	
}

#endif		
	
	

#include "planner.h"

#define DEG_TO_RAD (aPI/180)
#define RAD_TO_DEG (180/aPI)

avcPlanner::avcPlanner(void) {
		
	// setup some example waypoints to play with
	// this might be better to read from a file (or website) later
	/*
	 //example race course map. This will be read in from a gpx file or something
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
	*/
	
	
}

/////////////////////////////////////////////////////////////////////////////

avcPlanner::~avcPlanner(void) {}

/////////////////////////////////////////////////////////////////////////////

aErr
avcPlanner::init(aIOLib ioRef, aSettingFileRef settings) {
	
	//initialize the logger
	m_logger = logger::getInstance();
	
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
						   aKEY_NORMALIZE_MOTIVATION_VECTOR,
						   &normalizeMotivationVector,
						   1,
						   &e);
	// Copy flag to member variable.
	m_normalizeMotivationVector = normalizeMotivationVector;
	
	float maxUnPassedDistanceToWaypoint = 0;
	aSettingFile_GetFloat (ioRef, settings,
						 aKEY_MAX_UNPASSED_DISTANCE,
						 &maxUnPassedDistanceToWaypoint,
						 10,
						 &e);
	// Copy flag to member variable.
	m_maxUnPassedDistanceToWaypoint = maxUnPassedDistanceToWaypoint;
	
	float minUnPassedDistanceToWaypoint = 0;
	aSettingFile_GetFloat (ioRef, settings,
						 aKEY_MIN_UNPASSED_DISTANCE,
						 &minUnPassedDistanceToWaypoint,
						 1,
						 &e);
	// Copy flag to member variable.
	m_minUnPassedDistanceToWaypoint = minUnPassedDistanceToWaypoint;
	
	float unpassedZetaSliceDeg = 0;
	aSettingFile_GetFloat (ioRef, settings,
						   aKEY_UNPASSED_SLICE_ZETA,
						   &unpassedZetaSliceDeg,
						   90,
						   &e);
	// Copy flag to member variable.
	m_unpassedZetaSliceDeg = unpassedZetaSliceDeg;
	
	return e;
}

/////////////////////////////////////////////////////////////////////////////

avcForceVector 
avcPlanner::getMotivation(const avcStateVector& pos,
			  const avcForceVector& repulse) {
	
	avcForceVector motivationVector, goal;
	avcWaypointVector nextUnpassedWaypoint;
	
	// check to see if we've passed the current waypoint
	// this could pontentially mark an added bonus waypoint as passed before
	// we attempt to drive to it. This seems to be the right behavior, since we
	// don't want to drive far off course to get to a bonus waypoint
	G_CHK_ERR(m_logger, checkForPassedWayPoints(pos), "Checking for passed waypoints");
	
	try {
		//get the next unpassed waypoint
		nextUnpassedWaypoint = m_waypoints[getFirstUnpassedWayPoint()];
	}
	catch (int &e) {
		// probably couldn't find any unpassed waypoints. We might have
		// completed the map.
		// set the goal vector to 0,0 so we just look for sensor inputs
		LOG_ERROR(m_logger, "Error while getting first unpassed waypoint\n\t\t\
				  Next point will be 0,0.");
		nextUnpassedWaypoint.state.x = 0.0;
		nextUnpassedWaypoint.state.y = 0.0;
	}
	
	// calculate a vector between the current position to the next waypoint
	if( aErrNone != calcForceVectorBetweenStates (pos,
								  nextUnpassedWaypoint.state,
									 &goal) )
	{
		// set the goal vector to 0,0 so we just look for sensor inputs
		LOG_ERROR(m_logger, "Error while calculating goal vector\n\t\t\
				  Goal vector will be 0,0.");
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
		LOG_ERROR(m_logger, "Insering new waypoint failed.\n\t\t\
				  Adding new waypoint at end of map.");
		m_waypoints.push_back(newWaypoint);
		return aErrUnknown;
	}

	return aErrNone;
}

/////////////////////////////////////////////////////////////////////////////

aErr
avcPlanner::checkForPassedWayPoints(const avcStateVector& pos) {
	// compare current position to unpassed waypoints to determine if
	// points should be marked as passed
	
	// options:
	// a) has bot passed line perpendicular to waypoint heading
	//		(corners should be 45˚ headings)
	// b) ^ or V towards waypoint heading within some distance
	// c) simple distance from waypoint (circle)
	
	// TODO: option b (^ or V to waypoint)
	// find next unpassed waypoint
	int firstUnpassedWaypoint = getFirstUnpassedWayPoint();
	if (-1 == firstUnpassedWaypoint) {
		// all waypoints have already been marked as passed or the map is empty
		// nothing to see here.
		return aErrNone;
	}
	
	// compute a dimensional vector between the current position and
	// the next waypoint
	double distanceToWaypoint = 0.0, thetaRad = 0.0, thetaDeg=0.0;
	calcPolarVectorBetweenStates(pos, m_waypoints[firstUnpassedWaypoint].state,
							&distanceToWaypoint, &thetaRad);
	// convert to degrees and unwrap
	thetaDeg = unwrapAngleDeg(thetaRad * RAD_TO_DEG);
		
	// if we are within a minimum distance to the waypoint, mark it as passed; recurse
	if (distanceToWaypoint <= m_minUnPassedDistanceToWaypoint) {
		LOG_INFO(m_logger, "Passed waypoint minUnPassedDistance");
		m_waypoints[firstUnpassedWaypoint].waypointPassed = 1;
		checkForPassedWayPoints(pos);
		return aErrNone;
	}
	
	// if we are more than a max distance to the waypoint -> not passed; return
	if (distanceToWaypoint > m_maxUnPassedDistanceToWaypoint) {
		LOG_INFO(m_logger, "Not in donut");
		return aErrNone;
	}
	
	// (we are in the donut of doubt)
	// if the heading to the waypoint is outside ±(180˚-zeta) of waypoint heading -> passed
	double maxTheta=0.0, minTheta=0.0;
	maxTheta = unwrapAngleDeg(m_waypoints[firstUnpassedWaypoint].state.h + 180 +
						   m_unpassedZetaSliceDeg/2.0);
	minTheta = unwrapAngleDeg(m_waypoints[firstUnpassedWaypoint].state.h + 180 - 
						   m_unpassedZetaSliceDeg/2.0);
	if (thetaDeg > maxTheta || thetaDeg < minTheta) {
		LOG_INFO(m_logger, "Passed waypoint in donut outside of slice");
		m_waypoints[firstUnpassedWaypoint].waypointPassed = 1;
		checkForPassedWayPoints(pos);
		return aErrNone;
	}
	// not passed
	else {
		LOG_INFO(m_logger, "In donut slice");
		return aErrNone;
	}
	
	return aErrNone;

}

/////////////////////////////////////////////////////////////////////////////

aErr
avcPlanner::calcPolarVectorBetweenStates(const avcStateVector& state1, 
										 const avcStateVector& state2,
										 double *distanceToWaypoint,
										 double *thetaRad) {

	double x=0.0, y=0.0;
	
	// check pointer arguments
	CHECK_ARG_RETURN(m_logger, distanceToWaypoint);
	CHECK_ARG_RETURN(m_logger, thetaRad);
	
	double dLon = (state2.x - state1.x) * DEG_TO_RAD;
	double dLat = (state2.y - state1.y) * DEG_TO_RAD;
	const double earthRadiusKM = 6371.0;
	
	// calculate the heading from state1 to state2
	/*
	 var y = Math.sin(dLon) * Math.cos(lat2);
	 var x = Math.cos(lat1)*Math.sin(lat2) -
	 Math.sin(lat1)*Math.cos(lat2)*Math.cos(dLon);
	 var brng = Math.atan2(y, x).toDeg();
	 */
	y = sin( dLon) * cos(state2.y * DEG_TO_RAD);
	x = cos(state1.y * DEG_TO_RAD) * sin(state2.y * DEG_TO_RAD) - 
		sin(state1.y * DEG_TO_RAD) * cos(state2.y * DEG_TO_RAD) * cos( dLon );
	*thetaRad = atan2(y, x);
	
	//m_logger.logInfo("dLon, dLat: %.2e, %.2e", dLon, dLat);
	//m_logger.logInfo("x, y: %.2e, %.2e", x, y);
	
	// calculate the distance between state1 and state2
	/*
	var a = Math.sin(dLat/2) * Math.sin(dLat/2) + Math.cos(lat1.toRad()) *
	 Math.cos(lat2.toRad()) * Math.sin(dLon/2) * Math.sin(dLon/2); 
	var c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a)); 
	var d = R * c;
	 */
	double a, c;
	a = sin(dLat/2) * sin(dLat/2) + cos(state1.y * DEG_TO_RAD) *
		cos(state2.y * DEG_TO_RAD) * sin(dLon/2) * sin(dLon/2);
	c = 2.0 * atan2(sqrt(a), sqrt(1-a));
	*distanceToWaypoint = earthRadiusKM * c;
	
	return aErrNone;
}
	


int
avcPlanner::getFirstUnpassedWayPoint(void) {
	for (int i = 0; i < (int) m_waypoints.size(); i++) {
		if(0 == m_waypoints[i].waypointPassed)
			return i;
	}
	
	return -1;
}

/////////////////////////////////////////////////////////////////////////////

aErr
avcPlanner::calcForceVectorBetweenStates(const avcStateVector& state1, const avcStateVector& state2, 
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
	double dist=0.0, headingToNextStateRad=0.0, goalHeading=0.0;
	
	// check the force vector pointer validity
	CHECK_ARG_RETURN(m_logger, pGoalForceVec);
	
	calcPolarVectorBetweenStates(state1, state2, &dist, &headingToNextStateRad);
	
	// the vector difference between the goal heading
	// and the current heading is the direction the "force" should applied
	// in order to reach the next waypoint goal; this goal heading is a
	// a heading relative to the bot's current heading
	goalHeading = headingToNextStateRad - (state1.h * DEG_TO_RAD);
	
	//convert bearing to an x-y force unit vector
	// this conversion to cartesian will eliminate >|pi| headings
	pGoalForceVec->x = cos(goalHeading);
	pGoalForceVec->y = sin(goalHeading);
	
	//normalize the force vector (this shouldn't really be needed here)
	normalizeForceVector(pGoalForceVec);
	
	return aErrNone;
	
}

/////////////////////////////////////////////////////////////////////////////

aErr
avcPlanner::normalizeForceVector(avcForceVector *pForceVector) {
	// normalizes force vector to unit length
	
	//check input parameter
	CHECK_ARG_RETURN(m_logger, pForceVector);
	
	//calculate the vector magnitude
	double mag = sqrt( (pForceVector->x * pForceVector->x) +
					   (pForceVector->y * pForceVector->y) );
		
	//check for near zero vectors and return a 0 length vector
	if (mag < 10e-8) {
		
		LOG_INFO(m_logger, "Truncating near zero magnitude vector.");
		pForceVector->x = 0.0;
		pForceVector->y = 0.0;
	}
	else {
		pForceVector->x = pForceVector->x/mag;
		pForceVector->y = pForceVector->y/mag;
	}
	
	return aErrNone;
}

/////////////////////////////////////////////////////////////////////////////

double
avcPlanner::unwrapAngleDeg(double phi){
	double unwrapped = fmod((phi+360.0), 360.0);
	unwrapped = unwrapped < 0 ? unwrapped+360.0 : unwrapped;
	return unwrapped;
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
	logger* log = logger::getInstance();
	
	log->log(INFO, "\n\nTesting normalizing vectors");
	avcForceVector tempVector1 = avcForceVector(1.1, 1.1);
	log->append(tempVector1, INFO);
	planner.normalizeForceVector(&tempVector1);
	log->append(tempVector1, INFO);
	
	avcForceVector tempVector2 = avcForceVector(1.0e-12, 1.0e-12);
	log->append(tempVector2);
	planner.normalizeForceVector(&tempVector2);
	log->append(tempVector2, INFO);
	
	log->log(INFO, "Testing normalizing NULL force vector");
	planner.normalizeForceVector(NULL);
	
	
	log->log(INFO, "\n\nTesting unwrapAngleDeg()");
	double angles[] = {-500, -270, -100, -90, 0, 70, 90, 135, 180, 200, 270, 300, 359, 360, 500};
	for (unsigned int i=0; i<sizeof(angles)/sizeof(double); i++) {
		log->log(INFO, "%.1f unwrapped is %.1f", angles[i], planner.unwrapAngleDeg(angles[i]));
	}
	
	log->log(INFO, "\n\nTesting calcPolarVectorBetweenStates()");
	double r, theta;
	planner.calcPolarVectorBetweenStates(planner.m_waypoints[0].state, planner.m_waypoints[1].state, &r, &theta);
	log->log(INFO, "R, theta is %.2e, %.5f", r, planner.unwrapAngleDeg(theta * RAD_TO_DEG));
	planner.calcPolarVectorBetweenStates(planner.m_waypoints[3].state, planner.m_waypoints[4].state, &r, &theta);
	log->log(INFO, "R, theta is %.2e, %.5f", r, planner.unwrapAngleDeg(theta * RAD_TO_DEG));
	planner.calcPolarVectorBetweenStates(planner.m_waypoints[4].state, planner.m_waypoints[5].state, &r, &theta);
	log->log(INFO, "R, theta is %.2e, %.5f", r, planner.unwrapAngleDeg(theta * RAD_TO_DEG));


	
	log->log(INFO, "\n\nTesting calcPolarVectorBetweenStates()");
	planner.calcForceVectorBetweenStates(planner.m_waypoints[0].state, planner.m_waypoints[1].state, &tempVector1);
	log->append(tempVector1, INFO);
	planner.calcForceVectorBetweenStates(planner.m_waypoints[3].state, planner.m_waypoints[4].state, &tempVector1);
	log->append(tempVector1, INFO);
	planner.calcForceVectorBetweenStates(planner.m_waypoints[4].state, planner.m_waypoints[5].state, &tempVector1);
	log->append(tempVector1, INFO);

	
	return 0;
	
}

#endif		
	
	

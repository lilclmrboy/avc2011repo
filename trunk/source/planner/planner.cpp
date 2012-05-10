#include "planner.h"

#define DEG_TO_RAD (aPI/180)
#define RAD_TO_DEG (180/aPI)

avcPlanner::avcPlanner(void) {
	m_waypoints.push_back(avcWaypointVector(0.0,0.0,0.0));	
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
	
	//m_waypoints.push_back(avcWaypointVector(1.946912e-06,-2.306507e-08,0.0));
	//m_waypoints.push_back(avcWaypointVector(-1.932234e-06,-5.429704e-08,0.0));
	//m_waypoints.push_back(avcWaypointVector(0.0,0.0,0.0));
	//m_waypoints.push_back(avcWaypointVector(1.636801e-06,0.0,0.0));					  
	
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
						   0,
						   &e);
	// Copy repulse weight to member variable.
	m_repulseVectorWeight = repulse_weight;
	

	int normalizeMotivationVector = 0;
	aSettingFile_GetInt (ioRef, settings,
						   aKEY_NORMALIZE_MOTIVATION_VECTOR,
						   &normalizeMotivationVector,
						   0,
						   &e);
	// Copy flag to member variable.
	m_normalizeMotivationVector = normalizeMotivationVector;
	
	float maxUnPassedDistanceToWaypoint = 0;
	aSettingFile_GetFloat (ioRef, settings,
						 aKEY_MAX_UNPASSED_DISTANCE,
						 &maxUnPassedDistanceToWaypoint,
						 0.5f,
						 &e);
	// Copy flag to member variable.
	m_maxUnPassedDistanceToWaypoint = maxUnPassedDistanceToWaypoint;
	
	float minUnPassedDistanceToWaypoint = 0;
	aSettingFile_GetFloat (ioRef, settings,
						 aKEY_MIN_UNPASSED_DISTANCE,
						 &minUnPassedDistanceToWaypoint,
						 0.5f,
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
 
	//Okay, lets get some waypoints from a file.
	char* pFile;
	aSettingFile_GetString(ioRef, settings,
				aKEY_TRACKFILE,
				&pFile,
				"maps/NStest.track",
				&e);
	
        loadMap(pFile);

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
		m_logger->log(INFO, "At last waypoint. Next point will be current position");
		 
		nextUnpassedWaypoint.state.x = pos.x;
		nextUnpassedWaypoint.state.y = pos.y;

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
	motivationVector.x = (goal.x + (double)m_repulseVectorWeight * repulse.x);
	motivationVector.y = (goal.y + (double)m_repulseVectorWeight * repulse.y);
	
	//limit the motivation vector to ±1
	motivationVector.x = motivationVector.x >  1.0 ?  1.0 : motivationVector.x;
	motivationVector.x = motivationVector.x < -1.0 ? -1.0 : motivationVector.x;
	motivationVector.y = motivationVector.y >  1.0 ?  1.0 : motivationVector.y;
	motivationVector.y = motivationVector.y < -1.0 ? -1.0 : motivationVector.y;
	
	m_logger->log(INFO, "Goal\t%1.3f\t%1.3f\tRepulse\t%1.3f\t%1.3f", goal.x, goal.y, repulse.x, repulse.y);
	
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
	
	try {
		// get the first way point not marked as passed
		int currentWaypointIndex = getFirstUnpassedWayPoint();
		
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
	int firstUnpassedWaypoint =0;
	try {
		firstUnpassedWaypoint = getFirstUnpassedWayPoint();
	}
	catch (int &e) {
		m_logger->log(INFO, "All waypoints in map are passed.");
		firstUnpassedWaypoint = (int)m_waypoints.size()-1;
	}

	if (-1 == firstUnpassedWaypoint) {
		// all waypoints have already been marked as passed or the map is empty
		// nothing to see here.
		return aErrNone;
	}
	
	//m_logger->log(INFO, "curPos: %.8e,%.8e,%e\ttarPos: %.8e,%.8e", pos.x, pos.y, pos.h, m_waypoints[firstUnpassedWaypoint].state.x, m_waypoints[firstUnpassedWaypoint].state.y);
	
	// compute a dimensional vector between the current position and
	// the next waypoint
	double distanceToWaypoint = 0.0, thetaRad = 0.0, thetaDeg=0.0;
	calcPolarVectorBetweenStates(pos, m_waypoints[firstUnpassedWaypoint].state,
							&distanceToWaypoint, &thetaRad);
	// convert to degrees and unwrap
	thetaDeg = unwrapAngleDeg(thetaRad * RAD_TO_DEG);
		
	// if we are within a minimum distance to the waypoint, mark it as passed; recurse
	if (distanceToWaypoint <= m_minUnPassedDistanceToWaypoint) {
		m_logger->log(INFO, "Passed waypoint minUnPassedDistance %d/%d", firstUnpassedWaypoint, (int)m_waypoints.size()-1);
		m_waypoints[firstUnpassedWaypoint].waypointPassed = 1;
		
		//m_logger->log(INFO, "First unpassed waypoint (map size): %d (%d)", firstUnpassedWaypoint, (int)m_waypoints.size()-1);
		
		//if we're not at the last waypoint, recurse to the next waypoint
		if (firstUnpassedWaypoint < (int)m_waypoints.size()-1) {
			checkForPassedWayPoints(pos);
		}
		
		return aErrNone;
	}
	
	// if we are more than a max distance to the waypoint -> not passed; return
	if (distanceToWaypoint > m_maxUnPassedDistanceToWaypoint) {
		m_logger->log(INFO,"Not in donut");
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
		m_logger->log(INFO, "Passed waypoint in donut outside of slice %d/%d", firstUnpassedWaypoint, (int)m_waypoints.size()-1);
		m_waypoints[firstUnpassedWaypoint].waypointPassed = 1;
		
		//m_logger->log(INFO, "First unpassed waypoint (map size): %d (%d)", firstUnpassedWaypoint, (int)m_waypoints.size()-1);
		
		//if we're not at the last waypoint, recurse to the next waypoint
		if (firstUnpassedWaypoint < (int)m_waypoints.size()-1) {
			checkForPassedWayPoints(pos);
		}
		
		return aErrNone;
	}
	
	// not passed
	else {
		m_logger->log(INFO, "In donut slice. Drive FAST!");
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
	*thetaRad = atan2(y, x);// - aPI/2;
	
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
	*distanceToWaypoint = earthRadiusKM * c * 1000;
	
	return aErrNone;
}
	

/////////////////////////////////////////////////////////////////////////////

int
avcPlanner::getFirstUnpassedWayPoint(void) {
	for (int i = 0; i < (int) m_waypoints.size(); i++) {
		if(0 == m_waypoints[i].waypointPassed)
			return i;
	}
	
	throw -1;
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
	
	//m_logger->log(INFO, "curPos: %.8e,%.8e,%e\ttarPos: %.8e,%.8e", state1.x, state1.y, state1.h, state2.x, state2.y);
	
	calcPolarVectorBetweenStates(state1, state2, &dist, &headingToNextStateRad);
	
	// the vector difference between the goal heading
	// and the current heading is the direction the "force" should applied
	// in order to reach the next waypoint goal; this goal heading is a
	// a heading relative to the bot's current heading
	//goalHeading = (state1.h * DEG_TO_RAD) - headingToNextStateRad;
	goalHeading = headingToNextStateRad - (state1.h * DEG_TO_RAD);
	
	//convert bearing to an x-y force unit vector
	// this conversion to cartesian will eliminate >|pi| headings
	pGoalForceVec->x = cos(goalHeading);
	pGoalForceVec->y = sin(goalHeading);
	
	// if the distance to the next point is within the minimum "passed" distance
	// then we must be at the end of the list. scale the force by the distance (or set to 0)
	if (dist < m_maxUnPassedDistanceToWaypoint) {
		//pGoalForceVec->x *= dist/m_maxUnPassedDistanceToWaypoint;
		//pGoalForceVec->y *= dist/m_maxUnPassedDistanceToWaypoint;
    m_logger->log(INFO, "Within maxUnPassedDistanceToWaypoint; must be at end of map. Did we win?");
		pGoalForceVec->x *= 0.5;
		pGoalForceVec->y *= 0.5;
	}
	
	m_logger->log(INFO, "dist: %3.2f\tbearing(deg): %3.2f\theading(deg): %3.2f", dist, unwrapAngleDeg(headingToNextStateRad*RAD_TO_DEG), unwrapAngleDeg(goalHeading*RAD_TO_DEG));
	//m_logger->log(INFO, "goal.x: %e\tgoal.y: %e", pGoalForceVec->x, pGoalForceVec->y);
	

	
	//normalize the force vector (this shouldn't really be needed here)
	//normalizeForceVector(pGoalForceVec);
	// instead, let's scale this in getMotivation()
	
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
		
		m_logger->log(INFO, "Truncating near zero magnitude vector.");
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

/////////////////////////////////////////////////////////////////////////////

void
avcPlanner::loadMap(const char* mapfile) {

  FILE* map;
  int numberOfMapPoints=0;
  map = fopen(mapfile, "r");
  
  if (map == NULL){
    char path[PATH_MAX];
    getcwd(path, PATH_MAX);
    m_logger->log(ERROR, "Map file not found: %s/%s", path, mapfile);
  }
    
  if (map != NULL)  {
    //empty the current map vector.
    m_waypoints.clear();
    char pointstr[256];
    while(fgets(pointstr, 256, map)) {
      double x,y,h;
      x = y = h = 0.0;
      sscanf(pointstr, "%le, %le, %le", &x, &y, &h);
      m_logger->log(INFO, "adding waypoint(%le, %le, %le) to map.", x, y, h);
      m_waypoints.push_back(avcWaypointVector(x, y, h));
      numberOfMapPoints++;
    }
    //make sure there is at least one waypoint
    if (!m_waypoints.size()) {
      m_waypoints.push_back(avcWaypointVector(0.0,0.0,0.0));
    }
  } else {
    m_waypoints.push_back(avcWaypointVector(0.0,0.0,0.0));
  }
  
  m_logger->log(INFO, "Added %d points to the map", numberOfMapPoints);
  
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

  aSettingFileRef settings;
	aErr e = aErrNone;
	aIOLib ioRef;
	
	// Grab an aIO reference object to gain the setting to talk to the stem.
	aIO_GetLibRef(&ioRef, &e);
	
	// Read from a settings file if it exists.
	if (aSettingFile_Create(ioRef, 
													"console.config",
													&settings,
													&e))
		throw acpException(e, "creating settings");

	planner.init(ioRef, settings);

	log->log(INFO, "\n\nTesting normalizing vectors");
	avcForceVector tempVector1 = avcForceVector(1.1, 1.1);
	log->append(tempVector1, INFO);
	planner.normalizeForceVector(&tempVector1);
	log->append(tempVector1, INFO);
	planner.normalizeForceVector(&tempVector1);
	log->append(tempVector1, INFO);
	
	avcForceVector tempVector2 = avcForceVector(1.0e-12, 1.0e-12);
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
  log->log(INFO, "Loading acroSquare track");
  planner.loadMap("maps/acroSquare.track");
  log->log(INFO, "Adding dummy last point to map");
  planner.m_waypoints.push_back(planner.m_waypoints[4].state);
	double r, theta;
	planner.calcPolarVectorBetweenStates(planner.m_waypoints[0].state, planner.m_waypoints[1].state, &r, &theta);
	log->log(INFO, "From 0 to 1, R, theta is %.2e, %.5f", r, planner.unwrapAngleDeg(theta * RAD_TO_DEG));
  planner.calcPolarVectorBetweenStates(planner.m_waypoints[1].state, planner.m_waypoints[2].state, &r, &theta);
	log->log(INFO, "From 1 to 2, R, theta is %.2e, %.5f", r, planner.unwrapAngleDeg(theta * RAD_TO_DEG));
  planner.calcPolarVectorBetweenStates(planner.m_waypoints[2].state, planner.m_waypoints[3].state, &r, &theta);
	log->log(INFO, "From 2 to 3, R, theta is %.2e, %.5f", r, planner.unwrapAngleDeg(theta * RAD_TO_DEG));
	planner.calcPolarVectorBetweenStates(planner.m_waypoints[3].state, planner.m_waypoints[4].state, &r, &theta);
	log->log(INFO, "From 3 to 4, R, theta is %.2e, %.5f", r, planner.unwrapAngleDeg(theta * RAD_TO_DEG));
	planner.calcPolarVectorBetweenStates(planner.m_waypoints[4].state, planner.m_waypoints[5].state, &r, &theta);
	log->log(INFO, "From 4 to 5, R, theta is %.2e, %.5f", r, planner.unwrapAngleDeg(theta * RAD_TO_DEG));


	
	log->log(INFO, "\n\nTesting calcPolarVectorBetweenStates()");
	planner.calcForceVectorBetweenStates(planner.m_waypoints[0].state, planner.m_waypoints[1].state, &tempVector1);
	log->append(tempVector1, INFO);
  planner.calcForceVectorBetweenStates(planner.m_waypoints[1].state, planner.m_waypoints[2].state, &tempVector1);
  log->append(tempVector1, INFO);
  planner.calcForceVectorBetweenStates(planner.m_waypoints[2].state, planner.m_waypoints[3].state, &tempVector1);
  log->append(tempVector1, INFO);
  planner.calcForceVectorBetweenStates(planner.m_waypoints[3].state, planner.m_waypoints[4].state, &tempVector1);
	log->append(tempVector1, INFO);
	planner.calcForceVectorBetweenStates(planner.m_waypoints[4].state, planner.m_waypoints[5].state, &tempVector1);
	log->append(tempVector1, INFO);

	
	log->log(INFO, "\n\nTest planner with N-S state vectors and one waypoint");
  log->log(INFO, "Loading NS track");
  planner.loadMap("maps/NStest.track");
	log->log(INFO, "Robot will be pushed north while facing north");
	planner.getMotivation(avcStateVector(-105.241227814428,40.02664202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241227814428,40.02665202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241227814428,40.02666202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241227814428,40.02667202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241227814428,40.02668202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241227814428,40.02669202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241227814428,40.02670202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241227814428,40.02671202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241227814428,40.02672202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241227814428,40.02673202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241227814428,40.02674202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241227814428,40.02675202181311,0), avcForceVector(0.0,0.0));
	log->log(INFO, "\n\nRobot will be pushed south while facing north");
	planner.getMotivation(avcStateVector(-105.241227814428,40.02674202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241227814428,40.02673202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241227814428,40.02672202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241227814428,40.02671202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241227814428,40.02670202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241227814428,40.02669202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241227814428,40.02668202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241227814428,40.02667202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241227814428,40.02666202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241227814428,40.02665202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241227814428,40.02664202181311,0), avcForceVector(0.0,0.0));
	
	
	log->log(INFO, "\n\n");
  log->log(INFO, "Creating dummy 1 point map");
  planner.m_waypoints.clear();
  planner.m_waypoints.push_back(avcStateVector(-105.241227814428,40.02664202181311,0));
  log->log(INFO, "Robot try to drive northeast while facing north (faking motion to the SW)");
	//planner.getMotivation(avcStateVector(-105.241227814428,40.02664202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241237814428,40.02663202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241247814428,40.02662202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241257814428,40.02661202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241267814428,40.02660202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241277814428,40.02659202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241287814428,40.02658202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241297814428,40.02657202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241307814428,40.02656202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241317814428,40.02655202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241327814428,40.02654202181311,0), avcForceVector(0.0,0.0));
	
	planner.getMotivation(avcStateVector(-105.241317814428,40.02655202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241307814428,40.02656202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241297814428,40.02657202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241287814428,40.02658202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241277814428,40.02659202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241267814428,40.02660202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241257814428,40.02661202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241247814428,40.02662202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241237814428,40.02663202181311,0), avcForceVector(0.0,0.0));
	planner.getMotivation(avcStateVector(-105.241227814428,40.02664202181311,0), avcForceVector(0.0,0.0));

	return 0;
	
}

#endif		
	
	

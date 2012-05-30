/////////////////////////////////////////////////////////////////////////////
// File: avcStructs.h
// This file contain type definitions used in multiple modules, such as
// component vectors, state vectors, and the like.
//
#ifndef _avcStructs_H_
#define _avcStructs_H_

using std::vector;

/////////////////////////////////////////////////////////////////////////////
// This holds the setpoint update.
typedef struct avcControlVector{
	short rSetPoint;
	short lSetPoint;
	avcControlVector(void) : rSetPoint(0), lSetPoint(0) {}
	avcControlVector(short r, short l) : rSetPoint(r), lSetPoint(l) {}
} avcControlVector;

/////////////////////////////////////////////////////////////////////////////

typedef struct avcForceVector {
  double x;
  double y;
	avcForceVector(void) : x(0.0), y(0.0) {}
	avcForceVector(double ux, double uy) : x(ux), y(uy) {}
} avcForceVector;

/////////////////////////////////////////////////////////////////////////////
// The state vector is a robot global position, heading and speed	
typedef struct avcStateVector {

	//Longitude
	double x;
	//latitude
	double y;
	//heading degrees from magnetic north.
	double h;
	//velocity in x direction
	double vx;
	//velocity in y direction
	double vy;
	//angular velocity.
	double vw;

	avcStateVector(void) :
		x(0.0),
		y(0.0),
		h(0.0),
		vx(0.0),
		vy(0.0),
		vw(0.0)
  {}
	
	avcStateVector(double lo, double la, 
                 double he, double sx = 0.0,
								 double sy = 0.0, double sw = 0.0) : 
		x(lo),
		y(la),
		h(he),
		vx(sx),
		vy(sy),
	  vw(sw)
  {}	

} avcStateVector;


typedef struct avcWaypointVector {
	avcStateVector state;
	int waypointPassed;

	avcWaypointVector (void) {
		waypointPassed=0;
	}
	avcWaypointVector ( avcStateVector s ) {
		state = s;
		waypointPassed=0;
	}
	avcWaypointVector ( double newx, double newy, double newh ) {
		state.x=newx;
		state.y=newy;
		state.h=newh;
		waypointPassed=0;
	}
} avcWaypointVector;

#endif //_avcStructs_H_

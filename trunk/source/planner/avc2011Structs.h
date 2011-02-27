/////////////////////////////////////////////////////////////////////////////
// File: avc2011Structs.h
// This file contain type definitions used in multiple modules, such as
// component vectors, state vectors, and the like.
//
#ifndef _avc2011Structs_H_
#define _avc2011Structs_H_
#include "aStem.h"
#include <vector>

#ifdef aDEBUG_H
	#define aDEBUG_PRINT(arg) printf(arg);fflush(stdout)
#else 
	#define aDEBUG_PRINT(arg)
#endif

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
	//speed
	double s;

	avcStateVector(void) :
		x(0.0),
		y(0.0),
		h(0.0),
		s(0.0)
  {}
	
	avcStateVector(double la, double lo, 
                 double he, double spd = 0.0) : 
		x(lo),
		y(la),
		h(he),
		s(spd)
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

#endif //_avc2011Structs_H_

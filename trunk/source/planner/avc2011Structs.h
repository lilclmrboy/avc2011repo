/////////////////////////////////////////////////////////////////////////////
// File: avc2011Structs.h
// This file contain type definitions used in multiple modules, such as
// component vectors, state vectors, and the like.
//
#ifndef _avc2011Structs_H_
#define _avc2011Structs_H_
#include "aStem.h"
#include <vector>

using std::vector;

typedef struct avcForceVector {
  double x;
  double y;
} avcForceVector;

typedef struct avcStateVector {
  double x;
  double y;
  double h; //heading
  avcStateVector(void) : x(0.0), y(0.0), h(0.0) {}
  avcStateVector(double newx, double newy, double newh) {
	  x=newx;
	  y=newy;
	  h=newh;
  }
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

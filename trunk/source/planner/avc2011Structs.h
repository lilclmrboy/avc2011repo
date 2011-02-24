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
  short x;
  short y;
} avcForceVector;

typedef struct avcStateVector {
  double x;
  double y;
  double h; //heading
} avcStateVector;

typedef struct avcControlUpdate {
  int setpointr;
  int setpointl;
} avcControlUpdate;

#endif //_avc2011Structs_H_

/////////////////////////////////////////////////////////////////////////////
// File: avc2011Structs.h
// This file contain type definitions used in multiple modules, such as
// component vectors, state vectors, and the like.
//
typedef struct avcForceVector {
  short x;
  short y;
} avcForceVector;

typedef struct avcStateVector {
  double x;
  double y;
  double h; //heading
} avcStateVector;

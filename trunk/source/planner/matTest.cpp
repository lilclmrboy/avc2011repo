/*
 *  matTest.cpp
 *  macx_ekfSlam
 *
 *  Created by Jeremiah Sullenger on 11/1/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */
#include "mat_math.h"
#include <cassert>
#include <stdio.h>

int main(int argc, char *argv[])
{
  
  
  Matrix a(3,3);
  assert(a.numCols() == 3 && a.numRows() == 3);
  
  assert(a(1,1) == 0 && a(1,2) == 0 && a(1,3) == 0 &&
         a(2,1) == 0 && a(2,2) == 0 && a(2,3) == 0 &&
         a(3,1) == 0 && a(3,2) == 0 && a(3,3) == 0);
  
  a(1,1) = 1; a(1,2) = 2; a(1,3) = 0;
  a(2,1) = -1; a(2,2) = 1; a(2,3) = 1;
  a(3,1) = 1; a(3,2) = 2; a(3,3) = 3;
  
  assert(a(1,1) == 1 && a(1,2) == 2 && a(1,3) == 0 &&
         a(2,1) == -1 && a(2,2) == 1 && a(2,3) == 1 &&
         a(3,1) == 1 && a(3,2) == 2 && a(3,3) == 3);
  
 
  Matrix cofacta(3,3);
  
  cofacta(1,1) = 1; cofacta(1,2) = 4; cofacta(1,3) = -3;
  cofacta(2,1) = -6; cofacta(2,2) = 3; cofacta(2,3) = -0;
  cofacta(3,1) = 2; cofacta(3,2) = -1; cofacta(3,3) = 3;
  
  
  Matrix transCof(3,3);
  
  transCof(1,1) = 1; transCof(1,2) = -6; transCof(1,3) = 2;
  transCof(2,1) = 4; transCof(2,2) = 3; transCof(2,3) = -1;
  transCof(3,1) = -3; transCof(3,2) = -0; transCof(3,3) = 3;
  
  Matrix b(1,1);
  assert(b.numCols() == 1 && b.numRows() == 1);
  
  
  bool exept = false;
  try {
    Matrix c(0,0);
  }
  catch (ArrayOutOfBounds err) {
    exept = true;
  }
  assert(exept);
  
  
  b(1,1) = 3;
  assert(b(1,1) == 3);
  
  assert(9 == Determinant(a, 3));
  assert(3 == Determinant(b, 1));
  
  b = CoFactor(a, 3);
  
  assert(b.numCols() == 3 && b.numRows() == 3);
  assert(b == cofacta);
  
  Matrix d(6,6);
  
  d = b.transpose();
  
  assert(d.numCols() == 3 && d.numRows() == 3);
  assert(d == transCof);
  
  d.resize(6, 6);
  assert(d.numCols() == 6 && d.numRows() == 6);
  assert(d(6,6) == 0);
  
  d.setElements(2);
  assert(d(1,3) == 2 && d(4,5) == 2);
  
  Matrix e(2,2);
  
  e(1,1) = 1; e(1,2) = 1;
  e(2,1) = 1; e(2,2) = 1;
  
  Matrix f(2,2);
  
  f(1,1) = 2; f(1,2) = 2;
  f(2,1) = 2; f(2,2) = 2;
  
  Matrix g(2,2);
  
  g = e + e;
  assert(g == f);
  
  g = f - e;
  assert(g == e);
  
  g = e * 2;
  assert(g == f);

  e.resize(2, 3);
  e(1,1) = 1; e(1,2) = 1; e(1,3) = 1;
  e(2,1) = 1; e(2,2) = 1; e(2,3) = 1;
  
  f.resize(3,2);
  f = e.transpose();
  assert(e.numCols() == 3 && e.numRows() == 2);
  assert(f.numCols() == 2 && f.numRows() == 3);
  
  g(1,1) = 3; g(1,2) = 3;
  g(2,1) = 3; g(2,2) = 3;
  
  e = (e * f);  
  assert(e.numCols() == 2 && e.numRows() == 2);
  assert(e == g);
  
  exept = false;
  try {
    g = (f * f);
  }
  catch (ArrayOutOfBounds err) {
    exept = true;
  }
  assert(exept);
  
  exept = false;
  try {
    g = f(6,1);
  }
  catch (ArrayOutOfBounds err) {
    exept = true;
  }
  assert(exept);
  
  exept = false;
  try {
    g = f(1,6);
  }
  catch (ArrayOutOfBounds err) {
    exept = true;
  }
  assert(exept);
  
  exept = false;
  try {
    g = f.invert();
  }
  catch (NotInvertableError err) {
    exept = true;
  }
  assert(exept);
  
  g = a.subMatrix(1, 2, 1, 2);
  assert(g.numCols() == 2 && g.numRows() == 2);
  
  assert(g(1,1) == 1 && g(2,1) == -1);
  
  d = a.invert();
  printMatrix(d);
  
  Vector h(3), i(3);
  
  h(1) = h(2) = h(3) = 1;
  i(1) = i(2) = i(3) = 2;
  
  Matrix l(2,3);
  
  
  l.push(g);
  l.push(g);
  printMatrix(l);
  
  printMatrix(d * h);
  
  assert( (h + h) == i );
  assert( (i - h) == h );
  assert( (h * 2) == i );
  assert( (2 * h) == i );
  
  assert( Vector(h).subVector(1,2) == Vector(h).resize(2) );
}

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
  
	//Create a 3X3 matrix.
  Matrix a(3,3);
	//assert that it has three rows and three columns.
  assert(a.numCols() == 3 && a.numRows() == 3);
	
	//Tests for a 1x1 matrix.
	Matrix b(1,1);
  assert(b.numCols() == 1 && b.numRows() == 1);
	
	b(1,1) = 3;
  assert(b(1,1) == 3);

	//Insert into a non existant row.
	bool exept = false;
  try {
    Matrix b(2,1)= 3;
  }
  catch (ArrayOutOfBounds err) {
    exept = true;
  }
  assert(exept);

	//Insert into a non existant column.
	exept = false;
  try {
    Matrix b(1,2)= 3;
  }
  catch (ArrayOutOfBounds err) {
    exept = true;
  }
  assert(exept);

	//Creating a matrix with 0 rows and columns should
	//trigger an error.
  exept = false;
  try {
    Matrix c(0,0);
  }
  catch (ArrayOutOfBounds err) {
    exept = true;
  }
  assert(exept);

	//Assert that the rows and columns have been set 0
  assert(a(1,1) == 0 && a(1,2) == 0 && a(1,3) == 0 &&
         a(2,1) == 0 && a(2,2) == 0 && a(2,3) == 0 &&
         a(3,1) == 0 && a(3,2) == 0 && a(3,3) == 0);
  
	//Now lets put some values in the matrix.
  a(1,1) = 1; a(1,2) = 2; a(1,3) = 0;
  a(2,1) = -1; a(2,2) = 1; a(2,3) = 1;
  a(3,1) = 1; a(3,2) = 2; a(3,3) = 3;
  
	//Assert that the matrix now holds the values.
  assert(a(1,1) == 1 && a(1,2) == 2 && a(1,3) == 0 &&
         a(2,1) == -1 && a(2,2) == 1 && a(2,3) == 1 &&
         a(3,1) == 1 && a(3,2) == 2 && a(3,3) == 3);
  
 
	//Create and fill matrix to test the cofactor op.
  Matrix cofacta(3,3);
  cofacta(1,1) = 1; cofacta(1,2) = 4; cofacta(1,3) = -3;
  cofacta(2,1) = -6; cofacta(2,2) = 3; cofacta(2,3) = -0;
  cofacta(3,1) = 2; cofacta(3,2) = -1; cofacta(3,3) = 3;
  
  //Create and fill a matrix to test the transpose op.
  Matrix transCof(3,3);
  transCof(1,1) = 1; transCof(1,2) = -6; transCof(1,3) = 2;
  transCof(2,1) = 4; transCof(2,2) = 3; transCof(2,3) = -1;
  transCof(3,1) = -3; transCof(3,2) = -0; transCof(3,3) = 3;
  
  //Test the Determinant for two 3x3 matrices.  
  assert(9 == Determinant(a, 3));
  assert(3 == Determinant(b, 1));
  
	//Generate the cofactor matrix from a.
  b = CoFactor(a, 3);
  
	//Test the outcome of the CoFactor funtion with the cofacror of
	//a.
  assert(b.numCols() == 3 && b.numRows() == 3);
  assert(b == cofacta);
  
	//test some resizing ops.
  Matrix d(6,6);
  
	//Transpose b into d.
  d = b.transpose();
  
	//Test that the rezise to 3X3 worked and that it contains the
	//transoposition of the cofactor.
  assert(d.numCols() == 3 && d.numRows() == 3);
  assert(d == transCof);
  
	//Resize the matrix and test that it zeroFilled.
  d.resize(6, 6);
  assert(d.numCols() == 6 && d.numRows() == 6);
  assert(d(6,6) == 0);
  
	//Test that setting elements to a scalar.
  d.setElements(2);
  assert(d(1,3) == 2 && d(4,5) == 2);
  
	//Lets do some additiona and subtraction.
  Matrix e(2,2);
  
  e(1,1) = 1; e(1,2) = 1;
  e(2,1) = 1; e(2,2) = 1;
  
  Matrix f(2,2);
  
  f(1,1) = 2; f(1,2) = 2;
  f(2,1) = 2; f(2,2) = 2;
  
  Matrix g(2,2);
  
	//add
  g = e + e;
  assert(g == f);
  
  g = f - e;
  assert(g == e);
  
  g = e * 2;
  assert(g == f);

  e.resize(2, 3);
  e(1,1) = 1; e(1,2) = 1; e(1,3) = 1;
  e(2,1) = 1; e(2,2) = 1; e(2,3) = 1;
  
  //f.resize(3,2);
  f = e.transpose();
  assert(e.numCols() == 3 && e.numRows() == 2);
  assert(f.numCols() == 2 && f.numRows() == 3);
  
  g(1,1) = 3; g(1,2) = 3;
  g(2,1) = 3; g(2,2) = 3;
  
	//Test array multiplication.
  e = (e * f);  
  assert(e.numCols() == 2 && e.numRows() == 2);
  assert(e == g);
  
	//Test error non-compatible array multiplication.
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

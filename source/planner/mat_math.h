/*
 *  matrix.h
 *  macx_ekfSlam
 *
 *  Created by Jeremiah Sullenger on 10/31/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */
#ifndef _mat_math_h
#define _mat_math_h
#include <stdexcept>

class Matrix {

public:
  
  ~Matrix() { if(m_data != NULL) delete[] m_data; }
  Matrix(): m_rows(0), m_cols(0), m_data(NULL){}
  Matrix(const Matrix& m);
  Matrix(const unsigned int row, const unsigned int col);
  
  const unsigned int numRows() const { return m_rows; }
  const unsigned int numCols() const { return m_cols; }
  
  Matrix resize(const unsigned int row, const unsigned int col);
  
  Matrix push(const Matrix& m);
  
  double& element(const unsigned int row, const unsigned int col);
  Matrix subMatrix(const unsigned int fr, 
                    const unsigned int lr, 
                    const unsigned int fc, 
                    const unsigned int lc);
  
  Matrix& setElements(const double a);
  
  Matrix transpose();
  
  Matrix invert();
  
  Matrix& operator=(const Matrix& m);
  Matrix& operator=(const double a);
  
  double& operator() (const unsigned int row, const unsigned int col);
  double  operator() (const unsigned int row, const unsigned int col) const;
  
  Matrix& operator+=(const Matrix& m);
  Matrix& operator+=(const double a);
  Matrix& operator-=(const Matrix& m);
  Matrix& operator-=(const double a);
  Matrix& operator*=(const Matrix& m);
  Matrix& operator*=(const double a);
  
  // friend operators that create new Matrix objects
  friend double Determinant(const Matrix& a, const unsigned int order);
  friend Matrix CoFactor(const Matrix& mat, const unsigned int order);
  friend Matrix operator+(const Matrix &m1, const Matrix& m2);
  friend Matrix operator+(const Matrix &m1, const double a);
  friend Matrix operator+(const double a, const Matrix &m1);
  friend Matrix operator-(const Matrix &m1, const Matrix& m2);
  friend Matrix operator-(const Matrix &m1, const double a);
  friend Matrix operator-(const double a, const Matrix &m1);
  friend Matrix operator*(const Matrix &m1, const Matrix &m2);
  friend Matrix operator*(const Matrix &m1, const double a);
  friend Matrix operator*(const double a, const Matrix &m1);
  
  //element-wise comparison opoerators.
  friend bool operator==(const Matrix &m1, const Matrix &m2);
  friend bool operator!=(const Matrix &m1, 
                         const Matrix &m2) { return !(m1 == m2); }
  
protected:
  
  unsigned int m_rows, m_cols;
  double* m_data;
  
};

class Vector : public Matrix {
public:
  
  ~Vector() {}
  Vector(): Matrix() {}
  Vector(const Vector& v);
  Vector(const unsigned int row);
  
  Vector resize(const unsigned int row);
  
  Vector push(const Vector& v); 
  
  double& element(const unsigned int row);
  Vector subVector(const unsigned int fr, const unsigned int lr);
  
  const unsigned int size() { return Matrix::numRows(); }
  
  Vector gaussian(const double mean, const Matrix& cov);
  
  double& operator() (const unsigned int row);
  double  operator() (const unsigned int row) const;
  
  // Dot (inner) product (vector * vector = scalar)
  //friend double operator*(const Vector &v1, const Vector& v2);
  
};

void printMatrix(const Matrix& mat);

class ArrayOutOfBounds : public std::runtime_error {
public:
  ArrayOutOfBounds() : std::runtime_error("Array out of Bounds Error.") { }
};

class NotInvertableError : public std::runtime_error {
public:
  NotInvertableError() : std::runtime_error("The array is not nxn.") { }
};

#endif

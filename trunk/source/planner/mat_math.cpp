/*
 *  matrix.cpp
 *  macx_ekfSlam
 *
 *  Created by Jeremiah Sullenger on 10/31/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#include "mat_math.h"
#include "string.h"
#include "math.h"

Matrix::Matrix(const Matrix& m) {
  m_rows = m.m_rows;
  m_cols = m.m_cols;
  
  m_data = new double[m_rows * m_cols];
  memcpy(m_data, m.m_data, m_rows * m_cols* sizeof(double));
  
}

/////////////////////////////////////////////////////////////////////

Matrix::Matrix(const unsigned int row, const unsigned int col) {
  m_rows = row;
  m_cols = col;
  if (row == 0 || col == 0) 
    throw ArrayOutOfBounds();
  
  m_data = new double[m_rows * m_cols];
  memset(m_data, 0, row * col * sizeof(double));
}

/////////////////////////////////////////////////////////////////////

Matrix 
Matrix::resize(const unsigned int row, const unsigned int col) {
  
  if (row == 0 || col == 0) {
    delete[] m_data;
    m_data = NULL;
    m_rows = m_cols = 0;
  }
  
  double *tmp = new double[row * col];
  memset(tmp, 0, row * col * sizeof(double));
  
  for (unsigned int i = 0; (i < m_rows && i < row); i++) {
    for (unsigned int j = 0; (j < m_cols && j< col); j++) {
      
      tmp[col*i + j] = m_data[m_cols*i + j];
    }
  }
  delete[] m_data;
  m_data = tmp;
  m_rows = row;
  m_cols = col;
  
  return *this; 
}

/////////////////////////////////////////////////////////////////////

Matrix
Matrix::push(const Matrix& m) {
  
  unsigned int lastR = m_rows;
  unsigned int lastC = m_cols;
  
  resize(m_rows + m.m_rows, m_cols + m.m_cols);
  
  for (unsigned int i= 1; i <= m.m_rows; ++i) {
   for (unsigned int j= 1; j <= m.m_cols; ++j) {
     this->element(i+lastR ,j+lastC) = m(i, j);
   } 
  }
  return *this;
}

/////////////////////////////////////////////////////////////////////

double& 
Matrix::element(const unsigned int row, const unsigned int col) {
  
  if (row-1 >= m_rows || col-1 >= m_cols || row == 0 || col == 0)
    throw ArrayOutOfBounds();
  
  return m_data[m_cols*(row-1) + (col-1)];
}

/////////////////////////////////////////////////////////////////////

Matrix 
Matrix::subMatrix(const unsigned int fr, 
                  const unsigned int lr, 
                  const unsigned int fc, 
                  const unsigned int lc) 
{
  
  if ((lr - fr) < 0 || (lc - fc) < 0) 
    throw ArrayOutOfBounds();
  
  unsigned int nrows = (lr - fr) + 1;
  unsigned int ncols = (lc - fc) + 1;
  
  Matrix tmp(nrows, ncols);
  
  int ii = 1;
  for (unsigned int i = 0; i < nrows; i++) {
    int jj = 1;
    for (unsigned int j = 0; j < ncols; j++) {
      tmp(ii,jj) = element(i+(fr), j+(fc));
      ++jj;
    }
    ++ii;
  }

  return tmp;
}

/////////////////////////////////////////////////////////////////////

Matrix& 
Matrix::setElements(double a) {
  for (unsigned int i=0; i< (m_rows * m_cols); i++) {
    m_data[i] = a; 
  }
  return *this;
}

Matrix 
Matrix::transpose() {
  
  Matrix tmp(m_cols,m_rows);
  
  for (unsigned int i=1; i<= m_rows; i++) {
    for (unsigned int j=1; j<= m_cols; j++) {
      tmp(j,i) = element(i, j);
    }
  }
  
  return tmp;
}

/////////////////////////////////////////////////////////////////////

Matrix 
Matrix::invert() {
  if (m_rows != m_cols) throw NotInvertableError();
  
  double det = Determinant(*this, m_rows);
  
  if (det == 0.0) throw NotInvertableError();
  
  Matrix adj = (CoFactor(*this, m_rows)).transpose();
  
  return adj * (1.0 / det);
}

/////////////////////////////////////////////////////////////////////

Matrix& 
Matrix::operator=(const Matrix& m) {
  
  if (this == &m) return *this;
  m_rows = m.m_rows;
  m_cols = m.m_cols;
  delete[] m_data;
  m_data = new double[m_rows * m_cols];
  memcpy(m_data, m.m_data, m_rows * m_cols * sizeof(double));
  
  return *this;
}

/////////////////////////////////////////////////////////////////////

Matrix& 
Matrix::operator=(const double a) {
  return setElements(a);
}

/////////////////////////////////////////////////////////////////////

double& 
Matrix::operator() (const unsigned int row, const unsigned int col) {
  return element(row, col);
}

/////////////////////////////////////////////////////////////////////

double  
Matrix::operator() (const unsigned int row, const unsigned int col) const {
  if (row-1 >= m_rows || col-1 >= m_cols || row == 0 || col == 0)
    throw ArrayOutOfBounds();
  
  return m_data[m_cols*(row-1) + (col-1)];
}

/////////////////////////////////////////////////////////////////////

Matrix& 
Matrix::operator+=(const Matrix& m) {
  if (m.m_rows != m_rows || m.m_cols != m_cols) throw ArrayOutOfBounds();
  
  for (unsigned int i=0; i < m_cols * m_rows; i++) {
    m_data[i] = m_data[i] + m.m_data[i];
  }
  return *this;
}

/////////////////////////////////////////////////////////////////////

Matrix& 
Matrix::operator+=(double a) {
  for (unsigned int i=0; i < m_cols * m_rows; i++) {
    m_data[i] = m_data[i] + a;
  }
  return *this;
}

/////////////////////////////////////////////////////////////////////

Matrix& 
Matrix::operator-=(const Matrix& m) {
  if (m.m_rows != m_rows || m.m_cols != m_cols) throw ArrayOutOfBounds();
  
  for (unsigned int i=0; i < m_cols * m_rows; i++) {
    m_data[i] = m_data[i] - m.m_data[i];
  }
  return *this;
}

/////////////////////////////////////////////////////////////////////

Matrix& 
Matrix::operator-=(double a) {
  
  return operator+=(-a);
}

/////////////////////////////////////////////////////////////////////

Matrix&
Matrix::operator*=(const Matrix& m) {
  
  if (m_cols != m.m_rows) throw ArrayOutOfBounds();
  Matrix tmp(m_rows, m.m_cols);
  
  for (unsigned int i=1; i<= m_rows; i++) {
    for (unsigned int j=1; j<= m.m_cols; j++) {
      tmp(i,j) = 0;
      for (unsigned int k=1; k<= m_cols; k++) {
        tmp(i,j) = tmp(i,j) + (element(i,k) * m(k,j));
      }
    } 
  }
  
  
  m_rows = tmp.m_rows;
  m_cols = tmp.m_cols;
  delete[] m_data;
  m_data = new double[m_rows * m_cols];
  memcpy(m_data, tmp.m_data, m_rows * m_cols * sizeof(double));
  
  return *this;
}

/////////////////////////////////////////////////////////////////////

Matrix& 
Matrix::operator*=(double a) {
  for (unsigned int i=0; i < m_cols * m_rows; i++) {
    m_data[i] = m_data[i] * a;
  }
  return *this;
}

//PRIVATE functions in Matrix ////////////////////////////////////////

/*
 Recursive definition of determinate using expansion by minors.
 http://local.wasp.uwa.edu.au/~pbourke/miscellaneous/determinant/
 */
double 
Determinant(const Matrix& mat, const unsigned int order) {
  
  if (mat.m_rows != mat.m_cols) throw NotInvertableError();
  
  unsigned int i,j,j1,j2;
  double det = 0.0;
  Matrix *t_mat = NULL;
  
  if (order < 1 || mat.m_rows != mat.m_cols) { /* Error */
    throw NotInvertableError();
    
  } else if (order == 1) { /* Shouldn't get used */
    det = mat(1,1);
    
  } else if (order == 2) {
    det = mat(1,1) * mat(2,2) - mat(2,1) * mat(1,2);
    
  } else {
    
    det = 0.0;
    
    for (j1=1; j1<= order; j1++) {
      t_mat = new Matrix(order-1, order-1);
      
      for (i=2;i<= order; i++) {
        j2 = 1;
        
        for (j=1; j<= order; j++) {
          if (j == j1)
            continue;
          (*t_mat)(i-1,j2) = mat(i,j);
          j2++;
        }
      }
      det += pow(-1.0, j1+1.0) * mat(1, j1) * Determinant(*t_mat, order-1);
    }
  }
  return(det); 
}

/////////////////////////////////////////////////////////////////////
/*
 Find the cofactor matrix of a square matrix
 http://local.wasp.uwa.edu.au/~pbourke/miscellaneous/determinant/
 */
Matrix 
CoFactor(const Matrix& mat,const unsigned int order)
{
  
  if (mat.m_rows != mat.m_cols) throw NotInvertableError();
  
  unsigned int i,j,ii,jj,i1,j1;
  double det;
  Matrix *c;
  Matrix ret(order, order);
  
  
  c = new Matrix(order-1,order-1);
  
  for (j=1; j <= order; j++) {
    for (i=1; i <= order; i++) {
      
      /* Form the adjoint a_ij */
      i1 = 1;      
      for (ii=1; ii <= order; ii++) {
        if (ii == i)
          continue;
        j1 = 1;
        for (jj=1; jj <= order; jj++) {
          if (jj == j)
            continue;
          (*c)(i1,j1) = mat(ii,jj);
          j1++;
        }
        i1++;
      }
      
      /* Calculate the determinate */
      det = Determinant(*c, order-1);
      
      /* Fill in the elements of the cofactor */
      ret(i,j) = pow(-1.0,i+j+2.0) * det;
    }
  }
  
  return ret;
}

//// FRIEND operators that create new Matrix objects
Matrix 
operator+(const Matrix &m1, const Matrix& m2) {
  return Matrix(m1) += m2;
}

/////////////////////////////////////////////////////////////////////
Matrix 
operator+(const Matrix &m1, double a) {
  return Matrix(m1) += a;
}

/////////////////////////////////////////////////////////////////////
Matrix
operator+(const double a, const Matrix &m1) {
  return m1 + a; 
}

/////////////////////////////////////////////////////////////////////
Matrix 
operator-(const Matrix &m1, const Matrix& m2) {
  return Matrix(m1) -= m2;
}

/////////////////////////////////////////////////////////////////////
Matrix 
operator-(const Matrix &m1, double a) {
  return Matrix(m1) -= a;
}

/////////////////////////////////////////////////////////////////////
Matrix 
operator-(double a, const Matrix &m1) {
  return Matrix(m1) -= a;
}

/////////////////////////////////////////////////////////////////////
Matrix
operator*(const Matrix &m1, const Matrix &m2) {
  return Matrix(m1) *= m2;
}

/////////////////////////////////////////////////////////////////////
Matrix 
operator*(const Matrix &m1, double a) {
  return Matrix(m1) *= a;
}

/////////////////////////////////////////////////////////////////////
Matrix
operator*(const double a, const Matrix &m1) {
  return Matrix(m1) *= a; 
}

bool
operator==(const Matrix &m1, const Matrix &m2) {
  bool ret = true;
  
  if (m1.m_rows != m1.m_rows || m1.m_cols != m2.m_cols ) {
    ret = false;
  }
  
  for (unsigned int i=1; i<= m1.m_rows; i++) {
    for(unsigned int j=1; j<= m1.m_cols; j++) {
      if (m1(i,j) != m2(i,j)) {
        ret = false;
        break;
      }
    }
    if(!ret) break;
  }
  return ret;
}


////////////////////////////////////////////////////////////////////////
// VECTOR
////////////////////////////////////////////////////////////////////////

Vector::Vector(const Vector& v) : Matrix(v) { }


/////////////////////////////////////////////////////////////////////

Vector::Vector(const unsigned int row) : Matrix(row, 1) { }

/////////////////////////////////////////////////////////////////////

Vector 
Vector::resize(const unsigned int row) {
  Matrix::resize(row,1);
  return *this;
}

/////////////////////////////////////////////////////////////////////

Vector
Vector::push(const Vector& v) {
  unsigned int lastR = m_rows;
  resize(m_rows + v.m_rows);
  for (unsigned int i = 1; i <= v.m_rows; ++i) {
    this->element(lastR+i) = v(i); 
  }
  return *this;
}

/////////////////////////////////////////////////////////////////////

double& 
Vector::element(const unsigned int row) {
  return Matrix::element(row,1);
}

/////////////////////////////////////////////////////////////////////

Vector 
Vector::subVector(const unsigned int fr, const unsigned int lr) {
  
  if ((lr - fr) < 0) 
    throw ArrayOutOfBounds();
  
  unsigned int nrows = (lr - fr) + 1;
  
  Vector tmp(nrows);
  
  int ii = 1;
  for (unsigned int i = 0; i < nrows; i++) {
    tmp(ii) = element(i+(fr));
    ++ii;
  }
  
  return tmp;
}

/////////////////////////////////////////////////////////////////////

Vector 
Vector::gaussian(const double mean,const Matrix& cov) {
  return Vector(1);
}

/////////////////////////////////////////////////////////////////////

double&
Vector::operator() (const unsigned int row) {
  return element(row);
}

/////////////////////////////////////////////////////////////////////

double  
Vector::operator() (const unsigned int row) const {
  if (row > m_rows || row == 0) 
    throw ArrayOutOfBounds();
  
  return m_data[m_cols * row-1];
}

/////////////////////////////////////////////////////////////////////

void printMatrix(const Matrix& mat) {
  
  for (unsigned int i=1; i<= mat.numRows(); i++) {
    for (unsigned int j=1; j<= mat.numCols(); j++) {
      printf(" %f ", mat(i,j));
    }
    printf("\n");
  }
}

// Dot (inner) product (vector * vector = scalar)
//double 
//operator*(const Vector &v1, const Vector& v2) {
//  return 1.0;
//}


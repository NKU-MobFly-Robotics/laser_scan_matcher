#ifndef LASER_SCAN_MATCHER_KARTO_TOOLS_H_
#define LASER_SCAN_MATCHER_KARTO_TOOLS_H_

#include <laser_scan_matcher/Math.h>

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <boost/thread.hpp>
#include <cstdlib>

namespace scan_tools {
/**
 * Represents a size (width, height) in 2-dimensional real space.
 */
template <typename T>
class Size2 {
 public:
  /**
   * Default constructor
   */
  Size2() : m_Width(0), m_Height(0) {}

  /**
   * Constructor initializing point location
   * @param width
   * @param height
   */
  Size2(T width, T height) : m_Width(width), m_Height(height) {}

  /**
   * Copy constructor
   * @param rOther
   */
  Size2(const Size2& rOther)
      : m_Width(rOther.m_Width), m_Height(rOther.m_Height) {}

 public:
  /**
   * Gets the width
   * @return the width
   */
  inline const T GetWidth() const { return m_Width; }

  /**
   * Sets the width
   * @param width
   */
  inline void SetWidth(T width) { m_Width = width; }

  /**
   * Gets the height
   * @return the height
   */
  inline const T GetHeight() const { return m_Height; }

  /**
   * Sets the height
   * @param height
   */
  inline void SetHeight(T height) { m_Height = height; }

  /**
   * Assignment operator
   */
  inline Size2& operator=(const Size2& rOther) {
    m_Width = rOther.m_Width;
    m_Height = rOther.m_Height;

    return (*this);
  }

  /**
   * Equality operator
   */
  inline bool operator==(const Size2& rOther) const {
    return (m_Width == rOther.m_Width && m_Height == rOther.m_Height);
  }

  /**
   * Inequality operator
   */
  inline bool operator!=(const Size2& rOther) const {
    return (m_Width != rOther.m_Width || m_Height != rOther.m_Height);
  }

 private:
  T m_Width;
  T m_Height;
};  // Size2<T>

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Represents a vector (x, y) in 2-dimensional real space.
 */
template <typename T>
class Vector2 {
 public:
  /**
   * Default constructor
   */
  Vector2() {
    m_Values[0] = 0;
    m_Values[1] = 0;
  }

  /**
   * Constructor initializing vector location
   * @param x
   * @param y
   */
  Vector2(T x, T y) {
    m_Values[0] = x;
    m_Values[1] = y;
  }

 public:
  /**
   * Gets the x-coordinate of this vector2
   * @return the x-coordinate of the vector2
   */
  inline const T& GetX() const { return m_Values[0]; }

  /**
   * Sets the x-coordinate of this vector2
   * @param x the x-coordinate of the vector2
   */
  inline void SetX(const T& x) { m_Values[0] = x; }

  /**
   * Gets the y-coordinate of this vector2
   * @return the y-coordinate of the vector2
   */
  inline const T& GetY() const { return m_Values[1]; }

  /**
   * Sets the y-coordinate of this vector2
   * @param y the y-coordinate of the vector2
   */
  inline void SetY(const T& y) { m_Values[1] = y; }

  /**
   * Floor point operator
   * @param rOther
   */
  inline void MakeFloor(const Vector2& rOther) {
    if (rOther.m_Values[0] < m_Values[0]) m_Values[0] = rOther.m_Values[0];
    if (rOther.m_Values[1] < m_Values[1]) m_Values[1] = rOther.m_Values[1];
  }

  /**
   * Ceiling point operator
   * @param rOther
   */
  inline void MakeCeil(const Vector2& rOther) {
    if (rOther.m_Values[0] > m_Values[0]) m_Values[0] = rOther.m_Values[0];
    if (rOther.m_Values[1] > m_Values[1]) m_Values[1] = rOther.m_Values[1];
  }

  /**
   * Returns the square of the length of the vector
   * @return square of the length of the vector
   */
  inline double SquaredLength() const {
    return math::Square(m_Values[0]) + math::Square(m_Values[1]);
  }

  /**
   * Returns the length of the vector (x and y).
   * @return length of the vector
   */
  inline double Length() const { return sqrt(SquaredLength()); }

  /**
   * Returns the square distance to the given vector
   * @returns square distance to the given vector
   */
  inline double SquaredDistance(const Vector2& rOther) const {
    return (*this - rOther).SquaredLength();
  }

  /**
   * Gets the distance to the other vector2
   * @param rOther
   * @return distance to other vector2
   */
  inline double Distance(const Vector2& rOther) const {
    return sqrt(SquaredDistance(rOther));
  }

 public:
  /**
   * In place Vector2 addition.
   */
  inline void operator+=(const Vector2& rOther) {
    m_Values[0] += rOther.m_Values[0];
    m_Values[1] += rOther.m_Values[1];
  }

  /**
   * In place Vector2 subtraction.
   */
  inline void operator-=(const Vector2& rOther) {
    m_Values[0] -= rOther.m_Values[0];
    m_Values[1] -= rOther.m_Values[1];
  }

  /**
   * Addition operator
   * @param rOther
   * @return vector resulting from adding this vector with the given vector
   */
  inline const Vector2 operator+(const Vector2& rOther) const {
    return Vector2(m_Values[0] + rOther.m_Values[0],
                   m_Values[1] + rOther.m_Values[1]);
  }

  /**
   * Subtraction operator
   * @param rOther
   * @return vector resulting from subtracting this vector from the given vector
   */
  inline const Vector2 operator-(const Vector2& rOther) const {
    return Vector2(m_Values[0] - rOther.m_Values[0],
                   m_Values[1] - rOther.m_Values[1]);
  }

  /**
   * In place scalar division operator
   * @param scalar
   */
  inline void operator/=(T scalar) {
    m_Values[0] /= scalar;
    m_Values[1] /= scalar;
  }

  /**
   * Divides a Vector2
   * @param scalar
   * @return scalar product
   */
  inline const Vector2 operator/(T scalar) const {
    return Vector2(m_Values[0] / scalar, m_Values[1] / scalar);
  }

  /**
   * Computes the dot product between the two vectors
   * @param rOther
   * @return dot product
   */
  inline double operator*(const Vector2& rOther) const {
    return m_Values[0] * rOther.m_Values[0] + m_Values[1] * rOther.m_Values[1];
  }

  /**
   * Scales the vector by the given scalar
   * @param scalar
   */
  inline const Vector2 operator*(T scalar) const {
    return Vector2(m_Values[0] * scalar, m_Values[1] * scalar);
  }

  /**
   * Subtract the vector by the given scalar
   * @param scalar
   */
  inline const Vector2 operator-(T scalar) const {
    return Vector2(m_Values[0] - scalar, m_Values[1] - scalar);
  }

  /**
   * In place scalar multiplication operator
   * @param scalar
   */
  inline void operator*=(T scalar) {
    m_Values[0] *= scalar;
    m_Values[1] *= scalar;
  }

  /**
   * Equality operator returns true if the corresponding x, y values of each
   * Vector2 are the same values.
   * @param rOther
   */
  inline bool operator==(const Vector2& rOther) const {
    return (m_Values[0] == rOther.m_Values[0] &&
            m_Values[1] == rOther.m_Values[1]);
  }

  /**
   * Inequality operator returns true if any of the corresponding x, y values of
   * each Vector2 not the same.
   * @param rOther
   */
  inline bool operator!=(const Vector2& rOther) const {
    return (m_Values[0] != rOther.m_Values[0] ||
            m_Values[1] != rOther.m_Values[1]);
  }

  /**
   * Less than operator
   * @param rOther
   * @return true if left vector is less than right vector
   */
  inline bool operator<(const Vector2& rOther) const {
    if (m_Values[0] < rOther.m_Values[0])
      return true;
    else if (m_Values[0] > rOther.m_Values[0])
      return false;
    else
      return (m_Values[1] < rOther.m_Values[1]);
  }

 private:
  T m_Values[2];
};  // Vector2<T>

/**
 * Type declaration of Vector2<double> vector
 */
typedef std::vector<Vector2<double> > PointVectorDouble;

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Stores x, y, width and height that represents the location and size of a
 * rectangle
 * (x, y) is at bottom left in mapper!
 */
template <typename T>
class Rectangle2 {
 public:
  /**
   * Default constructor
   */
  Rectangle2() {}

  /**
   * Constructor initializing rectangle parameters
   * @param x x-coordinate of left edge of rectangle
   * @param y y-coordinate of bottom edge of rectangle
   * @param width width of rectangle
   * @param height height of rectangle
   */
  Rectangle2(T x, T y, T width, T height)
      : m_Position(x, y), m_Size(width, height) {}

  /**
   * Constructor initializing rectangle parameters
   * @param rPosition (x,y)-coordinate of rectangle
   * @param rSize Size of the rectangle
   */
  Rectangle2(const Vector2<T>& rPosition, const Size2<T>& rSize)
      : m_Position(rPosition), m_Size(rSize) {}

  /**
   * Copy constructor
   */
  Rectangle2(const Rectangle2& rOther)
      : m_Position(rOther.m_Position), m_Size(rOther.m_Size) {}

 public:
  /**
   * Gets the x-coordinate of the left edge of this rectangle
   * @return the x-coordinate of the left edge of this rectangle
   */
  inline T GetX() const { return m_Position.GetX(); }

  /**
   * Sets the x-coordinate of the left edge of this rectangle
   * @param x the x-coordinate of the left edge of this rectangle
   */
  inline void SetX(T x) { m_Position.SetX(x); }

  /**
   * Gets the y-coordinate of the bottom edge of this rectangle
   * @return the y-coordinate of the bottom edge of this rectangle
   */
  inline T GetY() const { return m_Position.GetY(); }

  /**
   * Sets the y-coordinate of the bottom edge of this rectangle
   * @param y the y-coordinate of the bottom edge of this rectangle
   */
  inline void SetY(T y) { m_Position.SetY(y); }

  /**
   * Gets the width of this rectangle
   * @return the width of this rectangle
   */
  inline T GetWidth() const { return m_Size.GetWidth(); }

  /**
   * Sets the width of this rectangle
   * @param width the width of this rectangle
   */
  inline void SetWidth(T width) { m_Size.SetWidth(width); }

  /**
   * Gets the height of this rectangle
   * @return the height of this rectangle
   */
  inline T GetHeight() const { return m_Size.GetHeight(); }

  /**
   * Sets the height of this rectangle
   * @param height the height of this rectangle
   */
  inline void SetHeight(T height) { m_Size.SetHeight(height); }

  /**
   * Gets the position of this rectangle
   * @return the position of this rectangle
   */
  inline const Vector2<T>& GetPosition() const { return m_Position; }

  /**
   * Sets the position of this rectangle
   * @param rX x
   * @param rY y
   */
  inline void SetPosition(const T& rX, const T& rY) {
    m_Position = Vector2<T>(rX, rY);
  }

  /**
   * Sets the position of this rectangle
   * @param rPosition position
   */
  inline void SetPosition(const Vector2<T>& rPosition) {
    m_Position = rPosition;
  }

  /**
   * Gets the size of this rectangle
   * @return the size of this rectangle
   */
  inline const Size2<T>& GetSize() const { return m_Size; }

  /**
   * Sets the size of this rectangle
   * @param rSize size
   */
  inline void SetSize(const Size2<T>& rSize) { m_Size = rSize; }

  /**
   * Gets the center of this rectangle
   * @return the center of this rectangle
   */
  inline const Vector2<T> GetCenter() const {
    return Vector2<T>(m_Position.GetX() + m_Size.GetWidth() * 0.5,
                      m_Position.GetY() + m_Size.GetHeight() * 0.5);
  }

 public:
  /**
   * Assignment operator
   */
  Rectangle2& operator=(const Rectangle2& rOther) {
    m_Position = rOther.m_Position;
    m_Size = rOther.m_Size;

    return *this;
  }

  /**
   * Equality operator
   */
  inline bool operator==(const Rectangle2& rOther) const {
    return (m_Position == rOther.m_Position && m_Size == rOther.m_Size);
  }

  /**
   * Inequality operator
   */
  inline bool operator!=(const Rectangle2& rOther) const {
    return (m_Position != rOther.m_Position || m_Size != rOther.m_Size);
  }

 private:
  Vector2<T> m_Position;
  Size2<T> m_Size;
};  // Rectangle2

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Defines a position (x, y) in 2-dimensional space and heading.
 */
class Pose2 {
 public:
  /**
   * Default Constructor
   */
  Pose2() : m_Heading(0.0) {}

  /**
   * Constructor initializing pose parameters
   * @param rPosition position
   * @param heading heading
   */
  Pose2(const Vector2<double>& rPosition, double heading)
      : m_Position(rPosition), m_Heading(heading) {}

  /**
   * Constructor initializing pose parameters
   * @param x x-coordinate
   * @param y y-coordinate
   * @param heading heading
   */
  Pose2(double x, double y, double heading)
      : m_Position(x, y), m_Heading(heading) {}

  /**
   * Copy constructor
   */
  Pose2(const Pose2& rOther)
      : m_Position(rOther.m_Position), m_Heading(rOther.m_Heading) {}

 public:
  /**
   * Returns the x-coordinate
   * @return the x-coordinate of the pose
   */
  inline double GetX() const { return m_Position.GetX(); }

  /**
   * Sets the x-coordinate
   * @param x the x-coordinate of the pose
   */
  inline void SetX(double x) { m_Position.SetX(x); }

  /**
   * Returns the y-coordinate
   * @return the y-coordinate of the pose
   */
  inline double GetY() const { return m_Position.GetY(); }

  /**
   * Sets the y-coordinate
   * @param y the y-coordinate of the pose
   */
  inline void SetY(double y) { m_Position.SetY(y); }

  /**
   * Returns the position
   * @return the position of the pose
   */
  inline const Vector2<double>& GetPosition() const { return m_Position; }

  /**
   * Sets the position
   * @param rPosition of the pose
   */
  inline void SetPosition(const Vector2<double>& rPosition) {
    m_Position = rPosition;
  }

  /**
   * Returns the heading of the pose (in radians)
   * @return the heading of the pose
   */
  inline double GetHeading() const { return m_Heading; }

  /**
   * Sets the heading
   * @param heading of the pose
   */
  inline void SetHeading(double heading) { m_Heading = heading; }

  /**
   * Return the squared distance between two Pose2
   * @return squared distance
   */
  inline double SquaredDistance(const Pose2& rOther) const {
    return m_Position.SquaredDistance(rOther.m_Position);
  }

 public:
  /**
   * Assignment operator
   */
  inline Pose2& operator=(const Pose2& rOther) {
    m_Position = rOther.m_Position;
    m_Heading = rOther.m_Heading;

    return *this;
  }

  /**
   * Equality operator
   */
  inline bool operator==(const Pose2& rOther) const {
    return (m_Position == rOther.m_Position && m_Heading == rOther.m_Heading);
  }

  /**
   * Inequality operator
   */
  inline bool operator!=(const Pose2& rOther) const {
    return (m_Position != rOther.m_Position || m_Heading != rOther.m_Heading);
  }

  /**
   * In place Pose2 add.
   */
  inline void operator+=(const Pose2& rOther) {
    m_Position += rOther.m_Position;
    m_Heading = math::NormalizeAngle(m_Heading + rOther.m_Heading);
  }

  /**
   * Binary Pose2 add
   * @param rOther
   * @return Pose2 sum
   */
  inline Pose2 operator+(const Pose2& rOther) const {
    return Pose2(m_Position + rOther.m_Position,
                 math::NormalizeAngle(m_Heading + rOther.m_Heading));
  }

  /**
   * Binary Pose2 subtract
   * @param rOther
   * @return Pose2 difference
   */
  inline Pose2 operator-(const Pose2& rOther) const {
    return Pose2(m_Position - rOther.m_Position,
                 math::NormalizeAngle(m_Heading - rOther.m_Heading));
  }

 private:
  Vector2<double> m_Position;

  double m_Heading;
};  // Pose2

/**
 * Type declaration of Pose2 vector
 */
typedef std::vector<Pose2> Pose2Vector;

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Defines a Matrix 3 x 3 class.
 */
class Matrix3 {
 public:
  /**
   * Default constructor
   */
  Matrix3() { Clear(); }

  /**
   * Copy constructor
   */
  inline Matrix3(const Matrix3& rOther) {
    memcpy(m_Matrix, rOther.m_Matrix, 9 * sizeof(double));
  }

 public:
  /**
   * Sets this matrix to identity matrix
   */
  void SetToIdentity() {
    memset(m_Matrix, 0, 9 * sizeof(double));

    for (int i = 0; i < 3; i++) {
      m_Matrix[i][i] = 1.0;
    }
  }

  /**
   * Sets this matrix to zero matrix
   */
  void Clear() { memset(m_Matrix, 0, 9 * sizeof(double)); }

  /**
   * Sets this matrix to be the rotation matrix of rotation around given axis
   * @param x x-coordinate of axis
   * @param y y-coordinate of axis
   * @param z z-coordinate of axis
   * @param radians amount of rotation
   */
  void FromAxisAngle(double x, double y, double z, const double radians) {
    double cosRadians = cos(radians);
    double sinRadians = sin(radians);
    double oneMinusCos = 1.0 - cosRadians;

    double xx = x * x;
    double yy = y * y;
    double zz = z * z;

    double xyMCos = x * y * oneMinusCos;
    double xzMCos = x * z * oneMinusCos;
    double yzMCos = y * z * oneMinusCos;

    double xSin = x * sinRadians;
    double ySin = y * sinRadians;
    double zSin = z * sinRadians;

    m_Matrix[0][0] = xx * oneMinusCos + cosRadians;
    m_Matrix[0][1] = xyMCos - zSin;
    m_Matrix[0][2] = xzMCos + ySin;

    m_Matrix[1][0] = xyMCos + zSin;
    m_Matrix[1][1] = yy * oneMinusCos + cosRadians;
    m_Matrix[1][2] = yzMCos - xSin;

    m_Matrix[2][0] = xzMCos - ySin;
    m_Matrix[2][1] = yzMCos + xSin;
    m_Matrix[2][2] = zz * oneMinusCos + cosRadians;
  }

  /**
   * Returns transposed version of this matrix
   * @return transposed matrix
   */
  Matrix3 Transpose() const {
    Matrix3 transpose;

    for (int row = 0; row < 3; row++) {
      for (int col = 0; col < 3; col++) {
        transpose.m_Matrix[row][col] = m_Matrix[col][row];
      }
    }

    return transpose;
  }

  /**
   * Returns the inverse of the matrix
   */
  Matrix3 Inverse() const {
    Matrix3 kInverse = *this;
    bool haveInverse = InverseFast(kInverse, 1e-14);
    if (haveInverse == false) {
      assert(false);
    }
    return kInverse;
  }

  /**
   * Internal helper method for inverse matrix calculation
   * This code is lifted from the OgreMatrix3 class!!
   */
  bool InverseFast(Matrix3& rkInverse, double fTolerance = KT_TOLERANCE) const {
    // Invert a 3x3 using cofactors.  This is about 8 times faster than
    // the Numerical Recipes code which uses Gaussian elimination.
    rkInverse.m_Matrix[0][0] =
        m_Matrix[1][1] * m_Matrix[2][2] - m_Matrix[1][2] * m_Matrix[2][1];
    rkInverse.m_Matrix[0][1] =
        m_Matrix[0][2] * m_Matrix[2][1] - m_Matrix[0][1] * m_Matrix[2][2];
    rkInverse.m_Matrix[0][2] =
        m_Matrix[0][1] * m_Matrix[1][2] - m_Matrix[0][2] * m_Matrix[1][1];
    rkInverse.m_Matrix[1][0] =
        m_Matrix[1][2] * m_Matrix[2][0] - m_Matrix[1][0] * m_Matrix[2][2];
    rkInverse.m_Matrix[1][1] =
        m_Matrix[0][0] * m_Matrix[2][2] - m_Matrix[0][2] * m_Matrix[2][0];
    rkInverse.m_Matrix[1][2] =
        m_Matrix[0][2] * m_Matrix[1][0] - m_Matrix[0][0] * m_Matrix[1][2];
    rkInverse.m_Matrix[2][0] =
        m_Matrix[1][0] * m_Matrix[2][1] - m_Matrix[1][1] * m_Matrix[2][0];
    rkInverse.m_Matrix[2][1] =
        m_Matrix[0][1] * m_Matrix[2][0] - m_Matrix[0][0] * m_Matrix[2][1];
    rkInverse.m_Matrix[2][2] =
        m_Matrix[0][0] * m_Matrix[1][1] - m_Matrix[0][1] * m_Matrix[1][0];

    double fDet = m_Matrix[0][0] * rkInverse.m_Matrix[0][0] +
                  m_Matrix[0][1] * rkInverse.m_Matrix[1][0] +
                  m_Matrix[0][2] * rkInverse.m_Matrix[2][0];

    if (fabs(fDet) <= fTolerance) {
      return false;
    }

    double fInvDet = 1.0 / fDet;
    for (size_t row = 0; row < 3; row++) {
      for (size_t col = 0; col < 3; col++) {
        rkInverse.m_Matrix[row][col] *= fInvDet;
      }
    }

    return true;
  }

 public:
  /**
   * Assignment operator
   */
  inline Matrix3& operator=(const Matrix3& rOther) {
    memcpy(m_Matrix, rOther.m_Matrix, 9 * sizeof(double));
    return *this;
  }

  /**
   * Matrix element access, allows use of construct mat(r, c)
   * @param row
   * @param column
   * @return reference to mat(r,c)
   */
  inline double& operator()(int row, int column) {
    return m_Matrix[row][column];
  }

  /**
   * Read-only matrix element access, allows use of construct mat(r, c)
   * @param row
   * @param column
   * @return mat(r,c)
   */
  inline double operator()(int row, int column) const {
    return m_Matrix[row][column];
  }

  /**
   * Binary Matrix3 multiplication.
   * @param rOther
   * @return Matrix3 product
   */
  Matrix3 operator*(const Matrix3& rOther) const {
    Matrix3 product;

    for (size_t row = 0; row < 3; row++) {
      for (size_t col = 0; col < 3; col++) {
        product.m_Matrix[row][col] =
            m_Matrix[row][0] * rOther.m_Matrix[0][col] +
            m_Matrix[row][1] * rOther.m_Matrix[1][col] +
            m_Matrix[row][2] * rOther.m_Matrix[2][col];
      }
    }

    return product;
  }

  /**
   * Matrix3 and Pose2 multiplication - matrix * pose [3x3 * 3x1 = 3x1]
   * @param rPose2
   * @return Pose2 product
   */
  inline Pose2 operator*(const Pose2& rPose2) const {
    Pose2 pose2;

    pose2.SetX(m_Matrix[0][0] * rPose2.GetX() + m_Matrix[0][1] * rPose2.GetY() +
               m_Matrix[0][2] * rPose2.GetHeading());
    pose2.SetY(m_Matrix[1][0] * rPose2.GetX() + m_Matrix[1][1] * rPose2.GetY() +
               m_Matrix[1][2] * rPose2.GetHeading());
    pose2.SetHeading(m_Matrix[2][0] * rPose2.GetX() +
                     m_Matrix[2][1] * rPose2.GetY() +
                     m_Matrix[2][2] * rPose2.GetHeading());

    return pose2;
  }

  /**
   * In place Matrix3 add.
   * @param rkMatrix
   */
  inline void operator+=(const Matrix3& rkMatrix) {
    for (int row = 0; row < 3; row++) {
      for (int col = 0; col < 3; col++) {
        m_Matrix[row][col] += rkMatrix.m_Matrix[row][col];
      }
    }
  }

 private:
  double m_Matrix[3][3];
};  // Matrix3

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Defines a bounding box in 2-dimensional real space.
 */
class BoundingBox2 {
 public:
  /*
   * Default constructor
   */
  BoundingBox2()
      : m_Minimum(999999999999999999.99999, 999999999999999999.99999),
        m_Maximum(-999999999999999999.99999, -999999999999999999.99999) {}

 public:
  /**
   * Get bounding box minimum
   */
  inline const Vector2<double>& GetMinimum() const { return m_Minimum; }

  /**
   * Set bounding box minimum
   */
  inline void SetMinimum(const Vector2<double>& mMinimum) {
    m_Minimum = mMinimum;
  }

  /**
   * Get bounding box maximum
   */
  inline const Vector2<double>& GetMaximum() const { return m_Maximum; }

  /**
   * Set bounding box maximum
   */
  inline void SetMaximum(const Vector2<double>& rMaximum) {
    m_Maximum = rMaximum;
  }

  /**
   * Get the size of the bounding box
   */
  inline Size2<double> GetSize() const {
    Vector2<double> size = m_Maximum - m_Minimum;

    return Size2<double>(size.GetX(), size.GetY());
  }

  /**
   * Add vector to bounding box
   */
  inline void Add(const Vector2<double>& rPoint) {
    m_Minimum.MakeFloor(rPoint);
    m_Maximum.MakeCeil(rPoint);
  }

  /**
   * Add other bounding box to bounding box
   */
  inline void Add(const BoundingBox2& rBoundingBox) {
    Add(rBoundingBox.GetMinimum());
    Add(rBoundingBox.GetMaximum());
  }

  /**
   * Whether the given point is in the bounds of this box
   * @param rPoint
   * @return in bounds?
   */
  inline bool IsInBounds(const Vector2<double>& rPoint) const {
    return (math::InRange(rPoint.GetX(), m_Minimum.GetX(), m_Maximum.GetX()) &&
            math::InRange(rPoint.GetY(), m_Minimum.GetY(), m_Maximum.GetY()));
  }

 private:
  Vector2<double> m_Minimum;
  Vector2<double> m_Maximum;
};  // BoundingBox2

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Implementation of a Pose2 transform
 */
class Transform {
 public:
  /**
   * Constructs a transformation from the origin to the given pose
   * @param rPose pose
   */
  Transform(const Pose2& rPose) { SetTransform(Pose2(), rPose); }

  /**
   * Constructs a transformation from the first pose to the second pose
   * @param rPose1 first pose
   * @param rPose2 second pose
   */
  Transform(const Pose2& rPose1, const Pose2& rPose2) {
    SetTransform(rPose1, rPose2);
  }

 public:
  /**
   * Transforms the pose according to this transform
   * @param rSourcePose pose to transform from
   * @return transformed pose
   */
  inline Pose2 TransformPose(const Pose2& rSourcePose) {
    Pose2 newPosition = m_Transform + m_Rotation * rSourcePose;
    double angle = math::NormalizeAngle(rSourcePose.GetHeading() +
                                        m_Transform.GetHeading());

    return Pose2(newPosition.GetPosition(), angle);
  }

  /**
   * Inverse transformation of the pose according to this transform
   * @param rSourcePose pose to transform from
   * @return transformed pose
   */
  inline Pose2 InverseTransformPose(const Pose2& rSourcePose) {
    Pose2 newPosition = m_InverseRotation * (rSourcePose - m_Transform);
    double angle = math::NormalizeAngle(rSourcePose.GetHeading() -
                                        m_Transform.GetHeading());

    // components of transform
    return Pose2(newPosition.GetPosition(), angle);
  }

 private:
  /**
   * Sets this to be the transformation from the first pose to the second pose
   * @param rPose1 first pose
   * @param rPose2 second pose
   */
  void SetTransform(const Pose2& rPose1, const Pose2& rPose2) {
    if (rPose1 == rPose2) {
      m_Rotation.SetToIdentity();
      m_InverseRotation.SetToIdentity();
      m_Transform = Pose2();
      return;
    }

    // heading transformation
    m_Rotation.FromAxisAngle(0, 0, 1,
                             rPose2.GetHeading() - rPose1.GetHeading());
    m_InverseRotation.FromAxisAngle(0, 0, 1,
                                    rPose1.GetHeading() - rPose2.GetHeading());

    // position transformation
    Pose2 newPosition;
    if (rPose1.GetX() != 0.0 || rPose1.GetY() != 0.0) {
      newPosition = rPose2 - m_Rotation * rPose1;
    } else {
      newPosition = rPose2;
    }

    m_Transform = Pose2(newPosition.GetPosition(),
                        rPose2.GetHeading() - rPose1.GetHeading());
  }

 private:
  // pose transformation
  Pose2 m_Transform;

  Matrix3 m_Rotation;
  Matrix3 m_InverseRotation;
};  // Transform

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * The CoordinateConverter class is used to convert coordinates between world
 * and grid coordinates
 * In world coordinates 1.0 = 1 meter where 1 in grid coordinates = 1 pixel!
 * Default scale for coordinate converter is 20 that converters to 1 pixel =
 * 0.05 meter
 */
class CoordinateConverter {
 public:
  /**
   * Default constructor
   */
  CoordinateConverter() : m_Scale(20.0) {}

 public:
  /**
   * Scales the value
   * @param value
   * @return scaled value
   */
  inline double Transform(double value) { return value * m_Scale; }

  /**
   * Converts the point from world coordinates to grid coordinates
   * @param rWorld world coordinate
   * @param flipY
   * @return grid coordinate
   */
  inline Vector2<int> WorldToGrid(const Vector2<double>& rWorld,
                                  bool flipY = false) const {
    double gridX = (rWorld.GetX() - m_Offset.GetX()) * m_Scale;
    double gridY = 0.0;

    if (flipY == false) {
      gridY = (rWorld.GetY() - m_Offset.GetY()) * m_Scale;
    } else {
      gridY = (m_Size.GetHeight() / m_Scale - rWorld.GetY() + m_Offset.GetY()) *
              m_Scale;
    }

    return Vector2<int>(static_cast<int>(math::Round(gridX)),
                        static_cast<int>(math::Round(gridY)));
  }

  /**
   * Converts the point from grid coordinates to world coordinates
   * @param rGrid world coordinate
   * @param flipY
   * @return world coordinate
   */
  inline Vector2<double> GridToWorld(const Vector2<int>& rGrid,
                                     bool flipY = false) const {
    double worldX = m_Offset.GetX() + rGrid.GetX() / m_Scale;
    double worldY = 0.0;

    if (flipY == false) {
      worldY = m_Offset.GetY() + rGrid.GetY() / m_Scale;
    } else {
      worldY = m_Offset.GetY() + (m_Size.GetHeight() - rGrid.GetY()) / m_Scale;
    }

    return Vector2<double>(worldX, worldY);
  }

  /**
   * Gets the scale
   * @return scale
   */
  inline double GetScale() const { return m_Scale; }

  /**
   * Sets the scale
   * @param scale
   */
  inline void SetScale(double scale) { m_Scale = scale; }

  /**
   * Gets the offset
   * @return offset
   */
  inline const Vector2<double>& GetOffset() const { return m_Offset; }

  /**
   * Sets the offset
   * @param rOffset
   */
  inline void SetOffset(const Vector2<double>& rOffset) { m_Offset = rOffset; }

  /**
   * Sets the size
   * @param rSize
   */
  inline void SetSize(const Size2<int>& rSize) { m_Size = rSize; }

  /**
   * Gets the size
   * @return size
   */
  inline const Size2<int>& GetSize() const { return m_Size; }

  /**
   * Gets the resolution
   * @return resolution
   */
  inline double GetResolution() const { return 1.0 / m_Scale; }

  /**
   * Sets the resolution
   * @param resolution
   */
  inline void SetResolution(double resolution) { m_Scale = 1.0 / resolution; }

  /**
   * Gets the bounding box
   * @return bounding box
   */
  inline BoundingBox2 GetBoundingBox() const {
    BoundingBox2 box;

    double minX = GetOffset().GetX();
    double minY = GetOffset().GetY();
    double maxX = minX + GetSize().GetWidth() * GetResolution();
    double maxY = minY + GetSize().GetHeight() * GetResolution();

    box.SetMinimum(GetOffset());
    box.SetMaximum(Vector2<double>(maxX, maxY));
    return box;
  }

 private:
  Size2<int> m_Size;
  double m_Scale;

  Vector2<double> m_Offset;
};  // CoordinateConverter

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Type declaration of range readings vector
 */
typedef std::vector<double> RangeReadingsVector;

/**
 * LaserRangeScan representing the range readings from a laser range finder
 * sensor.
 */
class LaserRangeScan {
 public:
  /**
   * Constructs a scan from the given sensor with the given readings
   * @param rSensorName
   */
  LaserRangeScan() : m_pRangeReadings(NULL), m_NumberOfRangeReadings(0) {}

  /**
   * Constructs a scan from the given sensor with the given readings
   * @param rSensorName
   * @param rRangeReadings
   */
  LaserRangeScan(const RangeReadingsVector& rRangeReadings)
      : m_pRangeReadings(NULL), m_NumberOfRangeReadings(0) {
    SetRangeReadings(rRangeReadings);
  }

  /**
   * Destructor
   */
  virtual ~LaserRangeScan() { delete[] m_pRangeReadings; }

 public:
  /**
   * Gets the range readings of this scan
   * @return range readings of this scan
   */
  inline double* GetRangeReadings() const { return m_pRangeReadings; }

  inline RangeReadingsVector GetRangeReadingsVector() const {
    return RangeReadingsVector(m_pRangeReadings,
                               m_pRangeReadings + m_NumberOfRangeReadings);
  }

  /**
   * Sets the range readings for this scan
   * @param rRangeReadings
   */
  inline void SetRangeReadings(const RangeReadingsVector& rRangeReadings) {
    if (!rRangeReadings.empty()) {
      if (rRangeReadings.size() != m_NumberOfRangeReadings) {
        // delete old readings
        delete[] m_pRangeReadings;

        // store size of array!
        m_NumberOfRangeReadings = static_cast<int>(rRangeReadings.size());

        // allocate range readings
        m_pRangeReadings = new double[m_NumberOfRangeReadings];
      }

      // copy readings
      int index = 0;
      for (RangeReadingsVector::const_iterator iter = rRangeReadings.begin();
           iter != rRangeReadings.end(); iter++) {
        m_pRangeReadings[index++] = *iter;
      }
    } else {
      delete[] m_pRangeReadings;
      m_pRangeReadings = NULL;
    }
  }

  /**
   * Gets the number of range readings
   * @return number of range readings
   */
  inline int GetNumberOfRangeReadings() const {
    return m_NumberOfRangeReadings;
  }

 private:
  LaserRangeScan(const LaserRangeScan&);
  const LaserRangeScan& operator=(const LaserRangeScan&);

 private:
  double* m_pRangeReadings;
  int m_NumberOfRangeReadings;
};  // LaserRangeScan

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * The LaserRangeFinder defines a laser sensor that provides the pose offset
 * position of a localized range scan relative to the robot.
 *
 * The user can set an offset pose for the sensor relative to the robot
 * coordinate system. If no value is provided by the user, the sensor is set to
 * be at the origin of the robot coordinate system.
 *
 * The LaserRangeFinder contains parameters for physical laser sensor used by
 * the mapper for scan matching.
 * Also contains information about the maximum range of the sensor and provides
 * a threshold for limiting the range of readings.
 *
 * The optimal value for the range threshold depends on the angular resolution
 * of the scan and the desired map resolution.  RangeThreshold should be set as
 * large as possible while still providing "solid" coverage between consecutive
 * range readings.
 */
class LaserRangeFinder {
 public:
  /**
   * Destructor
   */
  virtual ~LaserRangeFinder() {}

 public:
  /**
   * Gets this range finder sensor's offset
   * @return offset pose
   */
  inline const Pose2& GetOffsetPose() const { return m_OffsetPose; }

  /**
   * Sets this range finder sensor's offset
   * @param rPose
   */
  inline void SetOffsetPose(const Pose2& rPose) { m_OffsetPose = rPose; }

  /**
   * Gets this range finder sensor's minimum range
   * @return minimum range
   */
  inline double GetMinimumRange() const { return m_MinimumRange; }

  /**
   * Sets this range finder sensor's minimum range
   * @param minimumRange
   */
  inline void SetMinimumRange(double minimumRange) {
    m_MinimumRange = minimumRange;

    SetRangeThreshold(GetRangeThreshold());
  }

  /**
   * Gets this range finder sensor's maximum range
   * @return maximum range
   */
  inline double GetMaximumRange() const { return m_MaximumRange; }

  /**
   * Sets this range finder sensor's maximum range
   * @param maximumRange
   */
  inline void SetMaximumRange(double maximumRange) {
    m_MaximumRange = maximumRange;

    SetRangeThreshold(GetRangeThreshold());
  }

  /**
   * Gets the range threshold
   * @return range threshold
   */
  inline double GetRangeThreshold() const { return m_RangeThreshold; }

  /**
   * Sets the range threshold
   * @param rangeThreshold
   */
  inline void SetRangeThreshold(double rangeThreshold) {
    // make sure rangeThreshold is within laser range finder range
    m_RangeThreshold =
        math::Clip(rangeThreshold, GetMinimumRange(), GetMaximumRange());

    if (math::DoubleEqual(GetRangeThreshold(), rangeThreshold) == false) {
      std::cout << "Info: clipped range threshold to be within minimum and "
                   "maximum range!"
                << std::endl;
    }
  }

  /**
   * Gets this range finder sensor's minimum angle
   * @return minimum angle
   */
  inline double GetMinimumAngle() const { return m_MinimumAngle; }

  /**
   * Sets this range finder sensor's minimum angle
   * @param minimumAngle
   */
  inline void SetMinimumAngle(double minimumAngle) {
    m_MinimumAngle = minimumAngle;

    Update();
  }

  /**
   * Gets this range finder sensor's maximum angle
   * @return maximum angle
   */
  inline double GetMaximumAngle() const { return m_MaximumAngle; }

  /**
   * Sets this range finder sensor's maximum angle
   * @param maximumAngle
   */
  inline void SetMaximumAngle(double maximumAngle) {
    m_MaximumAngle = maximumAngle;

    Update();
  }

  /**
   * Gets this range finder sensor's angular resolution
   * @return angular resolution
   */
  inline double GetAngularResolution() const { return m_AngularResolution; }

  /**
   * Sets this range finder sensor's angular resolution
   * @param angularResolution
   */
  inline void SetAngularResolution(double angularResolution) {
    m_AngularResolution = angularResolution;

    Update();
  }

  /**
   * Gets the number of range readings each localized range scan must contain to
   * be a valid scan.
   * @return number of range readings
   */
  inline int GetNumberOfRangeReadings() const {
    return m_NumberOfRangeReadings;
  }

  virtual bool Validate() {
    Update();

    if (math::InRange(GetRangeThreshold(), GetMinimumRange(),
                      GetMaximumRange()) == false) {
      std::cout << "Please set range threshold to a value between ["
                << GetMinimumRange() << ";" << GetMaximumRange() << "]"
                << std::endl;
      return false;
    }

    return true;
  }

  virtual bool Validate(LaserRangeScan* pLaserRangeScan) {
    // verify number of range readings in LaserRangeScan matches the number of
    // expected range readings
    if (pLaserRangeScan->GetNumberOfRangeReadings() !=
        GetNumberOfRangeReadings()) {
      std::cout << "LaserRangeScan contains "
                << pLaserRangeScan->GetNumberOfRangeReadings()
                << " range readings, expected " << GetNumberOfRangeReadings()
                << std::endl;
      return false;
    }
    return true;
  }

 public:
  /**
   * Create a laser range finder of the given type and ID
   * @param type
   * @param rName name of sensor - if no name is specified default name will be
   * assigned
   * @return laser range finder
   */
  static LaserRangeFinder* CreateLaserRangeFinder() {
    LaserRangeFinder* pLrf = NULL;

    pLrf = new LaserRangeFinder();

    // Sensing range is 80 meters.
    pLrf->m_MinimumRange = 0.0;
    pLrf->m_MaximumRange = 80.0;

    // 180 degree range
    pLrf->m_MinimumAngle = math::DegreesToRadians(-90);
    pLrf->m_MaximumAngle = math::DegreesToRadians(90);

    // 1.0 degree angular resolution
    pLrf->m_AngularResolution = math::DegreesToRadians(1.0);

    pLrf->m_NumberOfRangeReadings = 181;

    Pose2 defaultOffset;
    pLrf->SetOffsetPose(defaultOffset);

    return pLrf;
  }

 private:
  /**
   * Constructs a LaserRangeFinder object with given ID
   */
  LaserRangeFinder() : m_NumberOfRangeReadings(0) {
    m_MinimumRange = 0.0;
    m_MaximumRange = 80.0;

    m_MinimumAngle = -KT_PI_2;
    m_MaximumAngle = KT_PI_2;

    m_AngularResolution = math::DegreesToRadians(1);

    m_RangeThreshold = 12.0;
  }

  /**
   * Set the number of range readings based on the minimum and
   * maximum angles of the sensor and the angular resolution
   */
  void Update() {
    m_NumberOfRangeReadings =
        static_cast<int>(math::Round((GetMaximumAngle() - GetMinimumAngle()) /
                                     GetAngularResolution()) +
                         1);
  }

 private:
  LaserRangeFinder(const LaserRangeFinder&);
  const LaserRangeFinder& operator=(const LaserRangeFinder&);

 private:
  double m_MinimumAngle;
  double m_MaximumAngle;

  double m_AngularResolution;

  double m_MinimumRange;
  double m_MaximumRange;

  double m_RangeThreshold;

  int m_NumberOfRangeReadings;

  Pose2 m_OffsetPose;
};  // LaserRangeFinder

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * The LocalizedRangeScan contains range data from a single sweep of a laser
 * range finder sensor
 * in a two-dimensional space and position information. The odometer position is
 * the position
 * reported by the robot when the range data was recorded. The corrected
 * position is the position
 * calculated by the mapper (or localizer)
 */
class LocalizedRangeScan : public LaserRangeScan {
 public:
  /**
   * Constructs a range scan from the given range finder with the given readings
   */
  LocalizedRangeScan(LaserRangeFinder* pLaserRangeFinder,
                     const RangeReadingsVector& rReadings)
      : LaserRangeScan(rReadings),
        m_pLaserRangeFinder(pLaserRangeFinder),
        m_IsDirty(true) {
    SetLocalPointReadings();
  }

  /**
   * Destructor
   */
  virtual ~LocalizedRangeScan() {}

 private:
  mutable boost::shared_mutex m_Lock;

 public:
  /**
   * Gets the odometric pose of this scan
   * @return odometric pose of this scan
   */
  inline const Pose2& GetOdometricPose() const { return m_OdometricPose; }

  /**
   * Sets the odometric pose of this scan
   * @param rPose
   */
  inline void SetOdometricPose(const Pose2& rPose) { m_OdometricPose = rPose; }

  /**
   * Gets the (possibly corrected) robot pose at which this scan was taken.  The
   * corrected robot pose of the scan
   * is usually set by an external module such as a localization or mapping
   * module when it is determined
   * that the original pose was incorrect.  The external module will set the
   * correct pose based on
   * additional sensor data and any context information it has.  If the pose has
   * not been corrected,
   * a call to this method returns the same pose as GetOdometricPose().
   * @return corrected pose
   */
  inline const Pose2& GetCorrectedPose() const { return m_CorrectedPose; }

  /**
   * Moves the scan by moving the robot pose to the given location.
   * @param rPose new pose of the robot of this scan
   */
  inline void SetCorrectedPose(const Pose2& rPose) {
    m_CorrectedPose = rPose;

    m_IsDirty = true;
  }

  /**
   * Computes the position of the sensor
   * @return scan pose
   */
  inline Pose2 GetSensorPose() const { return GetSensorAt(m_CorrectedPose); }

  /**
   * Computes the robot pose given the corrected scan pose
   * @param rScanPose pose of the sensor
   */
  void SetSensorPose(const Pose2& rScanPose) {
    Pose2 deviceOffsetPose2 = GetLaserRangeFinder()->GetOffsetPose();
    double offsetLength = deviceOffsetPose2.GetPosition().Length();
    double offsetHeading = deviceOffsetPose2.GetHeading();
    double angleoffset =
        atan2(deviceOffsetPose2.GetY(), deviceOffsetPose2.GetX());
    double correctedHeading = math::NormalizeAngle(rScanPose.GetHeading());
    Pose2 worldSensorOffset = Pose2(
        offsetLength * cos(correctedHeading + angleoffset - offsetHeading),
        offsetLength * sin(correctedHeading + angleoffset - offsetHeading),
        offsetHeading);

    m_CorrectedPose = rScanPose - worldSensorOffset;

    Update();
  }

  /**
   * Computes the position of the sensor if the robot were at the given pose
   * @param rPose
   * @return sensor pose
   */
  inline Pose2 GetSensorAt(const Pose2& rPose) const {
    return Transform(rPose).TransformPose(
        GetLaserRangeFinder()->GetOffsetPose());
  }

  /**
   * Gets the bounding box of this scan
   * @return bounding box of this scan
   */
  inline const BoundingBox2& GetBoundingBox() const {
    boost::shared_lock<boost::shared_mutex> lock(m_Lock);
    if (m_IsDirty) {
      // throw away constness and do an update!
      lock.unlock();
      boost::unique_lock<boost::shared_mutex> uniqueLock(m_Lock);
      const_cast<LocalizedRangeScan*>(this)->Update();
    }

    return m_BoundingBox;
  }

  /**
   * Get point readings in global coordinates
   */
  inline const PointVectorDouble& GetPointReadings(
      bool wantFiltered = false) const {
    boost::shared_lock<boost::shared_mutex> lock(m_Lock);
    if (m_IsDirty) {
      // throw away constness and do an update!
      lock.unlock();
      boost::unique_lock<boost::shared_mutex> uniqueLock(m_Lock);
      const_cast<LocalizedRangeScan*>(this)->Update();
    }

    if (wantFiltered == true) {
      return m_PointReadings;
    } else {
      return m_UnfilteredPointReadings;
    }
  }

  /**
   * Get point readings in local coordinates
   */
  inline const PointVectorDouble& GetLocalPointReadings() const {
    return m_LocalPointReadings;
  }

  /**
   * Gets the laser range finder sensor that generated this scan
   * @return laser range finder sensor of this scan
   */
  inline LaserRangeFinder* GetLaserRangeFinder() const {
    return m_pLaserRangeFinder;
  }

 private:
  /**
   * Compute point readings based on range readings
   * Only range readings within [minimum range; range threshold] are returned
   */
  virtual void Update() {
    LaserRangeFinder* pLaserRangeFinder = GetLaserRangeFinder();

    if (pLaserRangeFinder != NULL) {
      m_PointReadings.clear();
      m_UnfilteredPointReadings.clear();

      double rangeThreshold = pLaserRangeFinder->GetRangeThreshold();
      double minimumAngle = pLaserRangeFinder->GetMinimumAngle();
      double angularResolution = pLaserRangeFinder->GetAngularResolution();
      Pose2 scanPose = GetSensorPose();

      // compute point readings
      int beamNum = 0;
      for (int i = 0; i < pLaserRangeFinder->GetNumberOfRangeReadings();
           i++, beamNum++) {
        double rangeReading = GetRangeReadings()[i];
        if (!math::InRange(rangeReading, pLaserRangeFinder->GetMinimumRange(),
                           rangeThreshold)) {
          double angle = scanPose.GetHeading() + minimumAngle +
                         beamNum * angularResolution;

          Vector2<double> point;
          point.SetX(scanPose.GetX() + (rangeReading * cos(angle)));
          point.SetY(scanPose.GetY() + (rangeReading * sin(angle)));

          m_UnfilteredPointReadings.push_back(point);
          continue;
        }

        double angle =
            scanPose.GetHeading() + minimumAngle + beamNum * angularResolution;

        Vector2<double> point;
        point.SetX(scanPose.GetX() + (rangeReading * cos(angle)));
        point.SetY(scanPose.GetY() + (rangeReading * sin(angle)));

        m_PointReadings.push_back(point);
        m_UnfilteredPointReadings.push_back(point);
      }

      // calculate bounding box of scan
      m_BoundingBox = BoundingBox2();
      m_BoundingBox.Add(scanPose.GetPosition());
      for (PointVectorDouble::iterator iter = m_PointReadings.begin();
           iter != m_PointReadings.end(); iter++) {
        m_BoundingBox.Add(*iter);
      }
    }

    m_IsDirty = false;
  }

  /**
   * Transform the range readings to point readings in the local coordinates
   */
  void SetLocalPointReadings() {
    LaserRangeFinder* pLaserRangeFinder = GetLaserRangeFinder();

    if (pLaserRangeFinder != NULL) {
      double rangeThreshold = pLaserRangeFinder->GetRangeThreshold();
      double minimumAngle = pLaserRangeFinder->GetMinimumAngle();
      double angularResolution = pLaserRangeFinder->GetAngularResolution();

      // compute point readings
      int beamNum = 0;
      for (int i = 0; i < pLaserRangeFinder->GetNumberOfRangeReadings();
           i++, beamNum++) {
        double rangeReading = GetRangeReadings()[i];
        if (!math::InRange(rangeReading, pLaserRangeFinder->GetMinimumRange(),
                           rangeThreshold)) {
          continue;
        }

        double angle = minimumAngle + beamNum * angularResolution;

        Vector2<double> point;
        point.SetX(rangeReading * cos(angle));
        point.SetY(rangeReading * sin(angle));

        m_LocalPointReadings.push_back(point);
      }
    }
  }

 private:
  LocalizedRangeScan(const LocalizedRangeScan&);
  const LocalizedRangeScan& operator=(const LocalizedRangeScan&);

 private:
  /**
   * Odometric pose of robot
   */
  Pose2 m_OdometricPose;

  /**
   * Corrected pose of robot calculated by mapper (or localizer)
   */
  Pose2 m_CorrectedPose;

 protected:
  /**
   * Vector of point readings
   */
  PointVectorDouble m_PointReadings;

  /**
   * Vector of unfiltered point readings
   */
  PointVectorDouble m_UnfilteredPointReadings;

  /**
   * Vector of point readings in local coordinates
   */
  PointVectorDouble m_LocalPointReadings;

  /**
   * Bounding box of localized range scan
   */
  BoundingBox2 m_BoundingBox;

  /**
   * Internal flag used to update point readings, barycenter and bounding box
   */
  bool m_IsDirty;

  LaserRangeFinder* m_pLaserRangeFinder;

};  // LocalizedRangeScan

/**
 * Type declaration of LocalizedRangeScan vector
 */
typedef std::vector<LocalizedRangeScan*> LocalizedRangeScanVector;

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Defines a grid class
 */
template <typename T>
class Grid {
 public:
  /**
   * Creates a grid of given size and resolution
   * @param width
   * @param height
   * @param resolution
   * @return grid pointer
   */
  static Grid* CreateGrid(int width, int height, double resolution) {
    Grid* pGrid = new Grid(width, height);

    pGrid->GetCoordinateConverter()->SetScale(1.0 / resolution);

    return pGrid;
  }

  /**
   * Destructor
   */
  virtual ~Grid() {
    delete[] m_pData;
    delete m_pCoordinateConverter;
  }

 public:
  /**
   * Clear out the grid data
   */
  void Clear() { memset(m_pData, 0, GetDataSize() * sizeof(T)); }

  /**
   * Returns a clone of this grid
   * @return grid clone
   */
  Grid* Clone() {
    Grid* pGrid = CreateGrid(GetWidth(), GetHeight(), GetResolution());
    pGrid->GetCoordinateConverter()->SetOffset(
        GetCoordinateConverter()->GetOffset());

    memcpy(pGrid->GetDataPointer(), GetDataPointer(), GetDataSize());

    return pGrid;
  }

  /**
   * Resizes the grid (deletes all old data)
   * @param width
   * @param height
   */
  virtual void Resize(int width, int height) {
    m_Width = width;  // m_Width = 31
    m_Height = height;
    m_WidthStep = math::AlignValue<int>(width, 8);  // m_WidthStep = 32

    if (m_pData != NULL) {
      delete[] m_pData;
      m_pData = NULL;
    }

    try {
      m_pData = new T[GetDataSize()];

      if (m_pCoordinateConverter == NULL) {
        m_pCoordinateConverter = new CoordinateConverter();
      }

      m_pCoordinateConverter->SetSize(Size2<int>(width, height));
    } catch (...) {
      m_pData = NULL;

      m_Width = 0;
      m_Height = 0;
      m_WidthStep = 0;
    }

    Clear();
  }

  /**
   * Checks whether the given coordinates are valid grid indices
   * @param rGrid
   */
  inline bool IsValidGridIndex(const Vector2<int>& rGrid) const {
    return (math::IsUpTo(rGrid.GetX(), m_Width) &&
            math::IsUpTo(rGrid.GetY(), m_Height));
  }

  /**
   * Gets the index into the data pointer of the given grid coordinate
   * @param rGrid
   * @param boundaryCheck default value is true
   * @return grid index
   */
  virtual int GridIndex(const Vector2<int>& rGrid,
                        bool boundaryCheck = true) const {
    if (boundaryCheck == true) {
      if (IsValidGridIndex(rGrid) == false) {
        std::stringstream error;
        error << "Index out of range.  Index must be between [0; " << m_Width
              << ") and [0; " << m_Height << ")";
        throw(error.str());
      }
    }

    int index = rGrid.GetX() + (rGrid.GetY() * m_WidthStep);

    if (boundaryCheck == true) {
      assert(math::IsUpTo(index, GetDataSize()));
    }

    return index;
  }

  /**
   * Gets the grid coordinate from an index
   * @param index
   * @return grid coordinate
   */
  Vector2<int> IndexToGrid(int index) const {
    Vector2<int> grid;

    grid.SetY(index / m_WidthStep);
    grid.SetX(index - grid.GetY() * m_WidthStep);

    return grid;
  }

  /**
   * Converts the point from world coordinates to grid coordinates
   * @param rWorld world coordinate
   * @param flipY
   * @return grid coordinate
   */
  inline Vector2<int> WorldToGrid(const Vector2<double>& rWorld,
                                  bool flipY = false) const {
    return GetCoordinateConverter()->WorldToGrid(rWorld, flipY);
  }

  /**
   * Converts the point from grid coordinates to world coordinates
   * @param rGrid world coordinate
   * @param flipY
   * @return world coordinate
   */
  inline Vector2<double> GridToWorld(const Vector2<int>& rGrid,
                                     bool flipY = false) const {
    return GetCoordinateConverter()->GridToWorld(rGrid, flipY);
  }

  /**
   * Gets pointer to data at given grid coordinate
   * @param rGrid grid coordinate
   * @return grid point
   */
  T* GetDataPointer(const Vector2<int>& rGrid) {
    int index = GridIndex(rGrid, true);
    return m_pData + index;
  }

  /**
   * Gets pointer to data at given grid coordinate
   * @param rGrid grid coordinate
   * @return grid point
   */
  T* GetDataPointer(const Vector2<int>& rGrid) const {
    int index = GridIndex(rGrid, true);
    return m_pData + index;
  }

  /**
   * Gets the width of the grid
   * @return width of the grid
   */
  inline int GetWidth() const { return m_Width; }

  /**
   * Gets the height of the grid
   * @return height of the grid
   */
  inline int GetHeight() const { return m_Height; }

  /**
   * Get the size as a Size2<int>
   * @return size of the grid
   */
  inline const Size2<int> GetSize() const {
    return Size2<int>(m_Width, m_Height);
  }

  /**
   * Gets the width step in bytes
   * @return width step
   */
  inline int GetWidthStep() const { return m_WidthStep; }

  /**
   * Gets the grid data pointer
   * @return data pointer
   */
  inline T* GetDataPointer() { return m_pData; }

  /**
   * Gets const grid data pointer
   * @return data pointer
   */
  inline T* GetDataPointer() const { return m_pData; }

  /**
   * Gets the allocated grid size in bytes
   * @return data size
   */
  inline int GetDataSize() const { return m_WidthStep * m_Height; }

  /**
   * Get value at given grid coordinate
   * @param rGrid grid coordinate
   * @return value
   */
  inline T GetValue(const Vector2<int>& rGrid) const {
    int index = GridIndex(rGrid);
    return m_pData[index];
  }

  /**
   * Gets the coordinate converter for this grid
   * @return coordinate converter
   */
  inline CoordinateConverter* GetCoordinateConverter() const {
    return m_pCoordinateConverter;
  }

  /**
   * Gets the resolution
   * @return resolution
   */
  inline double GetResolution() const {
    return GetCoordinateConverter()->GetResolution();
  }

  /**
   * Gets the grids bounding box
   * @return bounding box
   */
  inline BoundingBox2 GetBoundingBox() const {
    return GetCoordinateConverter()->GetBoundingBox();
  }

  /**
   * Increments all the grid cells from (x0, y0) to (x1, y1);
   * if applicable, apply f to each cell traced
   * @param x0
   * @param y0
   * @param x1
   * @param y1
   * @param f
   */
  void TraceLine(int x0, int y0, int x1, int y1) {
    bool steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep) {
      std::swap(x0, y0);
      std::swap(x1, y1);
    }
    if (x0 > x1) {
      std::swap(x0, x1);
      std::swap(y0, y1);
    }

    int deltaX = x1 - x0;
    int deltaY = abs(y1 - y0);
    int error = 0;
    int ystep;
    int y = y0;

    if (y0 < y1) {
      ystep = 1;
    } else {
      ystep = -1;
    }

    int pointX;
    int pointY;
    for (int x = x0; x <= x1; x++) {
      if (steep) {
        pointX = y;
        pointY = x;
      } else {
        pointX = x;
        pointY = y;
      }

      error += deltaY;

      if (2 * error >= deltaX) {
        y += ystep;
        error -= deltaX;
      }

      Vector2<int> gridIndex(pointX, pointY);
      if (IsValidGridIndex(gridIndex)) {
        int index = GridIndex(gridIndex, false);
        T* pGridPointer = GetDataPointer();
        pGridPointer[index]++;
      }
    }
  }

 protected:
  /**
   * Constructs grid of given size
   * @param width
   * @param height
   */
  Grid(int width, int height) : m_pData(NULL), m_pCoordinateConverter(NULL) {
    Resize(width, height);
  }

 private:
  int m_Width;      // width of grid
  int m_Height;     // height of grid
  int m_WidthStep;  // 8 bit aligned width of grid
  T* m_pData;       // grid data

  // coordinate converter to convert between world coordinates and grid
  // coordinates
  CoordinateConverter* m_pCoordinateConverter;
};  // Grid

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Occupancy grid definition. See GridStates for possible grid values.
 */
class OccupancyGrid : public Grid<int> {
 public:
  /**
   * Constructs an occupancy grid of given size
   * @param width
   * @param height
   * @param rOffset
   * @param resolution
   */
  OccupancyGrid(int width, int height, const Vector2<double>& rOffset,
                double resolution)
      : Grid<int>(width, height),
        m_pCellPassCnt(Grid<int>::CreateGrid(0, 0, resolution)),
        m_pCellHitsCnt(Grid<int>::CreateGrid(0, 0, resolution)) {
    if (math::DoubleEqual(resolution, 0.0)) {
      throw("Resolution cannot be 0");
    }

    m_MinPassThrough = 2;
    m_OccupancyThreshold = 0.1;

    GetCoordinateConverter()->SetScale(1.0 / resolution);
    GetCoordinateConverter()->SetOffset(rOffset);
  }

  /**
   * Destructor
   */
  virtual ~OccupancyGrid() {
    delete m_pCellPassCnt;
    delete m_pCellHitsCnt;
  }

 public:
  /**
   * Create an occupancy grid from the given scans using the given resolution
   * @param rScans
   * @param resolution
   */
  static OccupancyGrid* CreateFromScans(const LocalizedRangeScanVector& rScans,
                                        double resolution) {
    if (rScans.empty()) {
      return NULL;
    }

    int width, height;
    Vector2<double> offset;
    ComputeDimensions(rScans, resolution, width, height, offset);
    OccupancyGrid* pOccupancyGrid =
        new OccupancyGrid(width, height, offset, resolution);
    pOccupancyGrid->CreateFromScans(rScans);

    return pOccupancyGrid;
  }

  /**
   * Make a clone
   * @return occupancy grid clone
   */
  OccupancyGrid* Clone() const {
    OccupancyGrid* pOccupancyGrid = new OccupancyGrid(
        GetWidth(), GetHeight(), GetCoordinateConverter()->GetOffset(),
        1.0 / GetCoordinateConverter()->GetScale());
    memcpy(pOccupancyGrid->GetDataPointer(), GetDataPointer(), GetDataSize());

    pOccupancyGrid->GetCoordinateConverter()->SetSize(
        GetCoordinateConverter()->GetSize());
    pOccupancyGrid->m_pCellPassCnt = m_pCellPassCnt->Clone();
    pOccupancyGrid->m_pCellHitsCnt = m_pCellHitsCnt->Clone();

    return pOccupancyGrid;
  }

  /**
   * Check if grid point is free
   * @param rPose
   * @return whether the cell at the given point is free space
   */
  virtual bool IsFree(const Vector2<int>& rPose) const {
    int* pOffsets = reinterpret_cast<int*>(GetDataPointer(rPose));
    if (*pOffsets == GridStates_Free) {
      return true;
    }

    return false;
  }

  /**
   * Casts a ray from the given point (up to the given max range)
   * and returns the distance to the closest obstacle
   * @param rPose2
   * @param maxRange
   * @return distance to closest obstacle
   */
  virtual double RayCast(const Pose2& rPose2, double maxRange) const {
    double scale = GetCoordinateConverter()->GetScale();

    double x = rPose2.GetX();
    double y = rPose2.GetY();
    double theta = rPose2.GetHeading();

    double sinTheta = sin(theta);
    double cosTheta = cos(theta);

    double xStop = x + maxRange * cosTheta;
    double xSteps = 1 + fabs(xStop - x) * scale;

    double yStop = y + maxRange * sinTheta;
    double ySteps = 1 + fabs(yStop - y) * scale;

    double steps = math::Maximum(xSteps, ySteps);
    double delta = maxRange / steps;
    double distance = delta;

    for (int i = 1; i < steps; i++) {
      double x1 = x + distance * cosTheta;
      double y1 = y + distance * sinTheta;

      Vector2<int> gridIndex = WorldToGrid(Vector2<double>(x1, y1));
      if (IsValidGridIndex(gridIndex) && IsFree(gridIndex)) {
        distance = (i + 1) * delta;
      } else {
        break;
      }
    }

    return (distance < maxRange) ? distance : maxRange;
  }

  /**
   * Sets the minimum number of beams that must pass through a cell before it
   * will be considered to be occupied or unoccupied.
   * This prevents stray beams from messing up the map.
   */
  void SetMinPassThrough(int count) { m_MinPassThrough = count; }

  /**
   * Sets the minimum ratio of beams hitting cell to beams passing through
   * cell for cell to be marked as occupied.
   */
  void SetOccupancyThreshold(double thresh) { m_OccupancyThreshold = thresh; }

 protected:
  /**
   * Get cell hit grid
   * @return Grid<int>*
   */
  virtual Grid<int>* GetCellHitsCounts() { return m_pCellHitsCnt; }

  /**
   * Get cell pass grid
   * @return Grid<int>*
   */
  virtual Grid<int>* GetCellPassCounts() { return m_pCellPassCnt; }

 protected:
  /**
   * Calculate grid dimensions from localized range scans
   * @param rScans
   * @param resolution
   * @param rWidth
   * @param rHeight
   * @param rOffset
   */
  static void ComputeDimensions(const LocalizedRangeScanVector& rScans,
                                double resolution, int& rWidth, int& rHeight,
                                Vector2<double>& rOffset) {
    BoundingBox2 boundingBox;
    for (LocalizedRangeScanVector::const_iterator iter = rScans.begin();
         iter != rScans.end(); iter++) {
      boundingBox.Add((*iter)->GetBoundingBox());
    }

    double scale = 1.0 / resolution;
    Size2<double> size = boundingBox.GetSize();

    rWidth = static_cast<int>(math::Round(size.GetWidth() * scale));
    rHeight = static_cast<int>(math::Round(size.GetHeight() * scale));
    rOffset = boundingBox.GetMinimum();
  }

  /**
   * Create grid using scans
   * @param rScans
   */
  virtual void CreateFromScans(const LocalizedRangeScanVector& rScans) {
    m_pCellPassCnt->Resize(GetWidth(), GetHeight());
    m_pCellPassCnt->GetCoordinateConverter()->SetOffset(
        GetCoordinateConverter()->GetOffset());

    m_pCellHitsCnt->Resize(GetWidth(), GetHeight());
    m_pCellHitsCnt->GetCoordinateConverter()->SetOffset(
        GetCoordinateConverter()->GetOffset());

    for (LocalizedRangeScanVector::const_iterator iter = rScans.begin();
         iter != rScans.end(); iter++) {
      LocalizedRangeScan* pScan = *iter;
      AddScan(pScan);
    }

    Update();
  }

  /**
   * Adds the scan's information to this grid's counters (optionally
   * update the grid's cells' occupancy status)
   * @param pScan
   * @param doUpdate whether to update the grid's cell's occupancy status
   * @return returns false if an endpoint fell off the grid, otherwise true
   */
  virtual bool AddScan(LocalizedRangeScan* pScan, bool doUpdate = false) {
    double rangeThreshold = pScan->GetLaserRangeFinder()->GetRangeThreshold();
    double maxRange = pScan->GetLaserRangeFinder()->GetMaximumRange();
    double minRange = pScan->GetLaserRangeFinder()->GetMinimumRange();

    Vector2<double> scanPosition = pScan->GetSensorPose().GetPosition();

    // get scan point readings
    const PointVectorDouble& rPointReadings = pScan->GetPointReadings(false);

    bool isAllInMap = true;

    // draw lines from scan position to all point readings
    int pointIndex = 0;
    for (PointVectorDouble::const_iterator iter = rPointReadings.begin();
         iter != rPointReadings.end(); iter++) {
      Vector2<double> point = *iter;
      double rangeReading = pScan->GetRangeReadings()[pointIndex];
      bool isEndPointValid = rangeReading < (rangeThreshold - KT_TOLERANCE);

      if (rangeReading <= minRange || rangeReading >= maxRange ||
          std::isnan(rangeReading)) {
        // ignore these readings
        pointIndex++;
        continue;
      } else if (rangeReading >= rangeThreshold) {
        // trace up to range reading
        double ratio = rangeThreshold / rangeReading;
        double dx = point.GetX() - scanPosition.GetX();
        double dy = point.GetY() - scanPosition.GetY();
        point.SetX(scanPosition.GetX() + ratio * dx);
        point.SetY(scanPosition.GetY() + ratio * dy);
      }

      bool isInMap = RayTrace(scanPosition, point, isEndPointValid, doUpdate);
      if (!isInMap) {
        isAllInMap = false;
      }

      pointIndex++;
    }

    return isAllInMap;
  }

  /**
   * Traces a beam from the start position to the end position marking
   * the bookkeeping arrays accordingly.
   * @param rWorldFrom start position of beam
   * @param rWorldTo end position of beam
   * @param isEndPointValid is the reading within the range threshold?
   * @param doUpdate whether to update the cells' occupancy status immediately
   * @return returns false if an endpoint fell off the grid, otherwise true
   */
  virtual bool RayTrace(const Vector2<double>& rWorldFrom,
                        const Vector2<double>& rWorldTo, bool isEndPointValid,
                        bool doUpdate = false) {
    assert(m_pCellPassCnt != NULL && m_pCellHitsCnt != NULL);

    Vector2<int> gridFrom = m_pCellPassCnt->WorldToGrid(rWorldFrom);
    Vector2<int> gridTo = m_pCellPassCnt->WorldToGrid(rWorldTo);

    // CellUpdater* pCellUpdater = doUpdate ? m_pCellUpdater : NULL;
    m_pCellPassCnt->TraceLine(gridFrom.GetX(), gridFrom.GetY(), gridTo.GetX(),
                              gridTo.GetY());

    // for the end point
    if (isEndPointValid) {
      if (m_pCellPassCnt->IsValidGridIndex(gridTo)) {
        int index = m_pCellPassCnt->GridIndex(gridTo, false);

        int* pCellPassCntPtr = m_pCellPassCnt->GetDataPointer();
        int* pCellHitCntPtr = m_pCellHitsCnt->GetDataPointer();

        // increment cell pass through and hit count
        pCellPassCntPtr[index]++;
        pCellHitCntPtr[index]++;
      }
    }

    return m_pCellPassCnt->IsValidGridIndex(gridTo);
  }

  /**
   * Updates a single cell's value based on the given counters
   * @param pCell
   * @param cellPassCnt
   * @param cellHitCnt
   */
  virtual void UpdateCell(int* pCell, int cellPassCnt, int cellHitCnt) {
    if (cellPassCnt > m_MinPassThrough) {
      double hitRatio =
          static_cast<double>(cellHitCnt) / static_cast<double>(cellPassCnt);

      if (hitRatio > m_OccupancyThreshold) {
        *pCell = GridStates_Occupied;
      } else {
        *pCell = GridStates_Free;
      }
    }
  }

  /**
   * Update the grid based on the values in m_pCellHitsCnt and m_pCellPassCnt
   */
  virtual void Update() {
    assert(m_pCellPassCnt != NULL && m_pCellHitsCnt != NULL);

    // clear grid
    Clear();

    // set occupancy status of cells
    int* pDataPtr = GetDataPointer();
    int* pCellPassCntPtr = m_pCellPassCnt->GetDataPointer();
    int* pCellHitCntPtr = m_pCellHitsCnt->GetDataPointer();

    int nBytes = GetDataSize();
    for (int i = 0; i < nBytes;
         i++, pDataPtr++, pCellPassCntPtr++, pCellHitCntPtr++) {
      UpdateCell(pDataPtr, *pCellPassCntPtr, *pCellHitCntPtr);
    }
  }

  /**
   * Resizes the grid (deletes all old data)
   * @param width
   * @param height
   */
  virtual void Resize(int width, int height) {
    Grid<int>::Resize(width, height);
    m_pCellPassCnt->Resize(width, height);
    m_pCellHitsCnt->Resize(width, height);
  }

 protected:
  /**
   * Counters of number of times a beam passed through a cell
   */
  Grid<int>* m_pCellPassCnt;

  /**
   * Counters of number of times a beam ended at a cell
   */
  Grid<int>* m_pCellHitsCnt;

 private:
  /**
   * Restrict the copy constructor
   */
  OccupancyGrid(const OccupancyGrid&);

  /**
   * Restrict the assignment operator
   */
  const OccupancyGrid& operator=(const OccupancyGrid&);

 private:
  ////////////////////////////////////////////////////////////
  // NOTE: These two values are dependent on the resolution.  If the resolution
  // is too small,
  // then not many beams will hit the cell!

  // Number of beams that must pass through a cell before it will be considered
  // to be occupied
  // or unoccupied.  This prevents stray beams from messing up the map.
  int m_MinPassThrough;

  // Minimum ratio of beams hitting cell to beams passing through cell for cell
  // to be marked as occupied
  double m_OccupancyThreshold;
};  // OccupancyGrid

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Manages the scan data for a device
 */
class ScanManager {
 public:
  /**
   * Default constructor
   */
  ScanManager(int runningBufferMaximumSize, double runningBufferMaximumDistance)
      : m_pLastScan(NULL),
        m_RunningBufferMaximumSize(runningBufferMaximumSize),
        m_RunningBufferMaximumDistance(runningBufferMaximumDistance) {}

  /**
   * Destructor
   */
  virtual ~ScanManager() { Clear(); }

 public:
  /**
   * Adds scan to vector of processed scans tagging scan with given unique id
   * @param pScan
   */
  inline void AddScan(LocalizedRangeScan* pScan) {
    // add it to scan buffer
    m_Scans.push_back(pScan);
  }

  /**
   * Gets last scan
   * @param deviceId
   * @return last localized range scan
   */
  inline LocalizedRangeScan* GetLastScan() { return m_pLastScan; }

  /**
   * Sets the last scan
   * @param pScan
   */
  inline void SetLastScan(LocalizedRangeScan* pScan) { m_pLastScan = pScan; }

  /**
   * Gets scans
   * @return scans
   */
  inline LocalizedRangeScanVector& GetScans() { return m_Scans; }

  /**
   * Gets running scans
   * @return running scans
   */
  inline LocalizedRangeScanVector& GetRunningScans() { return m_RunningScans; }

  /**
   * Adds scan to vector of running scans
   * @param pScan
   */
  void AddRunningScan(LocalizedRangeScan* pScan) {
    m_RunningScans.push_back(pScan);

    // vector has at least one element (first line of this function), so this is
    // valid
    Pose2 frontScanPose = m_RunningScans.front()->GetSensorPose();
    Pose2 backScanPose = m_RunningScans.back()->GetSensorPose();

    // cap vector size and remove all scans from front of vector that are too
    // far from end of vector
    double squaredDistance =
        frontScanPose.GetPosition().SquaredDistance(backScanPose.GetPosition());
    while (m_RunningScans.size() > m_RunningBufferMaximumSize ||
           squaredDistance >
               math::Square(m_RunningBufferMaximumDistance) - KT_TOLERANCE) {
      // remove front of running scans
      m_RunningScans.erase(m_RunningScans.begin());

      // recompute stats of running scans
      frontScanPose = m_RunningScans.front()->GetSensorPose();
      backScanPose = m_RunningScans.back()->GetSensorPose();
      squaredDistance = frontScanPose.GetPosition().SquaredDistance(
          backScanPose.GetPosition());
    }
  }

  /**
   * Deletes data of this buffered device
   */
  void Clear() {
    m_Scans.clear();
    m_RunningScans.clear();
  }

 private:
  LocalizedRangeScanVector m_Scans;
  LocalizedRangeScanVector m_RunningScans;
  LocalizedRangeScan* m_pLastScan;

  int m_RunningBufferMaximumSize;
  double m_RunningBufferMaximumDistance;
};  // ScanManager

}  // namespace scan_tools

#endif  // LASER_SCAN_MATCHER_KARTO_TOOLS_H_
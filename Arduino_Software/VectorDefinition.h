/*
 * Vector definitions
 * 
 * Standard maths definitions
 */

#pragma once

struct Vector3 {
  float x, y, z;
  Vector3(): x(0), y(0), z(0) {}
  Vector3(float x, float y, float z): x{x}, y{y}, z{z} {}

  // Overload * operator to scale by float
  Vector3 operator*(float scalar) const {
      return Vector3(x * scalar, y * scalar, z * scalar);
  }

  // Overload += operator to add another Vector3
  Vector3& operator+=(const Vector3& other) {
      x += other.x;
      y += other.y;
      z += other.z;
      return *this;
  }

  /*/ Multiply vector with float
  Vector3 operator * (float d) {
    Vector3 P;
    P.x = x * d;
    P.y = y * d;
    P.z = z * d;
    return P;
  }

  // Integrate vector
  Vector3 * operator += (Vector3 Vector) {
    x += Vector.x;
    y += Vector.y;
    z += Vector.z;
    return this;
  }*/
};


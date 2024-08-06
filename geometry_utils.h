#pragma once

#define _USE_MATH_DEFINES

#include <iostream>
#include <memory>
#include <vector>

const double EPS = 1e-9;

struct TPoint3D {
 public:
  double x;
  double y;
  double z;

  bool operator==(const TPoint3D& other) const {
    static const double eps = std::numeric_limits<double>::epsilon();

    return fabs(x - other.x) < eps && fabs(y - other.y) < eps &&
           fabs(z - other.z) < eps;
  }

  TPoint3D Cross(const TPoint3D& v2) const {
    return TPoint3D{y * v2.z - z * v2.y, z * v2.x - x * v2.z,
                    x * v2.y - y * v2.x};
  }

  double Dot(const TPoint3D& v2) const {
    return x * v2.x + y * v2.y + z * v2.z;
  }

  TPoint3D operator-(const TPoint3D& v2) const {
    return TPoint3D{x - v2.x, y - v2.y, z - v2.z};
  }

  TPoint3D operator*(double scalar) const {
    return {x * scalar, y * scalar, z * scalar};
  }

  double Lenght() const { return sqrt(x * x + y * y + z * z); }

  void Dump() const {
    std::cout << "TPoint3D(" << x << ", " << y << ", " << z << ")" << std::endl;
  }
};

bool CheckCollinear(const TPoint3D& s, const TPoint3D& q, const TPoint3D& p);
bool CheckSeparatingAxis(const TPoint3D& axis,
                         const std::vector<TPoint3D>& vertices1,
                         const std::vector<TPoint3D>& vertices2);

double Distance(const TPoint3D& a, const TPoint3D& b);
double LengthSquared(const TPoint3D& A, const TPoint3D& B);
double Angle(const TPoint3D& a, const TPoint3D& b);

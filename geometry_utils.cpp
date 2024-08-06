#include "geometry_utils.h"

bool CheckCollinear(const TPoint3D& s, const TPoint3D& q, const TPoint3D& p) {
  double r1 = (s.x - q.x) * (p.y - q.y) - (s.y - q.y) * (p.x - q.x);
  double r2 = (s.x - q.x) * (p.z - q.z) - (s.z - q.z) * (p.x - q.x);
  double r3 = (s.y - q.y) * (p.z - q.z) - (s.z - q.z) * (p.y - q.y);
  return fabs(r1) < EPS && fabs(r2) < EPS && fabs(r3) < EPS;
}

bool CheckSeparatingAxis(const TPoint3D& axis,
                         const std::vector<TPoint3D>& vertices1,
                         const std::vector<TPoint3D>& vertices2) {
  double min1 = std::numeric_limits<double>::max();
  double max1 = std::numeric_limits<double>::min();
  double min2 = std::numeric_limits<double>::max();
  double max2 = std::numeric_limits<double>::min();

  auto computeProjections = [&](const std::vector<TPoint3D>& vertices,
                                double& min, double& max) {
    for (const auto& vertex : vertices) {
      double projection = axis.Dot(vertex);
      min = std::min(min, projection);
      max = std::max(max, projection);
    }
  };

  computeProjections(vertices1, min1, max1);
  computeProjections(vertices2, min2, max2);

  return max1 < min2 || max2 < min1;
}

double Distance(const TPoint3D& a, const TPoint3D& b) {
  return sqrt(pow(a.x - b.x, 2.) + pow(a.y - b.y, 2.) + pow(a.z - b.z, 2.));
}

double LengthSquared(const TPoint3D& A, const TPoint3D& B) {
  double dx = B.x - A.x;
  double dy = B.y - A.y;
  double dz = B.z - A.z;
  return dx * dx + dy * dy + dz * dz;
}

double Angle(const TPoint3D& a, const TPoint3D& b) {
  double dotProduct = a.Dot(b);

  double thisLength = a.Lenght();
  double otherLength = b.Lenght();

  double angle = acos(dotProduct / (thisLength * otherLength));

  return angle;
}

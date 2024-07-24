#define _USE_MATH_DEFINES

#include <algorithm>
#include <cmath>
#include <iostream>

#include "geometry_utils.h"
#include "intersections.h"
#include "point3D.h"
#include "segment3D.h"
#include "triangle3D.h"

namespace NS_Intersections {
bool CheckIntersection(const TTriangle3D& triangle1,
                       const TTriangle3D& triangle2) {
  auto edges1 = triangle1.GetEdges();
  auto edges2 = triangle2.GetEdges();

  std::vector<TPoint3D> axes;

  axes.emplace_back(edges1[0].Cross(edges1[1]));
  axes.emplace_back(edges2[0].Cross(edges2[1]));

  for (const auto& edge1 : edges1) {
    for (const auto& edge2 : edges2) {
      axes.emplace_back(edge1.Cross(edge2));
    }
  }

  const auto vertexies1 = triangle1.GetVertices();
  const auto vertexies2 = triangle2.GetVertices();

  if (std::any_of(axes.begin(), axes.end(), [&](const TPoint3D& axis) {
        return CheckSeparatingAxis(axis, vertexies1, vertexies2);
      })) {
    return false;
  }

  return true;
}

bool CheckIntersection(const TSegment3D& segment, const TTriangle3D& triangle) {
  std::vector<TPoint3D> axes;

  auto edges = triangle.GetEdges();

  auto normal = edges[0].Cross(edges[1]);
  axes.emplace_back(normal);

  auto dir = segment.getDirection();
  for (const auto& edge : edges) {
    axes.emplace_back(dir.Cross(edge));
  }

  for (const auto& axis : axes) {
    if (CheckSeparatingAxis(axis, segment.getPoints(),
                            triangle.GetVertices())) {
      return false;
    }
  }

  const auto segmentVertexies = segment.getPoints();
  const auto triangleVertexies = triangle.GetVertices();

  if (std::any_of(axes.begin(), axes.end(), [&](const TPoint3D& axis) {
        return CheckSeparatingAxis(axis, segmentVertexies, triangleVertexies);
      })) {
    return false;
  }

  return true;
}

bool CheckIntersection(const TTriangle3D& triangle,
                       const TPointObject3D& point) {
  auto vector1 = triangle.GetVertices()[1] - triangle.GetVertices()[0];
  auto vector2 = triangle.GetVertices()[2] - triangle.GetVertices()[0];

  auto triangleNormal = vector1.Cross(vector2);

  if ((point.point_ - triangle.GetVertices()[0]).Dot(triangleNormal) != 0) {
    return false;
  }

  auto vectorToPoint1 = triangle.GetVertices()[0] - point.point_;
  auto vectorToPoint2 = triangle.GetVertices()[1] - point.point_;
  auto vectorToPoint3 = triangle.GetVertices()[2] - point.point_;

  double angle1 = Angle(vectorToPoint1, vectorToPoint2);
  double angle2 = Angle(vectorToPoint2, vectorToPoint3);
  double angle3 = Angle(vectorToPoint3, vectorToPoint1);

  double totalAngle = angle1 + angle2 + angle3;
  constexpr auto angleTolerance = 0.0000001;
  return (std::abs(totalAngle - 2 * M_PI) < angleTolerance);
}

bool CheckIntersection(const TSegment3D& segment1, const TSegment3D& segment2) {
  TPoint3D p13, p43, p21;
  auto p1 = segment1.P();
  auto p2 = segment1.Q();
  auto p3 = segment2.P();
  auto p4 = segment2.Q();
  double d1343, d4321, d1321, d4343, d2121;
  double numer, denom;

  p13.x = p1.x - p3.x;
  p13.y = p1.y - p3.y;
  p13.z = p1.z - p3.z;
  p43.x = p4.x - p3.x;
  p43.y = p4.y - p3.y;
  p43.z = p4.z - p3.z;
  if (std::abs(p43.x) < EPS && std::abs(p43.y) < EPS && std::abs(p43.z) < EPS)
    return false;

  p21.x = p2.x - p1.x;
  p21.y = p2.y - p1.y;
  p21.z = p2.z - p1.z;
  if (std::abs(p21.x) < EPS && std::abs(p21.y) < EPS && std::abs(p21.z) < EPS)
    return false;

  d1343 = p13.x * p43.x + p13.y * p43.y + p13.z * p43.z;
  d4321 = p43.x * p21.x + p43.y * p21.y + p43.z * p21.z;
  d1321 = p13.x * p21.x + p13.y * p21.y + p13.z * p21.z;
  d4343 = p43.x * p43.x + p43.y * p43.y + p43.z * p43.z;
  d2121 = p21.x * p21.x + p21.y * p21.y + p21.z * p21.z;

  denom = d2121 * d4343 - d4321 * d4321;
  if (std::abs(denom) < EPS) {
    auto point1 = TPointObject3D(segment2.Q());
    auto point2 = TPointObject3D(segment2.P());
    if (CheckIntersection(segment1, point1) ||
        CheckIntersection(segment1, point2)) {
      return true;
    } else {
      return false;
    }
  }
  numer = d1343 * d4321 - d1321 * d4343;

  auto mua = numer / denom;
  auto mub = (d1343 + d4321 * mua) / d4343;

  if (mua < 0 || mua > 1 || mub < 0 || mub > 1) {
    return false;
  }

  return true;
}

bool CheckIntersection(const TSegment3D& segment, const TPointObject3D& point) {
  constexpr auto distanceTolerance = 0.0000001;
  return std::fabs(Distance(segment.P(), point.point_) +
                   Distance(point.point_, segment.Q()) -
                   Distance(segment.P(), segment.Q())) < distanceTolerance;
}

bool CheckIntersection(const TPointObject3D& point1,
                       const TPointObject3D& point2) {
  return point1.point_ == point2.point_;
}

bool CheckIntersection(const TTriangle3D& triangle, const TSegment3D& segment) {
  return CheckIntersection(segment, triangle);
}

bool CheckIntersection(const TPointObject3D& point, const TSegment3D& segment) {
  return CheckIntersection(segment, point);
}

bool CheckIntersection(const TPointObject3D& point,
                       const TTriangle3D& triangle) {
  return CheckIntersection(triangle, point);
}
}  // namespace NS_Intersections

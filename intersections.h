#pragma once

#include <vector>

class TTriangle3D;
class TSegment3D;
class TPointObject3D;

namespace NS_Intersections {
// The family of these functions checks whether triangles, segments and points
// are intersecting or not  (have at least 1 common point)
// The Math behind these functions for triangles is described by the link
// https://en.wikipedia.org/wiki/Hyperplane_separation_theorem
// and for segments https://paulbourke.net/geometry/pointlineplane/#i2l
bool CheckIntersection(const TTriangle3D& triangle1,
                       const TTriangle3D& triangle2);
bool CheckIntersection(const TSegment3D& segment, const TTriangle3D& triangle);
bool CheckIntersection(const TTriangle3D& triangle,
                       const TPointObject3D& point);
bool CheckIntersection(const TSegment3D& segment1, const TSegment3D& segment2);
bool CheckIntersection(const TSegment3D& segment, const TPointObject3D& point);
bool CheckIntersection(const TPointObject3D& point1,
                       const TPointObject3D& point2);
bool CheckIntersection(const TTriangle3D& triangle, const TSegment3D& segment);
bool CheckIntersection(const TPointObject3D& point, const TSegment3D& segment);
bool CheckIntersection(const TPointObject3D& point,
                       const TTriangle3D& triangle);
}  // namespace NS_Intersections

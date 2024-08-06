#pragma once

#include "intersections.h"
#include "object3D.h"

#include <limits>

class TPointObject3D : public IObject {
 public:
  TPointObject3D() = default;
  TPointObject3D(double x, double y, double z) : point_{x, y, z} {}
  TPointObject3D(const TPoint3D& point) : point_(point) {}

 public:
  bool Intersects(std::shared_ptr<IObject> other) const override {
    return other->Intersects(*this);
  }

  bool Intersects(const TPointObject3D& Dot) const override {
    return NS_Intersections::CheckIntersection(*this, Dot);
  }

  bool Intersects(const TSegment3D& segment) const override {
    return NS_Intersections::CheckIntersection(*this, segment);
  }

  bool Intersects(const TTriangle3D& triangle) const override {
    return NS_Intersections::CheckIntersection(*this, triangle);
  }

 public:
  TPoint3D point_;
};

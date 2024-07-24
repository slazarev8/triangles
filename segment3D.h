#pragma once

#include "geometry_utils.h"
#include "intersections.h"
#include "object3D.h"

class TSegment3D : public IObject {
 public:
  TSegment3D(const TPoint3D& first, const TPoint3D& second)
      : first_(first), second_(second) {}

  bool Intersects(std::shared_ptr<IObject> other) const override {
    return other->IntersectWithSegment(*this);
  }

  bool IntersectWithDot(const TPointObject3D& Dot) const override {
    return NS_Intersections::CheckIntersection(*this, Dot);
  }

  bool IntersectWithSegment(const TSegment3D& segment) const override {
    return NS_Intersections::CheckIntersection(*this, segment);
  }

  bool IntersectWithTriangle(const TTriangle3D& triangle) const override {
    return NS_Intersections::CheckIntersection(*this, triangle);
  }

  TPoint3D getDirection() const {
    return {second_.x - first_.x, second_.y - first_.y, second_.z - first_.z};
  }

  std::vector<TPoint3D> getPoints() const { return {first_, second_}; }

  TPoint3D P() const { return first_; }
  TPoint3D Q() const { return second_; }

 private:
  TPoint3D first_;
  TPoint3D second_;
};

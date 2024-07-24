#pragma once

#include "intersections.h"
#include "object3D.h"
#include "point3D.h"

class TTriangle3D : public IObject {
 public:
  TTriangle3D(const TPoint3D& s, const TPoint3D& q, const TPoint3D& p) {
    points_ = {s, q, p};
    edges_ = {points_[1] - points_[0], points_[2] - points_[1],
              points_[0] - points_[2]};
  }

  bool Intersects(std::shared_ptr<IObject> other) const override {
    return other->IntersectWithTriangle(*this);
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

  const std::vector<TPoint3D>& GetEdges() const { return edges_; }
  const std::vector<TPoint3D>& GetVertices() const { return points_; }

 private:
  std::vector<TPoint3D> points_;
  std::vector<TPoint3D> edges_;
};

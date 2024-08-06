#pragma once

class TPointObject3D;
class TSegment3D;
class TTriangle3D;

class IObject {
 public:
  virtual bool Intersects(std::shared_ptr<IObject> other) const = 0;
  virtual bool Intersects(const TPointObject3D& dot) const = 0;
  virtual bool Intersects(const TSegment3D& segment) const = 0;
  virtual bool Intersects(const TTriangle3D& triangle) const = 0;

  virtual ~IObject() = default;
};

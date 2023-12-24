// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "FlatBoundingBox.hpp"
#include "FlatRay.hpp"
#include "Math/FastMath.hpp"
#include "Math/Util.hpp"

#include <algorithm>

static constexpr unsigned
Distance1D(int a1, int a2, int b1, int b2) noexcept
{
  return a2 < b1
    ? b1 - a2 /* A is left of B */
    : (a1 >= b2
       ? a1 - b2 /* A is right of B */
       : 0); /* A and B overlap */
}

unsigned
FlatBoundingBox::SquareDistanceTo(FlatGeoPoint p) const noexcept
{
  if (IsInside(p))
    return 0;

  unsigned dx = Distance1D(lower_left.x, upper_right.x, p.x, p.x);
  unsigned dy = Distance1D(lower_left.y, upper_right.y, p.y, p.y);
  return Square(dx) + Square(dy);
}

unsigned
FlatBoundingBox::Distance(const FlatBoundingBox &f) const noexcept
{
  unsigned dx = Distance1D(lower_left.x, upper_right.x,
                           f.lower_left.x, f.upper_right.x);
  unsigned dy = Distance1D(lower_left.y, upper_right.y,
                           f.lower_left.y, f.upper_right.y);

  return ihypot(dx, dy);
}

bool
FlatBoundingBox::Intersects(const FlatRay& ray) const noexcept
{
  double tmin = 0, tmax = 1;

  // X
  if (ray.vector.x == 0) {
    // ray is parallel to slab. No hit if origin not within slab
    if (ray.point.x < lower_left.x || ray.point.x > upper_right.x)
      return false;
  } else {
    // compute intersection t value of ray with near/far plane of slab
    auto t1 = (lower_left.x - ray.point.x) * ray.fx;
    auto t2 = (upper_right.x - ray.point.x) * ray.fx;
    // make t1 be intersection with near plane, t2 with far plane
    if (t1 > t2)
      std::swap(t1, t2);

    tmin = std::max(tmin, t1);
    tmax = std::min(tmax, t2);
    // exit with no collision as soon as slab intersection becomes empty
    if (tmin > tmax)
      return false;
  }

  // Y
  if (ray.vector.y == 0) {
    // ray is parallel to slab. No hit if origin not within slab
    if (ray.point.y < lower_left.y || ray.point.y > upper_right.y)
      return false;
  } else {
    // compute intersection t value of ray with near/far plane of slab
    auto t1 = (lower_left.y - ray.point.y) * ray.fy;
    auto t2 = (upper_right.y - ray.point.y) * ray.fy;
    // make t1 be intersection with near plane, t2 with far plane
    if (t1 > t2)
      std::swap(t1, t2);

    tmin = std::max(tmin, t1);
    tmax = std::min(tmax, t2);
    // exit with no collision as soon as slab intersection becomes empty
    if (tmin > tmax)
      return false;
  }
  return true;
}

FlatGeoPoint
FlatBoundingBox::GetCenter() const noexcept
{
  /// @todo This will break if overlaps 360/0
  return (lower_left + upper_right) / 2;
}

bool
FlatBoundingBox::Overlaps(const FlatBoundingBox &other) const noexcept
{
  if (lower_left.x > other.upper_right.x)
    return false;
  if (upper_right.x < other.lower_left.x)
    return false;
  if (lower_left.y > other.upper_right.y)
    return false;
  if (upper_right.y < other.lower_left.y)
    return false;

  return true;
}

bool
FlatBoundingBox::IsInside(const FlatGeoPoint &loc) const noexcept
{
  if (loc.x < lower_left.x)
    return false;
  if (loc.x > upper_right.x)
    return false;
  if (loc.y < lower_left.y)
    return false;
  if (loc.y > upper_right.y)
    return false;

  return true;
}

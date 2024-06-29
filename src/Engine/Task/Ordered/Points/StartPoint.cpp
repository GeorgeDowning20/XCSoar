// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "StartPoint.hpp"
#include "Task/Ordered/Settings.hpp"
#include "Task/ObservationZones/Boundary.hpp"
#include "Task/TaskBehaviour.hpp"
#include "Geo/Math.hpp"
#include "Geo/GeoVector.hpp"

#include <cassert>

StartPoint::StartPoint(std::unique_ptr<ObservationZonePoint> &&_oz,
                       WaypointPtr &&wp,
                       const TaskBehaviour &tb,
                       const StartConstraints &_constraints)
  :OrderedTaskPoint(TaskPointType::START, std::move(_oz), std::move(wp), false),
   safety_height(tb.safety_height_arrival),
   margins(tb.start_margins),
   constraints(_constraints)
{
}

void
StartPoint::SetTaskBehaviour(const TaskBehaviour &tb) noexcept
{
  safety_height = tb.safety_height_arrival;
  margins = tb.start_margins;
}

double
StartPoint::GetElevation() const noexcept
{
  return GetBaseElevation() + safety_height;
}

void
StartPoint::SetOrderedTaskSettings(const OrderedTaskSettings &settings) noexcept
{
  OrderedTaskPoint::SetOrderedTaskSettings(settings);
  constraints = settings.start_constraints;
}

void
StartPoint::SetNeighbours(OrderedTaskPoint *_prev, OrderedTaskPoint *_next) noexcept
{
  assert(_prev==NULL);
  // should not ever have an inbound leg
  OrderedTaskPoint::SetNeighbours(_prev, _next);
}

bool FindIntersection(const GeoPoint &p1, const GeoPoint &p2,
                      const GeoPoint &p3, const GeoPoint &p4,
                      GeoPoint &intersection);

GeoPoint CalculateHeadingEndPoint(const GeoPoint &start, const Angle &heading, double distance);

bool IsPointOnLineSegment(const GeoPoint &p, const GeoPoint &p1, const GeoPoint &p2);

// Function Definitions
bool FindIntersection(const GeoPoint &p1, const GeoPoint &p2,
                      const GeoPoint &p3, const GeoPoint &p4,
                      GeoPoint &intersection) {
  // Line p1-p2 represented as a1x + b1y = c1
  double a1 = p2.latitude.Native() - p1.latitude.Native();
  double b1 = p1.longitude.Native() - p2.longitude.Native();
  double c1 = a1 * p1.longitude.Native() + b1 * p1.latitude.Native();

  // Line p3-p4 represented as a2x + b2y = c2
  double a2 = p4.latitude.Native() - p3.latitude.Native();
  double b2 = p3.longitude.Native() - p4.longitude.Native();
  double c2 = a2 * p3.longitude.Native() + b2 * p3.latitude.Native();

  double determinant = a1 * b2 - a2 * b1;

  if (determinant == 0) {
    // The lines are parallel
    return false;
  } else {
    double x = (b2 * c1 - b1 * c2) / determinant;
    double y = (a1 * c2 - a2 * c1) / determinant;
    intersection = GeoPoint(Angle::Radians(x), Angle::Radians(y));
    return true;
  }
}

GeoPoint CalculateHeadingEndPoint(const GeoPoint &start, const Angle &heading, double distance) {
  GeoVector vector(distance, heading);
  return vector.EndPoint(start);
}

bool IsPointOnLineSegment(const GeoPoint &p, const GeoPoint &p1, const GeoPoint &p2) {
  double min_lat = std::min(p1.latitude.Native(), p2.latitude.Native());
  double max_lat = std::max(p1.latitude.Native(), p2.latitude.Native());
  double min_lon = std::min(p1.longitude.Native(), p2.longitude.Native());
  double max_lon = std::max(p1.longitude.Native(), p2.longitude.Native());

  return (p.latitude.Native() >= min_lat && p.latitude.Native() <= max_lat &&
          p.longitude.Native() >= min_lon && p.longitude.Native() <= max_lon);
}

void
StartPoint::find_best_start(const AircraftState &state,
                            const OrderedTaskPoint &next,
                            const FlatProjection &projection)
{
  const OZBoundary boundary = GetBoundary();
  assert(boundary.begin() != boundary.end()); // Ensure boundary is not empty

  auto boundary_it = boundary.begin();
  const GeoPoint &start_point = *boundary_it++;   // GetSectorStart()
  assert(boundary_it != boundary.end());          // Ensure there is a second point
  const GeoPoint &end_point = *boundary_it;       // GetSectorEnd()

  const GeoPoint &next_location = next.GetLocationRemaining();
  GeoPoint best_location;
  double best_distance;

  // Check if the aircraft is in the start sector
  if (IsInSector(state)) {
    // Draw a line between the current position and the first turn point
    GeoPoint intersection;
    if (FindIntersection(state.location, next_location, start_point, end_point, intersection)) {
      best_location = intersection;
      best_distance = ::DoubleDistance(state.location, intersection, next_location);
    } else {
      // If no intersection found, fall back to minimum distance method
      best_location = *boundary.begin();
      best_distance = ::DoubleDistance(state.location, *boundary.begin(), next_location);

      for (auto it = boundary.begin(); it != boundary.end(); ++it) {
        auto distance = ::DoubleDistance(state.location, *it, next_location);
        if (distance < best_distance) {
          best_location = *it;
          best_distance = distance;
        }
      }
    }
  } else {
    // If not in the start sector, fall back to minimum distance method
    best_location = *boundary.begin();
    best_distance = ::DoubleDistance(state.location, *boundary.begin(), next_location);

    for (auto it = boundary.begin(); it != boundary.end(); ++it) {
      auto distance = ::DoubleDistance(state.location, *it, next_location);
      if (distance < best_distance) {
        best_location = *it;
        best_distance = distance;
      }
    }
  }

  SetSearchMin(SearchPoint(best_location, projection));
}

bool
StartPoint::IsInSector(const AircraftState &state) const noexcept
{
  return OrderedTaskPoint::IsInSector(state) &&
    // TODO: not using margins?
    constraints.CheckHeight(state, GetBaseElevation());
}

bool
StartPoint::CheckExitTransition(const AircraftState &ref_now,
                                const AircraftState &ref_last) const noexcept
{
  if (!constraints.open_time_span.HasBegun(RoughTime{ref_last.time}))
    /* the start gate is not yet open when we left the OZ */
    return false;

  if (constraints.open_time_span.HasEnded(RoughTime{ref_now.time}))
    /* the start gate was already closed when we left the OZ */
    return false;

  if (!constraints.CheckSpeed(ref_now.ground_speed, &margins) ||
      !constraints.CheckSpeed(ref_last.ground_speed, &margins))
    /* flying too fast */
    return false;

  // TODO: not using margins?
  const bool now_in_height =
    constraints.CheckHeight(ref_now, GetBaseElevation());
  const bool last_in_height =
    constraints.CheckHeight(ref_last, GetBaseElevation());

  if (now_in_height && last_in_height) {
    // both within height limit, so use normal location checks
    return OrderedTaskPoint::CheckExitTransition(ref_now, ref_last);
  }
  if (!TransitionConstraint(ref_now.location, ref_last.location)) {
    // don't allow vertical crossings for line OZ's
    return false;
  }

  // transition inside sector to above
  return !now_in_height && last_in_height
    && OrderedTaskPoint::IsInSector(ref_last)
    && CanStartThroughTop();
}

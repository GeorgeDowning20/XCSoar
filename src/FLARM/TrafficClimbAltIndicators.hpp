// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "FLARM/Traffic.hpp"
#include <cstdint>

/**
 * Classifies a #FlarmTraffic target's climb rate (relative to the
 * current MacCready / own 30s average) and relative altitude, for the
 * "Colourful traffic" map rendering mode.
 */
class TrafficClimbAltIndicators
{
public:
  enum class Climb : uint8_t {
    GOOD,
    UP,
    DOWN,
  };

  enum class RelAlt : uint8_t {
    ABOVE,
    SAME,
    BELOW,
  };

  TrafficClimbAltIndicators() noexcept
    :climb(Climb::DOWN), rel_alt(RelAlt::SAME) {}

  /**
   * Classify using the current MC setting and own 30s average climb.
   */
  static TrafficClimbAltIndicators
  GetClimbAltIndicators(const FlarmTraffic &traffic) noexcept;

  static TrafficClimbAltIndicators
  GetClimbAltIndicators(const FlarmTraffic &traffic, const double set_mc,
                        const double current_30s_vario) noexcept
  {
    TrafficClimbAltIndicators indicators;

    // largest of the set mc or current 30s average vario.
    const double reference_climb_rate = (set_mc > current_30s_vario)
      ? set_mc : current_30s_vario;

    if (traffic.climb_rate_avg30s >= reference_climb_rate)
      indicators.climb = Climb::GOOD;
    else if (traffic.climb_rate_avg30s > 0.0)
      indicators.climb = Climb::UP;
    else
      indicators.climb = Climb::DOWN;

    if (traffic.relative_altitude > (const RoughAltitude)similar_altitude_threshold_meters)
      indicators.rel_alt = RelAlt::ABOVE;
    else if (traffic.relative_altitude > (const RoughAltitude)-similar_altitude_threshold_meters)
      indicators.rel_alt = RelAlt::SAME;
    else
      indicators.rel_alt = RelAlt::BELOW;

    return indicators;
  }

  Climb GetClimb() const noexcept { return climb; }
  RelAlt GetRelAlt() const noexcept { return rel_alt; }

private:
  Climb climb;
  RelAlt rel_alt;

  static constexpr int similar_altitude_threshold_meters = 50;
};

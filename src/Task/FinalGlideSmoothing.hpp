// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include <cmath>

/**
 * Distance from the finish beyond which the TE-compensated arrival
 * altitude difference is used unmodified by
 * SmoothFinalAltitudeDifference() [m].
 */
constexpr double FINAL_ALT_SMOOTH_TE_DISTANCE = 10000;

/**
 * Distance from the finish within which the plain (non-TE)
 * arrival altitude difference is used unmodified by
 * SmoothFinalAltitudeDifference() [m].
 */
constexpr double FINAL_ALT_SMOOTH_NON_TE_DISTANCE = 5000;

/**
 * Blend a total-energy-compensated arrival altitude difference into
 * the plain (non-TE) value as the glider approaches the finish, to
 * avoid a jump when the excess kinetic energy is dissipated close to
 * the ground.  Beyond #FINAL_ALT_SMOOTH_TE_DISTANCE from the finish,
 * the TE value is used unmodified; within
 * #FINAL_ALT_SMOOTH_NON_TE_DISTANCE, the plain value is used; in
 * between, the two are linearly interpolated.
 *
 * @param altitude_difference the non-TE-compensated arrival altitude
 * difference [m]
 * @param te_altitude_difference the TE-compensated arrival altitude
 * difference [m]; if not finite (e.g. because airspeed is
 * unavailable), the plain value is returned unmodified
 * @param distance distance remaining to the finish [m]; if not
 * finite, the plain value is returned unmodified
 */
inline double
SmoothFinalAltitudeDifference(double altitude_difference,
                               double te_altitude_difference,
                               double distance) noexcept
{
  if (!std::isfinite(te_altitude_difference) || !std::isfinite(distance))
    return altitude_difference;

  if (distance >= FINAL_ALT_SMOOTH_TE_DISTANCE)
    return te_altitude_difference;

  if (distance <= FINAL_ALT_SMOOTH_NON_TE_DISTANCE)
    return altitude_difference;

  const double fraction =
    (distance - FINAL_ALT_SMOOTH_NON_TE_DISTANCE) /
    (FINAL_ALT_SMOOTH_TE_DISTANCE - FINAL_ALT_SMOOTH_NON_TE_DISTANCE);
  return altitude_difference +
    fraction * (te_altitude_difference - altitude_difference);
}

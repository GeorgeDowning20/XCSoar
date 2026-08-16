// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "Tracking/Features.hpp"

#ifdef HAVE_TRACKING

#include "FLARM/Traffic.hpp"

/**
 * Settings for a direct in-app connection to the OGN (Open Glider
 * Network) APRS-IS feed.  Received traffic is displayed using the
 * regular FLARM traffic mechanism (#FlarmTraffic::SourceType::OGN).
 */
struct OGNSettings {
  static constexpr unsigned DEFAULT_RANGE_KM = 100;
  static constexpr unsigned MIN_RANGE_KM = 5;
  static constexpr unsigned MAX_RANGE_KM = 500;

  /** Connect directly to the OGN APRS-IS network? */
  bool enabled;

  /**
   * Only request traffic within this many kilometres of the current
   * GPS position (APRS-IS server-side range filter).
   */
  unsigned range_km;

  /**
   * Bitmask of #FlarmTraffic::AircraftType values to display (bit i =
   * 1u << i).  Defaults to gliders only.
   */
  unsigned aircraft_type_mask;

  void SetDefaults() {
    enabled = false;
    range_km = DEFAULT_RANGE_KM;
    aircraft_type_mask =
      1u << unsigned(FlarmTraffic::AircraftType::GLIDER);
  }

  [[gnu::pure]]
  bool IsTypeEnabled(FlarmTraffic::AircraftType type) const noexcept {
    return (aircraft_type_mask & (1u << unsigned(type))) != 0;
  }

  [[gnu::pure]]
  unsigned EffectiveRangeKm() const noexcept {
    if (range_km < MIN_RANGE_KM || range_km > MAX_RANGE_KM)
      return DEFAULT_RANGE_KM;

    return range_km;
  }
};

#endif

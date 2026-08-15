// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "Blackboard/BaseBlackboard.hpp"
#include "Blackboard/ComputerSettingsBlackboard.hpp"
#include "Blackboard/MapSettingsBlackboard.hpp"
#include "FLARM/FadingTraffic.hpp"
#include "thread/Debug.hpp"
#include "time/Stamp.hpp"
#include "UIState.hpp"

#include <chrono>
#include <deque>
#include <map>

struct FlarmTrailPoint {
  GeoPoint location;
  TimeStamp time;
  double climb_rate_avg30s;
};

struct FlarmTrailReference {
  FlarmId id;
  TimeStamp time;
};

/**
 * Blackboard used by map window: provides read-only access to local
 * copies of data required by map window
 * 
 */
class MapWindowBlackboard:
  public BaseBlackboard,
  public ComputerSettingsBlackboard,
  public MapSettingsBlackboard
{
  UIState ui_state;

  std::map<FlarmId, std::deque<FlarmTrailPoint>> flarm_trails;
  std::deque<FlarmTrailReference> flarm_trail_fifo;

protected:
  MapWindowBlackboard() noexcept {
    /* this needs to be initialised because ReadBlackboard() uses the
       previous FLARM traffic list */
    gps_info.Reset();
  }

  [[gnu::const]]
  const MoreData &Basic() const noexcept {
    assert(InDrawThread());

    return BaseBlackboard::Basic();
  }

  [[gnu::const]]
  const DerivedInfo &Calculated() const noexcept {
    assert(InDrawThread());

    return BaseBlackboard::Calculated();
  }

  /**
   * FLARM traffic that has disappeared, but will remain on the map
   * (greyed out) for some time.  See #FlarmFadingTraffic.
   */
  [[gnu::pure]]
  auto GetFadingFlarmTraffic() const noexcept {
    return FlarmFadingTraffic::GetAll();
  }

  [[gnu::pure]]
  const std::deque<FlarmTrailPoint> *
  GetFlarmTrail(FlarmId id) const noexcept {
    const auto i = flarm_trails.find(id);
    return i != flarm_trails.end() ? &i->second : nullptr;
  }

  [[gnu::const]]
  const ComputerSettings &GetComputerSettings() const noexcept {
    assert(InDrawThread());

    return ComputerSettingsBlackboard::GetComputerSettings();
  }

  [[gnu::const]]
  const MapSettings &GetMapSettings() const noexcept {
    assert(InDrawThread());

    return settings_map;
  }

  [[gnu::const]]
  const UIState &GetUIState() const noexcept {
    assert(InDrawThread());

    return ui_state;
  }

  void ReadBlackboard(const MoreData &nmea_info,
                      const DerivedInfo &derived_info) noexcept;
  void ReadComputerSettings(const ComputerSettings &settings) noexcept;
  void ReadMapSettings(const MapSettings &settings) noexcept;

  void ReadUIState(const UIState &new_value) noexcept {
    ui_state = new_value;
  }
};

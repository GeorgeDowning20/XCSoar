// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "MapWindowBlackboard.hpp"

void
MapWindowBlackboard::ReadComputerSettings(const ComputerSettings &settings) noexcept
{
  computer_settings = settings;
}

void
MapWindowBlackboard::ReadMapSettings(const MapSettings &settings) noexcept
{
  settings_map = settings;
}

void
MapWindowBlackboard::ReadBlackboard(const MoreData &nmea_info,
				    const DerivedInfo &derived_info) noexcept
{
  FlarmFadingTraffic::Update(settings_map.fade_traffic,
                            settings_map.traffic_fade_timeout_minutes,
                            gps_info.flarm.traffic,
                            nmea_info.flarm.traffic,
                            nmea_info.clock);

  gps_info = nmea_info;
  calculated_info = derived_info;
}


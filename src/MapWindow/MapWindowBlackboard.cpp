// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "MapWindowBlackboard.hpp"

#include <algorithm>

static constexpr size_t FLARM_TRAIL_METADATA_RESERVE = 4u * 1024u * 1024u;
static constexpr auto FLARM_TRAIL_SAMPLE_INTERVAL = std::chrono::seconds{1};
static constexpr size_t FLARM_TRAIL_MAX_SAMPLES_PER_TARGET = 3600;

static size_t
GetFlarmTrailMaxSamples(unsigned memory_limit_mb) noexcept
{
  const size_t memory_limit = memory_limit_mb * 1024u * 1024u;
  const size_t sample_memory_limit =
    std::max(memory_limit, FLARM_TRAIL_METADATA_RESERVE + 1u) -
    FLARM_TRAIL_METADATA_RESERVE;
  return sample_memory_limit /
    (sizeof(FlarmTrailPoint) + sizeof(FlarmTrailReference));
}

static void
UpdateFlarmTrails(std::map<FlarmId, std::deque<FlarmTrailPoint>> &trails,
                  std::deque<FlarmTrailReference> &fifo,
                  const TrafficList &traffic,
                  const std::map<FlarmId, FlarmTraffic> &fading,
                  TimeStamp now,
                  unsigned length_minutes,
                  unsigned memory_limit_mb) noexcept
{
  const auto max_age = std::chrono::minutes{length_minutes};
  const size_t max_samples = GetFlarmTrailMaxSamples(memory_limit_mb);

  std::erase_if(trails, [&traffic, &fading](const auto &item) {
    return traffic.FindTraffic(item.first) == nullptr &&
      fading.find(item.first) == fading.end();
  });

  for (const auto &target : traffic.list) {
    if (!target.location_available)
      continue;

    auto &trail = trails[target.id];
    if ((trail.empty() ||
         now - trail.back().time >= FLARM_TRAIL_SAMPLE_INTERVAL) &&
        (trail.empty() || trail.back().location != target.location)) {
      if (trail.size() == FLARM_TRAIL_MAX_SAMPLES_PER_TARGET)
        trail.pop_front();

      trail.push_back({target.location, now, target.climb_rate_avg30s});
      fifo.push_back({target.id, now});
    }
  }

  while (!fifo.empty() &&
         (now - fifo.front().time > max_age ||
      fifo.size() > max_samples)) {
    const auto reference = fifo.front();
    fifo.pop_front();

    if (auto i = trails.find(reference.id); i != trails.end() &&
        !i->second.empty() && i->second.front().time == reference.time) {
      i->second.pop_front();
      if (i->second.empty())
        trails.erase(i);
    }
  }
}

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

  const auto fading = FlarmFadingTraffic::GetAll();
  UpdateFlarmTrails(flarm_trails, flarm_trail_fifo,
                    nmea_info.flarm.traffic, fading, nmea_info.clock,
                    settings_map.traffic_trail_length_minutes,
                    settings_map.traffic_trail_memory_limit_mb);

  gps_info = nmea_info;
  calculated_info = derived_info;
}


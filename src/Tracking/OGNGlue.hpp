// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "Tracking/Features.hpp"

#ifdef HAVE_TRACKING

#include "Cloud/OGNClient.hpp"
#include "Tracking/OGNSettings.hpp"
#include "Geo/GeoPoint.hpp"

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <string_view>

class EventLoop;
struct NMEAInfo;
class TrackingGlue;

/**
 * Maintains a direct in-app connection to the OGN (Open Glider
 * Network) APRS-IS feed, filtered to a configurable range around the
 * current GPS position.  Received traffic is forwarded to
 * #TrackingGlue::OnOgnTraffic(), which injects it into the regular
 * FLARM traffic display.
 */
class OGNGlue final : public OGNAprsHandler {
  static constexpr const char *HOST = "aprs.glidernet.org";
  static constexpr unsigned PORT = 14580;

  /** Re-send the range filter when the fix has moved further than this. */
  static constexpr double FILTER_UPDATE_DISTANCE = 20000; // 20 km

  EventLoop &loop;
  TrackingGlue &tracking;

  std::unique_ptr<OGNClient> client;

  bool enabled = false;
  unsigned range_km = OGNSettings::DEFAULT_RANGE_KM;

  /** Written on the main thread, read from OnAprsLine() on the asio_thread. */
  std::atomic<unsigned> aircraft_type_mask{
    1u << unsigned(FlarmTraffic::AircraftType::GLIDER)};

  GeoPoint filter_location = GeoPoint::Invalid();

public:
  OGNGlue(EventLoop &_loop, TrackingGlue &_tracking) noexcept;
  ~OGNGlue() noexcept;

  void SetSettings(const OGNSettings &settings) noexcept;

  /** Called periodically (e.g. once per GPS fix) with the own-ship state. */
  void Tick(const NMEAInfo &basic) noexcept;

  void BeginShutdown() noexcept;

private:
  void UpdateFilter(const GeoPoint &location) noexcept;

  /** Build the initial APRS-IS filter spec around @p location. */
  std::string MakeFilter(const GeoPoint &location) const noexcept;

  /* virtual methods from OGNAprsHandler */
  void OnAprsLine(std::string_view line) noexcept override;
};

#endif

// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "OGNGlue.hpp"
#include "OGNSettings.hpp"
#include "TrackingGlue.hpp"
#include "Cloud/OGNAprs.hpp"
#include "Tracking/SkyLines/TrafficExtensions.hpp"
#include "io/async/GlobalAsioThread.hpp"
#include "NMEA/Info.hpp"
#include "FLARM/Id.hpp"

#include <fmt/format.h>

using namespace std::chrono;

/**
 * Fold an APRS station id into the OGN pilot-id namespace.  Mirrors
 * #OGNPilotIdFromStation() in Cloud/OGNTraffic.cpp (kept separate here
 * to avoid pulling the cloud-only aggregation code into the app).
 */
[[gnu::pure]]
static uint32_t
OGNPilotIdFromStation(std::string_view station_id) noexcept
{
  uint32_t h = 2166136261u;
  for (unsigned char ch : station_id)
    h = (h ^ uint32_t(ch)) * 16777619u;
  return SkyLinesTracking::OGN_PILOT_ID_MASK | (h & 0x7fffffffu);
}

[[gnu::pure]]
static uint32_t
OGNPilotIdFromFlarm(uint32_t flarm_id) noexcept
{
  return SkyLinesTracking::OGN_PILOT_ID_MASK | (flarm_id & 0xffffffu);
}

OGNGlue::OGNGlue(EventLoop &_loop, TrackingGlue &_tracking) noexcept
  :loop(_loop), tracking(_tracking) {}

OGNGlue::~OGNGlue() noexcept
{
  if (client)
    client->Stop();
}

void
OGNGlue::SetSettings(const OGNSettings &settings) noexcept
{
  range_km = settings.EffectiveRangeKm();
  aircraft_type_mask = settings.aircraft_type_mask;

  if (settings.enabled == enabled) {
    if (enabled && client && filter_location.IsValid())
      UpdateFilter(filter_location);
    return;
  }

  enabled = settings.enabled;

  if (enabled) {
    /* don't connect yet: without a GPS fix we have no range filter to
       send, which would otherwise subscribe to the entire global OGN
       firehose (see Tick(), which starts the client once we know our
       position). */
    filter_location = GeoPoint::Invalid();
  } else if (client) {
    client->Stop();
    client.reset();
  }
}

void
OGNGlue::Tick(const NMEAInfo &basic) noexcept
{
  if (!enabled || !basic.location_available)
    return;

  if (!client) {
    client = std::make_unique<OGNClient>(
      loop, *global_cares_channel, *this,
      std::string(HOST), PORT,
      std::string("N0CALL"), std::string("-1"),
      MakeFilter(basic.location));
    filter_location = basic.location;
    client->Start();
    return;
  }

  UpdateFilter(basic.location);
}

std::string
OGNGlue::MakeFilter(const GeoPoint &location) const noexcept
{
  return fmt::format("r/{:.4f}/{:.4f}/{}",
                     location.latitude.Degrees(),
                     location.longitude.Degrees(),
                     range_km);
}

void
OGNGlue::UpdateFilter(const GeoPoint &location) noexcept
{
  if (!client)
    return;

  if (filter_location.IsValid() &&
      filter_location.Distance(location) < FILTER_UPDATE_DISTANCE)
    return;

  filter_location = location;
  client->SetFilter(MakeFilter(location));
}

void
OGNGlue::BeginShutdown() noexcept
{
  if (client)
    client->Stop();
}

void
OGNGlue::OnAprsLine(std::string_view line) noexcept
{
  const OGNAprsParseResult r = ParseOGNAprsLine(line);
  if (!r.valid || !IsForwardableOgnTraffic(r, line))
    return;

  /* same aircraft_type -> AircraftType mapping as FlarmTrafficBuilder::Build() */
  const FlarmTraffic::AircraftType type =
    r.aircraft_type <= 15 && r.aircraft_type != 14
    ? FlarmTraffic::AircraftType(r.aircraft_type)
    : FlarmTraffic::AircraftType::UNKNOWN;
  if ((aircraft_type_mask & (1u << unsigned(type))) == 0)
    return;

  const uint32_t pilot_id = r.flarm_valid
    ? OGNPilotIdFromFlarm(r.flarm_id)
    : OGNPilotIdFromStation(r.station_id);

  const FlarmId flarm_id = r.flarm_valid
    ? FlarmId::FromValue(r.flarm_id & 0xffffffu)
    : FlarmId::Undefined();

  tracking.OnOgnTraffic(pilot_id, r.location, r.altitude, r.altitude_valid,
                       r.track_deg, r.track_valid, flarm_id,
                       r.aircraft_type);
}

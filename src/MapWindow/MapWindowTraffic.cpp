// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "MapWindow.hpp"
#include "OffscreenTrafficMarker.hpp"
#include "ui/canvas/Icon.hpp"
#include "Screen/Layout.hpp"
#include "Formatter/UserUnits.hpp"
#include "Look/TrafficLook.hpp"
#include "Renderer/TextInBox.hpp"
#include "Renderer/TrafficRenderer.hpp"
#include "FLARM/Friends.hpp"
#include "FLARM/TrafficClimbAltIndicators.hpp"
#include "MapSettings.hpp"
#include "util/StringCompare.hxx"

#include <algorithm>
#include <cassert>

#ifdef ENABLE_OPENGL
#include "ui/canvas/opengl/Scope.hpp"
#endif

static constexpr size_t MAX_RENDERED_FLARM_TRAIL_SEGMENTS = 256;
static constexpr Color FLARM_TRAIL_CLIMB_COLOR{0xff, 0x00, 0x00};
static constexpr Color FLARM_TRAIL_UP_COLOR{0xff, 0xff, 0x00};
static constexpr Color FLARM_TRAIL_SINK_COLOR{0x00, 0x00, 0xff};

/** AboveColors this many offscreen blob markers, hide their labels to avoid clutter. */
static constexpr unsigned MAX_OFFSCREEN_TRAFFIC_NAMES = 6;

/**
 * Best available identifier for a traffic target: its resolved name
 * if known, otherwise the FLARM/ICAO id, so traffic without a
 * database/callsign match is still identifiable on the map.
 */
static const char *
GetTrafficLabel(const FlarmTraffic &traffic, char (&buffer)[16]) noexcept
{
  if (traffic.HasName() && !StringIsEmpty(traffic.name))
    return traffic.name.c_str();

  if (traffic.id.IsDefined()) {
    traffic.id.Format(buffer);
    return buffer;
  }

  return nullptr;
}

static void
DrawFlarmTrail(Canvas &canvas, const WindowProjection &projection,
               const std::deque<FlarmTrailPoint> &trail,
               double set_mc,
               double current_30s_vario, unsigned width) noexcept
{
  if (trail.size() < 2)
    return;

  const double reference_climb_rate = std::max(set_mc, current_30s_vario);
  const auto get_color = [reference_climb_rate](double climb_rate) noexcept {
    return climb_rate >= reference_climb_rate
      ? FLARM_TRAIL_CLIMB_COLOR
      : climb_rate > 0.
        ? FLARM_TRAIL_UP_COLOR
        : FLARM_TRAIL_SINK_COLOR;
  };

  const auto draw_segment = [&](size_t begin, size_t end) noexcept {
    canvas.Select(Pen(Pen::Style::SOLID, Layout::ScalePenWidth(width),
                      get_color(trail[end].climb_rate_avg30s)));
    canvas.DrawLine(projection.GeoToScreen(trail[begin].location),
                    projection.GeoToScreen(trail[end].location));
  };

  const size_t segment_count = trail.size() - 1;
  const size_t step = std::max<size_t>(1,
    (segment_count + MAX_RENDERED_FLARM_TRAIL_SEGMENTS - 1) /
    MAX_RENDERED_FLARM_TRAIL_SEGMENTS);

  size_t previous = 0;
  for (size_t i = step; i < trail.size(); i += step) {
    draw_segment(previous, i);
    previous = i;
  }

  if (previous + 1 < trail.size())
    draw_segment(previous, trail.size() - 1);
}

static void
DrawOffscreenFlarmMarker(Canvas &canvas, PixelPoint position,
                         const PixelRect &map_rect, const TrafficLook &look,
                         bool fading, bool colorful_traffic,
                         const FlarmTraffic &traffic, double set_mc,
                         double current_30s_vario,
                         DisplayOnlineTrafficMapMode online_mode,
                         unsigned marker_scale_percent,
                         bool show_names) noexcept
{
  const auto indicators = TrafficClimbAltIndicators::GetClimbAltIndicators(
    traffic, set_mc, current_30s_vario);
  const Color color = fading
    ? ColorWithAlpha({0x99, 0x99, 0x99}, 0x80)
    : colorful_traffic
      ? look.GetColourfulTrafficColor(indicators)
      : look.GetBasicTrafficColor(indicators);

  canvas.Select(Pen(Pen::Style::SOLID, Layout::ScalePenWidth(1), color));
  const unsigned radius = GetOffscreenTrafficMarkerRadius(marker_scale_percent);
#ifdef ENABLE_OPENGL
  const ScopeAlphaBlend alpha_blend;
  canvas.Select(Brush(color));
#elif defined(USE_MEMORY_CANVAS)
  canvas.Select(Brush(color));
#else
  if (fading)
    canvas.SelectHollowBrush();
  else
    canvas.Select(Brush(color));
#endif
  canvas.DrawCircle(position, radius);

  const FlarmColor friend_color = FlarmFriends::GetFriendColor(traffic.id);

  if (friend_color != FlarmColor::NONE) {
    canvas.Select(look.GetTeamPen(friend_color));
    canvas.SelectHollowBrush();
    canvas.DrawCircle(position, radius + Layout::Scale(2) +
                      Layout::ScalePenWidth(1));
  }

  const bool show_name = show_names &&
    (!FlarmTraffic::IsInjectedSource(traffic.source) ||
     online_mode == DisplayOnlineTrafficMapMode::SYMBOL_NAME);
  char label_buffer[16];
  const char *label = show_name ? GetTrafficLabel(traffic, label_buffer) : nullptr;
  if (label != nullptr) {
    TextInBoxMode mode;
    if (!fading)
      mode.shape = LabelShape::OUTLINED;
    mode.align = TextInBoxMode::CENTER;
    mode.vertical_position = TextInBoxMode::ABOVE;
    mode.move_in_view = true;
    TextInBox(canvas, label, position, mode, map_rect);
  }
}

static void
DrawFlarmTraffic(Canvas &canvas, const WindowProjection &projection,
                 const TrafficLook &look, bool fading, bool colorful_traffic,
                 const PixelPoint aircraft_pos,
                 const FlarmTraffic &traffic,
                 DisplayOnlineTrafficMapMode online_mode,
                 const double set_mc, const double current_30s_vario,
                 unsigned scale_percent) noexcept
{
  assert(traffic.location_available);

  // Points for the screen coordinates for the icon, name and average climb
  PixelPoint sc;

  // If FLARM target not on the screen, move to the next one
  if (auto p = projection.GeoToScreenIfVisible(traffic.location))
    sc = *p;
  else
    return;

  TextInBoxMode mode;
  if (!fading)
    mode.shape = LabelShape::OUTLINED;

  // JMW TODO enhancement: decluttering of FLARM altitudes (sort by max lift)

  // only draw labels if not close to aircraft
  const TrafficRenderer::MapTrafficLabelLayout layout =
    TrafficRenderer::MapLabelLayout(scale_percent);
  if ((sc - aircraft_pos).MagnitudeSquared() >
      layout.min_label_distance * layout.min_label_distance) {
    const bool show_name =
      (!FlarmTraffic::IsInjectedSource(traffic.source) ||
       online_mode == DisplayOnlineTrafficMapMode::SYMBOL_NAME);
    char label_buffer[16];
    const char *label = show_name ? GetTrafficLabel(traffic, label_buffer) : nullptr;

    if (label != nullptr) {
      auto sc_name = sc;
      sc_name.y -= layout.name_offset_y;

      TextInBox(canvas, label, sc_name,
                mode, projection.GetScreenRect());
    }

    if (!fading && traffic.climb_rate_avg30s >= 0.1) {
      auto sc_av = sc;
      sc_av.y += layout.climb_offset_y;

      TextInBox(canvas,
                FormatUserVerticalSpeed(traffic.climb_rate_avg30s, false),
                sc_av, mode,
                projection.GetScreenRect());
    }
  }

  auto color = FlarmFriends::GetFriendColor(traffic.id);

  const TrafficClimbAltIndicators indicators =
    TrafficClimbAltIndicators::GetClimbAltIndicators(traffic, set_mc,
                                                     current_30s_vario);

  TrafficRenderer::Draw(canvas, look, fading, colorful_traffic, traffic,
                        traffic.track - projection.GetScreenAngle(),
                        color, sc, indicators, scale_percent);
}

/**
 * Draws the FLARM traffic icons onto the given canvas
 * @param canvas Canvas for drawing
 */
void
MapWindow::DrawFLARMTraffic(Canvas &canvas,
                            const PixelPoint aircraft_pos) const noexcept
{
  // Return if FLARM icons on moving map are disabled
  if (!GetMapSettings().show_flarm_on_map)
    return;

  // Return if FLARM data is not available
  const TrafficList &flarm = Basic().flarm.traffic;

  const WindowProjection &projection = render_projection;

  canvas.Select(*traffic_look.font);

  const DisplayOnlineTrafficMapMode online_mode =
    GetMapSettings().online_traffic_map_mode;
  const unsigned scale_percent = (unsigned)GetMapSettings().traffic_icon_scale;
  const bool colorful_traffic = GetMapSettings().use_detailed_flarm_colours;
  const bool trails_enabled = GetMapSettings().traffic_trail_enabled;
  const unsigned trail_width = (unsigned)GetMapSettings().traffic_trail_width;
  const unsigned marker_scale_percent =
    (unsigned)GetMapSettings().traffic_offscreen_marker_size;
  const double set_mc = GetComputerSettings().polar.glide_polar_task.GetMC();
  const double current_30s_vario = Calculated().average;

  // Count offscreen blob markers first so we can hide their labels when
  // there would be too many of them cluttering the map edge.
  unsigned offscreen_count = 0;
  for (const auto &traffic : flarm.list) {
    if (!traffic.location_available)
      continue;
    if (FlarmTraffic::IsInjectedSource(traffic.source) &&
        online_mode == DisplayOnlineTrafficMapMode::OFF)
      continue;
    if (GetOffscreenTrafficMarkerPosition(projection, traffic_visible_rect,
                                          traffic.location,
                                          marker_scale_percent))
      ++offscreen_count;
  }
  const bool show_offscreen_names = offscreen_count <= MAX_OFFSCREEN_TRAFFIC_NAMES;

  // Circle through the traffic targets
  for (const auto &traffic : flarm.list) {
    if (!traffic.location_available)
      continue;

    if (FlarmTraffic::IsInjectedSource(traffic.source) &&
        online_mode == DisplayOnlineTrafficMapMode::OFF)
      continue;

    if (trails_enabled)
      if (const auto *trail = GetFlarmTrail(traffic.id))
        DrawFlarmTrail(canvas, projection, *trail, set_mc,
                       current_30s_vario, trail_width);

    if (const auto marker =
          GetOffscreenTrafficMarkerPosition(projection, traffic_visible_rect,
                                            traffic.location,
                                            marker_scale_percent)) {
      DrawOffscreenFlarmMarker(canvas, *marker, traffic_visible_rect,
                               traffic_look, false, colorful_traffic, traffic,
                               set_mc, current_30s_vario, online_mode,
                               marker_scale_percent, show_offscreen_names);
      continue;
    }

    /* Historically, we skipped targets with both relative vectors
       zero to avoid drawing "no position" FLARM targets.  Absolute-
       position traffic (e.g. ADS-B) may legitimately have relatives
       not computed yet, so allow those as well.  Require either
       component non-zero so due-north/south targets still draw. */
    if (traffic.absolute_location ||
        traffic.relative_north != 0 || traffic.relative_east != 0)
      DrawFlarmTraffic(canvas, projection, traffic_look, false, colorful_traffic,
                       aircraft_pos, traffic, online_mode,
                       set_mc, current_30s_vario, scale_percent);
  }

  if (const auto &fading = GetFadingFlarmTraffic(); !fading.empty()) {
    for (const auto &[id, traffic] : fading) {
      assert(traffic.location_available);

      if (FlarmTraffic::IsInjectedSource(traffic.source) &&
          online_mode == DisplayOnlineTrafficMapMode::OFF)
        continue;

      if (trails_enabled)
        if (const auto *trail = GetFlarmTrail(traffic.id))
          DrawFlarmTrail(canvas, projection, *trail, set_mc,
                         current_30s_vario, trail_width);

      if (const auto marker =
        GetOffscreenTrafficMarkerPosition(projection, traffic_visible_rect,
                                              traffic.location,
                                              marker_scale_percent)) {
        DrawOffscreenFlarmMarker(canvas, *marker, traffic_visible_rect,
                                 traffic_look, true, colorful_traffic, traffic,
                                 set_mc, current_30s_vario, online_mode,
                                 marker_scale_percent, show_offscreen_names);
        continue;
      }

      if (traffic.absolute_location ||
          traffic.relative_north != 0 || traffic.relative_east != 0)
        DrawFlarmTraffic(canvas, projection, traffic_look, true, colorful_traffic,
                         aircraft_pos, traffic, online_mode,
                         set_mc, current_30s_vario, scale_percent);
    }
  }
}


/**
 * Draws the GliderLink traffic icons onto the given canvas
 * @param canvas Canvas for drawing
 */
void
MapWindow::DrawGLinkTraffic([[maybe_unused]] Canvas &canvas) const noexcept
{
#ifdef ANDROID

  // Return if FLARM icons on moving map are disabled
  if (!GetMapSettings().show_flarm_on_map)
    return;

  const GliderLinkTrafficList &traffic = Basic().glink_data.traffic;
  if (traffic.IsEmpty())
    return;

  const MoreData &basic = Basic();

  const WindowProjection &projection = render_projection;

  canvas.Select(*traffic_look.font);

  // Circle through the GliderLink targets
  for (const auto &traf : traffic.list) {

    // Points for the screen coordinates for the icon, name and average climb
    PixelPoint sc;

    // If FLARM target not on the screen, move to the next one
    if (auto p = projection.GeoToScreenIfVisible(traf.location))
      sc = *p;
    else
      continue;

    TextInBoxMode mode;
    mode.shape = LabelShape::OUTLINED;
    mode.align = TextInBoxMode::Alignment::RIGHT;

    // If callsign/name available draw it to the canvas
    if (traf.HasName() && !StringIsEmpty(traf.name)) {
      // Draw the callsign above the icon
      auto sc_name = sc;
      sc_name.x -= Layout::Scale(10);
      sc_name.y -= Layout::Scale(15);

      TextInBox(canvas, traf.name, sc_name,
                mode, GetClientRect());
    }

    if (traf.climb_rate_received) {

      // If average climb data available draw it to the canvas
      mode.align = TextInBoxMode::Alignment::LEFT;

      // Draw the average climb to the right of the icon
      auto sc_av = sc;
      sc_av.x += Layout::Scale(10);
      sc_av.y -= Layout::Scale(8);

      TextInBox(canvas,
                FormatUserVerticalSpeed(traf.climb_rate, false),
                sc_av, mode, GetClientRect());
    }

    // use GPS altitude to be consistent with GliderLink
    if(basic.gps_altitude_available && traf.altitude_received
        && fabs(double(traf.altitude) - basic.gps_altitude) >= 100.0) {
      // If average climb data available draw it to the canvas
      char label_alt[100];
      double alt = (double(traf.altitude) - basic.gps_altitude) / 100.0;
      FormatRelativeUserAltitude(alt, label_alt, false);

      // Location of altitude label
      auto sc_alt = sc;
      sc_alt.x -= Layout::Scale(10);
      sc_alt.y -= Layout::Scale(0);

      mode.align = TextInBoxMode::Alignment::RIGHT;
      TextInBox(canvas, label_alt, sc_alt, mode, GetClientRect());
    }

    TrafficRenderer::Draw(canvas, traffic_look, traf,
                          traf.track - projection.GetScreenAngle(), sc,
                          (unsigned)GetMapSettings().traffic_icon_scale);
  }
#endif
}

/**
 * Draws the teammate icon to the given canvas
 * @param canvas Canvas for drawing
 */
void
MapWindow::DrawTeammate(Canvas &canvas) const noexcept
{
  const TeamInfo &teamcode_info = Calculated();

  if (teamcode_info.teammate_available) {
    if (auto p = render_projection.GeoToScreenIfVisible(teamcode_info.teammate_location))
      traffic_look.teammate_icon.Draw(canvas, *p);
  }
}

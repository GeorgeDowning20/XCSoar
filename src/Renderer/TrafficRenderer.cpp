// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "TrafficRenderer.hpp"
#include "ui/canvas/Canvas.hpp"
#include "Screen/Layout.hpp"
#include "Look/TrafficLook.hpp"
#include "FLARM/Traffic.hpp"
#include "GliderLink/Traffic.hpp"
#include "Math/Screen.hpp"
#include "util/Macros.hpp"
#include "Asset.hpp"

#include <algorithm>

#ifdef ENABLE_OPENGL
#include "ui/canvas/opengl/Scope.hpp"
#endif

/** The arrow template spans 14 units vertically (-8 to +6). */
static constexpr unsigned ARROW_SPAN = 14;

/**
 * Target map traffic symbol height in virtual points.  Scales with
 * display DPI (and small-screen viewing distance) but not window size.
 * ~Scale(100) arrow size at the 240 px design baseline.
 */
static constexpr unsigned MAP_TRAFFIC_ICON_VPT = 50;

struct MapTrafficScale {
  int arrow_scale;
  unsigned circle_radius;
};

static MapTrafficScale
MapTrafficScaleFromIconSize(unsigned icon_size) noexcept
{
  return {
    std::max(int(icon_size) * 50 / int(ARROW_SPAN), 1),
    std::max(icon_size / 3U, Layout::ScalePenWidth(1)),
  };
}

[[gnu::pure]]
static MapTrafficScale
GetMapTrafficScale(unsigned scale_percent) noexcept
{
  return MapTrafficScaleFromIconSize(TrafficRenderer::MapIconSize(scale_percent));
}

static void
DrawFlarmArrow(Canvas &canvas, const TrafficLook &traffic_look,
               bool fading, bool colorful_traffic,
               const FlarmTraffic &traffic,
               const Angle angle, const FlarmColor color,
               const PixelPoint pt,
               const TrafficClimbAltIndicators &indicators,
               int arrow_scale, unsigned circle_radius) noexcept
{
  BulkPixelPoint arrow[] = {
    { -4, 6 },
    { 0, -8 },
    { 4, 6 },
    { 0, 3 },
  };

  PolygonRotateShift(arrow, pt, angle, arrow_scale);

  if (fading) {
    canvas.Select(traffic_look.fading_pen);

#ifdef ENABLE_OPENGL
    canvas.Select(traffic_look.fading_brush);
#else
    /* we have no alpha blending - don't fill the shape */
    canvas.SelectHollowBrush();
#endif

#ifdef ENABLE_OPENGL
    const ScopeAlphaBlend alpha_blend;
#endif
    canvas.DrawPolygon(arrow, ARRAY_SIZE(arrow));
  } else {
    switch (traffic.alarm_level) {
    case FlarmTraffic::AlarmType::LOW:
    case FlarmTraffic::AlarmType::INFO_ALERT:
      canvas.Select(traffic_look.warning_brush);
      break;
    case FlarmTraffic::AlarmType::IMPORTANT:
    case FlarmTraffic::AlarmType::URGENT:
      canvas.Select(traffic_look.alarm_brush);
      break;
    case FlarmTraffic::AlarmType::NONE:
      if (colorful_traffic)
        canvas.Select(traffic_look.GetColourfulTrafficBrush(indicators));
      else
        canvas.Select(traffic_look.GetBasicTrafficBrush(indicators));
      break;
    }

    canvas.SelectBlackPen();
    canvas.DrawPolygon(arrow, ARRAY_SIZE(arrow));
  }

  if (color == FlarmColor::NONE)
    return;

  canvas.Select(traffic_look.GetTeamPen(color));

  canvas.SelectHollowBrush();
  canvas.DrawCircle(pt, circle_radius);
}

unsigned
TrafficRenderer::MapIconSize(unsigned scale_percent) noexcept
{
  return Layout::VptScale(MAP_TRAFFIC_ICON_VPT) * scale_percent / 100;
}

TrafficRenderer::MapTrafficLabelLayout
TrafficRenderer::MapLabelLayout(unsigned scale_percent) noexcept
{
  const unsigned icon_size = MapIconSize(scale_percent);
  const int half = int(icon_size) / 2;

  return {
    icon_size,
    half + int(Layout::VptScale(3)),
    half + int(Layout::VptScale(1)),
    half + int(Layout::VptScale(30)),
  };
}

void
TrafficRenderer::Draw(Canvas &canvas, const TrafficLook &traffic_look,
                      bool fading, bool colorful_traffic,
                      const FlarmTraffic &traffic, const Angle angle,
                      const FlarmColor color, const PixelPoint pt,
                      const TrafficClimbAltIndicators &indicators,
                      unsigned scale_percent) noexcept
{
  const MapTrafficScale scale = GetMapTrafficScale(scale_percent);
  DrawFlarmArrow(canvas, traffic_look, fading, colorful_traffic, traffic,
                angle, color, pt, indicators,
                scale.arrow_scale, scale.circle_radius);
}

void
TrafficRenderer::DrawList(Canvas &canvas, const TrafficLook &traffic_look,
                          bool colorful_traffic,
                          const FlarmTraffic &traffic, const Angle angle,
                          const FlarmColor color, const PixelPoint pt,
                          unsigned icon_size,
                          const TrafficClimbAltIndicators &indicators) noexcept
{
  const MapTrafficScale scale = MapTrafficScaleFromIconSize(icon_size);

  DrawFlarmArrow(canvas, traffic_look, false, colorful_traffic, traffic,
                angle, color, pt, indicators,
                scale.arrow_scale, scale.circle_radius);
}

void
TrafficRenderer::Draw(Canvas &canvas, const TrafficLook &traffic_look,
                      [[maybe_unused]] const GliderLinkTraffic &traffic,
                      const Angle angle, const PixelPoint pt,
                      unsigned scale_percent) noexcept
{
  BulkPixelPoint arrow[] = {
    { -4, 6 },
    { 0, -8 },
    { 4, 6 },
    { 0, 3 },
  };

  canvas.Select(traffic_look.basic_traffic_brushes.same);

  if (IsDithered())
    canvas.Select(Pen(Layout::ScalePenWidth(2), COLOR_BLACK));
  else
    canvas.SelectBlackPen();

  const MapTrafficScale scale = GetMapTrafficScale(scale_percent);
  PolygonRotateShift(arrow, pt, angle, scale.arrow_scale);
  canvas.DrawPolygon(arrow, ARRAY_SIZE(arrow));

  canvas.SelectHollowBrush();
  canvas.DrawCircle(pt, scale.circle_radius);
}

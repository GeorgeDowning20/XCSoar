// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "TrafficLook.hpp"
#include "Screen/Layout.hpp"
#include "Resources.hpp"

constexpr Color TrafficLook::team_color_green;
constexpr Color TrafficLook::team_color_magenta;
constexpr Color TrafficLook::team_color_blue;
constexpr Color TrafficLook::team_color_yellow;

void
TrafficLook::Initialise(const Font &_font)
{
  basic_traffic_brushes.above.Create(above_color);
  basic_traffic_brushes.same.Create(same_color);
  basic_traffic_brushes.below.Create(below_color);

  colorful_traffic_brushes.above.climb_good.Create(ColorfulTrafficColors::Above::climb_good);
  colorful_traffic_brushes.above.climb_up.Create(ColorfulTrafficColors::Above::climb_up);
  colorful_traffic_brushes.above.climb_down.Create(ColorfulTrafficColors::Above::climb_down);

  colorful_traffic_brushes.same.climb_good.Create(ColorfulTrafficColors::Same::climb_good);
  colorful_traffic_brushes.same.climb_up.Create(ColorfulTrafficColors::Same::climb_up);
  colorful_traffic_brushes.same.climb_down.Create(ColorfulTrafficColors::Same::climb_down);

  colorful_traffic_brushes.below.climb_good.Create(ColorfulTrafficColors::Below::climb_good);
  colorful_traffic_brushes.below.climb_up.Create(ColorfulTrafficColors::Below::climb_up);
  colorful_traffic_brushes.below.climb_down.Create(ColorfulTrafficColors::Below::climb_down);

  warning_brush.Create(warning_color);
  alarm_brush.Create(alarm_color);

  fading_pen.Create(Pen::Style::DASH1, Layout::ScalePenWidth(1), fading_outline_color);

#ifdef ENABLE_OPENGL
  fading_brush.Create(fading_fill_color);
#endif

  unsigned width = Layout::ScalePenWidth(2);
  team_pen_green.Create(width, team_color_green);
  team_pen_blue.Create(width, team_color_blue);
  team_pen_yellow.Create(width, team_color_yellow);
  team_pen_magenta.Create(width, team_color_magenta);

  teammate_icon.LoadResource(IDB_TEAMMATE_POS_ALL);

  font = &_font;
}

const Brush &
TrafficLook::GetBasicTrafficBrush(const TrafficClimbAltIndicators &indicators) const noexcept
{
  switch (indicators.GetRelAlt()) {
  case TrafficClimbAltIndicators::RelAlt::ABOVE:
    return basic_traffic_brushes.above;
  case TrafficClimbAltIndicators::RelAlt::BELOW:
    return basic_traffic_brushes.below;
  case TrafficClimbAltIndicators::RelAlt::SAME:
    break;
  }

  return basic_traffic_brushes.same;
}

const Brush &
TrafficLook::GetColourfulTrafficBrush(const TrafficClimbAltIndicators &indicators) const noexcept
{
  const ClimbBrushes &climb_brushes = [this, &indicators]() -> const ClimbBrushes & {
    switch (indicators.GetRelAlt()) {
    case TrafficClimbAltIndicators::RelAlt::ABOVE:
      return colorful_traffic_brushes.above;
    case TrafficClimbAltIndicators::RelAlt::BELOW:
      return colorful_traffic_brushes.below;
    case TrafficClimbAltIndicators::RelAlt::SAME:
      break;
    }

    return colorful_traffic_brushes.same;
  }();

  switch (indicators.GetClimb()) {
  case TrafficClimbAltIndicators::Climb::GOOD:
    return climb_brushes.climb_good;
  case TrafficClimbAltIndicators::Climb::UP:
    return climb_brushes.climb_up;
  case TrafficClimbAltIndicators::Climb::DOWN:
    break;
  }

  return climb_brushes.climb_down;
}

// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "TrafficLook.hpp"
#include "Screen/Layout.hpp"
#include "Resources.hpp"

void
TrafficLook::Initialise(const Font &_font)
{
  basic_traffic_brushes.above.Create(above_color);
  basic_traffic_brushes.same.Create(same_color);
  basic_traffic_brushes.below.Create(below_color);

  colorful_traffic_brushes.above.climb_good.Create(ColorfulTrafficColors::AboveColors::climb_good);
  colorful_traffic_brushes.above.climb_up.Create(ColorfulTrafficColors::AboveColors::climb_up);
  colorful_traffic_brushes.above.climb_down.Create(ColorfulTrafficColors::AboveColors::climb_down);

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
  for (size_t i = 0; i < TEAM_COLOR_COUNT; ++i)
    team_pens[i].Create(width, team_colors[i]);

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

Color
TrafficLook::GetBasicTrafficColor(const TrafficClimbAltIndicators &indicators) const noexcept
{
  switch (indicators.GetRelAlt()) {
  case TrafficClimbAltIndicators::RelAlt::ABOVE:
    return above_color;
  case TrafficClimbAltIndicators::RelAlt::BELOW:
    return below_color;
  case TrafficClimbAltIndicators::RelAlt::SAME:
    return same_color;
  }

  return same_color;
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

Color
TrafficLook::GetColourfulTrafficColor(const TrafficClimbAltIndicators &indicators) const noexcept
{
  switch (indicators.GetRelAlt()) {
  case TrafficClimbAltIndicators::RelAlt::ABOVE:
    switch (indicators.GetClimb()) {
    case TrafficClimbAltIndicators::Climb::GOOD:
      return ColorfulTrafficColors::AboveColors::climb_good;
    case TrafficClimbAltIndicators::Climb::UP:
      return ColorfulTrafficColors::AboveColors::climb_up;
    case TrafficClimbAltIndicators::Climb::DOWN:
      return ColorfulTrafficColors::AboveColors::climb_down;
    }
    break;
  case TrafficClimbAltIndicators::RelAlt::BELOW:
    switch (indicators.GetClimb()) {
    case TrafficClimbAltIndicators::Climb::GOOD:
      return ColorfulTrafficColors::Below::climb_good;
    case TrafficClimbAltIndicators::Climb::UP:
      return ColorfulTrafficColors::Below::climb_up;
    case TrafficClimbAltIndicators::Climb::DOWN:
      return ColorfulTrafficColors::Below::climb_down;
    }
    break;
  case TrafficClimbAltIndicators::RelAlt::SAME:
    switch (indicators.GetClimb()) {
    case TrafficClimbAltIndicators::Climb::GOOD:
      return ColorfulTrafficColors::Same::climb_good;
    case TrafficClimbAltIndicators::Climb::UP:
      return ColorfulTrafficColors::Same::climb_up;
    case TrafficClimbAltIndicators::Climb::DOWN:
      return ColorfulTrafficColors::Same::climb_down;
    }
    break;
  }

  return ColorfulTrafficColors::Same::climb_down;
}

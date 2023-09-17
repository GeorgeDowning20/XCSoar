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
  for (unsigned i = 0; i < num_alt_based_colour_options; i++) {
    for (unsigned j = 0; j < num_vario_based_colour_options; j++) {
      colorful_traffic_brushes[i][j].Create(colorful_traffic_colors[i][j]); // 3x3 array of brushes
    }
  }

  for (unsigned i = 0; i < num_alt_based_colour_options; i++)
  {
    basic_traffic_brushes[i].Create(basic_traffic_colors[i]); // 2x3 array of brushes
  }

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

  teammate_icon.LoadResource(IDB_TEAMMATE_POS, IDB_TEAMMATE_POS_HD);

  font = &_font;
}

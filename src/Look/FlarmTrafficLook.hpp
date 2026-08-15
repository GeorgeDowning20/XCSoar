// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "ui/canvas/Color.hpp"
#include "ui/canvas/Pen.hpp"
#include "ui/canvas/Brush.hpp"
#include "ui/canvas/Font.hpp"
#include "FLARM/Color.hpp"

#include <array>

struct TrafficLook;

struct FlarmTrafficLook {
  Color warning_color;
  Color alarm_color;
  Color default_color;
  Color passive_color;
  Color selection_color;
  Color background_color;
  Color radar_color;
  Color safe_above_color;
  Color safe_below_color;
  Color warning_in_altitude_range_color;
  static constexpr size_t TEAM_COLOR_COUNT =
    static_cast<size_t>(FlarmColor::COUNT) - 1;
  std::array<Color, TEAM_COLOR_COUNT> team_colors;

  Brush warning_brush;
  Brush alarm_brush;
  Brush default_brush;
  Brush passive_brush;
  Brush selection_brush;
  Brush radar_brush;
  std::array<Brush, TEAM_COLOR_COUNT> team_brushes;
  Brush safe_above_brush;
  Brush safe_below_brush;
  Brush warning_in_altitude_range_brush;

  Pen warning_pen;
  Pen alarm_pen;
  Pen default_pen;
  Pen passive_pen;
  Pen selection_pen;

  std::array<Pen, TEAM_COLOR_COUNT> team_pens;

  Pen plane_pen, radar_pen;

  Pen unit_fraction_pen;

  Font label_font, side_info_font, no_traffic_font;
  Font info_values_font, info_units_font, info_labels_font, call_sign_font;

  void Initialise(const TrafficLook &other, bool small, bool inverse = false);

  /**
   * Reload pens and fonts after #Layout::Initialise() (DPI / resize).
   */
  void ReinitialiseLayout() noexcept;

  [[gnu::pure]]
  const Pen &GetTeamPen(FlarmColor color) const noexcept {
    return team_pens[static_cast<size_t>(color) - 1];
  }

  [[gnu::pure]]
  const Brush &GetTeamBrush(FlarmColor color) const noexcept {
    return team_brushes[static_cast<size_t>(color) - 1];
  }

private:
  bool small;
  bool inverse;

  void InitialisePensAndFonts() noexcept;
};

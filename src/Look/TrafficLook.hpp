// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "ui/canvas/Color.hpp"
#include "ui/canvas/Pen.hpp"
#include "ui/canvas/Brush.hpp"
#include "ui/canvas/Icon.hpp"
#include "FLARM/Color.hpp"
#include "FLARM/TrafficClimbAltIndicators.hpp"

#include <array>

class Font;

struct TrafficLook {
  static constexpr size_t TEAM_COLOR_COUNT =
    static_cast<size_t>(FlarmColor::COUNT) - 1;

  /** Basic (single colour per zone) relative-altitude colours. */
  static constexpr Color above_color{0x1d,0x9b,0xc5};
  static constexpr Color same_color{0xff,0x00,0xff};
  static constexpr Color below_color{0x1d,0xc5,0x10};

  static constexpr Color warning_color{0xfe,0x84,0x38};
  static constexpr Color alarm_color{0xfb,0x35,0x2f};

  /** One brush per relative-altitude zone (basic traffic colours). */
  struct BasicTrafficBrushes {
    Brush above, same, below;
  } basic_traffic_brushes;

  /** Climb-rate colours within one relative-altitude zone. */
  struct ClimbBrushes {
    Brush climb_good, climb_up, climb_down;
  };

  /** One #ClimbBrushes set per relative-altitude zone ("Colourful traffic"). */
  struct ColorfulTrafficBrushes {
    ClimbBrushes above, same, below;
  } colorful_traffic_brushes;

  /** "Colourful traffic" palette, keyed by relative-altitude zone. */
  struct ColorfulTrafficColors {
    struct Above {
      static constexpr Color climb_good{0xff, 0x66, 0x66}; // light red
      static constexpr Color climb_up{0xff, 0xff, 0x66};   // light yellow
      static constexpr Color climb_down{0x66, 0x66, 0xff}; // light blue
    };

    struct Same {
      static constexpr Color climb_good{0xff, 0x00, 0x00}; // red
      static constexpr Color climb_up{0xff, 0xff, 0x00};   // yellow
      static constexpr Color climb_down{0x00, 0x00, 0xff}; // blue
    };

    struct Below {
      static constexpr Color climb_good{0x99, 0x00, 0x00}; // dark red
      static constexpr Color climb_up{0x99, 0x99, 0x00};   // dark yellow
      static constexpr Color climb_down{0x00, 0x00, 0x99}; // dark blue
    };
  };

  Brush warning_brush;
  Brush alarm_brush;

  static constexpr Color fading_outline_color = ColorWithAlpha({0x60, 0x60, 0x60}, 0xa0);
  Pen fading_pen;

#ifdef ENABLE_OPENGL
  static constexpr Color fading_fill_color = ColorWithAlpha({0xc0, 0xc0, 0xc0}, 0x60);
  Brush fading_brush;
#endif

  static constexpr std::array<Color, TEAM_COLOR_COUNT> team_colors{{
    {0x74, 0xff, 0x00}, {0x00, 0x90, 0xff}, {0xff, 0xe8, 0x00},
    {0xff, 0x00, 0xcb}, {0xff, 0x35, 0x2f}, {0x00, 0xe0, 0xe0},
    {0xff, 0x84, 0x38}, {0x90, 0x40, 0xd0}, {0xa0, 0xe8, 0x20},
    {0x00, 0xa0, 0x90}, {0xff, 0x70, 0xa0}, {0xff, 0xff, 0xff},
  }};

  std::array<Pen, TEAM_COLOR_COUNT> team_pens;

  MaskedIcon teammate_icon;

  const Font *font;

  void Initialise(const Font &font);

  [[gnu::const]]
  static Color GetTeamColor(FlarmColor color) noexcept {
    return team_colors[static_cast<size_t>(color) - 1];
  }

  [[gnu::pure]]
  const Pen &GetTeamPen(FlarmColor color) const noexcept {
    return team_pens[static_cast<size_t>(color) - 1];
  }

  /** Single colour per relative-altitude zone. */
  [[gnu::pure]]
  const Brush &GetBasicTrafficBrush(const TrafficClimbAltIndicators &indicators) const noexcept;

  /** Colour used by the basic traffic palette. */
  [[gnu::pure]]
  Color GetBasicTrafficColor(const TrafficClimbAltIndicators &indicators) const noexcept;

  /** Colour depends on both relative altitude and climb rate. */
  [[gnu::pure]]
  const Brush &GetColourfulTrafficBrush(const TrafficClimbAltIndicators &indicators) const noexcept;

  /** Colour used by the colourful traffic palette. */
  [[gnu::pure]]
  Color GetColourfulTrafficColor(const TrafficClimbAltIndicators &indicators) const noexcept;
};


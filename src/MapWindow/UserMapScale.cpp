// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "UserMapScale.hpp"
#include "Interface.hpp"
#include "UIState.hpp"
#include "Profile/Profile.hpp"
#include "Profile/Keys.hpp"
#include "Message.hpp"
#include "Language/Language.hpp"

#ifdef ENABLE_OPENGL
#include "ui/canvas/opengl/Globals.hpp"
#endif

#include <algorithm>

double
ClampUserMapScale([[maybe_unused]] double scale, [[maybe_unused]] DisplayMode mode,
                   [[maybe_unused]] double vmin) noexcept
{
  constexpr double scale_100m = 10;
  double scale_1600km = 1600 * 100;

#ifdef ENABLE_OPENGL
  if (OpenGL::max_map_scale > 0)
    scale_1600km = std::min(scale_1600km, double(OpenGL::max_map_scale));
#endif

  /* use the same minimum (most zoomed-in) scale in cruise mode as in
     circling mode, instead of limiting cruise zoom based on vmin */
  double minreasonable = scale_100m;

  /* a small GPU max_map_scale can push the upper bound below the
     lower one; keep std::clamp() defined */
  minreasonable = std::min(minreasonable, scale_1600km);

  return std::clamp(scale, minreasonable, scale_1600km);
}

double
ClampUserMapScale(double scale) noexcept
{
  const DisplayMode mode = CommonInterface::GetUIState().display_mode;
  const auto vmin =
    CommonInterface::GetComputerSettings().polar.glide_polar_task.GetVMin();
  return ClampUserMapScale(scale, mode, vmin);
}

bool
DisableAutoZoomForManualScale() noexcept
{
  MapSettings &settings = CommonInterface::SetMapSettings();
  const DisplayMode mode = CommonInterface::GetUIState().display_mode;

  if (!settings.auto_zoom_enabled)
    return false;

  if (mode == DisplayMode::CIRCLING && settings.circle_zoom_enabled)
    return false;

  settings.auto_zoom_enabled = false;
  Profile::Set(ProfileKeys::AutoZoom, false);
  Message::AddMessage(_("Auto. zoom off"));
  return true;
}

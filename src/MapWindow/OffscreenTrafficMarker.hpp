// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "Projection/WindowProjection.hpp"
#include "Screen/Layout.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

inline unsigned
GetOffscreenTrafficMarkerRadius(unsigned scale_percent) noexcept
{
  return (unsigned)std::max(1, Layout::Scale(5) * (int)scale_percent / 200);
}

inline std::optional<PixelPoint>
GetOffscreenTrafficMarkerPosition(const WindowProjection &projection,
                                  const PixelRect &visible_rect,
                                  const GeoPoint &location,
                                  unsigned scale_percent) noexcept
{
  const PixelPoint target = projection.GeoToScreen(location);
  if (visible_rect.Contains(target))
    return std::nullopt;

  const PixelPoint origin = projection.GetScreenOrigin();
  const double dx = target.x - origin.x;
  const double dy = target.y - origin.y;

  if (dx == 0. && dy == 0.)
    return std::nullopt;

  const unsigned radius = GetOffscreenTrafficMarkerRadius(scale_percent);
  const PixelRect edge_rect =
    visible_rect.WithPadding((int)radius + Layout::Scale(2));
  const double x_limit = dx < 0.
    ? std::max(1, origin.x - edge_rect.left)
    : std::max(1, edge_rect.right - origin.x);
  const double y_limit = dy < 0.
    ? std::max(1, origin.y - edge_rect.top)
    : std::max(1, edge_rect.bottom - origin.y);
  const double scale = std::min(x_limit / std::abs(dx),
                                y_limit / std::abs(dy));
  return PixelPoint{
    (int)std::lround(origin.x + dx * scale),
    (int)std::lround(origin.y + dy * scale),
  };
}

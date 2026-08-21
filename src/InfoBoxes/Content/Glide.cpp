// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "InfoBoxes/Content/Glide.hpp"
#include "Engine/Util/Gradient.hpp"
#include "Formatter/GlideRatioFormatter.hpp"
#include "InfoBoxes/Data.hpp"
#include "Interface.hpp"

void
UpdateInfoBoxGRInstant(InfoBoxData &data) noexcept
{
  const auto gr = CommonInterface::Calculated().gr;

  if (!::GradientValid(gr)) {
    data.SetInvalid();
    return;
  }

  // Set Value
  data.SetValueFromGlideRatio(gr);
}

void
UpdateInfoBoxGRCruise(InfoBoxData &data) noexcept
{
  const auto &basic = CommonInterface::Basic();
  const auto &calculated = CommonInterface::Calculated();
  const auto cruise_gr = calculated.cruise_gr;

  if (!::GradientValid(cruise_gr)) {
    data.SetInvalid();
    return;
  }

  // Set Value
  data.SetValueFromGlideRatio(cruise_gr);

  if (basic.location_available)
    data.SetCommentFromDistance(basic.location.DistanceS(calculated.cruise_start_location));
  else
    data.SetCommentInvalid();
}

void
UpdateInfoBoxGRAvg(InfoBoxData &data) noexcept
{
  const auto average_gr = CommonInterface::Calculated().average_gr;

  if (average_gr == 0) {
    data.SetInvalid();
    return;
  }

  // Set Value
  if (average_gr < 0)
    data.SetValue("^^^");
  else if (!::GradientValid(average_gr))
    data.SetValue("+++");
  else
    data.SetValueFromGlideRatio(average_gr);
}

void
UpdateInfoBoxGRAvgTE(InfoBoxData &data) noexcept
{
  const auto &calculated = CommonInterface::Calculated();
  const auto average_gr_te = calculated.average_gr_te;

  if (average_gr_te == 0) {
    data.SetInvalid();
    return;
  }

  // Set Value
  if (average_gr_te < 0)
    data.SetValue("^^^");
  else if (!::GradientValid(average_gr_te))
    data.SetValue("+++");
  else {
    data.SetValueFromGlideRatio(average_gr_te);

    const auto &task_stats = calculated.task_stats;
    const auto final_gr = task_stats.total.gradient;
    data.SetCommentInvalid();
    if (task_stats.task_valid && final_gr > 0 && ::GradientValid(final_gr)) {
      FormatGlideRatio(data.comment.buffer(), data.comment.capacity(),
                       final_gr);
      if (average_gr_te > final_gr)
        data.SetAllColors(3);
    }
  }
}

void
UpdateInfoBoxLDVario(InfoBoxData &data) noexcept
{
  const auto ld_vario = CommonInterface::Calculated().ld_vario;

  if (!::GradientValid(ld_vario) ||
      !CommonInterface::Basic().total_energy_vario_available ||
      !CommonInterface::Basic().airspeed_available) {
    data.SetInvalid();
    return;
  }

  // Set Value
  data.SetValueFromGlideRatio(ld_vario);
}

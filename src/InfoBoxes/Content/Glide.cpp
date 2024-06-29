// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "InfoBoxes/Content/Glide.hpp"
#include "Engine/Util/Gradient.hpp"
#include "InfoBoxes/Data.hpp"
#include "Interface.hpp"

#include <tchar.h>

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
  // const auto &basic = CommonInterface::Basic();
  const TaskStats &task_stats = CommonInterface::Calculated().task_stats;
  const auto &calculated = CommonInterface::Calculated();
  const auto cruise_gr = calculated.cruise_gr;

  auto gradient = task_stats.total.gradient;
  const auto average_gr = CommonInterface::Calculated().average_gr;

  if (!::GradientValid(cruise_gr)) {
    data.SetInvalid();
    return;
  }

  // Set Value
  data.SetValueFromGlideRatio(cruise_gr);

    // comment:
    if (gradient <= 0) {
      data.SetComment(_T("+++"));
      return;
    }

     if (::GradientValid(gradient))
      data.SetCommentFromGR(gradient);
    else
      data.SetCommentInvalid();

    //  if cruise > required make the intire info box greed
      data.SetValueColor((cruise_gr > gradient) ? 3 : 0);
      data.SetCommentColor((average_gr > gradient) ? 3 : 0);
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
    data.SetValue(_T("^^^"));
  else if (!::GradientValid(average_gr))
    data.SetValue(_T("+++"));
  else
    data.SetValueFromGlideRatio(average_gr);
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

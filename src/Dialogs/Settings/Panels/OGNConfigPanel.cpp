// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "OGNConfigPanel.hpp"
#include "Profile/Keys.hpp"
#include "Profile/Profile.hpp"
#include "Language/Language.hpp"
#include "Tracking/OGNSettings.hpp"
#include "Tracking/TrackingSettings.hpp"
#include "FLARM/Traffic.hpp"
#include "Widget/RowFormWidget.hpp"
#include "Form/DataField/Boolean.hpp"
#include "Form/DataField/Listener.hpp"
#include "Interface.hpp"
#include "UIGlobals.hpp"
#include "Components.hpp"
#include "NetComponents.hpp"
#include "Tracking/TrackingGlue.hpp"

namespace {

struct TypeRow {
  FlarmTraffic::AircraftType type;
  const char *label;
};

/** Order here defines the ControlIndex TYPE_FIRST.. layout below. */
constexpr TypeRow type_rows[] = {
  { FlarmTraffic::AircraftType::GLIDER, N_("Glider") },
  { FlarmTraffic::AircraftType::TOW_PLANE, N_("Tow plane") },
  { FlarmTraffic::AircraftType::HELICOPTER, N_("Helicopter") },
  { FlarmTraffic::AircraftType::PARACHUTE, N_("Parachute") },
  { FlarmTraffic::AircraftType::DROP_PLANE, N_("Drop plane") },
  { FlarmTraffic::AircraftType::HANG_GLIDER, N_("Hang glider") },
  { FlarmTraffic::AircraftType::PARA_GLIDER, N_("Paraglider") },
  { FlarmTraffic::AircraftType::POWERED_AIRCRAFT, N_("Powered aircraft") },
  { FlarmTraffic::AircraftType::JET_AIRCRAFT, N_("Jet aircraft") },
  { FlarmTraffic::AircraftType::FLYING_SAUCER, N_("Flying saucer") },
  { FlarmTraffic::AircraftType::BALLOON, N_("Balloon") },
  { FlarmTraffic::AircraftType::AIRSHIP, N_("Airship") },
  { FlarmTraffic::AircraftType::UAV, N_("Unmanned aerial vehicle") },
  { FlarmTraffic::AircraftType::STATIC_OBJECT, N_("Static object") },
  { FlarmTraffic::AircraftType::UNKNOWN, N_("Unknown") },
};

} // namespace

enum ControlIndex {
  ENABLED,
  RANGE,
  TYPE_FIRST,
};

class OGNConfigPanel final
  : public RowFormWidget, DataFieldListener {
public:
  OGNConfigPanel()
    :RowFormWidget(UIGlobals::GetDialogLook()) {}

  void SetEnabled(bool enabled);

  /* virtual methods from class Widget */
  void Prepare(ContainerWindow &parent, const PixelRect &rc) noexcept override;
  bool Save(bool &changed) noexcept override;

private:
  /* methods from DataFieldListener */
  void OnModified(DataField &df) noexcept override;
};

void
OGNConfigPanel::SetEnabled(bool enabled)
{
  SetRowEnabled(RANGE, enabled);
  for (unsigned i = 0; i < std::size(type_rows); ++i)
    SetRowEnabled(TYPE_FIRST + i, enabled);
}

void
OGNConfigPanel::OnModified(DataField &df) noexcept
{
  if (IsDataField(ENABLED, df)) {
    const DataFieldBoolean &dfb = (const DataFieldBoolean &)df;
    SetEnabled(dfb.GetValue());
  }
}

void
OGNConfigPanel::Prepare(ContainerWindow &parent,
                        const PixelRect &rc) noexcept
{
  RowFormWidget::Prepare(parent, rc);

  const auto &settings =
    CommonInterface::GetComputerSettings().tracking.ogn;

  AddBoolean(_("Enable OGN traffic"),
             _("Connect directly to the Open Glider Network (OGN) "
               "APRS-IS feed and show received traffic using the "
               "regular FLARM traffic display."),
             settings.enabled, this);

  AddInteger(_("Range"),
             _("Only request OGN traffic within this distance of the "
               "current GPS position."),
             "%u km", "%u",
             OGNSettings::MIN_RANGE_KM, OGNSettings::MAX_RANGE_KM, 5,
             settings.EffectiveRangeKm());

  for (const auto &row : type_rows)
    AddBoolean(gettext(row.label),
              _("Show this aircraft type as OGN traffic."),
              settings.IsTypeEnabled(row.type));

  SetEnabled(settings.enabled);
}

bool
OGNConfigPanel::Save(bool &_changed) noexcept
{
  bool changed = false;

  auto &settings =
    CommonInterface::SetComputerSettings().tracking.ogn;

  changed |= SaveValue(ENABLED, ProfileKeys::OGNEnabled, settings.enabled);

  unsigned range_km = settings.EffectiveRangeKm();
  if (SaveValueInteger(RANGE, ProfileKeys::OGNRangeKM, range_km)) {
    settings.range_km = range_km;
    changed = true;
  }

  unsigned mask = 0;
  for (unsigned i = 0; i < std::size(type_rows); ++i)
    if (GetValueBoolean(TYPE_FIRST + i))
      mask |= 1u << unsigned(type_rows[i].type);

  if (mask != settings.aircraft_type_mask) {
    settings.aircraft_type_mask = mask;
    Profile::Set(ProfileKeys::OGNAircraftTypeMask, mask);
    changed = true;
  }

  _changed |= changed;

#ifdef HAVE_TRACKING
  if (changed && net_components != nullptr && net_components->tracking != nullptr)
    net_components->tracking->SetSettings(
      CommonInterface::GetComputerSettings().tracking);
#endif

  return true;
}


std::unique_ptr<Widget>
CreateOGNConfigPanel()
{
  return std::make_unique<OGNConfigPanel>();
}

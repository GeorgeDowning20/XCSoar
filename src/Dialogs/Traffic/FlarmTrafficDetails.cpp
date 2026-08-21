// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

/**
 * @file
 * The FLARM Traffic Details dialog displaying extended information about
 * the FLARM targets from the FLARMnet database
 * @todo Button that opens the Waypoint details dialog of the
 * home airport (if found in FLARMnet and local waypoint database)
 */

#include "TrafficDialogs.hpp"
#include "Look/TrafficLook.hpp"
#include "Dialogs/TextEntry.hpp"
#include "Dialogs/Message.hpp"
#include "Dialogs/WidgetDialog.hpp"
#include "Widget/RowFormWidget.hpp"
#include "FLARM/FlarmNetRecord.hpp"
#include "FLARM/Traffic.hpp"
#include "FLARM/List.hpp"
#include "FLARM/Details.hpp"
#include "FLARM/Friends.hpp"
#include "FLARM/Glue.hpp"
#include "FLARM/FadingTraffic.hpp"
#include "Geo/GeoVector.hpp"
#include "Renderer/ColorButtonRenderer.hpp"
#include "UIGlobals.hpp"
#include "Components.hpp"
#include "Formatter/UserUnits.hpp"
#include "Formatter/AngleFormatter.hpp"
#include "Formatter/TimeFormatter.hpp"
#include "util/StringBuilder.hxx"
#include "util/StringCompare.hxx"
#include "util/Macros.hpp"
#include "Language/Language.hpp"
#include "Interface.hpp"
#include "Blackboard/LiveBlackboard.hpp"
#include "Blackboard/BlackboardListener.hpp"
#include "TeamActions.hpp"

#include <optional>

class FlarmTrafficDetailsWidget final
  : public RowFormWidget, NullBlackboardListener {
  enum Controls {
    CALLSIGN,
    CHANGE_CALLSIGN_BUTTON,
    SPACER1,
    DISTANCE,
    ALTITUDE,
    VARIO,
    LAST_SEEN,
    SPACER2,
    PILOT,
    AIRPORT,
    RADIO,
    PLANE,
    SOURCE,
    TRAFFIC_SOURCE,
  };

  WndForm &dialog;

  const FlarmId target_id;
  bool open_traffic_list = false;

public:
  FlarmTrafficDetailsWidget(WndForm &_dialog, FlarmId _target_id)
    :RowFormWidget(_dialog.GetLook()), dialog(_dialog),
     target_id(_target_id) {}

  void CreateButtons(WidgetDialog &buttons);

  bool ShouldOpenTrafficList() const noexcept {
    return open_traffic_list;
  }

  /* virtual methods from Widget */
  void Prepare(ContainerWindow &parent, const PixelRect &rc) noexcept override;
  void Show(const PixelRect &rc) noexcept override;
  void Hide() noexcept override;

private:
  void UpdateChanging(const MoreData &basic);
  void Update();

  /**
   * Look up the target in the live traffic list, falling back to the
   * last known ("fading"/grey) snapshot if it has disappeared.  The
   * fallback snapshot, if used, is stored in @p fallback so its
   * lifetime matches the returned pointer.
   */
  static const FlarmTraffic *
  ResolveTarget(const TrafficList &live, FlarmId target_id,
               std::optional<FlarmTraffic> &fallback) noexcept;

  void OnCallsignClicked();
  void OnTeamClicked();
  void OnFriendColorClicked(FlarmColor color);
  void OnTrafficListClicked();

  /* virtual methods from BlackboardListener */
  void OnGPSUpdate([[maybe_unused]] const MoreData &basic) override {
    Update();
  }
};

inline void
FlarmTrafficDetailsWidget::CreateButtons(WidgetDialog &buttons)
{
  const ButtonLook &button_look = buttons.GetButtonLook();

  for (unsigned i = 1; i < (unsigned)FlarmColor::COUNT; ++i) {
    const FlarmColor color = (FlarmColor)i;
    buttons.AddButton(std::make_unique<ColorButtonRenderer>(
                        button_look, TrafficLook::GetTeamColor(color)),
                      [this, color](){ OnFriendColorClicked(color); });
  }

  buttons.AddButton(_("Clear"), [this](){ OnFriendColorClicked(FlarmColor::NONE); });
  buttons.AddButton(_("Team"), [this](){ OnTeamClicked(); });
  buttons.AddButton(_("Traffic list"), [this](){ OnTrafficListClicked(); });
}

void
FlarmTrafficDetailsWidget::Prepare([[maybe_unused]] ContainerWindow &parent,
                                   [[maybe_unused]] const PixelRect &rc) noexcept
{
  AddReadOnly(_("Callsign"));
  AddButton(_("Change callsign"), [this](){ OnCallsignClicked(); });
  AddSpacer();
  AddReadOnly(_("Distance"));
  AddReadOnly(_("Altitude"));
  AddReadOnly(_("Vario"));
  AddReadOnly(_("Last seen"));
  AddSpacer();
  AddReadOnly(_("Pilot"));
  AddReadOnly(_("Airport"));
  AddReadOnly(_("Radio frequency"));
  AddReadOnly(_("Plane type"));
  AddReadOnly(_("Data source"));
  AddReadOnly(_("Traffic source"));

  Update();
}

void
FlarmTrafficDetailsWidget::Show(const PixelRect &rc) noexcept
{
  RowFormWidget::Show(rc);
  Update();
  CommonInterface::GetLiveBlackboard().AddListener(*this);
}

void
FlarmTrafficDetailsWidget::Hide() noexcept
{
  CommonInterface::GetLiveBlackboard().RemoveListener(*this);
  RowFormWidget::Hide();
}

/**
 * Updates all the dialogs fields, that are changing frequently.
 * e.g. climb speed, distance, height
 */
const FlarmTraffic *
FlarmTrafficDetailsWidget::ResolveTarget(const TrafficList &live,
                                         FlarmId target_id,
                                         std::optional<FlarmTraffic> &fallback) noexcept
{
  if (const FlarmTraffic *target = live.FindTraffic(target_id);
      target != nullptr)
    return target;

  fallback = FlarmFadingTraffic::Find(target_id);
  return fallback ? &*fallback : nullptr;
}

void
FlarmTrafficDetailsWidget::UpdateChanging(const MoreData &basic)
{
  char tmp[40];
  const char *value;

  std::optional<FlarmTraffic> fallback;
  const FlarmTraffic *target =
    ResolveTarget(basic.flarm.traffic, target_id, fallback);
  const bool is_fading = fallback.has_value();

  bool target_ok = target && target->IsDefined();

  // Fill distance/direction field
  if (target_ok) {
    RoughDistance distance = target->distance;
    Angle bearing = target->Bearing();

    if (target->absolute_location && target->location.IsValid() &&
        basic.location_available) {
      const GeoVector vec{basic.location, target->location};
      distance = vec.distance;
      bearing = vec.bearing;
    }

    FormatUserDistanceSmart(distance, tmp, true, 20, 1000);
    char *p = tmp + strlen(tmp);
    *p++ = ' ';
    FormatAngleDelta(p, 20, bearing - basic.track);
    value = tmp;
  } else
    value = "--";

  SetText(DISTANCE, value);

  // Fill altitude field
  if (target_ok) {
    char *p = tmp;
    if (target->altitude_available) {
      FormatUserAltitude(target->altitude, p);
      p += strlen(p);
      *p++ = ' ';
    }

    RoughAltitude relative_altitude = target->relative_altitude;
    if (target->absolute_altitude && target->altitude_available) {
      if (const auto ownship_altitude = basic.GetAnyAltitude())
        relative_altitude =
          target->altitude - RoughAltitude(*ownship_altitude);
    }

    RoughDistance distance = target->distance;
    if (target->absolute_location && target->location.IsValid() &&
        basic.location_available)
      distance = GeoVector{basic.location, target->location}.distance;

    Angle dir = Angle::FromXY(distance, relative_altitude);
    FormatVerticalAngleDelta(p, 20, dir);

    value = tmp;
  } else
    value = "--";

  SetText(ALTITUDE, value);

  // Fill climb speed field
  if (target_ok && target->climb_rate_avg30s_available) {
    FormatUserVerticalSpeed(target->climb_rate_avg30s, tmp);
    value = tmp;
  } else
    value = "--";

  SetText(VARIO, value);

  // Fill "last seen" field (only meaningful while greyed out/fading)
  if (target_ok && is_fading) {
    const auto elapsed = Validity(basic.clock).GetTimeDifference(target->valid);
    StringFormat(tmp, sizeof(tmp), "%s %s",
                 FormatTimespanSmart(elapsed).c_str(), _("ago"));
    value = tmp;
  } else
    value = "--";

  SetText(LAST_SEEN, value);
}

/**
 * Updates all the dialogs fields.
 * Should be called on dialog opening as it closes the dialog when the
 * target does not exist.
 */
void
FlarmTrafficDetailsWidget::Update()
{
  char tmp[200], tmp_id[7];
  const char *value;

  // Set the dialog caption
  StringFormatUnsafe(tmp, "%s (%s)",
                     _("Traffic Details"), target_id.Format(tmp_id));
  dialog.SetCaption(tmp);

  std::optional<FlarmTraffic> fallback;
  const FlarmTraffic *target =
    ResolveTarget(CommonInterface::Basic().flarm.traffic, target_id, fallback);

  const ResolvedInfo info = FlarmDetails::ResolveInfo(target_id);

  // Shared fields: pilot/plane/airfield direct from resolver
  SetText(PILOT, !info.pilot.empty() ? info.pilot.c_str() : "--");

  const char *plane_value = !info.plane_type.empty() ? info.plane_type.c_str() : nullptr;
  if (plane_value == nullptr && target != nullptr)
    plane_value = FlarmTraffic::GetTypeString(target->type);
  SetText(PLANE, plane_value != nullptr ? plane_value : "--");

  SetText(AIRPORT, !info.airfield.empty() ? info.airfield.c_str() : "--");

  char fbuf[16];
  const char *freq = info.frequency.Format(fbuf, 16);
  value = freq != nullptr ? UnsafeBuildString(tmp, freq, " MHz") : "--";
  SetText(RADIO, value);

  // Fill the callsign field (+ registration). Prefer resolved
  // callsign; fall back to live traffic name (e.g. ADS-B).
  const char *cs = !info.callsign.empty() ? info.callsign.c_str() : nullptr;
  if (cs == nullptr &&
      target != nullptr && target->HasName() && !StringIsEmpty(target->name))
    cs = target->name.c_str();

  if (cs != nullptr && cs[0] != 0) {
    try {
      BasicStringBuilder<char> builder(tmp, ARRAY_SIZE(tmp));
      builder.Append(cs);
      if (!info.registration.empty())
        builder.Append(" (", info.registration.c_str(), ")");
      value = tmp;
    } catch (BasicStringBuilder<char>::Overflow) {
      value = cs;
    }
  } else
    value = "--";
  SetText(CALLSIGN, value);

  const char *data_source = FlarmDetails::ToString(info.source);
  if (info.source == ResolvedSource::NONE && target != nullptr)
    data_source = FlarmTraffic::GetSourceString(target->source);
  SetText(SOURCE, data_source);

  // Traffic source type (FLARM, ADS-B, Mode-S, etc.) and signal strength
  if (target != nullptr) {
    StaticString<64> source_str;
    source_str = FlarmTraffic::GetSourceString(target->source);
    if (target->rssi_available)
      source_str.AppendFormat(" (%d dBm)", (int)target->rssi);
    SetText(TRAFFIC_SOURCE, source_str);
  } else {
    SetText(TRAFFIC_SOURCE, "--");
  }

  // Update the frequently changing fields too
  UpdateChanging(CommonInterface::Basic());
}

/**
 * This event handler is called when the "Team" button is pressed
 */
inline void
FlarmTrafficDetailsWidget::OnTeamClicked()
{
  const FlarmTraffic *target =
    CommonInterface::Basic().flarm.traffic.FindTraffic(target_id);
  if (target != nullptr && target->no_track) {
    ShowMessageBox(_("This target has NoTrack enabled and may not be persisted."),
                   _("Privacy"), MB_OK);
    return;
  }

  if (ShowMessageBox(_("Do you want to set this FLARM contact as your new teammate?"),
                  _("New Teammate"), MB_YESNO) != IDYES)
    return;

  TeamActions::TrackFlarm(target_id);

  dialog.SetModalResult(mrOK);
}

/**
 * This event handler is called when the "Change Callsign" button is pressed
 */
inline void
FlarmTrafficDetailsWidget::OnCallsignClicked()
{
  const FlarmTraffic *target =
    CommonInterface::Basic().flarm.traffic.FindTraffic(target_id);
  if (target != nullptr && target->no_track) {
    ShowMessageBox(_("This target has NoTrack enabled and may not be persisted."),
                   _("Privacy"), MB_OK);
    return;
  }

  StaticString<21> newName;
  newName.clear();

  const char* cs = FlarmDetails::LookupCallsign(target_id);
  if (cs != nullptr && cs[0] != 0)
    newName = cs;

  if (TextEntryDialog(newName, _("Callsign")) &&
      FlarmDetails::AddSecondaryItem(target_id, newName))
    SaveFlarmNames();

  Update();
}

void
FlarmTrafficDetailsWidget::OnFriendColorClicked(FlarmColor color)
{
  const FlarmTraffic *target =
    CommonInterface::Basic().flarm.traffic.FindTraffic(target_id);
  if (target != nullptr && target->no_track) {
    ShowMessageBox(_("This target has NoTrack enabled and may not be persisted."),
                   _("Privacy"), MB_OK);
    return;
  }

  FlarmFriends::SetFriendColor(target_id, color);
  dialog.SetModalResult(mrOK);
}

void
FlarmTrafficDetailsWidget::OnTrafficListClicked()
{
  open_traffic_list = true;
  dialog.SetModalResult(mrCancel);
}

/**
 * The function opens the FLARM Traffic Details dialog
 */
bool
dlgFlarmTrafficDetailsShowModal(FlarmId id) noexcept
{
  const DialogLook &look = UIGlobals::GetDialogLook();

  WidgetDialog dialog(WidgetDialog::Full{}, UIGlobals::GetMainWindow(),
                      look, _("Traffic Details"));

  FlarmTrafficDetailsWidget *widget =
    new FlarmTrafficDetailsWidget(dialog, id);
  widget->CreateButtons(dialog);
  dialog.AddButton(_("Close"), mrCancel);
  dialog.FinishPreliminary(widget);
  const bool result = dialog.ShowModal() == mrOK;
  if (widget->ShouldOpenTrafficList())
    TrafficListDialog();

  return result;
}

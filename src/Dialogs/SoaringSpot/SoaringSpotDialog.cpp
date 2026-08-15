// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "SoaringSpotDialog.hpp"
#include "Language/Language.hpp"

#ifdef HAVE_HTTP

#include "Dialogs/CoFunctionDialog.hpp"
#include "Dialogs/Error.hpp"
#include "Dialogs/ListPicker.hpp"
#include "Dialogs/Message.hpp"
#include "Dialogs/WidgetDialog.hpp"
#include "Components.hpp"
#include "BackendComponents.hpp"
#include "DataComponents.hpp"
#include "Interface.hpp"
#include "LocalPath.hpp"
#include "UIGlobals.hpp"
#include "UtilsSettings.hpp"
#include "Look/DialogLook.hpp"
#include "Operation/PluggableOperationEnvironment.hpp"
#include "Profile/Keys.hpp"
#include "Profile/Profile.hpp"
#include "Renderer/TwoTextRowsRenderer.hpp"
#include "Repository/FileType.hpp"
#include "Task/ProtectedTaskManager.hpp"
#include "Widget/RowFormWidget.hpp"
#include "ui/control/List.hpp"
#include "system/FileUtil.hpp"
#include "system/Path.hpp"
#include "net/client/SoaringSpot/Client.hpp"
#include "net/http/Init.hpp"
#include "lib/curl/Global.hxx"
#include "Engine/Waypoint/Waypoints.hpp"
#include "Geo/AltitudeReference.hpp"
#include "Geo/Math.hpp"
#include "Engine/Task/Ordered/OrderedTask.hpp"
#include "Engine/Task/Ordered/Points/StartPoint.hpp"
#include "Engine/Task/Ordered/Points/FinishPoint.hpp"
#include "Engine/Task/Ordered/Points/AATPoint.hpp"
#include "Engine/Task/Ordered/Points/ASTPoint.hpp"
#include "Engine/Task/Factory/AbstractTaskFactory.hpp"
#include "Task/ObservationZones/CylinderZone.hpp"
#include "Task/ObservationZones/KeyholeZone.hpp"
#include "Task/ObservationZones/LineSectorZone.hpp"
#include "Task/ObservationZones/SectorZone.hpp"
#include "Task/ObservationZones/SymmetricSectorZone.hpp"
#include "util/CharUtil.hxx"
#include "util/StringCompare.hxx"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

using std::string_view_literals::operator""sv;

namespace {

/**
 * The competition the user has picked, persisted in the profile.
 */
struct State {
  std::string contest_url{Profile::Get(ProfileKeys::SoaringSpotContestURL, "")};
  std::string contest_name{Profile::Get(ProfileKeys::SoaringSpotContestName, "")};
  std::string class_slug{Profile::Get(ProfileKeys::SoaringSpotClassSlug, "")};
  std::string class_name{Profile::Get(ProfileKeys::SoaringSpotClassName, "")};

  /** what the last update did, shown when the dialog reopens */
  std::string status;
};

struct ListRow {
  std::string first, second;
};

class RowListRenderer final : public ListItemRenderer {
  TwoTextRowsRenderer &row_renderer;
  const std::vector<ListRow> &rows;

public:
  RowListRenderer(TwoTextRowsRenderer &_row_renderer,
                  const std::vector<ListRow> &_rows) noexcept
    :row_renderer(_row_renderer), rows(_rows) {}

  void OnPaintItem(Canvas &canvas, const PixelRect rc,
                   unsigned i) noexcept override {
    const auto &row = rows[i];
    row_renderer.DrawFirstRow(canvas, rc, row.first.c_str());
    if (!row.second.empty())
      row_renderer.DrawSecondRow(canvas, rc, row.second.c_str());
  }
};

int
PickRow(const char *caption, const std::vector<ListRow> &rows) noexcept
{
  const auto &look = UIGlobals::GetDialogLook();
  TwoTextRowsRenderer row_renderer;
  const unsigned height = row_renderer.CalculateLayout(*look.list.font,
                                                       look.small_font);
  RowListRenderer renderer{row_renderer, rows};
  return ListPicker(caption, rows.size(), 0, height, renderer);
}

/**
 * Run a coroutine in the I/O thread while showing a modal progress
 * dialog.  The factory is invoked with the ProgressListener which
 * remains valid for the whole run.
 */
template<typename F>
auto
RunNetwork(const char *caption, F &&make_task)
{
  PluggableOperationEnvironment env;
  return ShowCoFunctionDialog(UIGlobals::GetMainWindow(),
                              UIGlobals::GetDialogLook(),
                              caption, make_task(env), &env);
}

void
AppendLine(std::string &status, const char *label, const char *value) noexcept
{
  status += label;
  status += ": ";
  status += value;
  status += "\n";
}

/**
 * Make the given file the only entry of a profile file list, so that
 * the files of a previous competition are deactivated.
 *
 * @return true if the profile was changed
 */
bool
SetProfileFile(std::string_view key, Path path) noexcept
{
  const auto contracted = ContractLocalPath(path);
  const char *value = contracted != nullptr ? contracted.c_str() : path.c_str();

  if (StringIsEqual(Profile::Get(key, ""), value))
    return false;

  Profile::Set(key, value);
  return true;
}

void
ReloadSiteFiles(bool waypoints, bool airspace) noexcept
{
  const UISettings old_ui_settings = CommonInterface::GetUISettings();

  SettingsEnter();
  WaypointFileChanged = waypoints;
  AirspaceFileChanged = airspace;
  SettingsLeave(old_ui_settings);
}

/**
 * A turn point name split into its trigraph and the rest, e.g.
 * "NAW-Naseby West" or "NAW Naseby West" into "NAW" and
 * "Naseby West".  Either part may be absent: SoaringSpot sometimes
 * prints the plain name ("Walton") for a point the SeeYou file calls
 * "WLN Walton".
 */
struct SplitName {
  std::string code, label;
};

bool
IsTurnPointCode(std::string_view s) noexcept
{
  if (s.empty() || s.size() > 5)
    return false;

  for (const char ch : s)
    if (!IsUpperAlphaNumericASCII(ch))
      return false;

  return true;
}

SplitName
SplitTurnPointName(std::string_view name) noexcept
{
  if (const auto i = name.find_first_of("- "sv);
      i != std::string_view::npos) {
    const auto code = name.substr(0, i);
    const auto label = name.substr(i + 1);

    if (!label.empty() && IsTurnPointCode(code))
      return {std::string{code}, std::string{label}};
  }

  return {{}, std::string{name}};
}

/**
 * How a turn point was found in the waypoint file.
 */
enum class MatchMethod : uint_least8_t {
  /** the trigraph and the name both agree */
  EXACT,

  /** only part of the name agreed */
  NAME,

  /** located by the leg distance and bearing */
  POSITION,
};

const char *
ToString(MatchMethod method) noexcept
{
  switch (method) {
  case MatchMethod::EXACT:
    break;

  case MatchMethod::NAME:
    return _("name matching");

  case MatchMethod::POSITION:
    return _("position matching");
  }

  return "";
}

struct NameMatch {
  WaypointPtr waypoint;
  MatchMethod method;
};

/**
 * Find the waypoint the competition file uses for a SoaringSpot turn
 * point name.  A matching trigraph wins; otherwise the remaining name
 * is compared, which is what resolves the names SoaringSpot prints
 * without their trigraph.
 */
NameMatch
LookupTurnPoint(const Waypoints &waypoints, const std::string &name) noexcept
{
  if (auto wp = waypoints.LookupName(name); wp != nullptr)
    return {std::move(wp), MatchMethod::EXACT};

  const auto wanted = SplitTurnPointName(name);

  if (!wanted.code.empty()) {
    /* "NAW-Naseby West" is spelled "NAW Naseby West" in the file;
       trigraph and name agree, so this is not worth reporting */
    if (auto wp = waypoints.LookupName(wanted.code + " " + wanted.label);
        wp != nullptr)
      return {std::move(wp), MatchMethod::EXACT};

    if (auto wp = waypoints.LookupName(wanted.label); wp != nullptr)
      return {std::move(wp), MatchMethod::NAME};
  }

  WaypointPtr by_label = nullptr;

  for (const auto &wp : waypoints) {
    const auto have = SplitTurnPointName(wp->name);

    if (!wanted.code.empty() &&
        (StringIsEqualIgnoreCase(wp->shortname, wanted.code) ||
         StringIsEqualIgnoreCase(have.code, wanted.code)))
      return {wp, MatchMethod::NAME};

    if (by_label == nullptr &&
        StringIsEqualIgnoreCase(have.label, wanted.label))
      by_label = wp;
  }

  return {std::move(by_label), MatchMethod::NAME};
}

/**
 * A sector plus an inner cylinder; use the well-known named zones
 * where the dimensions match so the task editor shows them as such.
 */
std::unique_ptr<ObservationZonePoint>
CreateKeyhole(const SoaringSpot::TurnPoint &tp,
              const GeoPoint &location) noexcept
{
  if (fabs(tp.inner_radius - 500) < 2) {
    if (fabs(tp.angle - 90) < 2 && fabs(tp.radius - 20000) < 2)
      return KeyholeZone::CreateBGAFixedCourseZone(location);

    if (fabs(tp.angle - 90) < 2 && fabs(tp.radius - 10000) < 2)
      return KeyholeZone::CreateDAeCKeyholeZone(location);

    if (fabs(tp.angle - 180) < 2 && fabs(tp.radius - 10000) < 2)
      return KeyholeZone::CreateBGAEnhancedOptionZone(location);
  }

  auto oz = KeyholeZone::CreateCustomKeyholeZone(location, tp.radius,
                                                 Angle::Degrees(tp.angle));
  oz->SetInnerRadius(tp.inner_radius);
  return oz;
}

std::unique_ptr<ObservationZonePoint>
CreateOZ(const SoaringSpot::TurnPoint &tp, const GeoPoint &location,
         bool is_start, bool is_intermediate) noexcept
{
  if (tp.line && !is_intermediate)
    return std::make_unique<LineSectorZone>(location, tp.radius * 2);

  if (tp.has_radials)
    return std::make_unique<SectorZone>(location, tp.radius,
                                        Angle::Degrees(tp.start_radial),
                                        Angle::Degrees(tp.end_radial));

  /* SoaringSpot prints the half angle for the start */
  const double angle = is_start
    ? std::min(tp.angle * 2, 360.0)
    : tp.angle;

  if (angle >= 359.5)
    return std::make_unique<CylinderZone>(location, tp.radius);

  if (tp.inner_radius > 0 && is_intermediate)
    return CreateKeyhole(tp, location);

  return SymmetricSectorZone::CreateSymmetricCircularSectorZone(location,
                                                                tp.radius,
                                                                Angle::Degrees(angle));
}

/**
 * What happened while resolving the turn point names against the
 * waypoint file.
 */
struct BuildReport {
  /** turn points which were not found under the name SoaringSpot prints */
  std::vector<std::string> derived;

  /** turn points which had to be left out of the task */
  std::vector<std::string> skipped;

  bool IsClean() const noexcept {
    return derived.empty() && skipped.empty();
  }
};

struct ResolvedPoint {
  const SoaringSpot::TurnPoint *tp;
  WaypointPtr waypoint;

  /** was this the first row of the task, whose angle is a half angle? */
  bool original_first;
};

/**
 * Find the waypoint at the position SoaringSpot's leg distance and
 * bearing point at.  Returns nullptr unless exactly one waypoint is
 * close enough, so that a neighbouring turn point can never be
 * substituted silently.
 */
WaypointPtr
LookupByPosition(const Waypoints &waypoints, const GeoPoint &previous,
                 const SoaringSpot::TurnPoint &tp, bool is_finish) noexcept
{
  /* SoaringSpot rounds the leg to 10 m and 0.1 degrees, which is
     worth about half a kilometre over a long leg */
  const double tolerance = 300 + tp.leg_distance * 0.005;

  /* the last leg is measured to the edge of the finish ring, not to
     its centre */
  const double distances[] = {
    tp.leg_distance,
    is_finish ? tp.leg_distance + tp.radius : tp.leg_distance,
  };

  for (const double distance : distances) {
    const auto estimate = FindLatitudeLongitude(previous,
                                                Angle::Degrees(tp.leg_bearing),
                                                distance);

    WaypointPtr found = nullptr;
    unsigned count = 0;
    waypoints.VisitWithinRange(estimate, tolerance,
                               [&found, &count, &estimate,
                                tolerance](const WaypointPtr &wp){
                                 if (wp->location.DistanceS(estimate) > tolerance)
                                   return;

                                 ++count;
                                 found = wp;
                               });

    if (count == 1)
      return found;
  }

  return nullptr;
}

/**
 * Resolve the turn point names against the waypoint file.  Where the
 * name does not match, the leg distance and bearing printed by
 * SoaringSpot locate the point instead, which recovers turn points
 * whose name is spelled differently in the competition file.
 */
std::vector<ResolvedPoint>
ResolvePoints(const SoaringSpot::TaskDetails &details, BuildReport &report)
{
  const auto &waypoints = *data_components->waypoints;

  std::vector<ResolvedPoint> resolved;
  resolved.reserve(details.points.size());

  /* the location of the preceding row, which the next leg refers to;
     invalid after a row we could not resolve */
  auto previous = GeoPoint::Invalid();

  for (std::size_t i = 0; i < details.points.size(); ++i) {
    const auto &tp = details.points[i];

    auto [wp, method] = LookupTurnPoint(waypoints, tp.name);

    if (wp == nullptr && previous.IsValid() && tp.leg_distance > 0) {
      wp = LookupByPosition(waypoints, previous, tp,
                            i == details.points.size() - 1);
      method = MatchMethod::POSITION;
    }

    if (wp == nullptr) {
      report.skipped.emplace_back(tp.name);
      previous.SetInvalid();
      continue;
    }

    if (method != MatchMethod::EXACT)
      report.derived.emplace_back(tp.name + " \u2192 " + wp->name +
                                  " (" + ToString(method) + ")");

    previous = wp->location;
    resolved.emplace_back(ResolvedPoint{&tp, std::move(wp), i == 0});
  }

  return resolved;
}

std::unique_ptr<OrderedTask>
BuildTask(const SoaringSpot::TaskDetails &details, BuildReport &report)
{
  const auto &settings = CommonInterface::GetComputerSettings();

  const auto resolved = ResolvePoints(details, report);
  const std::size_t n = resolved.size();

  if (n < 2)
    throw std::runtime_error{_("No turn point of this task is in the waypoint file")};

  const bool aat = details.duration.count() > 0;
  const auto factory_type = aat
    ? TaskFactoryType::AAT : TaskFactoryType::RACING;

  auto task = std::make_unique<OrderedTask>(settings.task);
  task->SetFactory(factory_type);

  {
    auto ordered_settings = task->GetOrderedTaskSettings();

    if (aat)
      ordered_settings.aat_min_time = details.duration;

    if (const double max_height = details.points.front().max_height;
        max_height > 0) {
      ordered_settings.start_constraints.max_height = (unsigned)max_height;
      ordered_settings.start_constraints.max_height_ref = AltitudeReference::MSL;
    }

    if (const double min_height = details.points.back().min_height;
        min_height > 0) {
      ordered_settings.finish_constraints.min_height = (unsigned)min_height;
      ordered_settings.finish_constraints.min_height_ref = AltitudeReference::MSL;
    }

    task->SetOrderedTaskSettings(ordered_settings);
  }

  auto &factory = task->GetFactory();

  for (std::size_t i = 0; i < n; ++i) {
    const bool is_intermediate = i > 0 && i < n - 1;
    auto wp = resolved[i].waypoint;
    auto oz = CreateOZ(*resolved[i].tp, wp->location,
                       resolved[i].original_first, is_intermediate);

    std::unique_ptr<OrderedTaskPoint> point;
    if (i == 0)
      point = factory.CreateStart(std::move(oz), std::move(wp));
    else if (i == n - 1)
      point = factory.CreateFinish(std::move(oz), std::move(wp));
    else if (aat)
      point = factory.CreateAATPoint(std::move(oz), std::move(wp));
    else
      point = factory.CreateASTPoint(std::move(oz), std::move(wp));

    if (point != nullptr)
      factory.Append(*point, false);
  }

  return task;
}

void
ShowBuildReport(const BuildReport &report) noexcept
{
  if (report.IsClean())
    return;

  std::string message;

  if (!report.skipped.empty()) {
    message += _("These turn points are not in the waypoint file and have been LEFT OUT. The task is incomplete - check it before you fly!");
    message += "\n\n";
    for (const auto &name : report.skipped)
      message += "    " + name + "\n";
  }

  if (!report.derived.empty()) {
    if (!message.empty())
      message += "\n";

    message += _("These turn points are not named as SoaringSpot spells them. Check they are right before you fly!");
    message += "\n\n";
    for (const auto &name : report.derived)
      message += "    " + name + "\n";
  }

  ShowMessageBox(message.c_str(), _("Check the task"),
                 MB_OK | MB_ICONEXCLAMATION);
}

/**
 * Download the competition's airspace and turn point files unless we
 * already have the current version, and make them the only active
 * ones.
 */
void
UpdateFiles(const State &state, std::string &status)
{
  const auto files = RunNetwork(_("Download"),
                                [&state](ProgressListener &env){
                                  return SoaringSpot::ListFiles(*Net::curl,
                                                                state.contest_url,
                                                                env);
                                });
  if (!files)
    return;

  bool waypoints_changed = false, airspace_changed = false;

  for (const bool airspace : {false, true}) {
    const auto kind = airspace
      ? SoaringSpot::FileKind::AIRSPACE : SoaringSpot::FileKind::WAYPOINT;
    const auto type = airspace ? FileType::AIRSPACE : FileType::WAYPOINT;
    const auto url_key = airspace
      ? ProfileKeys::SoaringSpotAirspaceURL
      : ProfileKeys::SoaringSpotWaypointURL;
    const auto list_key = airspace
      ? ProfileKeys::AirspaceFileList : ProfileKeys::WaypointFileList;
    const char *label = airspace ? _("Airspace") : _("Turn points");

    const auto i = std::find_if(files->begin(), files->end(),
                                [kind](const SoaringSpot::File &file){
                                  return file.kind == kind;
                                });
    if (i == files->end()) {
      AppendLine(status, label, _("not published"));
      continue;
    }

    const Path name{i->name.c_str()};
    if (!name.IsValidFilename())
      throw std::runtime_error{"Invalid file name"};

    const auto directory = LocalPath(GetFileTypeDefaultDir(type));
    Directory::CreateRecursive(directory);
    const auto path = AllocatedPath::Build(directory, name);

    const bool current = StringIsEqual(Profile::Get(url_key, ""),
                                       i->url.c_str()) && File::Exists(path);
    if (!current) {
      const auto ok = RunNetwork(_("Download"),
                                 [i, &path](ProgressListener &env){
                                   return SoaringSpot::DownloadFile(*Net::curl,
                                                                    i->url,
                                                                    AllocatedPath{path.c_str()},
                                                                    env);
                                 });
      if (!ok)
        break;

      Profile::Set(url_key, i->url.c_str());
    }

    const bool activated = SetProfileFile(list_key, path);
    if (airspace)
      airspace_changed = activated;
    else
      waypoints_changed = activated;

    std::string value{i->name};
    if (current && !activated) {
      value += " (";
      value += _("up to date");
      value += ")";
    }

    AppendLine(status, label, value.c_str());
  }

  if (waypoints_changed || airspace_changed) {
    Profile::Save();
    ReloadSiteFiles(waypoints_changed, airspace_changed);
  }
}

/**
 * Download one task and make it the active task.
 */
void
LoadTask(const SoaringSpot::TaskRef &task_ref, std::string &status)
{
  const std::string url = task_ref.url;

  const auto details = RunNetwork(_("Download"),
                                  [&url](ProgressListener &env){
                                    return SoaringSpot::DownloadTask(*Net::curl,
                                                                     url, env);
                                  });
  if (!details)
    return;

  BuildReport report;
  auto task = BuildTask(*details, report);

  if (backend_components->protected_task_manager == nullptr)
    throw std::runtime_error{"No task manager"};

  backend_components->protected_task_manager->TaskCommit(*task);

  Profile::Set(ProfileKeys::SoaringSpotTaskURL, url.c_str());
  Profile::Save();

  std::string value{task_ref.name};
  if (!task_ref.date.empty()) {
    value += ", ";
    value += task_ref.date;
  }

  if (!report.skipped.empty()) {
    value += " (";
    value += _("incomplete");
    value += ")";
  }

  AppendLine(status, _("Task"), value.c_str());

  ShowBuildReport(report);
}

/**
 * Load the newest task of the selected class.
 */
void
UpdateTask(const State &state, const std::vector<SoaringSpot::TaskRef> &tasks,
           std::string &status)
{
  /* the task list is newest first within each class */
  const auto i = std::find_if(tasks.begin(), tasks.end(),
                              [&state](const SoaringSpot::TaskRef &task){
                                return state.class_slug.empty() ||
                                  task.class_slug == state.class_slug;
                              });
  if (i == tasks.end()) {
    AppendLine(status, _("Task"), _("not published"));
    return;
  }

  LoadTask(*i, status);
}

void
RunUpdate(State &state) noexcept
{
  state.status.clear();

  try {
    /* the turn point file must be in place before the task can be
       resolved against it */
    UpdateFiles(state, state.status);

    const auto results = RunNetwork(_("Download"),
                                    [&state](ProgressListener &env){
                                      return SoaringSpot::ListResults(*Net::curl,
                                                                      state.contest_url,
                                                                      env);
                                    });
    if (results)
      UpdateTask(state, results->tasks, state.status);
  } catch (...) {
    ShowError(std::current_exception(), _("SoaringSpot"));
  }
}

bool
SelectClass(State &state) noexcept
try {
  const auto results = RunNetwork(_("Download"),
                                  [&state](ProgressListener &env){
                                    return SoaringSpot::ListResults(*Net::curl,
                                                                    state.contest_url,
                                                                    env);
                                  });
  if (!results)
    return false;

  if (results->classes.empty()) {
    ShowMessageBox(_("No classes"), _("SoaringSpot"),
                   MB_OK | MB_ICONINFORMATION);
    return false;
  }

  std::vector<ListRow> rows;
  for (const auto &cls : results->classes)
    rows.emplace_back(cls.name, "");

  const int i = PickRow(_("Class"), rows);
  if (i < 0)
    return false;

  state.class_slug = results->classes[i].slug;
  state.class_name = results->classes[i].name;

  Profile::Set(ProfileKeys::SoaringSpotClassSlug, state.class_slug.c_str());
  Profile::Set(ProfileKeys::SoaringSpotClassName, state.class_name.c_str());
  Profile::Save();
  return true;
} catch (...) {
  ShowError(std::current_exception(), _("SoaringSpot"));
  return false;
}

/**
 * Pick any task of the competition, so that an earlier day can be
 * loaded instead of the newest one.
 */
bool
SelectTask(State &state) noexcept
try {
  const auto results = RunNetwork(_("Download"),
                                  [&state](ProgressListener &env){
                                    return SoaringSpot::ListResults(*Net::curl,
                                                                    state.contest_url,
                                                                    env);
                                  });
  if (!results)
    return false;

  std::vector<const SoaringSpot::TaskRef *> tasks;
  std::vector<ListRow> rows;

  for (const auto &task : results->tasks) {
    if (!state.class_slug.empty() && task.class_slug != state.class_slug)
      continue;

    std::string info;
    if (state.class_slug.empty()) {
      const auto cls = std::find_if(results->classes.begin(),
                                    results->classes.end(),
                                    [&task](const SoaringSpot::Class &c){
                                      return c.slug == task.class_slug;
                                    });
      info = cls != results->classes.end() ? cls->name : task.class_slug;

      if (!task.date.empty()) {
        info += ", ";
        info += task.date;
      }
    } else
      info = task.date;

    tasks.emplace_back(&task);
    rows.emplace_back(task.name, std::move(info));
  }

  if (rows.empty()) {
    ShowMessageBox(_("No tasks"), _("SoaringSpot"), MB_OK | MB_ICONINFORMATION);
    return false;
  }

  const int i = PickRow(_("Task"), rows);
  if (i < 0)
    return false;

  state.status.clear();
  LoadTask(*tasks[i], state.status);
  return true;
} catch (...) {
  ShowError(std::current_exception(), _("SoaringSpot"));
  return false;
}

bool
SelectContest(State &state) noexcept
try {
  const auto contests = RunNetwork(_("Download"), [](ProgressListener &env){
    return SoaringSpot::ListContests(*Net::curl, env);
  });
  if (!contests)
    return false;

  if (contests->empty()) {
    ShowMessageBox(_("No competitions"), _("SoaringSpot"),
                   MB_OK | MB_ICONINFORMATION);
    return false;
  }

  std::vector<ListRow> rows;
  for (const auto &contest : *contests)
    rows.emplace_back(contest.title, contest.info);

  const int i = PickRow(_("Competition"), rows);
  if (i < 0)
    return false;

  state.contest_url = (*contests)[i].url;
  state.contest_name = (*contests)[i].title;

  /* the class and the downloads belong to the previous competition */
  state.class_slug.clear();
  state.class_name.clear();
  state.status.clear();

  Profile::Set(ProfileKeys::SoaringSpotContestURL, state.contest_url.c_str());
  Profile::Set(ProfileKeys::SoaringSpotContestName, state.contest_name.c_str());
  Profile::Set(ProfileKeys::SoaringSpotClassSlug, "");
  Profile::Set(ProfileKeys::SoaringSpotClassName, "");
  Profile::Set(ProfileKeys::SoaringSpotAirspaceURL, "");
  Profile::Set(ProfileKeys::SoaringSpotWaypointURL, "");
  Profile::Set(ProfileKeys::SoaringSpotTaskURL, "");
  Profile::Save();

  SelectClass(state);
  return true;
} catch (...) {
  ShowError(std::current_exception(), _("SoaringSpot"));
  return false;
}

class SoaringSpotWidget final : public RowFormWidget {
  enum Controls {
    COMPETITION,
    CLASS,
    STATUS,
  };

  State &state;
  bool &update_requested;

public:
  SoaringSpotWidget(const DialogLook &look, State &_state,
                    bool &_update_requested) noexcept
    :RowFormWidget(look), state(_state),
     update_requested(_update_requested) {}

  /* virtual methods from class Widget */
  void Prepare(ContainerWindow &parent, const PixelRect &rc) noexcept override {
    RowFormWidget::Prepare(parent, rc);

    AddReadOnly(_("Competition"));
    AddReadOnly(_("Class"));
    AddMultiLine("");

    AddButton(_("Select competition"), [this](){
      if (SelectContest(state))
        Refresh();
    });

    AddButton(_("Select class"), [this](){
      if (RequireContest() && SelectClass(state))
        Refresh();
    });

    AddButton(_("Select task"), [this](){
      if (RequireContest() && SelectTask(state))
        Refresh();
    });

    Refresh();
  }

  bool RequestUpdate() noexcept {
    if (!RequireContest())
      return false;

    update_requested = true;
    return true;
  }

private:
  bool RequireContest() noexcept {
    if (!state.contest_url.empty())
      return true;

    ShowMessageBox(_("Select a competition first"), _("SoaringSpot"),
                   MB_OK | MB_ICONINFORMATION);
    return false;
  }

  void Refresh() noexcept {
    SetText(COMPETITION, state.contest_name.empty()
            ? _("(none)") : state.contest_name.c_str());
    SetText(CLASS, state.class_name.empty()
            ? _("(all classes)") : state.class_name.c_str());
    SetMultiLineText(STATUS, state.status.c_str());
  }
};

} // anonymous namespace

void
ShowSoaringSpotDialog()
{
  const auto &look = UIGlobals::GetDialogLook();
  State state;

  while (true) {
    bool update_requested = false;

    {
      TWidgetDialog<SoaringSpotWidget>
        dialog(WidgetDialog::Auto{}, UIGlobals::GetMainWindow(),
               look, _("SoaringSpot"));
      dialog.SetWidget(look, state, update_requested);
      dialog.AddButton(_("Update"), [&dialog](){
        if (dialog.GetWidget().RequestUpdate())
          dialog.SetModalResult(mrOK);
      });
      dialog.AddButton(_("Close"), mrCancel);
      dialog.ShowModal();
    }

    if (!update_requested)
      break;

    /* run the update with no dialog on screen: reloading the airspace
       and waypoint databases suspends the threads and refocuses the
       main window */
    RunUpdate(state);
  }
}

#else /* !HAVE_HTTP */

#include "Dialogs/Message.hpp"

void
ShowSoaringSpotDialog()
{
  ShowMessageBox(_("This function is not available on your platform yet."),
                 _("SoaringSpot"), MB_OK | MB_ICONERROR);
}

#endif /* !HAVE_HTTP */

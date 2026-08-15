// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "Parser.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <utility>

using std::string_view_literals::operator""sv;

namespace SoaringSpot {

static constexpr auto BASE_URL = "https://www.soaringspot.com"sv;

/**
 * A forward-only cursor over a HTML document.
 */
class Scanner {
  std::string_view rest;

public:
  explicit constexpr Scanner(std::string_view _rest) noexcept
    :rest(_rest) {}

  constexpr std::string_view GetRest() const noexcept {
    return rest;
  }

  /**
   * Move the cursor behind the next occurrence of #needle.
   */
  bool SkipPast(std::string_view needle) noexcept {
    const auto i = rest.find(needle);
    if (i == std::string_view::npos) {
      rest = {};
      return false;
    }

    rest = rest.substr(i + needle.size());
    return true;
  }

  /**
   * Return everything up to the next occurrence of #needle and move
   * the cursor behind it.
   */
  std::string_view ReadUntil(std::string_view needle) noexcept {
    const auto i = rest.find(needle);
    if (i == std::string_view::npos)
      return std::exchange(rest, {});

    const auto value = rest.substr(0, i);
    rest = rest.substr(i + needle.size());
    return value;
  }
};

static void
AppendUTF8(std::string &dest, unsigned ch) noexcept
{
  if (ch < 0x80)
    dest.push_back(static_cast<char>(ch));
  else if (ch < 0x800) {
    dest.push_back(static_cast<char>(0xc0 | (ch >> 6)));
    dest.push_back(static_cast<char>(0x80 | (ch & 0x3f)));
  } else {
    dest.push_back(static_cast<char>(0xe0 | (ch >> 12)));
    dest.push_back(static_cast<char>(0x80 | ((ch >> 6) & 0x3f)));
    dest.push_back(static_cast<char>(0x80 | (ch & 0x3f)));
  }
}

/**
 * Expand one HTML entity (without the leading ampersand and without
 * the trailing semicolon).
 */
static void
AppendEntity(std::string &dest, std::string_view name) noexcept
{
  if (name == "amp"sv)
    dest.push_back('&');
  else if (name == "lt"sv)
    dest.push_back('<');
  else if (name == "gt"sv)
    dest.push_back('>');
  else if (name == "quot"sv)
    dest.push_back('"');
  else if (name == "apos"sv)
    dest.push_back('\'');
  else if (name == "nbsp"sv)
    dest.push_back(' ');
  else if (name == "ndash"sv || name == "mdash"sv)
    dest.push_back('-');
  else if (name == "deg"sv)
    AppendUTF8(dest, 0xb0);
  else if (name.size() > 1 && name.front() == '#') {
    name.remove_prefix(1);

    int base = 10;
    if (!name.empty() && (name.front() == 'x' || name.front() == 'X')) {
      name.remove_prefix(1);
      base = 16;
    }

    const std::string number{name};
    char *endptr;
    const auto value = std::strtoul(number.c_str(), &endptr, base);
    if (*endptr == 0 && value > 0 && value < 0x10000)
      AppendUTF8(dest, static_cast<unsigned>(value));
  }
}

/**
 * Strip all HTML tags, expand entities and collapse white space.
 */
static std::string
ExtractText(std::string_view html) noexcept
{
  std::string value;
  bool pending_space = false;

  for (std::size_t i = 0; i < html.size(); ++i) {
    const char ch = html[i];

    if (ch == '<') {
      const auto end = html.find('>', i);
      if (end == std::string_view::npos)
        break;
      i = end;
      pending_space = true;
    } else if (ch == '&') {
      const auto end = html.find(';', i);
      if (end == std::string_view::npos || end - i > 10) {
        pending_space = true;
        continue;
      }

      std::string expanded;
      AppendEntity(expanded, html.substr(i + 1, end - i - 1));
      i = end;

      if (expanded == " ") {
        pending_space = true;
      } else if (!expanded.empty()) {
        if (pending_space && !value.empty())
          value.push_back(' ');
        pending_space = false;
        value += expanded;
      }
    } else if (static_cast<unsigned char>(ch) <= ' ') {
      pending_space = true;
    } else {
      if (pending_space && !value.empty())
        value.push_back(' ');
      pending_space = false;
      value.push_back(ch);
    }
  }

  return value;
}

/**
 * Parse a number which follows the given key, e.g. "R=" in
 * "Cylinder R=25.00 km".
 *
 * @return the value or a negative number if the key was not found
 */
static double
ParseValue(std::string_view text, std::string_view key,
           bool with_unit) noexcept
{
  const auto i = text.find(key);
  if (i == std::string_view::npos)
    return -1;

  const std::string tail{text.substr(i + key.size())};
  char *endptr;
  const double value = std::strtod(tail.c_str(), &endptr);
  if (endptr == tail.c_str())
    return -1;

  if (!with_unit)
    return value;

  while (*endptr == ' ')
    ++endptr;

  if (*endptr == 'k')
    return value * 1000;
  if (endptr[0] == 'N' && endptr[1] == 'M')
    return value * 1852;

  return value;
}

static std::chrono::seconds
ParseDuration(std::string_view text) noexcept
{
  unsigned hours = 0, minutes = 0, seconds = 0;
  const std::string copy{text};
  if (std::sscanf(copy.c_str(), "%u:%u:%u", &hours, &minutes, &seconds) < 2)
    return {};

  return std::chrono::hours{hours} + std::chrono::minutes{minutes} +
    std::chrono::seconds{seconds};
}

static std::string
MakeAbsoluteURL(std::string_view href) noexcept
{
  std::string url;
  if (href.starts_with("http"sv))
    url = href;
  else
    url = std::string{BASE_URL} + std::string{href};

  while (!url.empty() && url.back() == '/')
    url.pop_back();

  return url;
}

std::vector<Contest>
ParseContests(std::string_view html) noexcept
{
  std::vector<Contest> list;

  Scanner scanner{html};
  while (scanner.SkipPast("class=\"contest\""sv)) {
    /* the whole contest element, up to the start of the next one */
    Scanner item{scanner.GetRest()};

    if (!item.SkipPast("<h3>"sv))
      break;

    const auto heading = item.ReadUntil("</h3>"sv);

    Scanner link{heading};
    if (!link.SkipPast("href=\""sv))
      continue;

    Contest contest;
    contest.url = MakeAbsoluteURL(link.ReadUntil("\""sv));

    const auto slash = contest.url.rfind('/');
    contest.id = slash == std::string::npos
      ? contest.url
      : contest.url.substr(slash + 1);

    if (!link.SkipPast(">"sv))
      continue;

    contest.title = ExtractText(link.ReadUntil("</a>"sv));
    if (contest.title.empty())
      continue;

    if (item.SkipPast("class=\"info\""sv) && item.SkipPast(">"sv))
      contest.info = ExtractText(item.ReadUntil("</div>"sv));

    list.emplace_back(std::move(contest));
  }

  return list;
}

std::vector<File>
ParseFiles(std::string_view html) noexcept
{
  std::vector<File> list;

  Scanner scanner{html};
  while (scanner.SkipPast("<ul class=\"contest-downloads\">"sv)) {
    Scanner group{scanner.ReadUntil("</ul>"sv)};

    while (group.SkipPast("<a href=\""sv)) {
      File file;
      file.url = MakeAbsoluteURL(group.ReadUntil("\""sv));

      if (!group.SkipPast(">"sv))
        break;

      file.name = ExtractText(group.ReadUntil("</a>"sv));

      if (file.name.ends_with(".txt"))
        file.kind = FileKind::AIRSPACE;
      else if (file.name.ends_with(".cup"))
        file.kind = FileKind::WAYPOINT;
      else
        continue;

      list.emplace_back(std::move(file));
    }
  }

  return list;
}

/**
 * Convert "task-8-on-2026-08-15" to "Task 8" and "2026-08-15".
 */
static void
SplitTaskSlug(std::string_view slug, std::string &name,
              std::string &date) noexcept
{
  static constexpr auto on = "-on-"sv;

  const auto i = slug.find(on);
  if (i == std::string_view::npos) {
    name = slug;
    return;
  }

  auto number = slug.substr(0, i);
  if (number.starts_with("task-"sv))
    number.remove_prefix(5);

  name = "Task ";
  name += number;
  date = slug.substr(i + on.size());
}

/**
 * Append the task referenced by a "/tasks/<class>/task-N-on-DATE"
 * link, ignoring duplicates (each task is repeated once per
 * available site language).
 */
static void
AddTaskRef(std::vector<TaskRef> &list, std::string_view href) noexcept
{
  static constexpr auto tasks = "/tasks/"sv;

  const auto i = href.find(tasks);
  if (i == std::string_view::npos)
    return;

  const auto tail = href.substr(i + tasks.size());
  const auto slash = tail.find('/');
  if (slash == std::string_view::npos)
    return;

  const auto class_slug = tail.substr(0, slash);
  const auto slug = tail.substr(slash + 1);
  if (!slug.starts_with("task-"sv) ||
      slug.find('/') != std::string_view::npos)
    return;

  if (std::any_of(list.begin(), list.end(),
                  [class_slug, slug](const TaskRef &other){
                    return other.class_slug == class_slug &&
                      other.url.ends_with(slug);
                  }))
    return;

  TaskRef task;
  task.url = MakeAbsoluteURL(href);
  task.class_slug = class_slug;
  SplitTaskSlug(slug, task.name, task.date);
  list.emplace_back(std::move(task));
}

Results
ParseResults(std::string_view html) noexcept
{
  Results results;

  Scanner classes{html};
  while (classes.SkipPast("<table class=\"result-overview\">"sv)) {
    Scanner head{classes.ReadUntil("</thead>"sv)};
    if (!head.SkipPast("href=\""sv))
      continue;

    const auto href = head.ReadUntil("\""sv);
    const auto i = href.rfind('/');
    if (i == std::string_view::npos)
      continue;

    Class cls;
    cls.slug = href.substr(i + 1);

    if (!head.SkipPast(">"sv))
      continue;

    cls.name = ExtractText(head.ReadUntil("</a>"sv));
    if (cls.slug.empty() || cls.name.empty())
      continue;

    if (std::none_of(results.classes.begin(), results.classes.end(),
                     [&cls](const Class &other){
                       return other.slug == cls.slug;
                     }))
      results.classes.emplace_back(std::move(cls));
  }

  Scanner scanner{html};
  while (scanner.SkipPast("href=\""sv))
    AddTaskRef(results.tasks, scanner.ReadUntil("\""sv));

  return results;
}


/**
 * Parse one row of the "Observation zone" column.  The formats
 * produced by SoaringSpot are:
 *
 * - "Cylinder R=25.00 km"
 * - "Line 10.00 km (Radius 5.00 km)"
 * - "..., R=5.00 km, Angle=90.0°, Max.alt. is 1.62 km"
 * - "..., Rmin=0.50 km, Rmax=20.00 km, Angle=90.0°, Cylinder R=0.50 km"
 * - "R=55.00 km, Radial1=70.0°, Radial2=30.0°"
 *
 * "Angle" is the full opening angle for turn points, but the half
 * angle for the start, where 90 describes the usual 180 degree zone.
 */
static TurnPoint
ParseTurnPoint(std::string_view name, std::string_view oz) noexcept
{
  TurnPoint tp;
  tp.name = name;

  tp.max_height = std::max(ParseValue(oz, "Max.alt. is"sv, true), 0.0);
  tp.min_height = std::max(ParseValue(oz, "Min.alt. is"sv, true), 0.0);

  if (oz.find("Line"sv) != std::string_view::npos) {
    tp.line = true;
    const double length = ParseValue(oz, "Line"sv, true);
    if (length > 0)
      tp.radius = length / 2;
    return tp;
  }

  /* "Rmin"/"Rmax" describe a keyhole: a sector of Rmax combined with
     a cylinder of Rmin (repeated as the trailing "Cylinder R=") */
  double radius = ParseValue(oz, "Rmax="sv, true);
  if (radius <= 0)
    radius = ParseValue(oz, "R="sv, true);
  if (radius <= 0)
    radius = ParseValue(oz, "Radius"sv, true);
  if (radius > 0)
    tp.radius = radius;

  tp.inner_radius = std::max(ParseValue(oz, "Rmin="sv, true), 0.0);

  const double radial1 = ParseValue(oz, "Radial1="sv, false);
  const double radial2 = ParseValue(oz, "Radial2="sv, false);
  if (radial1 >= 0 && radial2 >= 0) {
    tp.has_radials = true;

    /* SoaringSpot does not say which radial bounds which side, so
       pick the arc spanning less than half a circle */
    if (std::fmod(radial1 - radial2 + 360, 360.0) <= 180) {
      tp.start_radial = radial2;
      tp.end_radial = radial1;
    } else {
      tp.start_radial = radial1;
      tp.end_radial = radial2;
    }

    return tp;
  }

  const double angle = ParseValue(oz, "Angle="sv, false);
  if (angle > 0)
    tp.angle = std::min(angle, 360.0);

  return tp;
}

TaskDetails
ParseTaskDetails(std::string_view html)
{
  TaskDetails details;

  {
    Scanner scanner{html};
    if (scanner.SkipPast("<h2 class=\"pull-left\">"sv))
      details.title = ExtractText(scanner.ReadUntil("</h2>"sv));
  }

  const auto table = html.find("<table class=\"task"sv);
  if (table == std::string_view::npos)
    throw std::runtime_error{"No task table found"};

  const auto duration = html.find("task-info task-duration"sv);
  if (duration != std::string_view::npos && duration < table) {
    Scanner scanner{html.substr(duration)};
    if (scanner.SkipPast("<strong>"sv))
      details.duration =
        ParseDuration(ExtractText(scanner.ReadUntil("</strong>"sv)));
  }

  Scanner scanner{html.substr(table)};
  if (!scanner.SkipPast("<tbody>"sv))
    throw std::runtime_error{"No task table found"};

  Scanner body{scanner.ReadUntil("</tbody>"sv)};
  while (body.SkipPast("<tr>"sv)) {
    Scanner row{body.ReadUntil("</tr>"sv)};

    std::string cells[4];
    unsigned n = 0;
    while (n < 4 && row.SkipPast("<td>"sv))
      cells[n++] = ExtractText(row.ReadUntil("</td>"sv));

    if (n < 4 || cells[0].empty())
      continue;

    auto &tp = details.points.emplace_back(ParseTurnPoint(cells[0], cells[3]));

    /* the leg from the previous turn point, used to locate a turn
       point whose name cannot be found in the waypoint file */
    tp.leg_distance = std::max(ParseValue(cells[1], ""sv, true), 0.0);
    tp.leg_bearing = std::max(ParseValue(cells[2], ""sv, false), 0.0);
  }

  if (details.points.size() < 2)
    throw std::runtime_error{"Task has too few turn points"};

  return details;
}

} // namespace SoaringSpot

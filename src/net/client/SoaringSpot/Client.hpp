// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "co/Task.hxx"
#include "system/Path.hpp"

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

class CurlGlobal;
class ProgressListener;

/**
 * A scraper for the SoaringSpot competition website
 * (https://www.soaringspot.com/).  SoaringSpot has no public API for
 * anonymous users, so this parses the HTML pages, similar to what
 * openvario-compman does.
 */
namespace SoaringSpot {

struct Contest {
  /** the last path segment of the contest URL */
  std::string id;

  /** absolute URL of the contest, without trailing slash */
  std::string url;

  std::string title;

  /** venue, dates, number of competitors */
  std::string info;
};

enum class FileKind : uint_least8_t {
  AIRSPACE,
  WAYPOINT,
};

struct File {
  std::string name;
  std::string url;
  FileKind kind;
};

struct TaskRef {
  std::string url;

  /** the URL slug of the competition class, e.g. "15-meter" */
  std::string class_slug;

  /** e.g. "Task 8" */
  std::string name;

  /** e.g. "2026-08-15" */
  std::string date;
};

struct Class {
  /** the URL slug, e.g. "15-meter" */
  std::string slug;

  /** the name as printed, e.g. "15 Meter" */
  std::string name;
};

/**
 * The contents of a competition's "results" page.
 */
struct Results {
  std::vector<Class> classes;

  /** all tasks of all classes, newest first */
  std::vector<TaskRef> tasks;
};

struct TurnPoint {
  /** as printed by SoaringSpot, e.g. "NAW-Naseby West" */
  std::string name;

  /** observation zone radius [m]; half the gate width for a line */
  double radius = 500;

  /**
   * The sector angle [degrees] exactly as printed; 360 means
   * cylinder.  This is the full opening angle for turn points, but
   * the half angle for the start.  Ignored if #has_radials.
   */
  double angle = 360;

  /** radius of the keyhole cylinder ("Rmin") [m], 0 if none */
  double inner_radius = 0;

  /**
   * A sector given by two explicit radials ("Radial1"/"Radial2")
   * instead of an opening angle.
   */
  bool has_radials = false;

  /** most counter-clockwise radial [degrees] */
  double start_radial = 0;

  /** most clockwise radial [degrees] */
  double end_radial = 0;

  /** is this a start/finish line? */
  bool line = false;

  /** "Max.alt. is" of the start [m MSL], 0 if unlimited */
  double max_height = 0;

  /** "Min.alt. is" of the finish [m MSL], 0 if unlimited */
  double min_height = 0;

  /** length of the leg from the previous turn point [m] */
  double leg_distance = 0;

  /** bearing of the leg from the previous turn point [degrees] */
  double leg_bearing = 0;
};

struct TaskDetails {
  std::string title;

  /**
   * The assigned area task duration.  Zero means this is a racing
   * task.
   */
  std::chrono::seconds duration{};

  std::vector<TurnPoint> points;
};

/**
 * Fetch the list of contests from the SoaringSpot front page.
 */
Co::Task<std::vector<Contest>>
ListContests(CurlGlobal &curl, ProgressListener &progress);

/**
 * Fetch the list of downloadable airspace and waypoint files of the
 * given contest.
 */
Co::Task<std::vector<File>>
ListFiles(CurlGlobal &curl, std::string contest_url,
          ProgressListener &progress);

/**
 * Fetch the classes and the task list of the given contest.
 */
Co::Task<Results>
ListResults(CurlGlobal &curl, std::string contest_url,
            ProgressListener &progress);

/**
 * Fetch the turn point list of one task.
 */
Co::Task<TaskDetails>
DownloadTask(CurlGlobal &curl, std::string task_url,
             ProgressListener &progress);

/**
 * Download a contest file to the given path.  Always returns true;
 * throws on error.
 */
Co::Task<bool>
DownloadFile(CurlGlobal &curl, std::string url, AllocatedPath path,
             ProgressListener &progress);

} // namespace SoaringSpot

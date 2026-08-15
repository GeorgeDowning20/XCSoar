// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "Client.hpp"

#include <string_view>

/**
 * The pure HTML parsers for the SoaringSpot client; separated from
 * the network code so they can be tested standalone.
 */
namespace SoaringSpot {

std::vector<Contest>
ParseContests(std::string_view html) noexcept;

std::vector<File>
ParseFiles(std::string_view html) noexcept;

/**
 * Parse the competition's "results" page.
 */
Results
ParseResults(std::string_view html) noexcept;

/**
 * Throws if the page does not contain a usable task.
 */
TaskDetails
ParseTaskDetails(std::string_view html);

} // namespace SoaringSpot

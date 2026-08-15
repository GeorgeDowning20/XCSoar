// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "Id.hpp"
#include "Traffic.hpp"
#include "List.hpp"
#include "time/Stamp.hpp"

#include <map>
#include <optional>

/**
 * Globally accessible, thread-safe snapshot of recently-disappeared
 * ("fading"/grey) FLARM targets.  Maintained by MapWindowBlackboard
 * (draw thread) and consulted by the FLARM traffic details dialog
 * (main thread) so a target's last known altitude/vario/etc. and time
 * since last seen remain visible while it's still shown greyed out.
 */
namespace FlarmFadingTraffic {

/**
 * Recompute the fading-target snapshot from the previous and current
 * traffic lists.
 *
 * @param enabled whether fading traffic is enabled at all
 * @param timeout_minutes how long a target stays in the snapshot after
 * disappearing from @p new_list
 */
void
Update(bool enabled, unsigned timeout_minutes,
       const TrafficList &old_list, const TrafficList &new_list,
       TimeStamp now) noexcept;

/**
 * Thread-safe copy of all currently fading targets.
 */
[[gnu::pure]]
std::map<FlarmId, FlarmTraffic>
GetAll() noexcept;

/**
 * Thread-safe lookup of one target's last known snapshot.
 */
[[gnu::pure]]
std::optional<FlarmTraffic>
Find(FlarmId id) noexcept;

} // namespace FlarmFadingTraffic

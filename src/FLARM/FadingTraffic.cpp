// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "FadingTraffic.hpp"
#include "thread/Mutex.hxx"

#include <chrono>

namespace FlarmFadingTraffic {

static Mutex mutex;
static std::map<FlarmId, FlarmTraffic> snapshot;

void
Update(bool enabled, unsigned timeout_minutes,
       const TrafficList &old_list, const TrafficList &new_list,
       TimeStamp now) noexcept
{
  const std::lock_guard<Mutex> lock(mutex);

  if (!enabled) {
    snapshot.clear();
    return;
  }

  /* first add all items from the old list */
  for (const auto &traffic : old_list.list)
    if (traffic.location_available)
      snapshot.try_emplace(traffic.id, traffic);

  /* now remove all items that are in the new list; now only items
     remain that have disappeared */
  for (const auto &traffic : new_list.list)
    if (auto i = snapshot.find(traffic.id); i != snapshot.end())
      snapshot.erase(i);

  /* remove all items that haven't been seen again for too long */
  const auto max_age = std::chrono::minutes{timeout_minutes};
  std::erase_if(snapshot, [now, max_age](const auto &i){
    return i.second.valid.IsOlderThan(now, max_age);
  });
}

std::map<FlarmId, FlarmTraffic>
GetAll() noexcept
{
  const std::lock_guard<Mutex> lock(mutex);
  return snapshot;
}

std::optional<FlarmTraffic>
Find(FlarmId id) noexcept
{
  const std::lock_guard<Mutex> lock(mutex);

  if (auto i = snapshot.find(id); i != snapshot.end())
    return i->second;

  return std::nullopt;
}

} // namespace FlarmFadingTraffic

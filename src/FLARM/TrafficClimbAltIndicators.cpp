#include "FLARM/TrafficClimbAltIndicators.hpp"
#include "Interface.hpp"

TrafficClimbAltIndicators
TrafficClimbAltIndicators::GetClimbAltIndicators(const FlarmTraffic &traffic) noexcept
{
   return GetClimbAltIndicators(traffic,
                                CommonInterface::GetComputerSettings().polar.glide_polar_task.GetMC(),
                                CommonInterface::Calculated().average);
}

TrafficClimbAltIndicators
TrafficClimbAltIndicators::GetClimbAltIndicators(const FlarmTraffic &traffic, const double set_mc,
                                                 const double current_30s_vario) noexcept
{
   TrafficClimbAltIndicators indicators;

   const double reference_climb_rate = (set_mc > current_30s_vario) ? set_mc : current_30s_vario; // largest of the set mc or current 30s average vario.

   if (traffic.climb_rate_avg30s >= reference_climb_rate)
      indicators.climb = Climb::GOOD;
   else if (traffic.climb_rate_avg30s > 0.0)
      indicators.climb = Climb::UP;
   else
      indicators.climb = Climb::DOWN;

   if (traffic.relative_altitude > (const RoughAltitude)similar_altitude_threshold_meters)
      indicators.rel_alt = RelAlt::ABOVE;
   else if (traffic.relative_altitude > (const RoughAltitude)-similar_altitude_threshold_meters)
      indicators.rel_alt = RelAlt::SAME;
   else
      indicators.rel_alt = RelAlt::BELOW;

   return indicators;
}
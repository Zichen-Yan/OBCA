/*
 * \file time_tool.h
 * \brief develop tool kit 
 *
 * Util for time
 *
 * \author Hongjun Wang, email:wang.hongjun6@byd.com, Copyright BYD
 * \version 1.0
 * \date 2021-11-01
 */
 
#ifndef BYDAPA_COMMON_TIME_TOOL_H_
#define BYDAPA_COMMON_TIME_TOOL_H_

#include "def_types.h"

#include <chrono>
#include <ratio>
#include <ostream>

/// \brief tool for bydapa system time
/// 
/// namespace for time
namespace bydapa {
namespace common {

constexpr int64 kUTSEpochOffsetFromUnixEpochInSeconds = (719162ll * 24ll * 60ll * 60ll);
constexpr int64 kUTSEpochOffsetFromUnixEpochInMillisenconds = (kUTSEpochOffsetFromUnixEpochInSeconds * 10000000ll);

// unit=100 nanoseconds;
struct UniversalTimeScaleClock {
	using rep = int64;
	using period = std::ratio<1, 10000000>;
	using duration = std::chrono::duration<rep, period>;
	using time_point = std::chrono::time_point<UniversalTimeScaleClock>;
	static constexpr bool is_steady = true;
};

using Duration = UniversalTimeScaleClock::duration;
using Time = UniversalTimeScaleClock::time_point;

Duration FromSeconds(const double seconds);
double ToSeconds(const Duration duration);

Duration FromMilliseconds(const int64 milliseconds);
int64 ToMilliseconds(const Duration duration);

// Creates a time from a Univeral Time Scal
Time FromUniveral(const int64 ticks);
// Outputs the Universal Time Scale timestamp for a given Time
int64 ToUniversal(const Time time);

Duration unix_epoch();
int64 universal_now();
int64 milli_timestamp_now();

std::ostream& operator<<(std::ostream& os, const Time time);

	
} // namespace common
} // namespace bydapa

#endif // BYDAPA_COMMON_TIME_TOOL_H_
/*
 * \file util_time.h
 * \brief process time util
 *
 * Algorithm time
 *
 * \author Hongjun Wang, email:wang.hongjun6@byd.com, Copyright BYD
 * \version 1.0
 * \date 2021-10-20
 */
#ifndef BYDAPA_COMMON_TICTOC_H_
#define BYDAPA_COMMON_TICTOC_H_

#include <chrono>
#include <string>
#include <sstream>
#include <iomanip>

/// \brief c++11 time about tic/toc
/// 
/// tic/toc for some process.
/// like this:
///   bydapa::common::TicToc tictoc;
///   tictoc.tic();
///   some process...
///   double nseconds = tictoc.toc();
///   std::cout << "It took me " << nseconds << " nseconds.";
///   tictoc.tic();
///   some other...
///   double nsecondstmp = tictoc.toc();
///   std::cout << "It took me " << nsecondstmp << " nseconds.";
namespace bydapa {
namespace common {

/// \brief initialize call back function for pushing data realtime
///
/// \param hasMS if true, appending '.xxx'
/// \return string '2021-09-10_15-20-45.123' or '2021-09-10_15-20-45'
/*std::string getTimeString222222(bool hasMS) {
	auto tNow = std::chrono::system_clock::now();
	auto tt = std::chrono::system_clock::to_time_t(tNow);

	std::tm tm;
	localtime_r(&tt, &tm);

	std::ostringstream oss;
	oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
	if (hasMS) {
		auto tSeconds = std::chrono::duration_cast<std::chrono::seconds>(tNow.time_since_epoch());
		auto tMilli = std::chrono::duration_cast<std::chrono::milliseconds>(tNow.time_since_epoch());
		auto ms = tMilli - tSeconds;
		oss << "." << std::setfill('0') << std::setw(3) << ms.count();
	}

	return oss.str();
}*/

class TicToc {
public:
	explicit TicToc() : tictic(), toctoc() {}
	~TicToc() {}
	
	TicToc(const TicToc&) = delete;
	TicToc& operator=(const TicToc&) = delete;

	void tic() {
		tictic = std::chrono::steady_clock::now();
	}

	inline double toc() {
		toctoc = std::chrono::steady_clock::now();
		std::chrono::steady_clock::duration time_span = toctoc - tictic;
		return double(time_span.count());
	}

private:
	std::chrono::steady_clock::time_point tictic;		///> start time
	std::chrono::steady_clock::time_point toctoc;		///> stop time
};
	
} // namespace common
} // namespace bydapa

#endif // BYDAPA_COMMON_TICTOC_H_

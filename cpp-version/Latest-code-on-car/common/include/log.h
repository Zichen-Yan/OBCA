/*
 * \file log.h
 * \brief log util
 *
 * Algorithm log
 *
 * \author Hongjun Wang, email:wang.hongjun6@byd.com, Copyright BYD
 * \version 1.0
 * \date 2021-11-10
 */
#ifndef BYDAPA_COMMON_LOG_H_
#define BYDAPA_COMMON_LOG_H_

#include <chrono>
#include <string>
#include <sstream>
#include <iomanip>
#include <mutex>
#include <regex>


#ifndef LOG_DEBUG
#define LOG_DEBUG(format, ...) \
	fprintf(bydapa::common::Log::GetInstance()->fileptr(),"%s BYDAPA_Module DEBUG %s:%d: " format "\n",bydapa::common::GetTimeString(true).c_str(),__FILE__,__LINE__, ## __VA_ARGS__)
#endif

#ifndef LOG_INFO
#define LOG_INFO(format, ...) \
	fprintf(bydapa::common::Log::GetInstance()->fileptr(),"%s BYDAPA_Module INFO %s:%d: " format "\n",bydapa::common::GetTimeString(true).c_str(),__FILE__,__LINE__, ## __VA_ARGS__)
#endif

#ifndef LOG_WARNING
#define LOG_WARNING(format, ...) \
	fprintf(bydapa::common::Log::GetInstance()->fileptr(),"%s BYDAPA_Module WARNING %s:%d: " format "\n",bydapa::common::GetTimeString(true).c_str(),__FILE__,__LINE__, ## __VA_ARGS__)
#endif

#ifndef LOG_ERROR
#define LOG_ERROR(format, ...) \
	fprintf(bydapa::common::Log::GetInstance()->fileptr(),"%s BYDAPA_Module ERROR %s:%d: " format "\n",bydapa::common::GetTimeString(true).c_str(),__FILE__,__LINE__, ## __VA_ARGS__)
#endif

namespace bydapa {
namespace common {

namespace {
/// \brief initialize call back function for pushing data realtime
///
/// \param hasMS if true, appending '.xxx'
/// \return string '2021-09-10_15-20-45.123' or '2021-09-10_15-20-45'
std::string GetTimeString(bool hasMS) {
	auto tNow = std::chrono::system_clock::now();
	auto tt = std::chrono::system_clock::to_time_t(tNow);

	std::tm tm;
	localtime_r(&tt, &tm); // Linux
	//localtime_s(&tm, &tt); // Windows

	std::ostringstream oss;
	oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
	if (hasMS) {
		auto tSeconds = std::chrono::duration_cast<std::chrono::seconds>(tNow.time_since_epoch());
		auto tMilli = std::chrono::duration_cast<std::chrono::milliseconds>(tNow.time_since_epoch());
		auto ms = tMilli - tSeconds;
		oss << "." << std::setfill('0') << std::setw(3) << ms.count();
	}

	return oss.str();
}

}

template<class T>
class Singleton {
public:
	using object_type = T;
	struct object_creator {
		object_creator() { Singleton<T>::GetInstance(); }
	};
	static object_creator creator_object;

public:
	static object_type* GetInstance() {
		static object_type _instance;
		return &_instance;
	}
};
template<typename T> typename Singleton<T>::object_creator Singleton<T>::creator_object;

class Log : public Singleton<Log> {
public:
	Log() {
		std::string timestr = GetTimeString(true);
		std::string filename = "/userdata/cyclonedds_test/log/BYDAPA_log.txt";
		pF = fopen(filename.c_str(), "wt");
	}

	~Log() {
		fflush(pF);
		fclose(pF);
		pF = nullptr;
	}

	void debug(std::string&& oMessage) {
		std::lock_guard<std::mutex> lock(mutex_);
		fprintf(pF, "%s BYDAPA_Module DEBUG %s:%d: %s\n", GetTimeString(true).c_str(),__FILE__,__LINE__, oMessage.c_str());
		fflush(pF);
	}

	FILE* fileptr() {
		std::lock_guard<std::mutex> lock(mutex_);
		return pF;
	}

	/// \brief set log file name
	///
	/// default is BYDAPA_log.txt, when not calling this method
	/// filename = BYDAPA_log_ + name + time_suffix.txt
	/// \param name filename has been valid checked
	/// \param time_suffix filename + time suffix if true
	void SetFilename(const std::string& name, bool time_suffix) {
		std::lock_guard<std::mutex> lock(mutex_);
		if(pF != nullptr) {
			fflush(pF);
			fclose(pF);
		}

		if(name.empty()) {
			time_suffix = true;
		}

		std::string timestr = GetTimeString(true);
		std::string filename = "/userdata/cyclonedds_test/log/BYDAPA_log_" + name;
		if(time_suffix) filename += timestr;
		filename += ".txt";

		std::string valid_filename(filename.size(), '\0');
		std::regex express("/|:| |>|<|\"|\\*|\\?|\\|");
		std::regex_replace(valid_filename.begin(),
							filename.begin(),
							filename.end(),
							express,
							"_");
		pF = fopen(valid_filename.c_str(), "wt");
	}

private:
	FILE* pF;
	std::mutex mutex_;
};
	
} // namespace common
} // namespace bydapa

#endif // BYDAPA_COMMON_LOG_H_
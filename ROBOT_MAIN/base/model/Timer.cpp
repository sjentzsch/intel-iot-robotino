#include "Timer.h"

Timer::Timer() {
	isRunning_ = false;
}

Timer::~Timer() {

}

void Timer::start() {
	msTime = boost::posix_time::microsec_clock::local_time();
	isRunning_ = true;
}

// if this returns a float everything goes very nasty !!!! DON'T DO IT! (the API did it in their timer class...)
long Timer::msecsElapsed() {
	if(isRunning_)
	{
		boost::posix_time::time_duration msdiff = boost::posix_time::microsec_clock::local_time() - msTime;
		return msdiff.total_milliseconds();
	}
	else
		return 0;
}

void Timer::reset() {
	isRunning_ = false;
}

bool Timer::isRunning() {
	return isRunning_;
}

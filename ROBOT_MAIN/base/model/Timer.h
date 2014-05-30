#ifndef TIMER_H_
#define TIMER_H_

#include <iostream>
#include <boost/date_time/posix_time/posix_time.hpp>

class Timer
{
public:
	Timer();
	~Timer();

	void start();
	long msecsElapsed();
	void reset();
	bool isRunning();

private:
	boost::posix_time::ptime msTime;
	bool isRunning_;
};

#endif /* TIMER_H_ */

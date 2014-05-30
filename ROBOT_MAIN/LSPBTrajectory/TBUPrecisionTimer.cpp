/*
 * TBUPrecisionTimer.cpp
 *
 *  Created on: 12.05.2012
 *      Author: root
 */

#include "TBUPrecisionTimer.h"

TBUPrecisionTimer::TBUPrecisionTimer() {
	// TODO Auto-generated constructor stub

}

TBUPrecisionTimer::~TBUPrecisionTimer() {
	// TODO Auto-generated destructor stub
}

void TBUPrecisionTimer::measure(struct timespec &time)
{
	clock_gettime(CLOCK_MONOTONIC, &time);
}

void TBUPrecisionTimer::restart()
{
	measure(ts_start);
}

long TBUPrecisionTimer::get_muSeconds()
{
	struct timespec ts_current;
	struct timespec ts_diff;

	measure(ts_current);
	ts_diff = subtract(ts_current, ts_start);

	return ts_diff.tv_sec * 1000000 + ts_diff.tv_nsec / 1000;
}

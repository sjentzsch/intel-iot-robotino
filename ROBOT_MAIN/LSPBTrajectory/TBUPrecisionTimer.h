/*
 * TBUPrecisionTimer.h
 *
 *  Created on: 12.05.2012
 *      Author: root
 */

#ifndef TBUPRECISIONTIMER_H_
#define TBUPRECISIONTIMER_H_

#include <time.h>
#include "timespec_utils.h"
#include <stdio.h>

class TBUPrecisionTimer {
private:
	struct timespec ts_start;

	void measure(struct timespec &time);
public:
	TBUPrecisionTimer();
	virtual ~TBUPrecisionTimer();
	void restart();
	long get_muSeconds(); // elapsed time in microseconds
};

#endif /* TBUPRECISIONTIMER_H_ */

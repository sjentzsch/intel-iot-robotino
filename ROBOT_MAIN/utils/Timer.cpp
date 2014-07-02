//
// Timer.cpp
//
// Authors:
//   Sören Jentzsch <soren.jentzsch@gmail.com>
//
// Copyright (c) 2014 Sören Jentzsch
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

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

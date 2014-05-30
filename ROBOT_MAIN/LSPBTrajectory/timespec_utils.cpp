#include "timespec_utils.h"

struct timespec subtract(struct timespec a, struct timespec b)
{
	struct timespec result;
	result.tv_sec  = a.tv_sec - b.tv_sec;
	result.tv_nsec  = a.tv_nsec - b.tv_nsec;

	while(result.tv_nsec >= 1000000000) // carry over nanoseconds to seconds if nano-diff is larger than
	{
		result.tv_sec += 1;
		result.tv_nsec -= 1000000000;
	}

	while(result.tv_nsec < 0) // if nano-diff is <0 carry over a second
	{
		result.tv_sec -= 1;
		result.tv_nsec += 1000000000;
	}

	return result;
}

struct timespec add(struct timespec a, struct timespec b)
{
	struct timespec result;
	result.tv_sec  = a.tv_sec + b.tv_sec;
	result.tv_nsec  = a.tv_nsec + b.tv_nsec;

	while(result.tv_nsec >= 1000000000) // carry over nanoseconds to seconds if nano-diff is larger than
	{
		result.tv_sec += 1;
		result.tv_nsec -= 1000000000;
	}

	while(result.tv_nsec < 0) // if nano-diff is <0 carry over a second
	{
		result.tv_sec -= 1;
		result.tv_nsec += 1000000000;
	}

	return result;
}

struct timespec measure()
{
	struct timespec result;
	clock_gettime(CLOCK_MONOTONIC, &result);
	return result;
}

double measureInMicroSeconds()
{
	struct timespec now = measure();
	double result = now.tv_sec * 1000000 + now.tv_nsec / 1000.0;
	return result;
}


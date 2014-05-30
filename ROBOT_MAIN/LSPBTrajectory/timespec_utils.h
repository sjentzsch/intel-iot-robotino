/*
 * timespec_utils.hpp
 *
 *  Created on: 17.05.2012
 *      Author: root
 */

#ifndef TIMESPEC_UTILS_HPP_
#define TIMESPEC_UTILS_HPP_

#include <time.h>

struct timespec subtract(struct timespec a, struct timespec b);

struct timespec add(struct timespec a, struct timespec b);

struct timespec measure();

double measureInMicroSeconds();


#endif /* TIMESPEC_UTILS_HPP_ */

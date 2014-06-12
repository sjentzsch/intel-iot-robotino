/*
 * FileLogger.h
 *
 *  Created on: 27.05.2011
 *      Author: root
 */

#ifndef FILELOGGER_H_
#define FILELOGGER_H_
#include <pantheios/pantheios.hpp>

//Inserters
#include <pantheios/inserters.hpp>

namespace FileLog = pantheios;
#define log_MotorController	pantheios::notice(1)
#define log_SensorEventGenerator pantheios::notice(2)
#define log_Communication pantheios::notice(4)
#define log_Camera pantheios::notice(5)
#define log_Job pantheios::notice(6)

#endif /* FILELOGGER_H_ */

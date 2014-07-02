//
// FileLoggerConfig.h
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

#ifndef FILELOGGERCONFIG_H_
#define FILELOGGERCONFIG_H_

#include "FileLogger.h"
#include <pantheios/frontends/stock.h>
#include <pantheios/backends/be.N.h> //include backend splitter
#include <pantheios/backends/bec.file.h> // include file backend
#include <pantheios/backends/bec.fprintf.h> // include console backend

const PAN_CHAR_T PANTHEIOS_FE_PROCESS_IDENTITY[] = "ROBOT_MAIN"; // set process name

PANTHEIOS_CALL(void) pantheios_be_file_getAppInit(int backEndId, pan_be_file_init_t* init) // configure file backend
{
    init->flags |=  PANTHEIOS_BE_INIT_F_NO_PROCESS_ID; // hide process name
    init->flags |=  PANTHEIOS_BE_INIT_F_HIDE_DATE; // hide date
    init->flags |=  PANTHEIOS_BE_INIT_F_HIGH_RESOLUTION; // use high resolution timer
    init->flags |=  PANTHEIOS_BE_INIT_F_NO_THREAD_ID; // hide thread id (is basically only a number, therefore not very humanreadable), better use e.g. [MotorController]
 //   init->flags |= PANTHEIOS_BE_INIT_F_NO_SEVERITY; // hide severity-level, turns prefix [time, NOTICE] into [time] and the same for all other severity-levels like EMERGENCY, ERROR
}

PANTHEIOS_CALL(void) pantheios_be_fprintf_getAppInit(int backEndId, pan_be_fprintf_init_t* init) // configure console backend
{
    init->flags |=  PANTHEIOS_BE_INIT_F_NO_PROCESS_ID;
    init->flags |=  PANTHEIOS_BE_INIT_F_HIDE_DATE;
    init->flags |=  PANTHEIOS_BE_INIT_F_HIGH_RESOLUTION;
    init->flags |=  PANTHEIOS_BE_INIT_F_NO_THREAD_ID;
    init->flags |=  PANTHEIOS_BE_INIT_F_NO_SEVERITY;
}

pan_be_N_t  PAN_BE_N_BACKEND_LIST[] = // configure backend splitter
{
	PANTHEIOS_BE_N_STDFORM_ENTRY(0, pantheios_be_fprintf, PANTHEIOS_BE_N_F_ID_MUST_MATCH_CUSTOM28),
	PANTHEIOS_BE_N_STDFORM_ENTRY(0, pantheios_be_file, PANTHEIOS_BE_N_F_ID_MUST_MATCH_CUSTOM28),
    PANTHEIOS_BE_N_STDFORM_ENTRY(1, pantheios_be_file, PANTHEIOS_BE_N_F_ID_MUST_MATCH_CUSTOM28),
    PANTHEIOS_BE_N_STDFORM_ENTRY(2, pantheios_be_file, PANTHEIOS_BE_N_F_ID_MUST_MATCH_CUSTOM28),
    PANTHEIOS_BE_N_STDFORM_ENTRY(2, pantheios_be_fprintf, PANTHEIOS_BE_N_F_ID_MUST_MATCH_CUSTOM28),
    PANTHEIOS_BE_N_STDFORM_ENTRY(4, pantheios_be_file, PANTHEIOS_BE_N_F_ID_MUST_MATCH_CUSTOM28),
    //PANTHEIOS_BE_N_STDFORM_ENTRY(4, pantheios_be_fprintf, PANTHEIOS_BE_N_F_ID_MUST_MATCH_CUSTOM28),	// uncomment to log communication on console
    PANTHEIOS_BE_N_STDFORM_ENTRY(5, pantheios_be_file, PANTHEIOS_BE_N_F_ID_MUST_MATCH_CUSTOM28),
    PANTHEIOS_BE_N_STDFORM_ENTRY(5, pantheios_be_fprintf, PANTHEIOS_BE_N_F_ID_MUST_MATCH_CUSTOM28),
    PANTHEIOS_BE_N_STDFORM_ENTRY(6, pantheios_be_file, PANTHEIOS_BE_N_F_ID_MUST_MATCH_CUSTOM28),
    PANTHEIOS_BE_N_TERMINATOR_ENTRY
};

#endif /* FILELOGGERCONFIG_H_ */

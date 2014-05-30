/*
 * Jobs.h
 *
 *  Created on: 10.06.2011
 *      Author: root
 */

#ifndef JOBS_H_
#define JOBS_H_
#include "JobPlannerEnums.h"

class Job
{
public:
	JobClass::JobClass jobClass;
	JobType::JobType jobType;
	int priority;
	JobProgressStatus::JobProgressStatus status;
	Puck::Puck carriedGood;
	POIType::POIType targetPOIType;
	POIRequiredNext::POIRequiredNext targetRequiredNext;
	POIAccessFrom::POIAccessFrom accDir;
	POI *POIFrom;
	POI *POITo;
	Job(JobClass::JobClass jobClass_=JobClass::Deliver, JobType::JobType jobType_=JobType::S0_TO_T1, int priority_=0, JobProgressStatus::JobProgressStatus status_=JobProgressStatus::STARTING, Puck::Puck carriedGood_=Puck::S0, POIType::POIType targetPOIType_=POIType::UNKNOWN, POIRequiredNext::POIRequiredNext targetRequiredNext_=POIRequiredNext::S0, POIAccessFrom::POIAccessFrom accDir_=POIAccessFrom::FRONT):
		jobClass(jobClass_),
		jobType(jobType_),
		priority(priority_),
		status(status_),
		carriedGood(carriedGood_),
		targetPOIType(targetPOIType_),
		targetRequiredNext(targetRequiredNext_),
		accDir(accDir_),
		POIFrom(NULL),
		POITo(NULL)
	{};
};

#endif /* JOBS_H_ */

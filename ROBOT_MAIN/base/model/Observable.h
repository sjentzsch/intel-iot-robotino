/*
 * Observable.h
 *
 *  Created on: Jun 2, 2011
 *      Author: Peter
 */

#ifndef OBSERVABLE_H_
#define OBSERVABLE_H_

namespace ID{
	enum ID { ROBO1=1, ROBO2, ROBO3, SERVER };
}

class Observable
{
public:
	void sync_notified(){}
    virtual void notify(void) = 0;
    virtual void notify(ID::ID id) = 0;
};

#endif /* OBSERVABLE_H_ */

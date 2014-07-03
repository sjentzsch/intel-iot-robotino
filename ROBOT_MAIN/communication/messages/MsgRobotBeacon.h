//
// MsgRobotBeacon.h
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

#ifndef MSGROBOTBEACON_H_
#define MSGROBOTBEACON_H_

#include <string>
#include <boost/property_tree/ptree.hpp>

using namespace std;

class MsgRobotBeacon
{
public:
	MsgRobotBeacon(unsigned long time_, double x_, double y_, double phi_, string state_, bool running_, unsigned long drinks_available_);
	MsgRobotBeacon(boost::property_tree::ptree& pt);
	virtual ~MsgRobotBeacon();

	static ::std::string Message() {return "msg_robot_beacon";};

	void load(boost::property_tree::ptree& pt);
	::std::string save();
	void print();

	string message;
	unsigned long time;
	double x;
	double y;
	double phi;
	string state;
	bool running;
	unsigned long drinks_available;
};

#endif /* MSGROBOTBEACON_H_ */

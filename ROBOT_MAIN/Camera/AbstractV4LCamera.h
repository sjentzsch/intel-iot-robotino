/*
 * V4LCamera.h
 *
 *  Created on: 30.06.2011
 *      Author: root
 */

#ifndef V4LCAMERA_H_
#define V4LCAMERA_H_

/* Daylight: 0 | Artificial Light: 1 | Twilight (Magdeburg 2013): 2 */
#define USE_LIGHT_PRESET 0
// till 19.30 daylight, till 20.30 twilight, afterwards artificial light

/* No Debug Information: 0 | Debug Mode (only Image Drawing): 1 */
#define CAMERA_DEBUG_MODE 0

/* Send RGB image: 0 | Send Binary Image: 1 */
#define SEND_BINARY_IMAGE 0

#include <iostream>
#include <stdlib.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <errno.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <linux/videodev2.h>
#include <boost/thread.hpp>
//TBU
#include "config.h"
#include "utils/FileLogger.h"
#include "../Api2Com.h"

const unsigned int CAM_STD_WIDTH = 320;		// 320 in simulation and on robotino (but 640 on real pc)
const unsigned int CAM_STD_HEIGHT = 240;	// 240 in simulation and on robotino (but 480 on real pc)
const unsigned int CAM_WIDTH = 160;			// 240
const unsigned int CAM_HEIGHT = 120;		// 180

struct ImageData {
	int id;
	unsigned char imageData[CAM_WIDTH*CAM_HEIGHT*3];
	ImageData(){};
	ImageData(int id_):id(id_){};
};

using namespace std;


#include <rec/robotino/api2/Camera.h>

#define SWITCH_SLEEP_TIME 200 // in ms

namespace AbstractV4LCameraSignal{
	enum AbstractV4LCamera {RUN, PAUSE};
}

class AbstractV4LCamera : public rec::robotino::api2::Camera{

protected:

	cv::Mat newImageRGB;
	cv::Mat newImageRGBGUI;

	boost::thread *execThread;
	AbstractV4LCameraSignal::AbstractV4LCamera signal; //represents the current control signal
	boost::mutex signal_mutex; //mutex for access control
	boost::condition_variable signal_cond; //condition variable, signals changes to control signal

	int initCounter;
	boost::mutex initCounter_mutex;

	void initCamera();
	void loop();
	bool queryImage();

	bool checkSignalStatus();
	void pause();
	void run();

	//virtual
	virtual void processImage() = 0;

public:
	AbstractV4LCamera();
	virtual ~AbstractV4LCamera();
	void startThread();
	bool switchToLightDetection();
	bool switchToPuckDetection();
	bool switchToDeliveryGateDetection();
};

#endif /* V4LCAMERA_H_ */

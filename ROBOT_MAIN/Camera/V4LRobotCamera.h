#ifndef V4LROBOTCAMERA_H_
#define V4LROBOTCAMERA_H_

#define _CRT_SECURE_NO_WARNINGS
#define _USE_MATH_DEFINES

#include <iostream>
#include <cmath>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "config.h"
#include "utils/FileLogger.h"
#include "utils/Timer.h"
#include "RobotCameraEnums.h"
#include "AbstractV4LCamera.h"

using namespace cv;
using namespace std;

struct puckContainer
{
	unsigned int imgX;
	unsigned int imgY;
	float worldRelX;
	float worldRelY;
	float worldAbsX;
	float worldAbsY;
	puckContainer(){}
	puckContainer(unsigned int imgX_, unsigned int imgY_, float worldRelX_, float worldRelY_) : imgX(imgX_), imgY(imgY_), worldRelX(worldRelX_), worldRelY(worldRelY_), worldAbsX(0), worldAbsY(0){}
	puckContainer(unsigned int imgX_, unsigned int imgY_, float worldRelX_, float worldRelY_, float worldAbsX_, float worldAbsY_) : imgX(imgX_), imgY(imgY_), worldRelX(worldRelX_), worldRelY(worldRelY_), worldAbsX(worldAbsX_), worldAbsY(worldAbsY_){}
};

class V4LRobotCamera: public AbstractV4LCamera
{
private:
	bool isStreaming;

	int dtToLastImageInMs;
	int fps;
	Timer timer;

	int dtToLastImageInMs2;
	int fps2;
	Timer timer2;

	CameraPuckDetection::CameraPuckDetection puckDetection;
	CameraPuckState::CameraPuckState puckState;

	CameraLightDetection::CameraLightDetection lightDetection;
	CameraLightState::CameraLightState lightState;
	CameraLampState::CameraLampState lampState;

	Timer lightTimer;
	unsigned int lightPassCounter;
	unsigned int* lightStateBuffer;

	unsigned char *pImageRaw;
	unsigned char *pImageNew;
	unsigned char *pImage;
	unsigned char *pImageGUI;

	// mutex for communication with the SensorEventGenerator-Thread
	boost::mutex m_mutex;

	vector<puckContainer> vPucksFinal;
	bool vPucksFinalProcessed;

	float* lampPosition;

	unsigned int divLeftSide;
	unsigned int divRightSide;

	/* given the hue [0-360] return the hue stored [0-180] in 8-Bit image depth */
	int huePS2HSV(int hue_ps) {return (int)((double)hue_ps/2.0);}

	/* given the saturation [0%-100%] return the saturation stored [0-255] in 8-Bit image depth */
	int satPS2HSV(int sat_ps) {return (int)((double)sat_ps/100*255);}

	/* given the value [0%-100%] return the value stored [0-255] in 8-Bit image depth */
	int valPS2HSV(int val_ps) {return (int)((double)val_ps/100*255);}

	/* adds an rectangle to the vector if the rectangle does not overlap with other ones */
	bool addRec(CameraPuckDetection::CameraPuckDetection actualPuckDetection, vector<vector<unsigned int> >& vRecs, unsigned int xp1, unsigned int yp1, unsigned int xp2, unsigned int yp2, unsigned int value);

	/*  */
	void processImage();

	/*  */
	void updateStatusPermanently();

	/* calculates the center points of all visible pucks and stores them in vPucks */
	void calcPuckDetection();

	/*  */
	void calcLampDetection();

	/*  */
	void calcLampDetectionAtGate();

	/*  */
	void calcLampDetectionAtGateOld();

	/*  */
	void calcLightDetection();

	/* calculates the division of the CAM_WIDTH */
	void calcBlueDivisionDetection();

public:
	/*
		CATCH_BOTTOM_LEFT: x-distance between the left side of the image and the left edge of the grabber + half of the biggest rectangle
		CATCH_BOTTOM_RIGHT: x-distance between the right side of the image and the right edge of the grabber + half of the biggest rectangle
		[GRABBER_MIDPOINT: x-position of the midpoint of the grabber, automatically determined using CATCH_BOTTOM_LEFT and CATCH_BOTTOM_RIGHT]
	*/
	unsigned int CATCH_BOTTOM_LEFT;
	unsigned int CATCH_BOTTOM_RIGHT;
	unsigned int GRABBER_MIDPOINT;

	V4LRobotCamera();
	virtual ~V4LRobotCamera();


	/* returns if the camera is streaming, used for waiting until first image is received */
	bool getIsStreaming() const {return this->isStreaming;}

	/*  */
	CameraPuckDetection::CameraPuckDetection getPuckDetection();

	/*  */
	CameraPuckState::CameraPuckState getPuckState();

	/*  */
	bool getVPucksFinalProcessed();

	/*  */
	void setPuckDetection(CameraPuckDetection::CameraPuckDetection puckDetection);

	/*  */
	CameraLampState::CameraLampState getLampState();

	/*  */
	CameraLightDetection::CameraLightDetection getLightDetection() const {return this->lightDetection;}

	/*  */
	CameraLightState::CameraLightState getLightState();

	/*  */
	void setLightDetection(CameraLightDetection::CameraLightDetection lightDetectionNew);

	/* returns the coordinates and the distance of the nearest puck if at least one is visible, else return false */
	//bool getNearestPuck(unsigned int* xPos, unsigned int* yPos, unsigned int* dist) const;

	//bool getPuckRelPos(float* xWorldPos, float* yWorldPos, float* worldDist);

	vector<puckContainer> getPucksFinalCloned();

	bool getLampRelPos(float* xWorldPos, float* yWorldPos);


	/* returns divLeftSide of the Division Calculation */
	unsigned int getDivLeftSide() const {return this->divLeftSide;}

	/* returns divRightSide of the Division Calculation */
	unsigned int getDivRightSide() const {return this->divRightSide;}

	/* checks if puck in front (-> RPC) [obsolete?] */
	//bool puckInFrontRPC();
};

#endif /* V4LROBOTCAMERA_H_ */

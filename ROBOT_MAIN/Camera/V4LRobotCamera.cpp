#include "V4LRobotCamera.h"

V4LRobotCamera::V4LRobotCamera() {
#if USE_CAMERA == 1
	open_device();
#endif

	isStreaming = false;
	lampPosition = new float[6];
	puckDetection = CameraPuckDetection::OFF;
	puckState = CameraPuckState::OFF;
	vPucksFinalProcessed = true;
	lampState = CameraLampState::OFF;
	lightDetection = CameraLightDetection::OFF;
	lightState = CameraLightState::OFF;
	lightPassCounter = 0;
	lightStateBuffer = new unsigned int[CameraLightState::nCameraLightState];
	for (int i = 0; i < CameraLightState::nCameraLightState; i++)
		lightStateBuffer[i] = 0;

	CATCH_BOTTOM_LEFT = 73;
	CATCH_BOTTOM_RIGHT = CAM_WIDTH - 78;
	GRABBER_MIDPOINT = CATCH_BOTTOM_LEFT + (CATCH_BOTTOM_RIGHT - CATCH_BOTTOM_LEFT) / 2;

	//Set up timer for fps calculation
	fps = 0;
	timer.start();

	fps2 = 0;
	timer2.start();
}

V4LRobotCamera::~V4LRobotCamera() {
	/*device->stopVideo();
	 device->stopDepth();*/
	delete[] lampPosition;
	delete[] lightStateBuffer;
}

/* adds an rectangle to the vector if the rectangle does not overlap with other ones */
bool V4LRobotCamera::addRec(CameraPuckDetection::CameraPuckDetection actualPuckDetection, vector<vector<unsigned int> >& vRecs, unsigned int xp1, unsigned int yp1, unsigned int xp2, unsigned int yp2, unsigned int value)
{
	//unsigned int FIX_PERCENTAGE = 205; // = 0.82*255

	// delete-list: contains the indices of the rectangles that should be removed from vRecs
	vector<int> indRecs;

	for(unsigned int i=0; i<vRecs.size(); i++)
	{
		// check if rectangles overlap
		if((vRecs[i][0] < xp2 && vRecs[i][2] > xp1 && vRecs[i][1] < yp2 && vRecs[i][3] > yp1))
		{
			// if the new rectangle has a lower value or it is fixed, then don't add it
			/*if(vRecs[i][4] >= FIX_PERCENTAGE || vRecs[i][4] > value)
				return false;*/
			if((actualPuckDetection != CameraPuckDetection::ALL_INPUT_LEFT && actualPuckDetection != CameraPuckDetection::ALL_INPUT_RIGHT) && vRecs[i][4] > value)
				return false;

			/*if(value < 3*vRecs[i][4]/4)
				return false;*/

			// else add the current rectangle to the delete-list
			indRecs.push_back(i);
		}
	}

	// now its clear that we have found a new rectangle

	// delete all the rectangles contained in the delete-list
	for(unsigned int i=0; i<indRecs.size(); i++)
		vRecs.erase(vRecs.begin() + indRecs[i] - i);

	// add the new rectangle
	vector<unsigned int> temp(5);
	temp[0] = xp1;
	temp[1] = yp1;
	temp[2] = xp2;
	temp[3] = yp2;
	temp[4] = value;
	vRecs.push_back(temp);

	return true;
}

void V4LRobotCamera::calcPuckDetection()
{
	CameraPuckDetection::CameraPuckDetection actualPuckDetection = puckDetection;

	//cout << "Camera: started" << endl;

	const unsigned int REC_WIDTH_START = 16;		// 10	 [16,5] in reality ~22, in simulator ~26 [all values dividable by 2]
	const unsigned int REC_HEIGHT_START = 10;		// 6	 [9] in reality ~12, in simulator ~14 [all values dividable by 2]
	const unsigned int REC_WIDTH_END = 32;			// 30	 24 (27. Juni)[36] in reality ~48, in simulator ~48 [all values dividable by 2]
	const unsigned int REC_HEIGHT_END = 24;			// 22	 16 (27. Juni)[25,5] in reality ~34, in simulator ~34 [all values dividable by 2]
	const unsigned int STEP_TO_RIGHT = 1;			// [7]
	const unsigned int STEP_TO_BOTTOM = 1;			// [5]
	const unsigned int MIN_START_LINE = CAM_HEIGHT/3;
	const unsigned int MAX_START_LINE = 3*CAM_HEIGHT/4;
	const unsigned int REC_START_LINE = CAM_HEIGHT/2;

	unsigned int START_LINE, DIVISION_PUCK;
	if(actualPuckDetection == CameraPuckDetection::ALL_INPUT_LEFT || actualPuckDetection == CameraPuckDetection::ALL_INPUT_RIGHT || actualPuckDetection == CameraPuckDetection::ALL_MACHINE)
	{
		START_LINE = MIN_START_LINE;
		DIVISION_PUCK = 2;
	} else if (actualPuckDetection == CameraPuckDetection::RECYCLING_FROM_MACHINE){
		START_LINE = REC_START_LINE;
		DIVISION_PUCK = 2;
	} else {
		START_LINE = MAX_START_LINE;
		DIVISION_PUCK = 4;
	}

	Mat mHSV(CAM_HEIGHT, CAM_WIDTH, CV_8UC3);

	Mat mBin1(CAM_HEIGHT, CAM_WIDTH, CV_8UC1);
	Mat mBin2(CAM_HEIGHT, CAM_WIDTH, CV_8UC1);

	Mat mSAT(CAM_HEIGHT, CAM_WIDTH, CV_32SC1);		// Summed Area Table (-> http://en.wikipedia.org/wiki/Summed_area_table)
	Mat mBinRecs(CAM_HEIGHT, CAM_WIDTH, CV_8UC1);
	vector<vector<unsigned int> > vRecs;

	cvtColor(newImageRGB, mHSV, CV_RGB2HSV);

	Mat temp;

	inRange(mHSV, Scalar(huePS2HSV(0), satPS2HSV(70), valPS2HSV(50)), Scalar(huePS2HSV(60)+1, satPS2HSV(100)+1, valPS2HSV(100)+1), mBin1);
	inRange(mHSV, Scalar(huePS2HSV(345), satPS2HSV(70), valPS2HSV(50)), Scalar(huePS2HSV(359)+1, satPS2HSV(100)+1, valPS2HSV(100)+1), temp);

	mBin1 += temp;

	if(actualPuckDetection == CameraPuckDetection::ALL_MACHINE)
	{
		dilate(mBin1, mBin2, Mat(), Point(-1,-1), 2);	// [4] ~ expand
		erode(mBin2, mBin2, Mat(), Point(-1,-1), 1);	// [5]  ~ shrink
	}
	else
	{
		dilate(mBin1, mBin2, Mat(), Point(-1,-1), 1);	// [4] ~ expand
		erode(mBin2, mBin2, Mat(), Point(-1,-1), 1);	// [5]  ~ shrink
	}
	//mBin2 = mBin1;

	// calculate mBinRecs: 1 if color matches, 0 if not
	for(int i=0; i<mBin2.rows; i++)
	{
		for(int j=0; j<mBin2.cols; j++)
		{
			if(mBin2.at<uchar>(i,j) == 255)
				mBinRecs.at<uchar>(i,j) = 1;
			else
				mBinRecs.at<uchar>(i,j) = 0;
		}
	}

	// calculate the Summed Area Table (SAT) [does the same as integral() ?]
	mSAT = Scalar(0);
	for(int i=0; i<mBinRecs.rows; i++)
	{
		for(int j=0; j<mBinRecs.cols; j++)
		{
			mSAT.at<int>(i,j) = mBinRecs.at<uchar>(i,j);
			if(i > 0)
				mSAT.at<int>(i,j) += mSAT.at<int>(i-1,j);
			if(j > 0)
				mSAT.at<int>(i,j) += mSAT.at<int>(i,j-1);
			if(i > 0 && j > 0)
				mSAT.at<int>(i,j) -= mSAT.at<int>(i-1,j-1);
		}
	}

	vRecs.clear();
	unsigned int currRecHeight, currRecWidth;
	for(unsigned int i=START_LINE; i<CAM_HEIGHT-REC_HEIGHT_END; i+=STEP_TO_BOTTOM)
	{
		currRecHeight = (REC_HEIGHT_END-REC_HEIGHT_START) * (i-MIN_START_LINE) / ((CAM_HEIGHT-REC_HEIGHT_END)-MIN_START_LINE)  + REC_HEIGHT_START;
		currRecWidth = (REC_WIDTH_END-REC_WIDTH_START) * (i-MIN_START_LINE) / ((CAM_HEIGHT-REC_HEIGHT_END)-MIN_START_LINE)  + REC_WIDTH_START;

		// TODO: if actualPuckDetection == CameraPuckDetection::ONLY_GRABBED_PUCK: don't consider the whole space, take CATCH_BOTTOM_LEFT and CATCH_BOTTOM_RIGHT

		/*if(actualPuckDetection == CameraPuckDetection::ALL_INPUT_LEFT)
		{
			for(unsigned int j=CAM_WIDTH-currRecWidth-STEP_TO_RIGHT; j>=STEP_TO_RIGHT; j-=STEP_TO_RIGHT)
			{
				uint32_t value = mSAT.at<uint32_t>(i,j) + mSAT.at<uint32_t>(i+currRecHeight,j+currRecWidth) - mSAT.at<uint32_t>(i,j+currRecWidth) - mSAT.at<uint32_t>(i+currRecHeight,j);
				if(value >= 3*(currRecHeight*currRecWidth)/4)
					addRec(actualPuckDetection, vRecs, j, i, j+currRecWidth, i+currRecHeight, value*255/(currRecWidth*currRecHeight));
			}
		}
		else
		{*/
			for(unsigned int j=0; j<CAM_WIDTH-currRecWidth; j+=STEP_TO_RIGHT)
			{
				uint32_t value = mSAT.at<uint32_t>(i,j) + mSAT.at<uint32_t>(i+currRecHeight,j+currRecWidth) - mSAT.at<uint32_t>(i,j+currRecWidth) - mSAT.at<uint32_t>(i+currRecHeight,j);
				//For oclusion by laser scanner
				//row > 100 and column in a range around CAM_WIDTH/2
				if(i > CAM_HEIGHT- REC_HEIGHT_END - 3 && j > (CAM_WIDTH/2 - currRecWidth - 10) && j < (CAM_WIDTH/2 + 10)){
					//decrease percentage of puck color that has to fill the rectangle
					if(value >= (currRecHeight*currRecWidth) / 8){
						addRec(actualPuckDetection, vRecs, j, i, j+currRecWidth, i+currRecHeight, value*255/(currRecWidth*currRecHeight));
					}
				} else {
					if(value >= (currRecHeight*currRecWidth) / DIVISION_PUCK){
						addRec(actualPuckDetection, vRecs, j, i, j+currRecWidth, i+currRecHeight, value*255/(currRecWidth*currRecHeight));
					}
				}
			}
		//}
	}

	// draw all the pucks
	for(unsigned int i=0; i<vRecs.size(); i++)
	{
//		cout << i << ": " << recs[i][0] << " - " << recs[i][1] << " - " << recs[i][2] << " - " << recs[i][3] << " - " << recs[i][4] << endl;

		//cout << "Rectangle " << i << " mid point: " << vRecs[i][0] +  (vRecs[i][2] - vRecs[i][0])/2 << " - " << vRecs[i][1] +  (vRecs[i][3] - vRecs[i][1])/2 << endl;

		#if CAMERA_DEBUG_MODE == 1
			rectangle(newImageRGBGUI, Point(vRecs[i][0],vRecs[i][1]), Point(vRecs[i][2],vRecs[i][3]), Scalar(0,0,0), 1, 8, 0);
		#endif
	}

	// save the final results for the SensorEventGenerator
	{
		boost::mutex::scoped_lock l(m_mutex);
		vPucksFinal.clear();
		unsigned int imgXtemp, imgYtemp;
		float worldRelXtemp, worldRelYtemp;
		for(unsigned int i=0; i<vRecs.size(); i++)
		{
			imgXtemp = (vRecs[i][0]+vRecs[i][2])/2;
			imgYtemp = (vRecs[i][1]+vRecs[i][3])/2;
//			worldRelXtemp = (42622.566/(11.841623 + imgYtemp) - 304.65018) - 50; // have 50mm between the puck and the grabber
//			worldRelYtemp = 12.61303 + (19090.492 - 241.92728*imgXtemp)/imgYtemp;
			//magdeburg 2014
			//in cm
			worldRelYtemp = (3021.72650875067 - 37.3054775323144*(imgXtemp))/(23.2016241213065 + (imgYtemp));
			worldRelXtemp = 5.86002636634567 + (3771.79816005607 - (imgXtemp))/(imgYtemp) - 7 - 22.5 - 6;
			//in mm
			worldRelYtemp *= 10;
			worldRelXtemp *= 10;

			//cout << "add puck " << i << ": " << imgXtemp << ", " << imgYtemp << ", " << worldRelXtemp << ", " << worldRelYtemp << endl;

			vPucksFinal.push_back(puckContainer(imgXtemp, imgYtemp, worldRelXtemp, worldRelYtemp));
		}
		vPucksFinalProcessed = true;
	}

	//cout << "Camera: finished" << endl;

	// fill the ImageData (swap the red and blue channel) and send it
	#if SEND_BINARY_IMAGE == 1
	ID::ID id = ModelProvider::getInstance()->getID();
	unsigned char newByteValue = 0;
	for(unsigned int i=0; i<CAM_HEIGHT; i++)
	{
		for(unsigned int u=0; u<CAM_WIDTH*3; u+=3)
		{
			if(u%3 == 0)
			{
				if(mBin2.ptr<uchar>(i)[u/3] == 255)
					newByteValue = 255;
				else
					newByteValue = 0;

				DynDataProvider::getInstance()->getImageData(id)->imageData[i*CAM_WIDTH*3+(u+2)] = newByteValue;
				DynDataProvider::getInstance()->getImageData(id)->imageData[i*CAM_WIDTH*3+(u+1)] = newByteValue;
				DynDataProvider::getInstance()->getImageData(id)->imageData[i*CAM_WIDTH*3+(u)] = newByteValue;
			}
		}
	}
	#endif

	/*if(puckCount > 0)
		cout << "Puck (world) (" << puckCount << "): x: " << nearestPuck[3] << " y: " << nearestPuck[4] << " dist: " << nearestPuck[5] << " image: " << nearestPuck[0] << "-" << nearestPuck[1] << endl;
	else
		cout << "No Puck" << endl;*/

	//mImageDebug = mBin2.clone();
	//imshow("CamImageDebug", mImageDebug);
}

CameraPuckDetection::CameraPuckDetection V4LRobotCamera::getPuckDetection() {
	boost::mutex::scoped_lock l(m_mutex);
	return this->puckDetection;
}

void V4LRobotCamera::setPuckDetection(CameraPuckDetection::CameraPuckDetection puckDetection) {
	boost::mutex::scoped_lock l(m_mutex);
	this->puckDetection = puckDetection;
	this->vPucksFinalProcessed = false;
}

CameraPuckState::CameraPuckState V4LRobotCamera::getPuckState() {
	boost::mutex::scoped_lock l(m_mutex);
	return this->puckState;
}

bool V4LRobotCamera::getVPucksFinalProcessed() {
	boost::mutex::scoped_lock l(m_mutex);
	return this->vPucksFinalProcessed;
}

CameraLampState::CameraLampState V4LRobotCamera::getLampState() {
	boost::mutex::scoped_lock l(m_mutex);
	return this->lampState;
}

CameraLightState::CameraLightState V4LRobotCamera::getLightState() {
	boost::mutex::scoped_lock l(m_mutex);
	return this->lightState;
}

/*bool RobotCamera::getNearestPuck(unsigned int* xPos, unsigned int* yPos, unsigned int* dist) const
 {
 if(numPucks() <= 0)
 return false;

 unsigned int distance = numeric_limits<unsigned int>::max();
 unsigned int actualDistance = distance;
 for(unsigned int i=0; i<vPucks.size(); i++)
 {
 actualDistance = (unsigned int)sqrt((float)SQUARE((int)(GRABBER_MIDPOINT-vPucks[i][0])) + (float)SQUARE(CAM_HEIGHT-vPucks[i][1]));
 if(actualDistance < distance)
 {
 *xPos = vPucks[i][0];
 *yPos = vPucks[i][1];
 *dist = actualDistance;
 distance = actualDistance;
 }
 }

 return true;
 }*/

vector<puckContainer> V4LRobotCamera::getPucksFinalCloned()
{
	boost::mutex::scoped_lock l(m_mutex);

	vector<puckContainer> vPucksCloned;

	for(unsigned int i=0; i<vPucksFinal.size(); i++)
		vPucksCloned.push_back(puckContainer(vPucksFinal.at(i).imgX, vPucksFinal.at(i).imgY, vPucksFinal.at(i).worldRelX, vPucksFinal.at(i).worldRelY));

	return vPucksCloned;
}

/*bool V4LRobotCamera::getPuckRelPos(float* xWorldPos, float* yWorldPos, float* worldDist) {
	boost::mutex::scoped_lock l(m_mutex);

	if (this->puckCount <= 0)
		return false;

	*xWorldPos = vecNearestPuck.at(0);
	*yWorldPos = vecNearestPuck.at(1);
	*worldDist = vecNearestPuck.at(2);

	return true;
}*/

bool V4LRobotCamera::getLampRelPos(float* xWorldPos, float* yWorldPos) {
	boost::mutex::scoped_lock l(m_mutex);

	if (lightDetection != CameraLightDetection::SEARCH_GATE || lampState == CameraLampState::NO_LAMP || lampState == CameraLampState::OFF)
		return false;

	*xWorldPos = lampPosition[3];
	*yWorldPos = lampPosition[4];

	return true;
}

/*bool RobotCamera::puckInFrontRPC()
 {
 for(unsigned int i=0; i<vPucks.size(); i++)
 if(vPucks[i][0] > 50 && vPucks[i][0] < 110 && vPucks[i][1] > 80)
 return true;

 return false;
 }*/

/*
 void RobotCamera::calcGreenLamp(float distToLamp)
 {
 //float distToLamp = 585.0f;

 const unsigned int REC_WIDTH_DIST1 = 12;					// 13 [26]
 const unsigned int REC_HEIGHT_DIST1 = 11;					// 12 [12]
 const unsigned int REC_WIDTH_DIST2 = 22;					// 19 [38]
 const unsigned int REC_HEIGHT_DIST2 = 19;					// 15 [30]
 const float REC_DIST1 = 500.0f;								// standing on the outer black line
 const float REC_DIST2 = 270.0f;								// standing in front of the lamp
 const unsigned int STOP_HEIGHT_DIST1 = CAM_HEIGHT/3;
 const unsigned int STOP_HEIGHT_DIST2 = 2*CAM_HEIGHT/3;
 const unsigned int REC_STEP_TO_RIGHT = 3;					// [5]
 const unsigned int REC_STEP_TO_BOTTOM = 4;					// [6]
 const unsigned int MAX_X_POS_DEVIATION = 20;				// smaller?

 Mat mHSV(CAM_HEIGHT, CAM_WIDTH, CV_8UC3);

 Mat mBinGreen(CAM_HEIGHT, CAM_WIDTH, CV_8UC1);
 Mat mBinWhite(CAM_HEIGHT, CAM_WIDTH, CV_8UC1);

 Mat mSatGreen(CAM_HEIGHT+1, CAM_WIDTH+1, CV_32SC1);
 Mat mSatWhite(CAM_HEIGHT+1, CAM_WIDTH+1, CV_32SC1);

 // store the midpoint (x,y) and the value (= white-pixel-count concerning the Bin-Image)
 vector<unsigned int> vGreen(3), vWhite(3);
 for(int i=0; i<3; i++)
 {
 vGreen[i] = 0;
 vWhite[i] = 0;
 }

 // reset all the lamp status variables
 this->lightState = CameraLightState::UNKNOWN;
 this->xPosLamp = 0;

 // make sure distToLamp is between REC_DIST1 and REC_DIST2
 if(distToLamp < REC_DIST2)
 distToLamp = REC_DIST2;
 else if(distToLamp > REC_DIST1)
 distToLamp = REC_DIST1;

 unsigned int currRecWidth = (unsigned int)(REC_WIDTH_DIST1 + (REC_WIDTH_DIST2-REC_WIDTH_DIST1) * (distToLamp-REC_DIST1) / (REC_DIST2-REC_DIST1));
 unsigned int currRecHeight = (unsigned int)(REC_HEIGHT_DIST1 + (REC_HEIGHT_DIST2-REC_HEIGHT_DIST1) * (distToLamp-REC_DIST1) / (REC_DIST2-REC_DIST1));
 unsigned int stop_height = (unsigned int)(STOP_HEIGHT_DIST1 + (STOP_HEIGHT_DIST2-STOP_HEIGHT_DIST1) * (distToLamp-REC_DIST1) / (REC_DIST2-REC_DIST1));
 unsigned int white_threshold_green = (currRecWidth*currRecHeight*255)/4;
 unsigned int rec_threshold = (currRecWidth*currRecHeight*255)*2/5;

 //cout << currRecWidth << " \t " << currRecHeight << " \t " << light_on_threshold << " \t " << stop_height << endl;

 // convert mImage to HSV and store it in mHSV
 cvtColor(newImageRGB, mHSV, CV_RGB2HSV);


 // create mBinGreen to filter the bright green light when green lamp is turned on, afterwards use dilate and erode
 inRange(mHSV, Scalar(huePS2HSV(100), satPS2HSV(65), valPS2HSV(60)), Scalar(huePS2HSV(150)+1, satPS2HSV(100)+1, valPS2HSV(100)+1), mBinGreen);
 //erode(mBinGreen, mBinGreen, Mat(), Point(-1,-1), 1);	// ~ shrink
 dilate(mBinGreen, mBinGreen, Mat(), Point(-1,-1), 1);	// ~ expand


 // create mBinWhite ...
 inRange(mHSV, Scalar(huePS2HSV(0), satPS2HSV(0), valPS2HSV(85)), Scalar(huePS2HSV(359)+1, satPS2HSV(30)+1, valPS2HSV(100)+1), mBinWhite);	// 0-20 und 90-100
 //erode(mBinWhite, mBinWhite, Mat(), Point(-1,-1), 1);	// ~ shrink
 dilate(mBinWhite, mBinWhite, Mat(), Point(-1,-1), 2);	// ~ expand


 // calculate the integral of mBinGreen and store it in mSatGreen
 integral(mBinGreen, mSatGreen, CV_32S);

 unsigned int currRecWidthWhiteCore, currRecHeightWhiteCore;
 uint32_t valueGreen, valueWhite;
 for(unsigned int i=0; i<stop_height; i+=REC_STEP_TO_BOTTOM)
 {
 for(unsigned int j=0; j<CAM_WIDTH-currRecWidth; j+=REC_STEP_TO_RIGHT)
 {
 valueGreen = mSatGreen.at<uint32_t>(i,j) + mSatGreen.at<uint32_t>(i+currRecHeight,j+currRecWidth) - mSatGreen.at<uint32_t>(i,j+currRecWidth) - mSatGreen.at<uint32_t>(i+currRecHeight,j);
 if(valueGreen > vGreen[2])
 {
 vGreen[0] = j;
 vGreen[1] = i;
 vGreen[2] = valueGreen;
 }
 }
 }

 #if CAMERA_DEBUG_MODE == 1
 rectangle(newImageRGBGUI, Point(vGreen[0],vGreen[1]), Point(vGreen[0]+currRecWidth,vGreen[1]+currRecHeight), Scalar(31,230,21), 1, 8, 0);
 #endif

 unsigned int whiteGreen = 0;

 if(vGreen[2] >= rec_threshold)
 {
 for(unsigned int i=vGreen[1]; i<vGreen[1]+currRecHeight; i++)
 {
 for(unsigned int j=vGreen[0]; j<vGreen[0]+currRecWidth; j++)
 {
 whiteGreen += mBinWhite.at<uchar>(i,j);
 }
 }
 }

 bool isGreen = false;

 if(whiteGreen > white_threshold_green)
 isGreen = true;

 if(isGreen)
 {
 this->xPosLamp = (vGreen[0]) + currRecWidth/2;
 this->lightState = CameraLightState::GREEN;
 }

 //cout << "calLamp done: " << xPosLamp << " \t " << LampStatus::cLampStatus[lampStatus] << "\t" << distToLamp << endl;

 #if DEBUG_CAMERA
 imshow("Green", mBinGreen.clone());
 imshow("White", mBinWhite.clone());
 #endif
 }*/

/*void RobotCamera::calcGreenLamp()
 {
 const unsigned int REC_WIDTH_DIST1 = 12;					// 13 [26]
 const unsigned int REC_HEIGHT_DIST1 = 11;					// 12 [12]
 const unsigned int REC_WIDTH_DIST2 = 20;					// 19 [38]
 const unsigned int REC_HEIGHT_DIST2 = 18;					// 15 [30]
 const float REC_DIST1 = 500.0f;								// standing on the outer black line
 const float REC_DIST2 = 270.0f;								// standing in front of the lamp
 const unsigned int STOP_HEIGHT_DIST1 = CAM_HEIGHT/3;
 const unsigned int STOP_HEIGHT_DIST2 = 2*CAM_HEIGHT/3;
 const unsigned int REC_STEP_TO_RIGHT = 3;					// [5]
 const unsigned int REC_STEP_TO_BOTTOM = 4;					// [6]
 const unsigned int MAX_X_POS_DEVIATION = 20;				// smaller?

 float distToLamp = 585.0f;

 Mat mHSV(CAM_HEIGHT, CAM_WIDTH, CV_8UC3);

 Mat mBinGreen(CAM_HEIGHT, CAM_WIDTH, CV_8UC1);
 Mat mBinWhite(CAM_HEIGHT, CAM_WIDTH, CV_8UC1);

 Mat mSatGreen(CAM_HEIGHT+1, CAM_WIDTH+1, CV_32SC1);
 Mat mSatWhite(CAM_HEIGHT+1, CAM_WIDTH+1, CV_32SC1);

 // store the midpoint (x,y) and the value (= white-pixel-count concerning the Bin-Image)
 vector<unsigned int> vGreen(3), vWhite(3);
 for(int i=0; i<3; i++)
 {
 vGreen[i] = 0;
 vWhite[i] = 0;
 }

 // reset all the lamp status variables
 this->lampStatus = LampStatus::UNKNOWN;
 this->xPosLamp = 0;

 // make sure distToLamp is between REC_DIST1 and REC_DIST2
 if(distToLamp < REC_DIST2)
 distToLamp = REC_DIST2;
 else if(distToLamp > REC_DIST1)
 distToLamp = REC_DIST1;

 unsigned int currRecWidth = (unsigned int)(REC_WIDTH_DIST1 + (REC_WIDTH_DIST2-REC_WIDTH_DIST1) * (distToLamp-REC_DIST1) / (REC_DIST2-REC_DIST1));
 unsigned int currRecHeight = (unsigned int)(REC_HEIGHT_DIST1 + (REC_HEIGHT_DIST2-REC_HEIGHT_DIST1) * (distToLamp-REC_DIST1) / (REC_DIST2-REC_DIST1));
 unsigned int stop_height = (unsigned int)(STOP_HEIGHT_DIST1 + (STOP_HEIGHT_DIST2-STOP_HEIGHT_DIST1) * (distToLamp-REC_DIST1) / (REC_DIST2-REC_DIST1));
 unsigned int light_on_threshold = (currRecWidth*currRecHeight*255)*2/5;

 //cout << currRecWidth << " \t " << currRecHeight << " \t " << light_on_threshold << " \t " << stop_height << endl;

 // convert mImage to HSV and store it in mHSV
 cvtColor(newImageRGB, mHSV, CV_RGB2HSV);

 // create mBinGreen to filter the bright green light when green lamp is turned on, afterwards use dilate and erode
 #ifdef TBU_DUMMIES
 inRange(mHSV, Scalar(huePS2HSV(90), satPS2HSV(20), valPS2HSV(30)), Scalar(huePS2HSV(180)+1, satPS2HSV(100)+1, valPS2HSV(100)+1), mBinGreen);
 #else
 inRange(mHSV, Scalar(huePS2HSV(100), satPS2HSV(65), valPS2HSV(60)), Scalar(huePS2HSV(150)+1, satPS2HSV(100)+1, valPS2HSV(100)+1), mBinGreen);
 #endif
 //erode(mBinGreen, mBinGreen, Mat(), Point(-1,-1), 2);	// ~ shrink
 dilate(mBinGreen, mBinGreen, Mat(), Point(-1,-1), 1);	// ~ expand

 // create mBinWhite ...
 inRange(mHSV, Scalar(huePS2HSV(0), satPS2HSV(0), valPS2HSV(90)), Scalar(huePS2HSV(359)+1, satPS2HSV(20)+1, valPS2HSV(100)+1), mBinWhite);
 erode(mBinWhite, mBinWhite, Mat(), Point(-1,-1), 1);	// ~ shrink
 dilate(mBinWhite, mBinWhite, Mat(), Point(-1,-1), 2);	// ~ expand

 // calculate the integral of mBinGreen and store it in mSatGreen
 integral(mBinGreen, mSatGreen, CV_32S);

 uint32_t valueGreen, valueWhite;
 for(unsigned int i=0; i<stop_height; i+=REC_STEP_TO_BOTTOM)
 {
 for(unsigned int j=0; j<CAM_WIDTH-currRecWidth; j+=REC_STEP_TO_RIGHT)
 {
 valueGreen = mSatGreen.at<uint32_t>(i,j) + mSatGreen.at<uint32_t>(i+currRecHeight,j+currRecWidth) - mSatGreen.at<uint32_t>(i,j+currRecWidth) - mSatGreen.at<uint32_t>(i+currRecHeight,j);
 if(valueGreen > vGreen[2])
 {
 vGreen[0] = j;
 vGreen[1] = i;
 vGreen[2] = valueGreen;
 }
 }
 }

 for(unsigned int i=vGreen[1]-3; i<vGreen[1]+currRecHeight+3; i++)
 {
 for(unsigned int j=vGreen[0]-3; j<vGreen[0]+currRecWidth+3; j++)
 {
 if(mBinGreen.at<uchar>(i,j) == 0 && mBinWhite.at<uchar>(i,j) == 255)
 mBinGreen.at<uchar>(i,j) = 255;
 mBinWhite.at<uchar>(i,j) = 0;
 }
 }

 if(vGreen[2] >= light_on_threshold)
 {
 xPosLamp = vGreen[0] + currRecWidth/2;
 lampStatus = LampStatus::GREEN;
 #if CAMERA_DEBUG_MODE == 1
 rectangle(newImageRGBGUI, Point(vGreen[0],vGreen[1]), Point(vGreen[0]+currRecWidth,vGreen[1]+currRecHeight), Scalar(31,230,21), 2, 8, 0);
 #endif

 //cout << "green: found: " << xPosLamp << ", " << (vGreen[1] + currRecHeight/2) << endl;
 }
 else
 {
 lampStatus = LampStatus::UNKNOWN;
 xPosLamp = 0;

 //cout << "green: not found" << endl;
 }

 cout << "calLamp done: " << xPosLamp << " \t " << LampStatus::cLampStatus[lampStatus] << "\t" << distToLamp << endl;

 #if DEBUG_CAMERA
 imshow("Green", mBinGreen.clone());
 imshow("White", mBinWhite.clone());
 #endif
 }*/

/*void V4LRobotCamera::calcLampDetection()
{
	// TODO: speicher die aktuelle rechtecksgröße und nehm sie als start, wir fahren ja nie WEG von der lampe! bei "off" dann reset

	Timer lampTimer;
	lampTimer.start();

	CameraLampDetection::CameraLampDetection actualLampDetection = lampDetection;

	float startDist = 560.0f;

	// average i.e. if everything is perfect: 560.0f
	const float DIST_FAR = 560.0f;								// standing on the outer black line
	const float DIST_NEAR = 270.0f;

	const unsigned int REC_WIDTH_STEP = 2;
	const unsigned int REC_HEIGHT_STEP = 2;

	const unsigned int REC_WIDTH_BLACK_FAR = 12;		// 10 12
	const unsigned int REC_WIDTH_BLACK_NEAR = 24;		// 20 24
	const unsigned int REC_HEIGHT_BLACK_FAR = 30;		// 10 30
	const unsigned int REC_HEIGHT_BLACK_NEAR = 45;		// 15 45
//	const unsigned int STOP_HEIGHT_DIST1 = 2*CAM_HEIGHT/3;
//	const unsigned int STOP_HEIGHT_DIST2 = 5*CAM_HEIGHT/6;

	Mat mHSV(CAM_HEIGHT, CAM_WIDTH, CV_8UC3);

	Mat mBinColor(CAM_HEIGHT, CAM_WIDTH, CV_8UC1);
	Mat mBinBlack(CAM_HEIGHT, CAM_WIDTH, CV_8UC1);

	Mat mSatColor(CAM_HEIGHT+1, CAM_WIDTH+1, CV_32SC1);
	Mat mSatBlack(CAM_HEIGHT+1, CAM_WIDTH+1, CV_32SC1);

	// store the midpoint (x,y) and the value (= white-pixel-count concerning the Bin-Image)
	vector<unsigned int> vColor(6), vWhite(6), vBlack(6);
	for(int i=0; i<6; i++)
	{
		vColor[i] = 0;
		vWhite[i] = 0;
		vBlack[i] = 0;
	}

	// make sure distToLamp is between REC_DIST1 and REC_DIST2
	if(startDist < DIST_NEAR)
		startDist = DIST_NEAR;
	else if(startDist > DIST_FAR)
		startDist = DIST_FAR;

	//cout << currRecWidth << " \t " << currRecHeight << " \t " << light_on_threshold << " \t " << stop_height << endl;

	// convert mImage to HSV and store it in mHSV
	cvtColor(newImageRGB, mHSV, CV_RGB2HSV);

	// create mBinColor ...
	Mat temp1, temp2, temp3, temp4;
	inRange(mHSV, Scalar(huePS2HSV(330), satPS2HSV(80), valPS2HSV(20)), Scalar(huePS2HSV(359)+1, satPS2HSV(100)+1, valPS2HSV(100)+1), mBinColor);	// changed 16:37 05.07.2011
	inRange(mHSV, Scalar(huePS2HSV(0), satPS2HSV(80), valPS2HSV(20)), Scalar(huePS2HSV(15)+1, satPS2HSV(100)+1, valPS2HSV(100)+1), temp1);			// changed 16:37 05.07.2011
	inRange(mHSV, Scalar(huePS2HSV(25), satPS2HSV(80), valPS2HSV(27)), Scalar(huePS2HSV(50)+1, satPS2HSV(100)+1, valPS2HSV(100)+1), temp2);			// changed 16:37 05.07.2011
	inRange(mHSV, Scalar(huePS2HSV(100), satPS2HSV(80), valPS2HSV(35)), Scalar(huePS2HSV(150)+1, satPS2HSV(100)+1, valPS2HSV(100)+1), temp3);		// changed 16:37 05.07.2011
	inRange(mHSV, Scalar(huePS2HSV(0), satPS2HSV(0), valPS2HSV(80)), Scalar(huePS2HSV(359)+1, satPS2HSV(40)+1, valPS2HSV(100)+1), temp4);	// was 0-20 and 88-100 (changed 16:37 05.07.2011)
	mBinColor += temp1 + temp2 + temp3 + temp4;
	//erode(mBinColor, mBinColor, Mat(), Point(-1,-1), 1);	// ~ shrink
	dilate(mBinColor, mBinColor, Mat(), Point(-1,-1), 1);	// ~ expand

	// create mBinBlack ...
	inRange(mHSV, Scalar(huePS2HSV(0), satPS2HSV(0), valPS2HSV(0)), Scalar(huePS2HSV(359)+1, satPS2HSV(100)+1, valPS2HSV(7)+1), mBinBlack);	// was 10 (changed 16:29 05.07.2011)
	//erode(mBinBlack, mBinBlack, Mat(), Point(-1,-1), 1);	// ~ shrink
	dilate(mBinBlack, mBinBlack, Mat(), Point(-1,-1), 1);	// ~ expand


	// calculate the integral of mBinColor and store it in mSatColor
	integral(mBinColor, mSatColor, CV_32S);

	// calculate the integral of mBinBlack and store it in mSatBlack
	integral(mBinBlack, mSatBlack, CV_32S);


	unsigned int currRecWidthBlack = 0, currRecHeightBlack = 0;
	unsigned int currRecThresholdColor = 0, currRecThresholdBlack = 0;
	uint32_t currValueColor = 0, currValueBlack = 0;
	//unsigned int stop_height = (unsigned int)(STOP_HEIGHT_DIST1 + (STOP_HEIGHT_DIST2-STOP_HEIGHT_DIST1) * (distToLamp-REC_DIST1) / (REC_DIST2-REC_DIST1));

	bool lampFound = false;
	bool currDistLampFound = false;
	for(unsigned int currDist=startDist; currDist>=DIST_NEAR; currDist-=10)
	{
		currRecWidthBlack = (unsigned int)(REC_WIDTH_BLACK_FAR + (REC_WIDTH_BLACK_NEAR-REC_WIDTH_BLACK_FAR) * (currDist-DIST_FAR) / (DIST_NEAR-DIST_FAR));
		currRecHeightBlack = (unsigned int)(REC_HEIGHT_BLACK_FAR + (REC_HEIGHT_BLACK_NEAR-REC_HEIGHT_BLACK_FAR) * (currDist-DIST_FAR) / (DIST_NEAR-DIST_FAR));
		//stop_height = (unsigned int)(STOP_HEIGHT_DIST1 + (STOP_HEIGHT_DIST2-STOP_HEIGHT_DIST1) * (dist-REC_DIST1) / (REC_DIST2-REC_DIST1));
		currRecThresholdBlack = (currRecWidthBlack*currRecHeightBlack*255)*7/10;
		currDistLampFound = false;

		// TODO: stop_height einbauen!
		//for(unsigned int i=0; i<stop_height; i+=REC_STEP_TO_BOTTOM)
		// TODO: start with ... ?
		for(unsigned int i=10; i<CAM_HEIGHT-currRecHeightBlack; i+=REC_HEIGHT_STEP)
		{
			for(unsigned int j=0; j<CAM_WIDTH-currRecWidthBlack; j+=REC_WIDTH_STEP)
			{
				currValueBlack = mSatBlack.at<uint32_t>(i,j) + mSatBlack.at<uint32_t>(i+currRecHeightBlack,j+currRecWidthBlack) - mSatBlack.at<uint32_t>(i,j+currRecWidthBlack) - mSatBlack.at<uint32_t>(i+currRecHeightBlack,j);
				if(currValueBlack >= vBlack[2] && currValueBlack >= currRecThresholdBlack)// && (!lampFound || i < vBlack[1]))
				{
					currValueColor = mSatColor.at<uint32_t>(0,j) + mSatColor.at<uint32_t>(i,j+currRecWidthBlack) - mSatColor.at<uint32_t>(0,j+currRecWidthBlack) - mSatColor.at<uint32_t>(i,j);
					currRecThresholdColor = (currRecWidthBlack*i*255)*1/2;
					if(currValueColor >= currRecThresholdColor)
					{
						vBlack[0] = j;
						vBlack[1] = i;
						vBlack[2] = currValueBlack;
						vBlack[3] = currRecWidthBlack;
						vBlack[4] = currRecHeightBlack;
						vBlack[5] = currDist;
						vBlack[6] = currValueColor;
						currDistLampFound = true;
						lampFound = true;
					}
				}
			}
		}

		if(!currDistLampFound)
			break;
	}

	if(lampFound)
	{
		lampPosition[0] = vBlack[0] + vBlack[3]/2;
		lampPosition[1] = vBlack[1] + vBlack[4]/2;
		lampPosition[2] = (unsigned int)sqrt((float)SQUARE((int)(GRABBER_MIDPOINT-lampPosition[0])) + (float)SQUARE(int(CAM_HEIGHT-lampPosition[1])));
		// TODO: adjust following two lines, need more samples!
		//lampPosition[3] = (19.502268 + 13312.334/lampPosition[1]) - 185; // stand still 18,5cm in front of the lamp
		lampPosition[3] = (11.927012 + 13326.621/lampPosition[1]) - 185;
		lampPosition[4] = (13756.292 - 184.97829*lampPosition[0])/(14.574833 + lampPosition[1]);
		lampPosition[5] = sqrt(SQUARE(lampPosition[3]) + SQUARE(lampPosition[4]));

		#ifdef IMAGE_DRAW_ON_TOP
			rectangle(newImageRGBGUI, Point(vBlack[0],vBlack[1]), Point(vBlack[0]+vBlack[3],vBlack[1]+vBlack[4]), Scalar(0,0,0), 1, 8, 0);
			rectangle(newImageRGBGUI, Point(vBlack[0],0), Point(vBlack[0]+vBlack[3],vBlack[1]), Scalar(42,42,236), 1, 8, 0);
			//rectangle(newImageRGBGUI, Point(vRed[0],vRed[1]), Point(vRed[0]+vRed[3],vRed[1]+vRed[4]), Scalar(42,42,236), 1, 8, 0);
			//rectangle(newImageRGBGUI, Point(vYellow[0],vYellow[1]), Point(vYellow[0]+vYellow[3],vYellow[1]+vYellow[4]), Scalar(67,208,238), 1, 8, 0);
			//rectangle(newImageRGBGUI, Point(vGreen[0],vGreen[1]), Point(vGreen[0]+vGreen[3],vGreen[1]+vGreen[4]), Scalar(31,230,21), 1, 8, 0);
		#endif

		lampState = CameraLampState::LAMP_IN_SIGHT;
	}
	else
	{
		for(int i=0; i<6; i++)
			lampPosition[i] = 0;

		lampState = CameraLampState::NO_LAMP;
	}

	// TODO: send image here
	// fill the ImageData (swap the red and blue channel) and send it
//	ID::ID id = ModelProvider::getInstance()->getID();
//	unsigned char newByteValue = 0;
//	for(unsigned int i=0; i<CAM_HEIGHT; i++)
//	{
//		for(unsigned int u=0; u<CAM_WIDTH*3; u+=3)
//		{
//			if(u%3 == 0)
//			{
//				if(mBinColor.ptr<uchar>(i)[u/3] == 255)
//					newByteValue = 255;
//				else
//					newByteValue = 0;
//
//				DynDataProvider::getInstance()->getImageData(id)->imageData[i*CAM_WIDTH*3+(u+2)] = newByteValue;
//				DynDataProvider::getInstance()->getImageData(id)->imageData[i*CAM_WIDTH*3+(u+1)] = newByteValue;
//				DynDataProvider::getInstance()->getImageData(id)->imageData[i*CAM_WIDTH*3+(u)] = newByteValue;
//			}
//		}
//	}

	//cout << "calcLampDetection result (" << lampTimer.msecsElapsed() << " ms): \t" << lampPosition[0] << "\t" << lampPosition[1] << "\t" << lampPosition[2] << "\t" << lampPosition[3] << "\t" << lampPosition[4] << "\t" << lampPosition[5] << endl;

	lampTimer.reset();
}*/

void V4LRobotCamera::calcLampDetectionAtGate()
{
	// TODO: speicher die aktuelle rechtecksgröße und nehm sie als start, wir fahren ja nie WEG von der lampe! bei "off" dann reset

	Timer lampTimer;
	lampTimer.start();

	float startDist = 560.0f;

	// average i.e. if everything is perfect: 560.0f
	const float DIST_FAR = 560.0f;								// standing on the outer black line
	const float DIST_NEAR = 270.0f;

	const unsigned int REC_WIDTH_FAR = 8;		// 10 12
	const unsigned int REC_WIDTH_NEAR = 18;		// 20 24
	const unsigned int REC_HEIGHT_FAR = 6;		// 10 30
	const unsigned int REC_HEIGHT_NEAR = 14;	// 15 45
//	const unsigned int STOP_HEIGHT_DIST1 = 2*CAM_HEIGHT/3;
//	const unsigned int STOP_HEIGHT_DIST2 = 5*CAM_HEIGHT/6;

	Mat mHSV(CAM_HEIGHT, CAM_WIDTH, CV_8UC3);

	Mat mBinGreen(CAM_HEIGHT, CAM_WIDTH, CV_8UC1);

	Mat mSatGreen(CAM_HEIGHT+1, CAM_WIDTH+1, CV_32SC1);

	/* store the midpoint (x,y) and the value (= white-pixel-count concerning the Bin-Image) */
	vector<unsigned int> vGreen(12);
	for(int i=0; i<12; i++)
	{
		vGreen[i] = 0;
	}

	// make sure distToLamp is between REC_DIST1 and REC_DIST2
	if(startDist < DIST_NEAR)
		startDist = DIST_NEAR;
	else if(startDist > DIST_FAR)
		startDist = DIST_FAR;

	//cout << currRecWidth << " \t " << currRecHeight << " \t " << light_on_threshold << " \t " << stop_height << endl;

	/* convert mImage to HSV and store it in mHSV */
	cvtColor(newImageRGB, mHSV, CV_RGB2HSV);


//	inRange(mHSV, Scalar(huePS2HSV(100), satPS2HSV(40), valPS2HSV(45)), Scalar(huePS2HSV(150)+1, satPS2HSV(100)+1, valPS2HSV(100)+1), mBinGreen);
	//magdeburg 2014
	inRange(mHSV, Scalar(huePS2HSV(100), satPS2HSV(40), valPS2HSV(40)), Scalar(huePS2HSV(160)+1, satPS2HSV(100)+1, valPS2HSV(100)+1), mBinGreen);

	//erode(mBinWhite, mBinWhite, Mat(), Point(-1,-1), 1);	// ~ shrink
	dilate(mBinGreen, mBinGreen, Mat(), Point(-1,-1), 2);	// ~ expand


	/* calculate the integral of mBinColor and store it in mSatColor */
	integral(mBinGreen, mSatGreen, CV_32S);



	unsigned int currRecWidth;
	unsigned int currRecHeight;
	uint32_t currGreenColor = 0;
	unsigned int currRecThresholdGreen = 0;
	bool lampFound = false;
	bool currDistLampFound = false;

	for(unsigned int currDist=startDist; currDist>=DIST_NEAR; currDist-=10)
	{
		currRecWidth = (unsigned int)(REC_WIDTH_FAR + (REC_WIDTH_NEAR-REC_WIDTH_FAR) * (currDist-DIST_FAR) / (DIST_NEAR-DIST_FAR));
		currRecHeight = (unsigned int)(REC_HEIGHT_FAR + (REC_HEIGHT_NEAR-REC_HEIGHT_FAR) * (currDist-DIST_FAR) / (DIST_NEAR-DIST_FAR));
		currRecThresholdGreen = (currRecWidth*currRecHeight*255)*6/10;
		currDistLampFound = false;

		for(unsigned int j=0; j<CAM_WIDTH-currRecWidth; j++)
		{
			for(unsigned int i=0; i<CAM_HEIGHT-currRecHeight; i++)
			{
				currGreenColor = mSatGreen.at<uint32_t>(i,j) + mSatGreen.at<uint32_t>(i+currRecHeight,j+currRecWidth) - mSatGreen.at<uint32_t>(i,j+currRecWidth) - mSatGreen.at<uint32_t>(i+currRecHeight,j);
				if(currGreenColor >= vGreen[2] && currGreenColor > currRecThresholdGreen)
				{
					vGreen[0] = j;
					vGreen[1] = i;
					vGreen[2] = currGreenColor;
					vGreen[3] = currRecWidth;
					vGreen[4] = currRecHeight;
					currDistLampFound = true;
					lampFound = true;
				}
			}
		}

		if(!currDistLampFound)
			break;
	}

	if(lampFound)
	{
		lampPosition[0] = vGreen[0] + vGreen[3]/2;
		lampPosition[1] = vGreen[1] + vGreen[4]/2;
		lampPosition[2] = (unsigned int)sqrt((float)SQUARE((int)(GRABBER_MIDPOINT-lampPosition[0])) + (float)SQUARE(int(CAM_HEIGHT-lampPosition[1])));
		// TODO: adjust following two lines, need more samples!
		//lampPosition[3] = (19.502268 + 13312.334/lampPosition[1]) - 185; // stand still 18,5cm in front of the lamp
		//lampPosition[3] = (11.927012 + 13326.621/lampPosition[1]) - 300;
		//lampPosition[4] = (13756.292 - 184.97829*lampPosition[0])/(14.574833 + lampPosition[1]);


		// 18.4
		// 9
		lampPosition[3] = (22.7482854711587 + 0.0320991630447918*lampPosition[0] + 263.393737504961/lampPosition[1] + 351.194277603173/(lampPosition[0]*lampPosition[0]*lampPosition[0]) - 0.656895299872049*lampPosition[1])*10 - 40;
		lampPosition[4] = (11.6672698932764/lampPosition[0] + (891.399354921567 - 11.3231043355679*lampPosition[0])/(12.6689243000754 + lampPosition[1]))*10;
		if(lampPosition[1] > CAM_WIDTH/2)
			lampPosition[4] = -lampPosition[4];

		lampPosition[5] = sqrt(SQUARE(lampPosition[3]) + SQUARE(lampPosition[4]));

		#if CAMERA_DEBUG_MODE == 1
			rectangle(newImageRGBGUI, Point(vGreen[0],vGreen[1]), Point(vGreen[0]+vGreen[3],vGreen[1]+vGreen[4]), Scalar(0,0,0), 1, 8, 0);
		#endif

		{
			boost::mutex::scoped_lock l(m_mutex);
			lampState = CameraLampState::LAMP_IN_SIGHT;
		}
	}
	else
	{
		for(int i=0; i<6; i++)
			lampPosition[i] = 0;

		{
			boost::mutex::scoped_lock l(m_mutex);
			lampState = CameraLampState::NO_LAMP;
		}
	}

	// fill the ImageData (swap the red and blue channel) and send it
	#if SEND_BINARY_IMAGE == 1
	ID::ID id = ModelProvider::getInstance()->getID();
	unsigned char newByteValue = 0;
	for(unsigned int i=0; i<CAM_HEIGHT; i++)
	{
		for(unsigned int u=0; u<CAM_WIDTH*3; u+=3)
		{
			if(u%3 == 0)
			{
				if(mBinGreen.ptr<uchar>(i)[u/3] == 255)
					newByteValue = 255;
				else
					newByteValue = 0;

				DynDataProvider::getInstance()->getImageData(id)->imageData[i*CAM_WIDTH*3+(u+2)] = newByteValue;
				DynDataProvider::getInstance()->getImageData(id)->imageData[i*CAM_WIDTH*3+(u+1)] = newByteValue;
				DynDataProvider::getInstance()->getImageData(id)->imageData[i*CAM_WIDTH*3+(u)] = newByteValue;
			}
		}
	}
	#endif

	FileLog::log(log_Camera, "[V4LRobotCamera] calcLampDetection result (", std::to_string(lampTimer.msecsElapsed()), " ms): \t", std::to_string(lampPosition[0]), "\t", std::to_string(lampPosition[1]), "\t", std::to_string(lampPosition[2]), "\t", std::to_string(lampPosition[3]), "\t", std::to_string(lampPosition[4]), "\t", std::to_string(lampPosition[5]));

	lampTimer.reset();
}

void V4LRobotCamera::calcLampDetectionAtGateOld() {
	// TODO: speicher die aktuelle rechtecksgröße und nehm sie als start, wir fahren ja nie WEG von der lampe! bei "off" dann reset

	Timer lampTimer;
	lampTimer.start();

	float startDist = 560.0f;

	// average i.e. if everything is perfect: 560.0f
	const float DIST_FAR = 560.0f; // standing on the outer black line
	const float DIST_NEAR = 270.0f;

	const unsigned int REC_WIDTH_STEP = 2;
	const unsigned int REC_HEIGHT_STEP = 2;

	const unsigned int REC_WIDTH_BLACK_FAR = 12; // 10 12
	const unsigned int REC_WIDTH_BLACK_NEAR = 24; // 20 24
	const unsigned int REC_HEIGHT_BLACK_FAR = 30; // 10 30
	const unsigned int REC_HEIGHT_BLACK_NEAR = 45; // 15 45
	//	const unsigned int STOP_HEIGHT_DIST1 = 2*CAM_HEIGHT/3;
	//	const unsigned int STOP_HEIGHT_DIST2 = 5*CAM_HEIGHT/6;

	Mat mHSV(CAM_HEIGHT, CAM_WIDTH, CV_8UC3);

	Mat mBinColor(CAM_HEIGHT, CAM_WIDTH, CV_8UC1);
	Mat mBinWhite(CAM_HEIGHT, CAM_WIDTH, CV_8UC1);
	Mat mBinBlack(CAM_HEIGHT, CAM_WIDTH, CV_8UC1);

	Mat mSatColor(CAM_HEIGHT + 1, CAM_WIDTH + 1, CV_32SC1);
	Mat mSatWhite(CAM_HEIGHT + 1, CAM_WIDTH + 1, CV_32SC1);
	Mat mSatBlack(CAM_HEIGHT + 1, CAM_WIDTH + 1, CV_32SC1);

	/* store the midpoint (x,y) and the value (= white-pixel-count concerning the Bin-Image) */
	vector<unsigned int> vColor(12), vWhite(12), vBlack(12);
	for (int i = 0; i < 12; i++) {
		vColor[i] = 0;
		vWhite[i] = 0;
		vBlack[i] = 0;
	}

	// make sure distToLamp is between REC_DIST1 and REC_DIST2
	if (startDist < DIST_NEAR)
		startDist = DIST_NEAR;
	else if (startDist > DIST_FAR)
		startDist = DIST_FAR;

	//cout << currRecWidth << " \t " << currRecHeight << " \t " << light_on_threshold << " \t " << stop_height << endl;

	/* convert mImage to HSV and store it in mHSV */
	cvtColor(newImageRGB, mHSV, CV_RGB2HSV);

	/* create mBinColor ... */
	inRange(
			mHSV,
			Scalar(huePS2HSV(0), satPS2HSV(50), valPS2HSV(80)),
			Scalar(huePS2HSV(50) + 1, satPS2HSV(100) + 1, valPS2HSV(100) + 1),
			mBinColor); // - 70 VAL	// - 65
	//erode(mBinColor, mBinColor, Mat(), Point(-1,-1), 1);	// ~ shrink
	dilate(mBinColor, mBinColor, Mat(), Point(-1, -1), 1); // ~ expand


	/* create mBinWhite ... */
	inRange(
			mHSV,
			Scalar(huePS2HSV(0), satPS2HSV(0), valPS2HSV(90)),
			Scalar(huePS2HSV(359) + 1, satPS2HSV(20) + 1, valPS2HSV(100) + 1),
			mBinWhite); // 0-30 und 85-100
	//erode(mBinWhite, mBinWhite, Mat(), Point(-1,-1), 1);	// ~ shrink
	dilate(mBinWhite, mBinWhite, Mat(), Point(-1, -1), 1); // ~ expand	// [2 times expand ?!]


	/* create mBinBlack ... */
	inRange(
			mHSV,
			Scalar(huePS2HSV(0), satPS2HSV(0), valPS2HSV(0)),
			Scalar(huePS2HSV(359) + 1, satPS2HSV(100) + 1, valPS2HSV(30) + 1),
			mBinBlack); // up to 22
	//erode(mBinBlack, mBinBlack, Mat(), Point(-1,-1), 1);	// ~ shrink
	dilate(mBinBlack, mBinBlack, Mat(), Point(-1, -1), 1); // ~ expand


	/* calculate the integral of mBinColor and store it in mSatColor */
	integral(mBinColor, mSatColor, CV_32S);

	/* calculate the integral of mBinWhite and store it in mSatWhite */
	integral(mBinWhite, mSatWhite, CV_32S);

	/* calculate the integral of mBinBlack and store it in mSatBlack */
	integral(mBinBlack, mSatBlack, CV_32S);

	unsigned int currRecWidthBlack = 0, currRecHeightBlack = 0;
	uint32_t currValueColor = 0, currValueBlack = 0, currValueWhite = 0;
	unsigned int currRecThresholdBlack = 0;
	unsigned int currYellowX = 0, currYellowWidth = 0, currYellowY = 0,
			currYellowHeight = 0;
	//unsigned int stop_height = (unsigned int)(STOP_HEIGHT_DIST1 + (STOP_HEIGHT_DIST2-STOP_HEIGHT_DIST1) * (distToLamp-REC_DIST1) / (REC_DIST2-REC_DIST1));

	bool lampFound = false;
	bool currDistLampFound = false;
	for (unsigned int currDist = startDist; currDist >= DIST_NEAR; currDist
			-= 10) {
		currRecWidthBlack = (unsigned int) (REC_WIDTH_BLACK_FAR
				+ (REC_WIDTH_BLACK_NEAR - REC_WIDTH_BLACK_FAR) * (currDist
						- DIST_FAR) / (DIST_NEAR - DIST_FAR));
		currRecHeightBlack = (unsigned int) (REC_HEIGHT_BLACK_FAR
				+ (REC_HEIGHT_BLACK_NEAR - REC_HEIGHT_BLACK_FAR) * (currDist
						- DIST_FAR) / (DIST_NEAR - DIST_FAR));
		//stop_height = (unsigned int)(STOP_HEIGHT_DIST1 + (STOP_HEIGHT_DIST2-STOP_HEIGHT_DIST1) * (dist-REC_DIST1) / (REC_DIST2-REC_DIST1));
		currRecThresholdBlack = (currRecWidthBlack * currRecHeightBlack * 255)
				* 7 / 10;
		currDistLampFound = false;

		// TODO: stop_height einbauen!
		//for(unsigned int i=0; i<stop_height; i+=REC_STEP_TO_BOTTOM)
		// TODO: start with ... ?
		for (unsigned int i = 10; i < CAM_HEIGHT - currRecHeightBlack
				- (REC_HEIGHT_STEP - 1); i += REC_HEIGHT_STEP) {
			for (unsigned int j = 5; j < CAM_WIDTH - currRecWidthBlack - 5
					- (REC_WIDTH_STEP - 1); j += REC_WIDTH_STEP) {
				currValueBlack = mSatBlack.at<uint32_t> (i, j)
						+ mSatBlack.at<uint32_t> (i + currRecHeightBlack,
								j + currRecWidthBlack) - mSatBlack.at<uint32_t> (i,
						j + currRecWidthBlack) - mSatBlack.at<uint32_t> (
						i + currRecHeightBlack, j);
				//cout << "currValueBlack: " << currValueBlack << endl;
				if (currValueBlack > vBlack[2] && currValueBlack
						>= currRecThresholdBlack)// && (!lampFound || i < vBlack[1]))
				{
					currValueColor = 0;
					currYellowX = 0;
					currYellowWidth = 0;
					currYellowY = 0;
					currYellowHeight = 0;

					for (unsigned int currYellowLength = 5; currYellowLength
							< currRecWidthBlack + 10; currYellowLength++) {
						for (unsigned int v = 0; v < i; v++) {
							for (unsigned int w = j - 5; w < j
									+ currRecWidthBlack + 5; w++) {
								if (v + currYellowLength < CAM_HEIGHT && w
										+ currYellowLength < CAM_WIDTH) {
									uint32_t
											currTempYellowValue =
													mSatColor.at<uint32_t> (v, w)
															+ mSatColor.at<uint32_t> (
																	v
																			+ currYellowLength,
																	w
																			+ currYellowLength)
															- mSatColor.at<uint32_t> (
																	v
																			+ currYellowLength,
																	w)
															- mSatColor.at<uint32_t> (
																	v,
																	w
																			+ currYellowLength);
									if (currTempYellowValue > currValueColor
											&& currTempYellowValue
													> (currYellowLength
															* currYellowLength
															* 255) * 5 / 6) {
										currYellowX = w;
										currYellowWidth = currYellowLength;
										currYellowY = v;
										currYellowHeight = currYellowLength;
										currValueColor = currTempYellowValue;
									}
								}
							}
						}
					}

					//currRecThresholdColor = (currRecWidthBlack*i*255)*1/2;
					if ((currYellowWidth * currYellowHeight)
							>= (currRecWidthBlack * currRecWidthBlack) / 4) {
						// TODO: delete?
						rectangle(
								newImageRGBGUI,
								Point(currYellowX, currYellowY),
								Point(currYellowX + currYellowWidth,
										currYellowY + currYellowHeight),
								Scalar(42, 42, 236), 1, 8, 0);
						rectangle(
								newImageRGBGUI,
								Point((currYellowX + j) / 2 + 1,
										(currYellowY + currYellowHeight - 3)),
								Point(
										(currYellowX + j) / 2 + currYellowWidth
												- 1,
										(currYellowY + currYellowHeight
												+ currYellowHeight - 5)),
								Scalar(31, 230, 21), 1, 8, 0);

						currValueWhite = mSatWhite.at<uint32_t> (
								(currYellowY + currYellowHeight - 3),
								(currYellowX + j) / 2 + 1)
								+ mSatWhite.at<uint32_t> (
										(currYellowY + currYellowHeight
												+ currYellowHeight - 5),
										(currYellowX + j) / 2 + currYellowWidth
												- 1) - mSatWhite.at<uint32_t> (
								(currYellowY + currYellowHeight
										+ currYellowHeight - 5),
								(currYellowX + j) / 2 + 1)
								- mSatWhite.at<uint32_t> (
										(currYellowY + currYellowHeight - 3),
										(currYellowX + j) / 2 + currYellowWidth
												- 1);
						//cout << currYellowX << "\t" << currYellowWidth << "\t" << currYellowY << "\t" << currYellowHeight << "\t" << currValueColor << "\t" << currValueWhite << "\t" << (((currYellowHeight-2)*(currYellowHeight-5)*255)/3) << endl;
						// TODO: uncomment last case and see what goes wrong! it goes wrong somewhere!
						if (currValueWhite >= ((currYellowHeight - 2)
								* (currYellowHeight - 5) * 255) * 4 / 10
								&& currValueWhite > vBlack[11]) {
							vBlack[0] = j;
							vBlack[1] = i;
							vBlack[2] = currValueBlack;
							vBlack[3] = currRecWidthBlack;
							vBlack[4] = currRecHeightBlack;
							vBlack[5] = currDist;
							vBlack[6] = currYellowX;
							vBlack[7] = currYellowWidth;
							vBlack[8] = currYellowY;
							vBlack[9] = currYellowHeight;
							vBlack[10] = currValueColor;
							vBlack[11] = currValueWhite;
							currDistLampFound = true;
							lampFound = true;
						}
					}
				}
			}
		}

		if (!currDistLampFound)
			break;
	}

	if (lampFound) {
		lampPosition[0] = vBlack[0] + vBlack[3] / 2;
		lampPosition[1] = vBlack[1] + vBlack[4] / 2;
		lampPosition[2] = (unsigned int) sqrt(
				(float) SQUARE((int)(GRABBER_MIDPOINT-lampPosition[0]))
						+ (float) SQUARE(int(CAM_HEIGHT-lampPosition[1])));
		// TODO: adjust following two lines,l need more samples!
		//lampPosition[3] = (19.502268 + 13312.334/lampPosition[1]) - 185; // stand still 18,5cm in front of the lamp
		lampPosition[3] = (11.927012 + 13326.621 / lampPosition[1]) - 250;
		lampPosition[4] = (13756.292 - 184.97829 * lampPosition[0])
				/ (14.574833 + lampPosition[1]);
		lampPosition[5] = sqrt(
				SQUARE(lampPosition[3]) + SQUARE(lampPosition[4]));

#if CAMERA_DEBUG_MODE == 1
		rectangle(newImageRGBGUI, Point(vBlack[0], vBlack[1]),
				Point(vBlack[0] + vBlack[3], vBlack[1] + vBlack[4]),
				Scalar(0, 0, 0), 1, 8, 0);
		rectangle(newImageRGBGUI, Point(vBlack[6], vBlack[8]),
				Point(vBlack[6] + vBlack[7], vBlack[8] + vBlack[9]),
				Scalar(42, 42, 236), 1, 8, 0);
		//rectangle(newImageRGBGUI, Point(vRed[0],vRed[1]), Point(vRed[0]+vRed[3],vRed[1]+vRed[4]), Scalar(42,42,236), 1, 8, 0);
		//rectangle(newImageRGBGUI, Point(vYellow[0],vYellow[1]), Point(vYellow[0]+vYellow[3],vYellow[1]+vYellow[4]), Scalar(67,208,238), 1, 8, 0);
		rectangle(
				newImageRGBGUI,
				Point((vBlack[6] + vBlack[0]) / 2 + 1,
						vBlack[8] + vBlack[9] - 3),
				Point((vBlack[6] + vBlack[0]) / 2 + vBlack[9] - 1,
						vBlack[8] + vBlack[9] + vBlack[9] - 5),
				Scalar(31, 230, 21), 1, 8, 0);
#endif

		{
			boost::mutex::scoped_lock l(m_mutex);
			lampState = CameraLampState::LAMP_IN_SIGHT;
		}
	} else {
		for (int i = 0; i < 6; i++)
			lampPosition[i] = 0;

		{
			boost::mutex::scoped_lock l(m_mutex);
			lampState = CameraLampState::NO_LAMP;
		}
	}

	// fill the ImageData (swap the red and blue channel) and send it
#if SEND_BINARY_IMAGE == 1
	ID::ID id = ModelProvider::getInstance()->getID();
	unsigned char newByteValue = 0;
	for (unsigned int i = 0; i < CAM_HEIGHT; i++) {
		for (unsigned int u = 0; u < CAM_WIDTH * 3; u += 3) {
			if (u % 3 == 0) {
				if (mBinWhite.ptr<uchar> (i)[u / 3] == 255)
					newByteValue = 255;
				else
					newByteValue = 0;

				DynDataProvider::getInstance()->getImageData(id)->imageData[i
						* CAM_WIDTH * 3 + (u + 2)] = newByteValue;
				DynDataProvider::getInstance()->getImageData(id)->imageData[i
						* CAM_WIDTH * 3 + (u + 1)] = newByteValue;
				DynDataProvider::getInstance()->getImageData(id)->imageData[i
						* CAM_WIDTH * 3 + (u)] = newByteValue;
			}
		}
	}
#endif

	//cout << "calcLampDetection result (" << lampTimer.msecsElapsed() << " ms): \t" << lampPosition[0] << "\t" << lampPosition[1] << "\t" << lampPosition[2] << "\t" << lampPosition[3] << "\t" << lampPosition[4] << "\t" << lampPosition[5] << endl;

	lampTimer.reset();
}

void V4LRobotCamera::setLightDetection(CameraLightDetection::CameraLightDetection lightDetectionNew)
{
	if(this->lightDetection == CameraLightDetection::OFF && lightDetectionNew != CameraLightDetection::OFF)
	{
		if(lightDetectionNew == CameraLightDetection::SEARCH_GATE)
			switchToDeliveryGateDetection();
		else
			switchToLightDetection();
		lightPassCounter = 0;
		for (int i = 0; i < CameraLightState::nCameraLightState; i++)
			lightStateBuffer[i] = 0;
		lightTimer.start();
	}
	else if(this->lightDetection != CameraLightDetection::OFF && lightDetectionNew == CameraLightDetection::OFF)
	{
		switchToPuckDetection();
		lightTimer.reset();
	}

	this->lightDetection = lightDetectionNew;
}

void V4LRobotCamera::calcLightDetection() {
	CameraLightDetection::CameraLightDetection actualLightDetection = lightDetection;

	const unsigned int REC_WIDTH = 21;
	const unsigned int REC_HEIGHT = 21;
	const unsigned int REC_STEP_TO_RIGHT = 1;
	const unsigned int REC_STEP_TO_BOTTOM = 1;
	const unsigned int START_HEIGHT = 90;
	unsigned int DIVISION_VERT_LEFT;
	unsigned int DIVISION_VERT_RIGHT;
	if(actualLightDetection == CameraLightDetection::NEAR)
	{
		DIVISION_VERT_LEFT = 50;
		DIVISION_VERT_RIGHT = 110;
	}
	else
	{
		DIVISION_VERT_LEFT = 20;
		DIVISION_VERT_RIGHT = 140;
	}

	Mat mHSV(CAM_HEIGHT, CAM_WIDTH, CV_8UC3);

	//red
	Mat mBinRed(CAM_HEIGHT, CAM_WIDTH, CV_8UC1);
	Mat mBinRed2(CAM_HEIGHT, CAM_WIDTH, CV_8UC1);
	Mat mBinWhiteRed(CAM_HEIGHT, CAM_WIDTH, CV_8UC1);
	//yellow
	Mat mBinYellow(CAM_HEIGHT, CAM_WIDTH, CV_8UC1);
	Mat mBinWhiteYellow(CAM_HEIGHT, CAM_WIDTH, CV_8UC1);
	//green
	Mat mBinGreen(CAM_HEIGHT, CAM_WIDTH, CV_8UC1);
	Mat mBinWhiteGreen(CAM_HEIGHT, CAM_WIDTH, CV_8UC1);

	//Summed area table (SAT)
	Mat mSatRed(CAM_HEIGHT + 1, CAM_WIDTH + 1, CV_32SC1);
	Mat mSatYellow(CAM_HEIGHT + 1, CAM_WIDTH + 1, CV_32SC1);
	Mat mSatGreen(CAM_HEIGHT + 1, CAM_WIDTH + 1, CV_32SC1);
	Mat mSatWhiteRed(CAM_HEIGHT + 1, CAM_WIDTH + 1, CV_32SC1);
	Mat mSatWhiteYellow(CAM_HEIGHT + 1, CAM_WIDTH + 1, CV_32SC1);
	Mat mSatWhiteGreen(CAM_HEIGHT + 1, CAM_WIDTH + 1, CV_32SC1);


	// store the midpoint (x,y) and the value (= white-pixel-count concerning the Bin-Image)
	vector<unsigned int> vRed(5), vYellow(5), vGreen(5);
	for (int i = 0; i < 5; i++) {
		vRed[i] = 0;
		vYellow[i] = 0;
		vGreen[i] = 0;
	}

	// convert mImage to HSV and store it in mHSV
	cvtColor(newImageRGB, mHSV, CV_RGB2HSV);

#if USE_LIGHT_PRESET == 1
	inRange(mHSV, Scalar(huePS2HSV(0), satPS2HSV(50), valPS2HSV(90)), Scalar(huePS2HSV(14)+1, satPS2HSV(100)+1, valPS2HSV(100)+1), mBinRed);
	inRange(mHSV, Scalar(huePS2HSV(310), satPS2HSV(50), valPS2HSV(90)), Scalar(huePS2HSV(359)+1, satPS2HSV(100)+1, valPS2HSV(100)+1), mBinRed2);
	mBinRed += mBinRed2;
	inRange(mHSV, Scalar(huePS2HSV(20), satPS2HSV(60), valPS2HSV(66)), Scalar(huePS2HSV(55)+1, satPS2HSV(100)+1, valPS2HSV(100)+1), mBinYellow);
	inRange(mHSV, Scalar(huePS2HSV(100), satPS2HSV(35), valPS2HSV(45)), Scalar(huePS2HSV(170)+1, satPS2HSV(100)+1, valPS2HSV(100)+1), mBinGreen);
	inRange(mHSV, Scalar(huePS2HSV(0), satPS2HSV(0), valPS2HSV(0)), Scalar(huePS2HSV(359)+1, satPS2HSV(40)+1, valPS2HSV(75)+1), mBinWhiteYellow);
	inRange(mHSV, Scalar(huePS2HSV(0), satPS2HSV(0), valPS2HSV(0)), Scalar(huePS2HSV(359)+1, satPS2HSV(20)+1, valPS2HSV(75)+1), mBinWhiteRed);
	// Saturday Night Values
	/*Mat temp;
	inRange(
			mHSV,
			Scalar(huePS2HSV(330), satPS2HSV(50), valPS2HSV(85)),
			Scalar(huePS2HSV(359) + 1, satPS2HSV(100) + 1, valPS2HSV(100) + 1),
			mBinRed);
	inRange(
			mHSV,
			Scalar(huePS2HSV(0), satPS2HSV(50), valPS2HSV(85)),
			Scalar(huePS2HSV(17) + 1, satPS2HSV(100) + 1, valPS2HSV(100) + 1),
			temp);
	mBinRed += temp;
	inRange(
			mHSV,
			//Scalar(huePS2HSV(25), satPS2HSV(50), valPS2HSV(75)), //june 12th 2012: change from 60->50 sat and 28-> 25 hue for dark yellow led
			Scalar(huePS2HSV(25), satPS2HSV(70), valPS2HSV(80)),
			Scalar(huePS2HSV(40) + 1, satPS2HSV(100) + 1, valPS2HSV(100) + 1),
			mBinYellow);
	inRange(
			mHSV,
			Scalar(huePS2HSV(100), satPS2HSV(45), valPS2HSV(60)),
			Scalar(huePS2HSV(150) + 1, satPS2HSV(100) + 1, valPS2HSV(100) + 1),
			mBinGreen);*/
#else

	//Magdeburg 2014
	//color rectangles
	inRange(mHSV, Scalar(huePS2HSV(330), satPS2HSV(40), valPS2HSV(80)), Scalar(huePS2HSV(359)+1, satPS2HSV(100)+1, valPS2HSV(100)+1), mBinRed);
	inRange(mHSV, Scalar(huePS2HSV(0), satPS2HSV(40), valPS2HSV(80)), Scalar(huePS2HSV(5)+1, satPS2HSV(100)+1, valPS2HSV(100)+1), mBinRed2);
	mBinRed += mBinRed2;
	inRange(mHSV, Scalar(huePS2HSV(10), satPS2HSV(45), valPS2HSV(45)), Scalar(huePS2HSV(60)+1, satPS2HSV(100)+1, valPS2HSV(100)+1), mBinYellow);
	inRange(mHSV, Scalar(huePS2HSV(130), satPS2HSV(30), valPS2HSV(30)), Scalar(huePS2HSV(180)+1, satPS2HSV(100)+1, valPS2HSV(100)+1), mBinGreen);
	//white cores
	inRange(mHSV, Scalar(huePS2HSV(0), satPS2HSV(0), valPS2HSV(85)), Scalar(huePS2HSV(359)+1, satPS2HSV(15)+1, valPS2HSV(100)+1), mBinWhiteRed);
	inRange(mHSV, Scalar(huePS2HSV(0), satPS2HSV(0), valPS2HSV(85)), Scalar(huePS2HSV(359)+1, satPS2HSV(30)+1, valPS2HSV(100)+1), mBinWhiteYellow);
	inRange(mHSV, Scalar(huePS2HSV(0), satPS2HSV(0), valPS2HSV(85)), Scalar(huePS2HSV(359)+1, satPS2HSV(30)+1, valPS2HSV(100)+1), mBinWhiteGreen);
#endif

	//erode(mBinRed, mBinRed, Mat(), Point(-1,-1), 1);	// ~ shrink
	dilate(mBinRed, mBinRed, Mat(), Point(-1, -1), 1); // ~ expand
	dilate(mBinYellow, mBinYellow, Mat(), Point(-1, -1), 1); // ~ expand
	dilate(mBinGreen, mBinGreen, Mat(), Point(-1, -1), 1); // ~ expand

	// calculate the integral of mBinWhite and store it in mSatWhite
	integral(mBinRed, mSatRed, CV_32S);
	integral(mBinYellow, mSatYellow, CV_32S);
	integral(mBinGreen, mSatGreen, CV_32S);
	integral(mBinWhiteRed, mSatWhiteRed, CV_32S);
	integral(mBinWhiteYellow, mSatWhiteYellow, CV_32S);
	integral(mBinWhiteGreen, mSatWhiteGreen, CV_32S);

	unsigned int currRecWidth = (unsigned int) REC_WIDTH;
	unsigned int currRecHeight = (unsigned int) REC_HEIGHT;

	unsigned int thresholdRed = 0, thresholdYellow = 0, thresholdGreen = 0;

	for (unsigned int j = DIVISION_VERT_LEFT; j < DIVISION_VERT_RIGHT
			- currRecWidth; j += REC_STEP_TO_RIGHT) {
		for (unsigned int i = 0; i < START_HEIGHT - currRecHeight; i
				+= REC_STEP_TO_BOTTOM) {
			thresholdRed = mSatRed.at<uint32_t> (i, j) + mSatRed.at<uint32_t> (
					i + currRecHeight, j + currRecWidth) - mSatRed.at<uint32_t> (i,
					j + currRecWidth) - mSatRed.at<uint32_t> (i + currRecHeight, j);
			if (thresholdRed > vRed[2]) {
				vRed[0] = j;
				vRed[1] = i;
				vRed[2] = thresholdRed;
				vRed[3] = currRecWidth;
				vRed[4] = currRecHeight;
			}

			thresholdYellow = mSatYellow.at<uint32_t> (i, j)
					+ mSatYellow.at<uint32_t> (i + currRecHeight, j + currRecWidth)
					- mSatYellow.at<uint32_t> (i, j + currRecWidth)
					- mSatYellow.at<uint32_t> (i + currRecHeight, j);
			if (thresholdYellow > vYellow[2]) {
				vYellow[0] = j;
				vYellow[1] = i;
				vYellow[2] = thresholdYellow;
				vYellow[3] = currRecWidth;
				vYellow[4] = currRecHeight;
			}

			thresholdGreen = mSatGreen.at<uint32_t> (i, j) + mSatGreen.at<uint32_t> (
					i + currRecHeight, j + currRecWidth) - mSatGreen.at<uint32_t> (
					i, j + currRecWidth) - mSatGreen.at<uint32_t> (
					i + currRecHeight, j);
			if (thresholdGreen > vGreen[2]) {
				vGreen[0] = j;
				vGreen[1] = i;
				vGreen[2] = thresholdGreen;
				vGreen[3] = currRecWidth;
				vGreen[4] = currRecHeight;
			}
		}
	}

	bool isRed = false;
	bool isYellow = false;
	bool isGreen = false;

	unsigned int isOn_threshold_red, isOn_threshold_yellow, isOn_threshold_green;

#if USE_LIGHT_PRESET == 1
	if(actualLightDetection == CameraLightDetection::FAR)
		isOn_threshold_red = 35000;
	else
		isOn_threshold_red = 40000;
	isOn_threshold_yellow = 30000;
	isOn_threshold_green = 40000;
#else
	if(actualLightDetection == CameraLightDetection::FAR)
		isOn_threshold_red = 30000;		// real: at 60000
	else
		isOn_threshold_red = 40000;		// real: at 80000
	isOn_threshold_yellow = 50000;		// real: at 90000
	isOn_threshold_green = 30000;		// real: at 50000
#endif

	if (vRed[2] > isOn_threshold_red)
		isRed = true;
	if (vYellow[2] > isOn_threshold_yellow)
		isYellow = true;
	if (vGreen[2] > isOn_threshold_green)
		isGreen = true;

	unsigned int whiteYellow = 0;
	unsigned int whiteRed = 0;
	unsigned int whiteGreen = 0;
	if(actualLightDetection == CameraLightDetection::NEAR)
	{

		if(isRed)
		{
			whiteRed = mSatWhiteRed.at<uint32_t> (vRed[1]+2, vRed[0]+2) + mSatWhiteRed.at<uint32_t> (vRed[1]+vRed[4]-2, vRed[0]+vRed[3]-2) - mSatWhiteRed.at<uint32_t> (vRed[1]+2, vRed[0]+vRed[3]-2) - mSatWhiteRed.at<uint32_t> (vRed[1]+vRed[4]-2, vRed[0]+2);
			if(whiteRed < 5000)	// real at 18000
				isRed = false;
		}

		if(isYellow)
		{
			whiteYellow = mSatWhiteYellow.at<uint32_t> (vYellow[1]+2, vYellow[0]+2) + mSatWhiteYellow.at<uint32_t> (vYellow[1]+vYellow[4]-2, vYellow[0]+vYellow[3]-2) - mSatWhiteYellow.at<uint32_t> (vYellow[1]+2, vYellow[0]+vYellow[3]-2) - mSatWhiteYellow.at<uint32_t> (vYellow[1]+vYellow[4]-2, vYellow[0]+2);
			if(whiteYellow < 3000)	// real at 10000
				isYellow = false;
		}

		if(isGreen)
		{
			whiteGreen = mSatWhiteGreen.at<uint32_t> (vGreen[1]+2, vGreen[0]+2) + mSatWhiteGreen.at<uint32_t> (vGreen[1]+vGreen[4]-2, vGreen[0]+vGreen[3]-2) - mSatWhiteGreen.at<uint32_t> (vGreen[1]+2, vGreen[0]+vGreen[3]-2) - mSatWhiteGreen.at<uint32_t> (vGreen[1]+vGreen[4]-2, vGreen[0]+2);
			if(vGreen[2] < 2000) // reat at 6000
				isGreen = false;
		}
	}

#if CAMERA_DEBUG_MODE == 1
	FileLog::log_NOTICE("[V4LRobotCamera] red: ", std::to_string(vRed[2]), "\t yellow: ", std::to_string(vYellow[2]), "\t green: ", std::to_string(vGreen[2]), "\t white (in red): ", std::to_string(whiteRed), "\t white (in yellow): ", std::to_string(whiteYellow), "\t white (in green): ", std::to_string(whiteGreen));
#endif

	FileLog::log(log_Camera, "[V4LRobotCamera] red: ", std::to_string(vRed[2]), "\t yellow: ", std::to_string(vYellow[2]), "\t green: ", std::to_string(vGreen[2]), "\t white (in red): ", std::to_string(whiteRed), "\t white (in yellow): ", std::to_string(whiteYellow), "\t white (in green): ", std::to_string(whiteGreen));

#if CAMERA_DEBUG_MODE == 1
	line(newImageRGBGUI, Point(0, START_HEIGHT),
			Point(CAM_WIDTH, START_HEIGHT), Scalar(255, 255, 255), 1, 8, 0);
	line(newImageRGBGUI, Point(DIVISION_VERT_LEFT, 0),
			Point(DIVISION_VERT_LEFT, CAM_HEIGHT), Scalar(255, 255, 255), 1, 8,
			0);
	line(newImageRGBGUI, Point(DIVISION_VERT_RIGHT, 0),
			Point(DIVISION_VERT_RIGHT, CAM_HEIGHT), Scalar(255, 255, 255), 1,
			8, 0);
	if (isRed)
		rectangle(newImageRGBGUI, Point(vRed[0], vRed[1]),
				Point(vRed[0] + vRed[3], vRed[1] + vRed[4]),
				Scalar(236, 42, 42), 1, 8, 0);
	if (isYellow)
		rectangle(newImageRGBGUI, Point(vYellow[0], vYellow[1]),
				Point(vYellow[0] + vYellow[3], vYellow[1] + vYellow[4]),
				Scalar(238, 67, 208), 1, 8, 0);
	if (isGreen)
		rectangle(newImageRGBGUI, Point(vGreen[0], vGreen[1]),
				Point(vGreen[0] + vGreen[3], vGreen[1] + vGreen[4]),
				Scalar(21, 31, 230), 1, 8, 0);
#endif

	if(isRed)
	{
		if(isYellow)
		{
			if(isGreen)
				lightStateBuffer[CameraLightState::RED_YELLOW_GREEN]++;
			else
				lightStateBuffer[CameraLightState::RED_YELLOW]++;
		}
		else
		{
			if(isGreen)
				lightStateBuffer[CameraLightState::RED_GREEN]++;
			else
				lightStateBuffer[CameraLightState::RED]++;
		}
	}
	else
	{
		if(isYellow)
		{
			if(isGreen)
				lightStateBuffer[CameraLightState::YELLOW_GREEN]++;
			else
				lightStateBuffer[CameraLightState::YELLOW]++;
		}
		else
		{
			if(isGreen)
				lightStateBuffer[CameraLightState::GREEN]++;
			else
				lightStateBuffer[CameraLightState::OFFLINE]++;
		}
	}

	lightPassCounter++;

//	//DEBUG
//	cout << "calcLightDetection result: " << lightPassCounter;
//	for(int i=0; i<CameraLightState::nCameraLightState; i++)
//		cout << "\t " << lightStateBuffer[i];
//	cout << endl;


	// fill the ImageData (swap the red and blue channel) and send it
#if SEND_BINARY_IMAGE == 1
	ID::ID id = ModelProvider::getInstance()->getID();
	unsigned char newByteValue = 0;
	for (unsigned int i = 0; i < CAM_HEIGHT; i++) {
		for (unsigned int u = 0; u < CAM_WIDTH * 3; u += 3) {
			if (u % 3 == 0) {
				if (mBinYellow.ptr<uchar> (i)[u / 3] == 255) // choose here which binary image to transfer
					newByteValue = 255;
				else
					newByteValue = 0;

				DynDataProvider::getInstance()->getImageData(id)->imageData[i
						* CAM_WIDTH * 3 + (u + 2)] = newByteValue;
				DynDataProvider::getInstance()->getImageData(id)->imageData[i
						* CAM_WIDTH * 3 + (u + 1)] = newByteValue;
				DynDataProvider::getInstance()->getImageData(id)->imageData[i
						* CAM_WIDTH * 3 + (u)] = newByteValue;
			}
		}
	}
#endif

//	//DEBUG
//	cout << "lightPassCounter: " << lightPassCounter << endl;
//	cout << "lightTimer: " << lightTimer.msecsElapsed() << endl;

	// only try to update when lightState was polled at least 8 times and 750 ms went by
	if (lightPassCounter >= 12 && lightTimer.msecsElapsed() > 750) {
		// store in newLightState the light state with the highest appearance count
		CameraLightState::CameraLightState newLightState =
				CameraLightState::OFF;
		for (int i = 0; i < CameraLightState::nCameraLightState; i++)
			if (lightStateBuffer[i] > lightStateBuffer[newLightState])
				newLightState = (CameraLightState::CameraLightState) i;

		// check for yellow flash and take it; else: only if best guess appeared at least in 75% of the cases, then take it!
		if ((newLightState == CameraLightState::OFFLINE
				&& lightStateBuffer[CameraLightState::YELLOW] >= 1
						* lightPassCounter / 4) || (newLightState
				== CameraLightState::YELLOW
				&& lightStateBuffer[CameraLightState::OFFLINE] >= 1
						* lightPassCounter / 4))
			newLightState = CameraLightState::YELLOW_FLASH;
		else if (lightStateBuffer[newLightState] < 3 * lightPassCounter / 4) {
			lightTimer.reset();
			lightPassCounter = 0;
			for (int i = 0; i < CameraLightState::nCameraLightState; i++)
				lightStateBuffer[i] = 0;
			lightTimer.start();
			return;
		}

		FileLog::log(log_Camera, "[V4LRobotCamera] Update lightState: lightPassCounter = ", std::to_string(lightPassCounter), " and lightTimer = ", std::to_string(lightTimer.msecsElapsed()), " and lightState = ", CameraLightState::cCameraLightState[newLightState]);
		FileLog::log_NOTICE("[V4LRobotCamera] Update lightState: lightPassCounter = ", std::to_string(lightPassCounter), " and lightTimer = ", std::to_string(lightTimer.msecsElapsed()), " and lightState = ", CameraLightState::cCameraLightState[newLightState]);

		// reset (i.e. reinitialize) the light detection process
		{
			boost::mutex::scoped_lock l(m_mutex);
			lightState = newLightState;
		}
		lightTimer.reset();
		lightPassCounter = 0;
		for (int i = 0; i < CameraLightState::nCameraLightState; i++)
			lightStateBuffer[i] = 0;
		lightTimer.start();
	}
}

void V4LRobotCamera::calcBlueDivisionDetection() {
	// important constant for this function
	unsigned int divWidth = 34;

	Mat mHSV(CAM_HEIGHT, CAM_WIDTH, CV_8UC3);
	Mat mBinBlue(CAM_HEIGHT, CAM_WIDTH, CV_8UC1);
	Mat mSatBlue(CAM_HEIGHT + 1, CAM_WIDTH + 1, CV_32SC1);

	/* reset all intern variables */
	this->divLeftSide = 0;
	this->divRightSide = 0;

	//cout << currRecWidth << " \t " << currRecHeight << " \t " << light_on_threshold << " \t " << stop_height << endl;

	/* convert mImage to HSV and store it in mHSV */
	cvtColor(newImageRGB, mHSV, CV_RGB2HSV);

	//erode(mBinBlue, mBinBlue, Mat(), Point(-1,-1), 2);

#ifdef TBU_SIM
	inRange(mHSV, Scalar(huePS2HSV(230), satPS2HSV(80), valPS2HSV(70)), Scalar(huePS2HSV(250)+1, satPS2HSV(100)+1, valPS2HSV(100)+1), mBinBlue);
#else
	inRange(mHSV, Scalar(huePS2HSV(210), satPS2HSV(50), valPS2HSV(30)),
			Scalar(huePS2HSV(230) + 1, satPS2HSV(80) + 1, valPS2HSV(60) + 1),
			mBinBlue);
#endif

	/* calculate the integral of mBinBlue and store it in mSatBlue */
	integral(mBinBlue, mSatBlue, CV_32S);

	this->divLeftSide
			= (unsigned int) (mSatBlue.at<uint32_t> (CAM_HEIGHT, divWidth)) / 255;
	this->divRightSide = (unsigned int) (mSatBlue.at<uint32_t> (CAM_HEIGHT,
			CAM_WIDTH) - mSatBlue.at<uint32_t> (CAM_HEIGHT, divWidth)) / 255;

	cout << "divLeftSide: " << divLeftSide << " \t divRightSide: "
			<< divRightSide << endl;

	//mImageDebug = mBinBlue.clone();
	//imshow("CamImageDebug", mImageDebug);
}

/*
 * Whenever a new image is available this method is being called and the current image resides in newImageRGB
 * Remark:
 * - 5ms sleep and image sending is already handled by AbstactV4LCamera base class
 */
void V4LRobotCamera::processImage() {
	if (puckDetection == CameraPuckDetection::OFF) {
		boost::mutex::scoped_lock l(m_mutex);
		puckState = CameraPuckState::OFF;
		vPucksFinal.clear();
		vPucksFinalProcessed = true;
	} else
		calcPuckDetection();

	if (lightDetection != CameraLightDetection::SEARCH_GATE) {
		boost::mutex::scoped_lock l(m_mutex);
		lampState = CameraLampState::OFF;
		for (int i = 0; i < 6; i++)
			lampPosition[i] = 0;
	} else
		calcLampDetectionAtGate();

	if (lightDetection == CameraLightDetection::OFF || lightDetection == CameraLightDetection::SEARCH_GATE) {
		boost::mutex::scoped_lock l(m_mutex);
		lightState = CameraLightState::OFF;
	} else
		calcLightDetection();

	//imageCounter=(imageCounter+1)%10;
	//imshow("mTest", mTest);
	//waitKey(5);

	//Make FPS calculation
	dtToLastImageInMs2 = (int) timer2.msecsElapsed();
	fps2 = 1000 / dtToLastImageInMs2;
	timer2.start();
	//cout << "FPS updateStatusPermanently: " << fps2 << "\t\t" << "FPS imageReceivedEvent: " << fps << endl;
	//FileLog::log_NOTICE("FPS updateStatusPermanently: ", FileLog::integer(fps2), "; FPS imageReceivedEvent-noNewNeeded: ", FileLog::integer(fps));

	//FileLog::log_NOTICE("[RobotCamera] updateStatusPermanently() ended");
}

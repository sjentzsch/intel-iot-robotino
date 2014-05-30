/*
 * V4LCamera.cpp
 *
 *  Created on: 30.06.2011
 *      Author: root
 */

#include "AbstractV4LCamera.h"

#define IMG_MEM_SIZE	5000000 //5MB

using namespace rec::robotino::api2;

AbstractV4LCamera::AbstractV4LCamera() {
	newImageRGB.create(CAM_HEIGHT, CAM_WIDTH, CV_8UC3);
	newImageRGBGUI.create(CAM_HEIGHT, CAM_WIDTH, CV_8UC3);
	initCamera();
	//TODO: still needed?
	boost::this_thread::sleep(boost::posix_time::milliseconds(SWITCH_SLEEP_TIME)); // give the drive some time to init
	switchToPuckDetection();
}

AbstractV4LCamera::~AbstractV4LCamera() {

}

//put further initStuff here
void AbstractV4LCamera::initCamera() {
	//tested with 320x240
	setFormat( CAM_WIDTH, CAM_HEIGHT, "mjpg" );
}

void AbstractV4LCamera::loop()
{
	while(true)
	{
		checkSignalStatus();

		if(queryImage()){
			initCounter_mutex.lock();
			if(++initCounter > 1)
			{
				initCounter_mutex.unlock();
				processImage();
			}
			else
			{
				initCounter_mutex.unlock();
				FileLog::log_NOTICE("[V4LCamera] Init phase, waiting ...");
			}

			#if SEND_BINARY_IMAGE == 0
			sendImage();	// comment if you want to send other images, e.g. binary ones
			#endif
		}
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
}

void AbstractV4LCamera::sendImage()
{
	// TODO: optimize me!!! --> memcpy;
	unsigned char *pnewImageRGBGUI;
	ID::ID id = ModelProvider::getInstance()->getID();
	for(unsigned int i=0; i<CAM_HEIGHT; i++)
	{
		pnewImageRGBGUI = newImageRGBGUI.ptr<uchar>(i);
		for(unsigned int u=0; u<CAM_WIDTH; ++u) // zwei Pixel pro schleife
		{
			DynDataProvider::getInstance()->getImageData(id)->imageData[(i*CAM_WIDTH*3)+(3*u)] = pnewImageRGBGUI[3*u]; // R
			DynDataProvider::getInstance()->getImageData(id)->imageData[(i*CAM_WIDTH*3)+(3*u+1)] = pnewImageRGBGUI[3*u+1]; // G
			DynDataProvider::getInstance()->getImageData(id)->imageData[(i*CAM_WIDTH*3)+(3*u+2)] = pnewImageRGBGUI[3*u+2]; // B
		}
	}
}

bool AbstractV4LCamera::queryImage()
{

//	if( isLocalConnection() )
//	{
////		std::cout << "[V4LCamera] Local connection" << std::endl;
//		//tested with 320x240
////		setFormat( CAM_WIDTH, CAM_HEIGHT, "mjpg" );
//	}

	unsigned int sizeOfNewImage = 0;
	bool imageAvailable = isNewImageAvailable(&sizeOfNewImage);
//	cout << "[V4LCamera] Size of image is: " << sizeOfNewImage << endl;

	if(imageAvailable){

		unsigned char* data = new unsigned char[IMG_MEM_SIZE];
		unsigned char** dataPointer = &data;
		unsigned int dataSize = IMG_MEM_SIZE;
		unsigned int width;
		unsigned int height;
		unsigned int step;

		bool imageLoadedFull = getImage(dataPointer, dataSize, &width, &height, &step);

		if(imageLoadedFull == false){
			FileLog::log_NOTICE("[V4LCamera] Error! Image could not be loaded!");
//			exit(-1);
		}

		cv::Mat cameraImg;
		cameraImg.create(height, width, CV_8UC3);

//		cout << "[V4LCamera] Current image size: " << width << ", " << height << endl;
//		cout << "[V4LCamera] Data size of image: " << dataSize << endl;

		for(uint row = 0; row < height; row++){
			for(uint col = 0; col < width; col++){
				cv::Vec3b currenPixel;
				//convert rgb to bgr
				//blue
				currenPixel.val[0] = data[ row * width * 3 + col * 3 + 0];
				//green
				currenPixel.val[1] = data[ row * width * 3 + col * 3 + 1];
				//red
				currenPixel.val[2] = data[ row * width * 3 + col * 3 + 2];
				cameraImg.at<cv::Vec3b> (row, col) = currenPixel;
			}
		}

		newImageRGB = cameraImg.clone();
		newImageRGBGUI = cameraImg.clone();

		delete[] data;

		return true;

	} else {
		//this case will happen as the loop is triggered more often then a new image is available
//		cout << "[V4LCamera] No image available" << endl;
		return false;
	}

}

bool AbstractV4LCamera::switchToLightDetection()
{
	if(BaseParameterProvider::getInstance()->getParams()->simulation_mode)
		return true;

	bool result = true;
	pause();

	initCounter_mutex.lock();
	initCounter = 0;
	initCounter_mutex.unlock();

	FileLog::log_NOTICE("[V4LCamera] Switching to LightDetectionMode!");
	for(int i = 0; i < 10; i++){
		setAutoFocusEnabled(false);
		setAutoExposureEnabled(false);
		setAutoWhiteBalanceEnabled(false);
		setExposure(83);
		#if USE_LIGHT_PRESET == 0
			setWhiteBalanceTemperature(4204);
		#elif USE_LIGHT_PRESET == 1
			setWhiteBalanceTemperature(0);
		#else
			setWhiteBalanceTemperature(4204);
		#endif
		setBacklightCompensation(0);
		setBrightness(128);
		setContrast(0);
		setSaturation(118);
		setGain(10);
		rec::robotino::api2::msleep( 300 );
	}

	if(result)
		FileLog::log_NOTICE("[V4LCamera] Switching to LightDetectionMode --> SUCCESS!");
	else
		FileLog::log_NOTICE("[V4LCamera] Switching to LightDetectionMode --> FAIL!");

	run();
	return result;
}

bool AbstractV4LCamera::switchToPuckDetection()
{
	if(BaseParameterProvider::getInstance()->getParams()->simulation_mode)
		return true;

	bool result = true;
	pause();

	for(int i = 0; i < 100; i++){
		setBrightness(127);
		setContrast(255);
		setSaturation(255);//255
		setGain(200); //10
		setWhiteBalanceTemperature(4200);
		setSharpness(0);
		setBacklightCompensation(0);
		setExposure(83); //83
		setFocus(0);

		setAutoFocusEnabled(false);
		setAutoWhiteBalanceEnabled(false);
		setAutoExposureEnabled(false);
		rec::robotino::api2::msleep( 50 );
	}

	boost::this_thread::sleep(boost::posix_time::milliseconds(SWITCH_SLEEP_TIME)); // give the drive some time to accomplish the changes
	if(result)
		FileLog::log_NOTICE("[V4LCamera] Switching to PuckDetectionMode --> SUCCESS!");
	else
		FileLog::log_NOTICE("[V4LCamera] Switching to PuckDetectionMode --> FAIL!");

	run();
	return result;
}

bool AbstractV4LCamera::switchToDeliveryGateDetection()
{
	if(BaseParameterProvider::getInstance()->getParams()->simulation_mode)
		return true;

	bool result = true;
	pause();

	for(int i = 0; i < 100; i++){
		setBrightness(100);
		setContrast(255);
		setSaturation(255);//255
		setGain(0); //10
		setWhiteBalanceTemperature(6500);
		setSharpness(0);
		setBacklightCompensation(0);
		setExposure(10); //83
		setFocus(250);
		setAutoFocusEnabled(false);
		setAutoWhiteBalanceEnabled(false);
		setAutoExposureEnabled(false);
		rec::robotino::api2::msleep( 50 );
	}

	boost::this_thread::sleep(boost::posix_time::milliseconds(SWITCH_SLEEP_TIME)); // give the drive some time to accomplish the changes
	if(result)
		FileLog::log_NOTICE("[V4LCamera] Switching to DeliveryGateDetectionMode --> SUCCESS!");
	else
		FileLog::log_NOTICE("[V4LCamera] Switching to DeliveryGateDetectionMode --> FAIL!");

	run();
	return result;
}

void AbstractV4LCamera::startThread()
{
	execThread = new boost::thread(&AbstractV4LCamera::loop, this);
}


bool AbstractV4LCamera::checkSignalStatus()
{
	bool wasPaused=false;
	boost::unique_lock<boost::mutex> lock(signal_mutex);
	while(signal == AbstractV4LCameraSignal::PAUSE) //wait if signal is PAUSE
	{
		wasPaused=true;
		//cout << "MotorController received signal PAUSED." << endl;
		FileLog::log_NOTICE("[V4LCamera] PAUSED");
		signal_cond.wait(lock); //waits for the notify, handles mutex locking/unlocking
	}

	if(wasPaused)
	{
		FileLog::log_NOTICE("[V4LCamera] RUN");
	}
	return true;
}


void AbstractV4LCamera::pause()
{
	{
		boost::lock_guard<boost::mutex> lock(signal_mutex);
		FileLog::log_NOTICE("V4LCamera is signaled PAUSED.");
		signal=AbstractV4LCameraSignal::PAUSE;
	}
}


void AbstractV4LCamera::run()
{
	{
		boost::lock_guard<boost::mutex> lock(signal_mutex);
		FileLog::log_NOTICE("V4LCamera is signaled RUN.");
		signal=AbstractV4LCameraSignal::RUN;
	}
	signal_cond.notify_all();
}

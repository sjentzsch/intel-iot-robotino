/*
 * ImageData.h
 *
 *  Created on: Jun 7, 2011
 *      Author: root
 */

#ifndef IMAGEDATA_H_
#define IMAGEDATA_H_

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

#endif /* IMAGEDATA_H_ */

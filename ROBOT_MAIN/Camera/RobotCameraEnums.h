/*
 * RobotCameraEnums.h
 *
 *  Created on: 30.06.2011
 *      Author: root
 */

#ifndef ROBOTCAMERAENUMS_H_
#define ROBOTCAMERAENUMS_H_

namespace CameraPuckDetection
{
	enum CameraPuckDetection { OFF, ALL_INPUT_LEFT, ALL_INPUT_RIGHT, ALL_MACHINE, NEARBY, SEARCH_CATCH_STRATEGY, RECYCLING_FROM_MACHINE };
	static const int nCameraPuckDetection = 7;
	static const char * cCameraPuckDetection[nCameraPuckDetection] = {"OFF", "ALL_INPUT_LEFT", "ALL_INPUT_RIGHT", "ALL_MACHINE", "NEARBY", "SEARCH_CATCH_STRATEGY", "RECYCLING_FROM_MACHINE"};
}

namespace CameraPuckState
{
	enum CameraPuckState { OFF, NO_PUCK, PUCK_IN_SIGHT, CATCH_PUCK_LEFT, CATCH_PUCK_STRAIGHT, CATCH_PUCK_RIGHT };
	static const int nCameraPuckState = 6;
	static const char * cCameraPuckState[nCameraPuckState] = {"OFF", "NO_PUCK", "PUCK_IN_SIGHT", "CATCH_PUCK_LEFT", "CATCH_PUCK_STRAIGHT", "CATCH_PUCK_RIGHT"};
}

namespace CameraLightDetection
{
	enum CameraLightDetection { OFF, NEAR, FAR, SEARCH_GATE };
	static const int nCameraLightDetection = 4;
	static const char * cCameraLightDetection[nCameraLightDetection] = {"OFF", "NEAR", "FAR", "SEARCH_GATE"};
}

namespace CameraLampState
{
	enum CameraLampState { OFF, NO_LAMP, LAMP_IN_SIGHT };
	static const int nCameraLampState = 3;
	static const char * cCameraLampState[nCameraLampState] = {"OFF", "NO_LAMP", "LAMP_IN_SIGHT"};
}

namespace CameraLightState
{
	enum CameraLightState { OFF=0, UNKNOWN, OFFLINE, RED, RED_FLASH, YELLOW, YELLOW_FLASH, GREEN, RED_YELLOW, RED_GREEN, YELLOW_GREEN, RED_YELLOW_GREEN };
	static const int nCameraLightState = 12;
	static const char * cCameraLightState[nCameraLightState] = {"OFF", "UNKNOWN", "OFFLINE", "RED", "RED_FLASH", "YELLOW", "YELLOW_FLASH", "GREEN", "RED_YELLOW", "RED_GREEN", "YELLOW_GREEN", "RED_YELLOW_GREEN"};
}

/*namespace CameraBlueDivisionDetection
{
	enum CameraBlueDivisionDetection { OFF, ON };
	static const int nCameraBlueDivisionDetection = 2;
	static const char * cCameraBlueDivisionDetection[nCameraBlueDivisionDetection] = {"OFF", "ON"};
}

namespace CameraBlueDivisionState
{
	enum CameraBlueDivisionState { OFF, ADVISE_LEFT, ADVISE_STOP, ADVISE_RIGHT };
	static const int nCameraBlueDivisionState = 4;
	static const char * cCameraBlueDivisionState[nCameraBlueDivisionState] = {"OFF", "ADVISE_LEFT", "ADVISE_STOP", "ADVISE_RIGHT"};
}*/

#endif /* ROBOTCAMERAENUMS_H_ */

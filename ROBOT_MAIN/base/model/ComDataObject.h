/*
 * ComDataObject.h
 *
 *  Created on: Apr 25, 2011
 *      Author: peter
 */

#ifndef COMDATAOBJECT_H_
#define COMDATAOBJECT_H_
#include <string.h>

namespace COMMAND
{
	enum COMMAND {START=0, PAUSE};
}

typedef unsigned char IP_ADDRESS[4];

struct ComDataObject{

		COMMAND::COMMAND command;
		IP_ADDRESS addresses[3];
		int client_ports[3]; //ports for WM-Server, ComDataObjectServer,DynamicDataServer-ports will be calculated with +100 respectively
		int simulation_ports[3];
		bool enabled[3];
		bool image_enabled[3];
		bool laserscanner_enabled[3];
		bool stateinfo_enabled[3];
		IP_ADDRESS server_address;
		int server_port;
		bool server_enabled;

		ComDataObject(){
			command = COMMAND::START;

			/*IP_ADDRESS robo1 = {172,26,104,1};
			IP_ADDRESS robo2 = {172,26,104,2};
			IP_ADDRESS robo3 = {172,26,104,3};
			IP_ADDRESS wmserver = {172,26,108,33};*/

			IP_ADDRESS robo1 = {172,36,40,1};
			IP_ADDRESS robo2 = {172,36,40,2};
			IP_ADDRESS robo3 = {172,36,40,3};
			IP_ADDRESS wmserver = {172,36,40,42};

			enabled[0] = false;
			enabled[1] = false;
			enabled[2] = false;

			image_enabled[0] = false;
			image_enabled[1] = false;
			image_enabled[2] = false;

			laserscanner_enabled[0] = false;
			laserscanner_enabled[1] = false;
			laserscanner_enabled[2] = false;

			stateinfo_enabled[0] = true;
			stateinfo_enabled[1] = true;
			stateinfo_enabled[2] = true;

			server_enabled = true;

			client_ports[0] = 8190;
			client_ports[1] = 8191;
			client_ports[2] = 8192;

			simulation_ports[0] = 7100;
			simulation_ports[1] = 7101;
			simulation_ports[2] = 7102;

			server_port = 8193;

			memcpy(addresses[0],robo1,4);
			memcpy(addresses[1],robo2,4);
			memcpy(addresses[2],robo3,4);
			memcpy(server_address,wmserver,4);

		}

	};

#endif /* COMDATAOBJECT_H_ */

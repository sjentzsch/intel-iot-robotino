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
		IP_ADDRESS robot_address;
		int client_ports;
		int simulation_ports;
		bool image_enabled;
		bool laserscanner_enabled;
		bool stateinfo_enabled;
		IP_ADDRESS server_address;
		int server_port;
		bool server_enabled;

		ComDataObject(){
			command = COMMAND::START;

			IP_ADDRESS robot = {172,36,40,1};
			IP_ADDRESS server = {172,36,40,42};

			memcpy(robot_address,robot,4);
			memcpy(server_address,server,4);

			image_enabled = false;

			laserscanner_enabled = false;

			stateinfo_enabled = true;

			server_enabled = true;

			client_ports = 8190;

			simulation_ports = 7100;

			server_port = 8193;
		}

	};

#endif /* COMDATAOBJECT_H_ */

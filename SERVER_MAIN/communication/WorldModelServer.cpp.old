/*
 * WorldModelServer.cpp
 *
 *  Created on: Apr 24, 2011
 *      Author: peter
 */

#include "WorldModelServer.h"
#include "../BaseParameterProvider.h"

WorldModelServer::WorldModelServer(int port) {
	this->port = port;

	if(BaseParameterProvider::getInstance()->getParams()->simulation_mode){
		this->milliseconds = 5000;
	}
	else{
		this->milliseconds = 20000;
	}
}

WorldModelServer::~WorldModelServer(){
	delete exec_deadlineThread;
}

uint64_t WorldModelServer::getAbsMsTimePlus(uint64_t addMsTime)
{
	struct timeval tv;
	gettimeofday(&tv,NULL);
	return (uint64_t)tv.tv_sec*(uint64_t)1000+(uint64_t)tv.tv_usec/(uint64_t)1000+addMsTime;
}

void WorldModelServer::deadlineThread()
{
	while(true)
	{
		{
			boost::unique_lock<boost::mutex> lock(mutexTime);

			//FileLog::log_NOTICE("[WorldModelClient] deadlineThread: ", std::to_string(expireTime), " ", std::to_string(getAbsMsTimePlus(0)));

			if(expireTime <= getAbsMsTimePlus(0))
			{
				FileLog::log(log_Communication, "[WorldModelServer] deadlineThread: Timer expired! ", std::to_string(expireTime), " ", std::to_string(getAbsMsTimePlus(0)));
				//FileLog::log_NOTICE("[WorldModelServer] deadlineThread: Timer expired!", std::to_string(expireTime), " ", std::to_string(getAbsMsTimePlus(0)));

				// TODO: close the current socket here -> need access to the current socket!

				//FileLog::log_NOTICE("[WorldModelServer]2 Set expire time to ", std::to_string(ULLONG_MAX));
				expireTime = ULLONG_MAX;
			}
		}

		boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	}
}

void WorldModelServer::handleWorldModelClients()
{
	exec_start = new boost::thread(&WorldModelServer::handleWorldModelClients_impl,this);

	{
		boost::unique_lock<boost::mutex> lock(mutexTime);
		FileLog::log(log_Communication, "[WorldModelServer]1 Set expire time to ", std::to_string(ULLONG_MAX));
		//FileLog::log_NOTICE("[WorldModelServer]1 Set expire time to ", std::to_string(ULLONG_MAX));
		expireTime = ULLONG_MAX;
	}
	exec_deadlineThread = new boost::thread(&WorldModelServer::deadlineThread, this);
}

void WorldModelServer::handleWorldModelClients_impl()
{
	boost::asio::io_service io_service;
	tcp::acceptor acceptor(io_service, tcp::endpoint(tcp::v4(), port));
	// SO_REUSEADDR: Specifies that the rules used in validating addresses supplied to bind() should allow reuse of local addresses, if this is supported by the protocol. This option takes an int value. This is a Boolean option.
	// acceptor.set_option(boost::asio::ip::tcp::acceptor::reuse_address(true));
	acceptor.listen(3);
	while(true)
	{
		tcp::socket socket(io_service);

		try
		{
			acceptor.accept(socket);

			ID::ID clientID;
			for(unsigned int i=0; i<4; i++)
			{
				IP_ADDRESS addr;
				if(i<3)
					memcpy(addr, ModelProvider::getInstance()->getComDataObject()->robot_address, 4);
				else
					memcpy(addr, ModelProvider::getInstance()->getComDataObject()->server_address, 4);
				std::stringstream* str = new std::stringstream("");
				for(int u=0;u<3;u++){
					*str<<(int)addr[u];
					*str<<".";
				}
				*str<<(int)addr[3];
				std::string tmpAddress(str->str().c_str());
				delete str;

				if(strcmp(socket.remote_endpoint().address().to_string().c_str(), tmpAddress.c_str()) == 0)
				{
					clientID = (ID::ID)(i+1);
					break;
				}
			}

			{
				boost::unique_lock<boost::mutex> lock(mutexTime);
				//FileLog::log_NOTICE("[WorldModelServer]3 Set expire time to ", std::to_string(getAbsMsTimePlus(milliseconds)));
				expireTime = getAbsMsTimePlus(milliseconds);
			}

			std::string logCommStr = "ip: " + socket.remote_endpoint().address().to_string() + ", robot: " + std::to_string((int)clientID) + ", port: " + std::to_string(socket.remote_endpoint().port());
			FileLog::log(log_Communication, "[WorldModelServer] Connection established: (", logCommStr, ")");
			//FileLog::log_NOTICE("[WorldModelServer] Connection established: (", logCommStr, ")");

			std::array<int, 1> recv_buf_version;
			int version;
			{
				boost::lock_guard<boost::mutex> lock(ModelProvider::getInstance()->wmMutex);
				version = ModelProvider::getInstance()->getWorldModel()->version;
			}

			boost::system::error_code ignored_error;
			//send version
			boost::asio::write(socket, boost::asio::buffer(&version,sizeof(version)), boost::asio::transfer_all(), ignored_error);
			//FileLog::log_NOTICE("[WorldModelServer] Send version number ", FileLog::integer(version), " to client");
			FileLog::log(log_Communication, "[WorldModelServer] Send version number ", FileLog::integer(version), " to client");

			//receive clients version
			boost::asio::read(socket,boost::asio::buffer(recv_buf_version));
			//FileLog::log_NOTICE("[WorldModelServer] Received version number ", FileLog::integer(recv_buf_version.at(0)), " from client");
			FileLog::log(log_Communication, "[WorldModelServer] Received version number ", FileLog::integer(recv_buf_version.at(0)), " from client");

			//compare version and if necessary send world-model to client
			if(version > recv_buf_version.at(0)){
				//FileLog::log_NOTICE("Own version higher than that of the client => Send world model to client!");
				FileLog::log(log_Communication, "Own version higher than that of the client => Send world model to client!");
				{
					boost::lock_guard<boost::mutex> lock(ModelProvider::getInstance()->wmMutex);
					char* byte_array = static_cast<char*>(static_cast<void*>(ModelProvider::getInstance()->getWorldModel()));
					boost::asio::write(socket, boost::asio::buffer(byte_array,sizeof(WorldModel)), boost::asio::transfer_all(), ignored_error);
				}
			}

			// TODO: the following does not work here, as we don't know how long we should wait for robots to make changes to the world model!
			{
				boost::unique_lock<boost::mutex> lock(mutexTime);
				//FileLog::log_NOTICE("[WorldModelServer]5 Set expire time to ", std::to_string(getAbsMsTimePlus(milliseconds)));
				expireTime = getAbsMsTimePlus(milliseconds);
			}

			// here the clients make changes to the world model

			// receive the world-model from the client after it was manipulated
			std::array<char, sizeof(WorldModel)> recv_buf;
			boost::asio::read(socket,boost::asio::buffer(recv_buf));

			FileLog::log(log_Communication, "[WorldModelServer] Reading WorldModel from client ... start");
			{
				boost::lock_guard<boost::mutex> lock(ModelProvider::getInstance()->wmMutex);
				memcpy(ModelProvider::getInstance()->getWorldModel(),recv_buf.data(),sizeof(WorldModel));
			}
			FileLog::log(log_Communication, "[WorldModelServer] Reading WorldModel from client ... done");

			{
				boost::unique_lock<boost::mutex> lock(mutexTime);
				//FileLog::log_NOTICE("[WorldModelServer]6 Set expire time to ", std::to_string(ULLONG_MAX));
				expireTime = ULLONG_MAX;
			}

			// notify the WorldModelTab that a new worldModel should be painted
			if(ModelProvider::getInstance()->getWorldModelChangeListener() != NULL){
				ModelProvider::getInstance()->getWorldModelChangeListener()->notify();
			}


			//FileLog::log_NOTICE("[WorldModelServer] Start closing socket ...");
			FileLog::log(log_Communication, "[WorldModelServer] Start closing socket ...");
			boost::system::error_code ec;
			if(socket.is_open())
			{
				socket.shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
				if(ec)
				{
					FileLog::log_NOTICE("[WorldModelServer] Socket Shutdown Error: ", ec.message());
					FileLog::log(log_Communication, "[WorldModelServer] Socket Shutdown Error: ", ec.message());
				}

				socket.close(ec);
				if(ec)
				{
					FileLog::log_NOTICE("[WorldModelServer] Socket Close Error: ", ec.message());
					FileLog::log(log_Communication, "[WorldModelServer] Socket Close Error: ", ec.message());
				}
			}
			//FileLog::log_NOTICE("[WorldModelServer] End closing socket.");
			FileLog::log(log_Communication, "[WorldModelServer] End closing socket.");
		}
		catch (std::exception& e)
		{
			FileLog::log_NOTICE("[WorldModelServer] Error: ", e.what());
			FileLog::log(log_Communication, "[WorldModelServer] Error: ", e.what());

			FileLog::log_NOTICE("[WorldModelServer] Start closing socket ...");
			FileLog::log(log_Communication, "[WorldModelServer] Start closing socket ...");
			boost::system::error_code ec;
			if(socket.is_open())
			{
				socket.shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
				if(ec)
				{
					FileLog::log_NOTICE("[WorldModelServer] Socket Shutdown Error: ", ec.message());
					FileLog::log(log_Communication, "[WorldModelServer] Socket Shutdown Error: ", ec.message());
				}

				socket.close(ec);
				if(ec)
				{
					FileLog::log_NOTICE("[WorldModelServer] Socket Close Error: ", ec.message());
					FileLog::log(log_Communication, "[WorldModelServer] Socket Close Error: ", ec.message());
				}
			}
			FileLog::log_NOTICE("[WorldModelServer] End closing socket.");
			FileLog::log(log_Communication, "[WorldModelServer] End closing socket.");
		}
	}
};

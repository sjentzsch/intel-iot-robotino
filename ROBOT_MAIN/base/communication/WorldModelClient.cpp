/*
 * WorldModelClient.cpp
 *
 *  Created on: Apr 24, 2011
 *      Author: peter
 */

#include "WorldModelClient.h"


using boost::asio::ip::tcp;
using boost::asio::deadline_timer;

WorldModelClient::WorldModelClient(ID::ID id,int milliseconds):clientHandle(io_service) {
	comDataObject = ModelProvider::getInstance()->getComDataObject();

	this->milliseconds = milliseconds;

	robotWasKicked = false;

	{
		boost::lock_guard<boost::mutex> lock(lockedMutex);
		locked = false;
	}

	{
		boost::lock_guard<boost::mutex> lock(mutexDeadlineThread);
		killDeadlineThread = false;
	}

	index = int(id)-1;
	IP_ADDRESS ownIPaddr = {127,0,0,1};

	if(id==ID::SERVER){
		port = comDataObject->server_port;
		enabled = comDataObject->server_enabled;
		if(id == ModelProvider::getInstance()->getHWID())
			memcpy(addr,ownIPaddr,4);
		else
			memcpy(addr,comDataObject->server_address,4);
	}else{
		port = comDataObject->client_ports[index];
		enabled = comDataObject->enabled[index];
		if(id == ModelProvider::getInstance()->getHWID())
			memcpy(addr,ownIPaddr,4);
		else
			memcpy(addr,comDataObject->addresses[index],4);
	}



	std::stringstream* str = new std::stringstream("");

	for(int i=0;i<3;i++){
		*str<<(int)addr[i];
		*str<<".";
	}

	*str<<(int)addr[3];
	std::string tmpAddress(str->str().c_str());
	FileLog::log(log_Communication, "[WorldModelClient] Client " , std::to_string(id), " connecting to IP ", tmpAddress);

	{
		boost::unique_lock<boost::mutex> lock(mutexTime);
		//FileLog::log_NOTICE("[WorldModelClient]8 Set expire time to ", std::to_string(ULLONG_MAX));
		expireTime = ULLONG_MAX;
	}
	exec_deadlineThread = new boost::thread(&WorldModelClient::deadlineThread, this);

	FileLog::log(log_Communication, "[WorldModelClient] Constructor called");
}

WorldModelClient::~WorldModelClient(){
	FileLog::log(log_Communication, "[WorldModelClient] Destructor called start");

	{
		boost::lock_guard<boost::mutex> lock(mutexDeadlineThread);
		killDeadlineThread = true;
	}
	exec_deadlineThread->join();

	delete exec_deadlineThread;
}

uint64_t WorldModelClient::getAbsMsTimePlus(uint64_t addMsTime)
{
	struct timeval tv;
	gettimeofday(&tv,NULL);
	return (uint64_t)tv.tv_sec*(uint64_t)1000+(uint64_t)tv.tv_usec/(uint64_t)1000+addMsTime;
}

void WorldModelClient::deadlineThread()
{
	mutexDeadlineThread.lock();
	while(!killDeadlineThread)
	{
		mutexDeadlineThread.unlock();
		{
			boost::unique_lock<boost::mutex> lock(mutexTime);
			//FileLog::log_NOTICE("[WorldModelClient] deadlineThread: ", std::to_string(expireTime), " ", std::to_string(getAbsMsTimePlus(0)));
			if(expireTime <= getAbsMsTimePlus(0))
			{
				FileLog::log(log_Communication, "[WorldModelClient] deadlineThread: Timer expired for client ", std::to_string(index+1), "! Close clientHandle!", std::to_string(expireTime), " ", std::to_string(getAbsMsTimePlus(0)));
				boost::system::error_code ec;
				if(clientHandle.is_open())
				{
					clientHandle.shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
					if(ec)
					{
						FileLog::log(log_Communication, "[WorldModelClient] Socket Shutdown Error: ", ec.message());
						FileLog::log_NOTICE("[WorldModelClient] Socket Shutdown Error: ", ec.message());
					}

					clientHandle.close(ec);
					if(ec)
					{
						FileLog::log(log_Communication, "[WorldModelClient] Socket Close Error: ", ec.message());
						FileLog::log_NOTICE("[WorldModelClient] Socket Close Error: ", ec.message());
					}
				}
				//FileLog::log_NOTICE("[WorldModelClient]1 Set expire time to ", std::to_string(ULLONG_MAX));
				expireTime = ULLONG_MAX;
			}
		}

		boost::this_thread::sleep(boost::posix_time::milliseconds(100));

		mutexDeadlineThread.lock();
	}
	mutexDeadlineThread.unlock();
}

void WorldModelClient::handleConnect(const boost::system::error_code& ec){

	{
		boost::unique_lock<boost::mutex> lock(mutexTime);
		//FileLog::log_NOTICE("[WorldModelClient]2 Set expire time to ", std::to_string(ULLONG_MAX));
		expireTime = ULLONG_MAX;
	}

	if(!clientHandle.is_open())
	{
		FileLog::log(log_Communication, "[WorldModelClient] Connect timed out");
		FileLog::log_NOTICE("[WorldModelClient] Connect timed out");
		if(index<3 && ModelProvider::getInstance()->getHWID() != (ID::ID)(index+1)){
			ModelProvider::getInstance()->kickRobot((ID::ID)(index+1));
			robotWasKicked = true;
		}
	}
	else if(ec)
	{
		FileLog::log(log_Communication, "[WorldModelClient] Connect error: ", ec.message(), " ", FileLog::integer(ec.value()), " ", ec.category().name());
		FileLog::log_NOTICE("[WorldModelClient] Connect error: ", ec.message(), " ", FileLog::integer(ec.value()), " ", ec.category().name());
		if(index<3 && ModelProvider::getInstance()->getHWID() != (ID::ID)(index+1)) {
			ModelProvider::getInstance()->kickRobot((ID::ID)(index+1));
			robotWasKicked = true;
		}
		boost::system::error_code ec;
		clientHandle.shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
		if(ec)
		{
			FileLog::log(log_Communication, "[WorldModelClient] Socket Shutdown Error: ", ec.message());
			FileLog::log_NOTICE("[WorldModelClient] Socket Shutdown Error: ", ec.message());
		}
		clientHandle.close(ec);
		if(ec)
		{
			FileLog::log(log_Communication, "[WorldModelClient] Socket Close Error: ", ec.message());
			FileLog::log_NOTICE("[WorldModelClient] Socket Close Error: ", ec.message());
		}
	}
	else
	{
		try
		{

			std::string logCommStr = "ip: " + clientHandle.remote_endpoint().address().to_string() + ", robot: " + std::to_string(index+1) + ", port: " + std::to_string(clientHandle.remote_endpoint().port());
			FileLog::log(log_Communication, "[WorldModelClient] Connection established: (", logCommStr, ")");
			//FileLog::log_NOTICE("[WorldModelClient] Connection established: (", logCommStr, ")");

			{
				boost::unique_lock<boost::mutex> lock(mutexTime);
				//FileLog::log_NOTICE("[WorldModelClient]3 Set expire time to ", std::to_string(getAbsMsTimePlus(milliseconds)));
				expireTime = getAbsMsTimePlus(milliseconds);
			}

			std::array<int, 1> recv_buf_version;
			int version;
			{
				boost::lock_guard<boost::mutex> lock(ModelProvider::getInstance()->wmMutex);
				FileLog::log(log_Communication, "[WorldModelClient] Reading current WorldModel Version number (start)");
				version = ModelProvider::getInstance()->getWorldModel()->version;
				FileLog::log(log_Communication, "[WorldModelClient] Reading current WorldModel Version number (end), it is ", FileLog::integer(version));
			}
			boost::system::error_code ignored_error;

			//receive servers version
			boost::asio::read(clientHandle, boost::asio::buffer(recv_buf_version));
			FileLog::log(log_Communication, "[WorldModelClient] The server version of world model (client: ", std::to_string(index+1), ") is " , FileLog::integer(recv_buf_version.at(0)));

			//send own version
			boost::asio::write(clientHandle, boost::asio::buffer(&version,sizeof(version)), boost::asio::transfer_all(), ignored_error);
			FileLog::log(log_Communication, "[WorldModelClient] The sent version of the own world model is " , FileLog::integer(version));

			//compare version and if necessary receive world-model from server
			if(recv_buf_version.at(0) > version)
			{
				FileLog::log(log_Communication, "[WorldModelClient] World model version of server ", std::to_string(index+1),  " higher => read world model from server!");
				std::array<char, sizeof(WorldModel)> recv_buf;
				boost::asio::read(clientHandle, boost::asio::buffer(recv_buf));
				{
					boost::lock_guard<boost::mutex> lock(ModelProvider::getInstance()->wmMutex);
					memcpy(ModelProvider::getInstance()->getWorldModel(),recv_buf.data(),sizeof(WorldModel));
				}
			} else {
				FileLog::log(log_Communication, "[WorldModelClient] Own world model version higher than that of the client ", std::to_string(index+1), " => continue!");
			}
		}
		catch(boost::system::system_error &e)
		{
			boost::system::error_code ec = e.code();
			FileLog::log(log_Communication, "[WorldModelClient] System error(1/2): ", e.what());
			FileLog::log(log_Communication, "[WorldModelClient] System error(2/2): ", ec.message(), " ", std::to_string(ec.value()), " ", ec.category().name());
			FileLog::log_NOTICE("[WorldModelClient] System error(1/2): ", e.what());
			FileLog::log_NOTICE("[WorldModelClient] System error(2/2): ", ec.message(), " ", std::to_string(ec.value()), " ", ec.category().name());
			if(index<3 && ModelProvider::getInstance()->getHWID() != (ID::ID)(index+1)){
				ModelProvider::getInstance()->kickRobot((ID::ID)(index+1));
				robotWasKicked = true;
			}
			if(clientHandle.is_open())
			{
				clientHandle.shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
				if(ec)
				{
					FileLog::log(log_Communication, "[WorldModelClient] Socket Shutdown Error: ", ec.message());
					FileLog::log_NOTICE("[WorldModelClient] Socket Shutdown Error: ", ec.message());
				}

				clientHandle.close(ec);
				if(ec)
				{
					FileLog::log(log_Communication, "[WorldModelClient] Socket Close Error: ", ec.message());
					FileLog::log_NOTICE("[WorldModelClient] Socket Close Error: ", ec.message());
				}
			}
		}
	}
	{
		boost::unique_lock<boost::mutex> lock(mutexTime);
		//FileLog::log_NOTICE("[WorldModelClient]4 Set expire time to ", std::to_string(ULLONG_MAX));
		expireTime = ULLONG_MAX;
	}
	{
		boost::lock_guard<boost::mutex> lock(lockedMutex);
		locked = true;
	}

	lockedCond.notify_all();
}

void WorldModelClient::lock(){

	if(enabled){
		std::stringstream* str = new std::stringstream("");

		for(int i=0;i<3;i++){
			*str<<(int)addr[i];
			*str<<".";
		}

		*str<<(int)addr[3];
		std::string tmpAddress(str->str().c_str());
		FileLog::log(log_Communication, "[WorldModelClient] Start locking client ", std::to_string(index+1), " with ip ", tmpAddress);
		boost::asio::ip::address tempAdd;
		try
		{
			tempAdd = boost::asio::ip::address::from_string(tmpAddress.c_str());
		} catch( ... )
		{
			FileLog::log(log_Communication, "[WorldModelClient] %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
			FileLog::log(log_Communication, "[WorldModelClient] addressString: ", tmpAddress);
			FileLog::log(log_Communication, "[WorldModelClient] %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
		}
		tcp::endpoint endpoint(tempAdd, port);

		delete str;
		FileLog::log(log_Communication, "[WorldModelClient] Try to connect to server with ip ", tmpAddress, ", port: ", std::to_string(port));
		{
			boost::unique_lock<boost::mutex> lock(mutexTime);
			//FileLog::log_NOTICE("[WorldModelClient]5 Set expire time to ", std::to_string(getAbsMsTimePlus(milliseconds)));
			expireTime = getAbsMsTimePlus(milliseconds);
		}
		clientHandle.async_connect(endpoint, boost::bind(&WorldModelClient::handleConnect,this,_1));
		io_service.run();
		waitForLock();
		FileLog::log(log_Communication, "[WorldModelClient] Finished locking client ", std::to_string(index+1));
	}else{
		{
			boost::lock_guard<boost::mutex> lock(lockedMutex);
			locked = true;
		}
	}
}

void WorldModelClient::unlock(){

	boost::system::error_code error;

	FileLog::log(log_Communication, "[WorldModelClient] Start unlocking client ", std::to_string(index+1));

	if(enabled && clientHandle.is_open())
	{
		{
			boost::unique_lock<boost::mutex> lock(mutexTime);
			//FileLog::log_NOTICE("[WorldModelClient]6 Set expire time to ", std::to_string(getAbsMsTimePlus(milliseconds)));
			expireTime = getAbsMsTimePlus(milliseconds);
		}

		{
			boost::lock_guard<boost::mutex> lock(ModelProvider::getInstance()->wmMutex);
			FileLog::log(log_Communication, "[WorldModelClient] Send world model to client with ip ", clientHandle.remote_endpoint().address().to_string());
			char* byte_array = static_cast<char*>(static_cast<void*>(ModelProvider::getInstance()->getWorldModel()));
			boost::asio::write(clientHandle, boost::asio::buffer(byte_array,sizeof(WorldModel)), boost::asio::transfer_all(), error);
		}

		if(error){
			FileLog::log(log_Communication, "[WorldModelClient] Sent error: ", error.message());
			FileLog::log_NOTICE("[WorldModelClient] Sent error: ", error.message());
		}else{
			//std::cout<<"[WorldModelClient] Sent WM to "<<clientHandle.remote_endpoint()<<"\n";
		}

		FileLog::log(log_Communication, "[WorldModelClient] waiting for shutdown");

		// wait for socket to be closed
		bool socketWasClosed = false;
		while(!socketWasClosed)
		{
			char buf[1];
			try
			{
				clientHandle.receive(boost::asio::buffer(buf, 1));
			}catch(boost::system::system_error& e)
			{
				//if(e.code() == boost::asio::error::eof)
					socketWasClosed = true;
			}
		}

		FileLog::log(log_Communication, "[WorldModelClient] shutdown finished");
	}
	else if(enabled)
	{
		FileLog::log_NOTICE("[WorldModelClient] unlock(): Did not send the world Model");
		FileLog::log(log_Communication, "[WorldModelClient] unlock(): Did not send the world Model");
	}

	{
		boost::unique_lock<boost::mutex> lock(mutexTime);
		//FileLog::log_NOTICE("[WorldModelClient]7 Set expire time to ", std::to_string(ULLONG_MAX));
		expireTime = ULLONG_MAX;
	}

	FileLog::log(log_Communication, "[WorldModelClient] Finished unlocking client ", std::to_string(index+1));
}

bool WorldModelClient::getRobotWasKicked()
{
	return robotWasKicked;
}

void WorldModelClient::waitForLock()
{
	boost::unique_lock<boost::mutex> lock(lockedMutex);
	while(locked == false)
	{
		lockedCond.wait(lock); //waits for the notify, handles mutex locking/unlocking
	}
}

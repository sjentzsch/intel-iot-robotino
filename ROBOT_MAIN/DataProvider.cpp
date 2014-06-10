/*
 * DataProvider.cpp
 *
 *  Created on: Jun 10, 2014
 *      Author: root
 */

#include "DataProvider.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

DataProvider *DataProvider::instance = NULL;

DataProvider::DataProvider()
{

}

DataProvider::~DataProvider()
{

}

DataProvider* DataProvider::getInstance()
{
	if(instance==NULL)
		instance = new DataProvider();
	return instance;
}

void DataProvider::processMsg(::std::stringstream& msg)
{
	boost::property_tree::ptree pt;

	//  try
	//  {
	boost::property_tree::json_parser::read_json(msg, pt);
	//  }
	//  catch(boost::property_tree::json_parser_error &exc)
	//  {
	//    std::cerr << "[TBUBaseParameters]: " << exc.what() << ". Default parameters will be used" << std::endl;
	//    setDefaultParameters();
	//    return;
	//  }

	string message = pt.get<string>("message");
	::std::cout << "message is: " << message << ::std::endl;
}

MsgEnvironment DataProvider::getLatestMsgEnvironment()
{
	boost::shared_lock<boost::shared_mutex> r_lock(mutexMsgEnvironment);
	return this->msgEnvironment;
}

/*
 * MsgEnvironment.cpp
 *
 *  Created on: Jun 10, 2014
 *      Author: root
 */

#include "MsgEnvironment.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#define PARAM_PRINT(__var__) #__var__ << ": " << __var__ << std::endl

MsgEnvironment::MsgEnvironment()
{
	this->message = "msg_environment";
	this->time_start = 0;
	this->x_max = 4.1;
	this->y_max = 8.7;
	this->x_robot = 0.5;
	this->y_robot = 2.4;
	this->phi_robot = 0.0;
	this->x_base = 3.5;
	this->y_base = 3.4;
	this->phi_base = 3.141;
}

MsgEnvironment::~MsgEnvironment()
{

}

void MsgEnvironment::load(::std::stringstream msg)
{
	// Create an empty property tree object
	using boost::property_tree::ptree;
	ptree pt;

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

	message = pt.get<string>("message");
	time_start = pt.get<unsigned long>("time_start");
	x_max = pt.get<double>("x_max");
	y_max = pt.get<double>("y_max");
	x_robot = pt.get<double>("x_robot");
	y_robot = pt.get<double>("y_robot");
	phi_robot = pt.get<double>("phi_robot");
	x_base = pt.get<double>("x_base");
	y_base = pt.get<double>("y_base");
	phi_base = pt.get<double>("phi_base");

	// example record parameters
	// if(pt.get_child_optional("tbu_base.records"))
	// {
	//   BOOST_FOREACH(ptree::value_type &v, pt.get_child("tbu_base.records")){
	//     Record record;
	//     record.record_string = v.second.get<std::string>("record_string");
	//     record.record_double = v.second.get<double>("record_double");
	//     records.push_back(record);
	//   }
	// }
}

void MsgEnvironment::print()
{
	std::cout << "####################:\n";
	std::cout << "MsgEnvironment:\n";
	std::cout << "####################:\n";
	std::cout << PARAM_PRINT(message);
	std::cout << PARAM_PRINT(time_start);
	std::cout << PARAM_PRINT(x_max);
	std::cout << PARAM_PRINT(y_max);
	std::cout << PARAM_PRINT(x_robot);
	std::cout << PARAM_PRINT(y_robot);
	std::cout << PARAM_PRINT(phi_robot);
	std::cout << PARAM_PRINT(x_base);
	std::cout << PARAM_PRINT(y_base);
	std::cout << PARAM_PRINT(phi_base);

	// std::cout << "Records:" << std::endl;
	// BOOST_FOREACH(Record &record, records){
	//   std::cout << "\t" << PARAM_PRINT(record.record_string);
	//   std::cout << "\t" << PARAM_PRINT(record.record_double);
	//   std::cout << "\t" << "---\n";
	// }
}

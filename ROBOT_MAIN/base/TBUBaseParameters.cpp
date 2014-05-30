#include "TBUBaseParameters.h"
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <stdexcept>
#include <sstream>
#include <ctime>

#include <sstream>

#define PARAM_PRINT(__var__) #__var__ << ": " << __var__ << std::endl

void TBUBaseParameters::save(std::string filename)
{
  // Create an empty property tree object
  using boost::property_tree::ptree;
  ptree pt;

  pt.put("tbu_base.team_number", team_number);
  pt.put("tbu_base.simulation_mode", simulation_mode);
  pt.put("tbu_base.simulation_odometry", simulation_odometry);
  pt.put("tbu_base.send_ir_sensor_events", send_ir_sensor_events);
  pt.put("tbu_base.garching_environment", garching_environment);

  // example record parameters
  // int i=0;
  // BOOST_FOREACH(const Record &data_record, records){
  //   std::stringstream ss;
  //   ss << "tbu_base.records.record_" << i++;

  //   pt.add(ss.str() + ".record_string", data_record.record_string);
  //   pt.add(ss.str() + ".record_double", data_record.record_double);
  // }

  // Write the property tree to the XML file.
  boost::property_tree::json_parser::write_json(filename, pt);
}



void TBUBaseParameters::load(std::string filename)
{
  // Create an empty property tree object
  using boost::property_tree::ptree;
  ptree pt;

//  try
//  {
    boost::property_tree::json_parser::read_json(filename, pt);
//  }
//  catch(boost::property_tree::json_parser_error &exc)
//  {
//    std::cerr << "[TBUBaseParameters]: " << exc.what() << ". Default parameters will be used" << std::endl;
//    setDefaultParameters();
//    return;
//  }

  team_number = pt.get<unsigned int>("tbu_base.team_number");
  simulation_mode = pt.get<bool>("tbu_base.simulation_mode");
  simulation_odometry = pt.get<bool>("tbu_base.simulation_odometry");
  send_ir_sensor_events = pt.get<bool>("tbu_base.send_ir_sensor_events");
  garching_environment = pt.get<bool>("tbu_base.garching_environment");

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

  std::cout << "[TBUBaseParameters]: loaded params from " << filename << std::endl;
}

void TBUBaseParameters::print()
{
  std::cout << "#####################################:\n";
  std::cout << "TBUBaseParameters configuration:\n";
  std::cout << "#####################################:\n";
  std::cout << PARAM_PRINT(team_number);
  std::cout << PARAM_PRINT(simulation_mode);
  std::cout << PARAM_PRINT(simulation_odometry);
  std::cout << PARAM_PRINT(send_ir_sensor_events);
  std::cout << PARAM_PRINT(garching_environment);

  // std::cout << "Records:" << std::endl;
  // BOOST_FOREACH(Record &record, records){
  //   std::cout << "\t" << PARAM_PRINT(record.record_string);
  //   std::cout << "\t" << PARAM_PRINT(record.record_double);
  //   std::cout << "\t" << "---\n";
  // }
}

void TBUBaseParameters::setDefaultParameters(){
  team_number = 1;
  simulation_mode = 0;
  simulation_odometry = 0;
  send_ir_sensor_events = 0;
  garching_environment = 0;
}

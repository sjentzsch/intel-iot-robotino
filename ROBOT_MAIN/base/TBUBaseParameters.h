#ifndef TBU_BASE_PARAMETERS
#define TBU_BASE_PARAMETERS

#include <string>
#include <vector>

/**
 * @brief An example record object containing some parameter items
 */
struct Record
{
  std::string record_string;
  double record_double;
};

/**
 * @brief The Parameters for the TBU_BASE class
 */
struct TBUBaseParameters{
  //////////////////////////////////////////
  // Parameters
  //////////////////////////////////////////

  /**
   * @brief Set Team Number: 1 | 2
   */
  unsigned int team_number;

  /**
   * @brief Set Simulation Mode: 1 ON | 0 OFF
   */
  bool simulation_mode;

  /**
   * @brief Set Simulation Mode: 1 ON | 0 OFF
   */
  bool simulation_odometry;

  /**
   * @brief Send IR-Sensor Events: 1 | Do not send IR-Sensor Events: 0
   */  
  bool send_ir_sensor_events;

  /**
   * @brief Set if using the testing environment in Garching Hochbrück: 1 YES | 0 NO
   */  
  bool garching_environment;
  
  /**
   * @brief record example
   */
  // std::vector<Record> records;

  //////////////////////////////////////////
  // Methods
  //////////////////////////////////////////

  /**
   * @brief load: loads this parameter struct from file
   * @param filename: /path/to/file
   */
  void load(std::string filename);

  /**
   * @brief save: saves this parameter struct to file
   * @param filename /path/to/file
   */
  void save(std::string filename);

  /**
   * @brief print: prints this parameter struct on std::cout
   */
  void print();

  /**
   * @brief setDefaultParameters: sets this parameter struct to its default values
   */
  void setDefaultParameters();
};

#endif

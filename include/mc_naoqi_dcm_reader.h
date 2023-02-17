#pragma once
#include <alcommon/almodule.h>
#include <boost/shared_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <althread/almutex.h>
#include "RobotModule.h"

#include <fstream>

namespace AL
{
class ALBroker;
class ALMemoryFastAccess;
class DCMProxy;
class ALMemoryProxy;
}

namespace mc_naoqi_dcm
{
/**
 * @brief Module to use fast method to get/set joints every 12ms with minimum delays.
 * Supported robots are PEPPER and NAO, but it should be suitable to for
 * extension to other Softbank Robotics robots.
 */
class MCNAOqiDCM : public AL::ALModule
{
 public:
  /**
   * @brief Module to use fast method to get/set joints every 12ms with minimum delays.
   *
   * @param broker A smart pointer to the broker (communication object)
   * @param name The name of the module
   */
  MCNAOqiDCM(boost::shared_ptr<AL::ALBroker> pBroker,
                const std::string &pName);

 private:
  /**
   * @brief Joint order in which the actuator values will be expressed
   *
   * @return
   * Reference joint order as defined in RobotModule::actuators
   */
  std::vector<std::string> getJointOrder() const;

  /**
   * @brief List of readable sensor names.
   * getSensors() will return sensor values corresponding to these.
   *
   * @return
   * Human-readable list of sensor names, as defined in RobotModule::sensors
   */
  std::vector<std::string> getSensorsOrder() const;

  /**
   * @brief Sensor values in the order expressed by getSensorsOrder()
   *
   * @return Vector of sensor values
   */
  std::vector<float> getSensors();

  /**
   * @brief Robot name (pepper or nao)
   *
   * @return robot name
   */
  std::string getRobotName() const;

  // Used for fast memory access
  boost::shared_ptr<AL::ALMemoryFastAccess> fMemoryFastAccess;

  // Store sensor values.
  std::vector<float> sensorValues;
  boost::shared_ptr<AL::DCMProxy> dcmProxy;

  // Memory proxy
  boost::shared_ptr<AL::ALMemoryProxy> memoryProxy;

  /**
   * \brief The RobotModule describes the sensors names and their corresponding
   * naoqi keys. The intent is to have a generic dcm module for both NAO and
   * PEPPER robots.
   */
  RobotModule robot_module;

  /**
   * Total number of sensors to be read from the memeory
   * Allows to pre-set apropriate vector size for storing and updating all sensor readings
   */
  int numSensors() const;

  std::ofstream logfile;
};

} /* mc_naoqi_dcm */

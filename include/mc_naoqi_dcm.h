#pragma once
#include <qi/anyvalue.hpp>
#include <qi/session.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include "RobotModule.h"

namespace mc_naoqi_dcm
{
/**
 * @brief Module to use fast method to get/set joints every 12ms with minimum delays.
 * Supported robots are PEPPER and NAO, but it should be suitable to for
 * extension to other Softbank Robotics robots.
 */
class MCNAOqiDCM
{
 public:
  /**
   * @brief Module to use fast method to get/set joints every 12ms with minimum delays.
   *
   * @param broker A smart pointer to the broker (communication object)
   * @param name The name of the module
   */
  MCNAOqiDCM(qi::SessionPtr session);

  virtual ~MCNAOqiDCM();

  /*! Initialisation of ALMemory/DCM link */
  void init();

  /**
   * @brief Set one hardness value to all joint
   *
   * @param stiffnessValue
   * Stiffness value that will be applied to all joints
   */
  void setStiffness(const float &stiffnessValue);

  /**
   * @brief Sets the desired actuator position to the specified one.
   *
   * @param jointValues
   * Joint values, specified in the same order as RobotModule::actuators.
   */
  void setJointAngles(std::vector<float> jointValues);

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

  /**
   * Create DCM alias and prepare command (ALValue structure) for it
   */
  void createAliasPrepareCommand(std::string aliasName,
                                 const std::vector<std::string> &mem_keys,
                                 std::vector<std::vector<std::vector<qi::AnyValue>>>& ledCommands,
                                 std::string updateType="ClearAll");

  /**
   * Total number of sensors to be read from the memeory
   * Allows to pre-set apropriate vector size for storing and updating all sensor readings
   */
  int numSensors() const;

 private:
  // Store sensor values.
  std::vector<float> sensorValues;
  std::map<std::string, int> sensorIndex;
  std::map<std::string, int> actuatorsIndex;
  qi::AnyObject dcmProxy;

  // Memory proxy
  qi::AnyObject memoryProxy;

  // Used for sending joint position commands every 12ms in callback
  std::vector<std::vector <std::vector <qi::AnyValue>>> jointPositionCommands;

  // joint stiffness command for DCM
  std::vector<std::vector <std::vector <qi::AnyValue>>> jointStiffnessCommands;

  /**
   *brief The RobotModule describes the sensors names and their corresponding
   * naoqi keys. The intent is to have a generic dcm module for both NAO and
   * PEPPER robots.
   */
  RobotModule robot_module;

  qi::SessionPtr session_;

};

} /* mc_naoqi_dcm */

// Bind methods to make them accessible through proxies
QI_REGISTER_MT_OBJECT(mc_naoqi_dcm::MCNAOqiDCM,
                        getJointOrder,
                        getRobotName,
                        getSensors,
                        getSensorsOrder,
                        numSensors,
                        setJointAngles,
                        setStiffness);
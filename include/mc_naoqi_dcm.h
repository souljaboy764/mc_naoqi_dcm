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

  /*! Enable/disable turning off wheels on bumper pressed */
  void bumperSafetyReflex(bool state);

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
   * Make robot say a sentence given in argument
   */
  void sayText(const std::string &toSay);

  /**
   * Create DCM alias and prepare command (ALValue structure) for it
   */
  void createAliasPrepareCommand(std::string aliasName,
                                 const std::vector<std::string> &mem_keys,
                                 std::vector<qi::AnyValue>& ledCommands,
                                 std::string updateType="ClearAll");
  // Create aliases for all leg groups defined in robot module
  void createLedAliases();

  /**
   * @brief Set one hardness value to all wheels
   *
   * @param stiffnessValue
   * Stiffness value that will be applied to all wheels
   */
  void setWheelsStiffness(const float &stiffnessValue);

  /**
   * @brief Set speed values to the wheels
   *
   * @param speed_fl
   * Speed value that will be applied to front left wheel
   * @param speed_fr
   * Speed value that will be applied to front right wheel
   * @param speed_b
   * Speed value that will be applied to back wheel
   */
  void setWheelSpeed(const float &speed_fl, const float &speed_fr, const float &speed_b);

  // one led set function for all groups
  void setLeds(std::string ledGroupName, const float &r, const float &g, const float &b);
  void setLedsDelay(std::string ledGroupName, const float &r, const float &g, const float &b, const int& delay);
  // Note: cannot bind mathods with the same name and different arguments
  void isetLeds(std::string ledGroupName, const float &intensity);
  // blink
  void blink();

  /**
  * This method will be called every time the bumper press event is raised
  */
  void onBumperPressed();

  /**
   * Total number of sensors to be read from the memeory
   * Allows to pre-set apropriate vector size for storing and updating all sensor readings
   */
  int numSensors() const;

  /**
   * Bumper sensor names
   */
  std::vector<std::string> bumperNames() const;

  /**
   * Tactile sensor names
   */
  std::vector<std::string> tactileSensorNames() const;

  /**
   * Ordered wheels actuator names
   */
  std::vector<std::string> wheelNames() const;

 private:
  // Store sensor values.
  std::vector<float> sensorValues;
  qi::AnyObject dcmProxy;

  // Memory proxy
  qi::AnyObject memoryProxy;

  // Used for sending joint position commands every 12ms in callback
  std::vector<qi::AnyValue> jointPositionCommands;

  // Used to store joint possition command to set via DCM every 12ms
  std::vector<qi::AnyValue> commands;

  // joint stiffness command for DCM
  std::vector<qi::AnyValue> jointStiffnessCommands;

  /**
   * Store command to send to leds
   */

  // map led group name to corresponding RGB or intensity commands
  std::map<std::string, std::vector<std::vector<qi::AnyValue>>> ledCmdMap;

  /**
   * Store commands to send to wheels (speed and stiffness)
   */
  std::vector<qi::AnyValue> wheelsCommands;
  std::vector<qi::AnyValue> wheelsStiffnessCommands;

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
                        setStiffness,
                        setJointAngles,
                        getJointOrder,
                        getSensorsOrder,
                        numSensors,
                        bumperNames,
                        tactileSensorNames,
                        wheelNames,
                        getSensors,
                        getRobotName,
                        sayText,
                        setLeds,
                        isetLeds,
                        blink,
                        onBumperPressed,
                        bumperSafetyReflex,
                        setWheelsStiffness,
                        setWheelSpeed);
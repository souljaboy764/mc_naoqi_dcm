#pragma once
#include <alcommon/almodule.h>
#include <boost/shared_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <althread/almutex.h>
#include "RobotModule.h"

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

  virtual ~MCNAOqiDCM();

  /*! Start the example */
  void startLoop();

  /*! Stop the example */
  void stopLoop();

 private:
  /*! Initialisation of ALMemory/DCM link */
  void init();

  /*! ALMemory fast access */
  void initFastAccess();

  /*!  Connect callback to the DCM preproccess */
  void connectToDCMloop();

  /**
  * @brief Callback called by the DCM every 12ms
  *
  *  Once this method is connected to DCM preprocess
  *  it will be called in Real Time every 12 milliseconds from DCM thread
  *  Dynamic allocation and system call are strictly forbidden in this method
  *  Computation time in this section must remain as short as possible to prevent
  *  erratic move or joint getting loose.
  *
  */
  void synchronisedDCMcallback();

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
                                 AL::ALValue& ledCommands,
                                 std::string updateType="ClearAll");
  // Create aliases for all leg groups defined in robot module
  void createLedAliases();

  // check if preProces is connected
  bool isPreProccessConnected();

 private:
  // Used for preprocess sync with the DCM
  ProcessSignalConnection fDCMPreProcessConnection;

  // Used to check id preprocess is connected
  bool preProcessConnected = false;

  // Used for fast memory access
  boost::shared_ptr<AL::ALMemoryFastAccess> fMemoryFastAccess;

  // Store sensor values.
  std::vector<float> sensorValues;
  boost::shared_ptr<AL::DCMProxy> dcmProxy;

  // Memory proxy
  boost::shared_ptr<AL::ALMemoryProxy> memoryProxy;

  // Used for sending joint position commands every 12ms in callback
  std::vector<float> jointPositionCommands;

  // Used to store joint possition command to set via DCM every 12ms
  AL::ALValue commands;

  // joint stiffness command for DCM
  AL::ALValue jointStiffnessCommands;

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
};

} /* mc_naoqi_dcm */

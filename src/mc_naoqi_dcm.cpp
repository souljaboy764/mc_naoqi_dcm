/// Module to communicate with mc_rtc_naoqi interface for whole-body control via mc_rtc framework
/// Enables connection of a preproccess callback to DCM loop for sending joint commands every 12ms
/// Implemented for both NAO and PEPPER robots.

#include "mc_naoqi_dcm.h"
#include "PepperRobotModule.h"

#include <alcommon/albroker.h>
#include <alcommon/almodule.h>
#include <alcommon/alproxy.h>
#include <alerror/alerror.h>
#include <almemoryfastaccess/almemoryfastaccess.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/dcmproxy.h>

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

namespace mc_naoqi_dcm
{
MCNAOqiDCM::MCNAOqiDCM(boost::shared_ptr<AL::ALBroker> broker, const std::string &name)
    : AL::ALModule(broker, name), fMemoryFastAccess(boost::shared_ptr<AL::ALMemoryFastAccess>(new AL::ALMemoryFastAccess()))
{
  setModuleDescription("Module to communicate with mc_rtc_naoqi interface for whole-body control via mc_rtc framework");

  // Bind methods to make them accessible through proxies
  functionName("startLoop", getName(), "connect a callback to DCM loop");
  BIND_METHOD(MCNAOqiDCM::startLoop);

  functionName("stopLoop", getName(), "disconnect a callback from DCM loop");
  BIND_METHOD(MCNAOqiDCM::stopLoop);

  functionName("isPreProccessConnected", getName(), "Check if preProccess is connected");
  setReturn("if connected", "boolean indicating if preProccess is connected to DCM loop");
  BIND_METHOD(MCNAOqiDCM::isPreProccessConnected);

  functionName("setStiffness", getName(), "change stiffness of all joint");
  addParam("value", "new stiffness value from 0.0 to 1.0");
  BIND_METHOD(MCNAOqiDCM::setStiffness);

  functionName("setJointAngles", getName(), "set joint angles");
  addParam("values", "new joint angles (in radian)");
  BIND_METHOD(MCNAOqiDCM::setJointAngles);

  functionName("getJointOrder", getName(), "get reference joint order");
  setReturn("joint order", "array containing names of all the joints");
  BIND_METHOD(MCNAOqiDCM::getJointOrder);

  functionName("getSensorsOrder", getName(), "get reference sensor order");
  setReturn("sensor names", "array containing names of all the sensors");
  BIND_METHOD(MCNAOqiDCM::getSensorsOrder);

  functionName("numSensors", getName(), "get total number of sensors");
  setReturn("sensors number", "int indicating total number of different sensors");
  BIND_METHOD(MCNAOqiDCM::numSensors);

  functionName("getSensors", getName(), "get all sensor values");
  setReturn("sensor values", "array containing values of all the sensors");
  BIND_METHOD(MCNAOqiDCM::getSensors);

  functionName("getRobotName", getName(), "get robot name");
  setReturn("robot name", "name of the robot for which module was built <pepper|nao>");
  BIND_METHOD(MCNAOqiDCM::getRobotName);

  // Create Pepper robot module
  robot_module = PepperRobotModule();
  
  // Get the DCM proxy
  try{
    dcmProxy = getParentBroker()->getDcmProxy();
  }catch (AL::ALError &e){
    throw ALERROR(getName(), "MCNAOqiDCM", "Impossible to create DCM Proxy : " + e.toString());
  }

  // Get the ALMemory proxy
  try{
    memoryProxy = getParentBroker()->getMemoryProxy();
  }catch (AL::ALError &e){
    throw ALERROR(getName(), "MCNAOqiDCM", "Impossible to create ALMemory Proxy : " + e.toString());
  }

  // Check that DCM is running
  signed long isDCMRunning;
  try{
    isDCMRunning = getParentBroker()->getProxy("ALLauncher")->call<bool>("isModulePresent", std::string("DCM"));
  }catch (AL::ALError &e){
    throw ALERROR(getName(), "MCNAOqiDCM", "Error when connecting to DCM : " + e.toString());
  }

  if (!isDCMRunning){
    throw ALERROR(getName(), "MCNAOqiDCM", "Error no DCM running ");
  }

  // initialize sensor reading/setting
  init();

  // Get all sensor values from ALMemory using fastaccess
  fMemoryFastAccess->GetValues(sensorValues);

  // Save initial sensor values into 'jointPositionCommands'
  for (int i = 0; i < robot_module.actuators.size(); i++){
    jointPositionCommands.push_back(sensorValues[i]);
  }

  // Send initial command to the actuators
  int DCMtime;
  try{
    // Get absolute time, at 0 ms in the future ( i.e. now )
    DCMtime = dcmProxy->getTime(0);
  }catch (const AL::ALError &e){
    throw ALERROR(getName(), "MCNAOqiDCM", "Error on DCM getTime : " + e.toString());
  }
  for (unsigned i = 0; i < robot_module.actuators.size(); i++){
    commands[3][i][0][0] = jointPositionCommands[i];
    commands[3][i][0][1] = DCMtime;
  }
  try{
    dcmProxy->setAlias(commands);
  }catch (const AL::ALError &e){
    throw ALERROR(getName(), "MCNAOqiDCM", "Error when sending command to DCM : " + e.toString());
  }
}

// Module destructor
MCNAOqiDCM::~MCNAOqiDCM()
{
  setStiffness(0.0f);
  stopLoop();
}

// Start loop
void MCNAOqiDCM::startLoop()
{
  connectToDCMloop();
  preProcessConnected = true;
}

// Stop loop
void MCNAOqiDCM::stopLoop()
{
  // Remove the preProcess callback connection
  fDCMPreProcessConnection.disconnect();
  preProcessConnected = false;
}

bool MCNAOqiDCM::isPreProccessConnected(){
  return preProcessConnected;
}

void MCNAOqiDCM::init()
{
  // Enable fast access of all robot_module.readSensorKeys from memory
  initFastAccess();
  // create 'jointActuator' alias to be used for sending joint possition commands
  createAliasPrepareCommand("jointActuator", robot_module.setActuatorKeys, commands);
  // create 'jointStiffness' alias to be used for setting joint stiffness commands
  createAliasPrepareCommand("jointStiffness", robot_module.setHardnessKeys, jointStiffnessCommands);
  // keep body joints turned off at initialization
  setStiffness(0.0f);
}

void MCNAOqiDCM::initFastAccess(){
  // Create the fast memory access to read sensor values
  fMemoryFastAccess->ConnectToVariables(getParentBroker(), robot_module.readSensorKeys, false);
}


void MCNAOqiDCM::createAliasPrepareCommand(std::string aliasName,
                const std::vector<std::string> &mem_keys,
                AL::ALValue& alias_command, std::string updateType){

  // create alias (unite group of memory keys under specific alis name)
  AL::ALValue alias;
  // 2 - for alias name and array of alias memory keys
  alias.arraySetSize(2);
  alias[0] = std::string(aliasName);
  alias[1].arraySetSize(mem_keys.size());

  // fill in array of memory keys of the alias
  for (unsigned i = 0; i < mem_keys.size(); ++i){
    alias[1][i] = mem_keys[i];
  }

  // Create alias in DCM
  try{
    dcmProxy->createAlias(alias);
  }catch (const AL::ALError &e){
    throw ALERROR(getName(), "createLedAlias()", "Error when creating Alias : " + e.toString());
  }

  // alias created succesfully
  // prepare command for this alias to be set via DCM 'setAlias' call
  alias_command.arraySetSize(6);
  alias_command[0] = std::string(aliasName);
  alias_command[1] = std::string(updateType);
  alias_command[2] = std::string("time-mixed");
  // placeholder for timed-command values
  alias_command[3].arraySetSize(mem_keys.size());
  for (int i = 0; i < mem_keys.size(); i++){
    // allocate space for a new value for a memory key to be set via setAlias call
    alias_command[3][i].arraySetSize(1);
    alias_command[3][i][0].arraySetSize(2);
  }
}

void MCNAOqiDCM::setStiffness(const float &stiffnessValue)
{
  int DCMtime;
  // increase stiffness with the "jointStiffness" Alias created at initialisation
  try{
    DCMtime = dcmProxy->getTime(0);
  }catch (const AL::ALError &e){
    throw ALERROR(getName(), "setStiffness()", "Error on DCM getTime : " + e.toString());
  }

  for(int i=0;i<robot_module.actuators.size();i++){
    jointStiffnessCommands[3][i][0][0] = stiffnessValue;
    jointStiffnessCommands[3][i][0][1] = DCMtime;
  }

  try{
    dcmProxy->setAlias(jointStiffnessCommands);
  }catch (const AL::ALError &e){
    throw ALERROR(getName(), "setStiffness()", "Error when sending stiffness to DCM : " + e.toString());
  }
}

void MCNAOqiDCM::setJointAngles(std::vector<float> jointValues)
{
  // update values in the vector that is used to send joint commands every 12ms
  jointPositionCommands = jointValues;
}

std::vector<std::string> MCNAOqiDCM::getJointOrder() const
{
  return robot_module.actuators;
}

std::vector<std::string> MCNAOqiDCM::getSensorsOrder() const
{
  return robot_module.sensors;
}

std::string MCNAOqiDCM::getRobotName() const
{
  return robot_module.name;
}

int MCNAOqiDCM::numSensors() const
{
  return robot_module.readSensorKeys.size();
}

// Method is not synchronized with DCM loop
// Better approach might be to call GetValues on postprocess of DCM loop
std::vector<float> MCNAOqiDCM::getSensors()
{
  // Get all values from ALMemory using fastaccess
  fMemoryFastAccess->GetValues(sensorValues);
  return sensorValues;
}

void MCNAOqiDCM::connectToDCMloop()
{
  // Connect callback to the DCM pre proccess
  try{
    //  onPreProcess is useful because it’s called just before the computation of orders sent to the chestboard (USB). Sending commands at this level means that you have the shortest delay to your command.
    fDCMPreProcessConnection =
        getParentBroker()->getProxy("DCM")->getModule()->atPreProcess(boost::bind(&MCNAOqiDCM::synchronisedDCMcallback, this));
        // what happens if I add it twice? add same callback while it is already added? try in python?
  }catch (const AL::ALError &e){
    throw ALERROR(getName(), "connectToDCMloop()", "Error when connecting to DCM preProccess: " + e.toString());
  }
}

// using 'jointActuator' alias created 'command'
// that will use data from 'jointPositionCommands' to send it to DCM every 12 ms
void MCNAOqiDCM::synchronisedDCMcallback()
{
  int DCMtime;

  try{
    // Get absolute time, at 0 ms in the future ( i.e. now )
    DCMtime = dcmProxy->getTime(0);
  }catch (const AL::ALError &e){
    throw ALERROR(getName(), "synchronisedDCMcallback()", "Error on DCM getTime : " + e.toString());
  }


  // XXX make this faster with memcpy?
  for (unsigned i = 0; i < robot_module.actuators.size(); i++){
    // new actuator value = latest values from jointPositionCommands
    commands[3][i][0][0] = jointPositionCommands[i];
    commands[3][i][0][1] = DCMtime;
  }

  try{
    dcmProxy->setAlias(commands);
  }catch (const AL::ALError &e){
    throw ALERROR(getName(), "synchronisedDCMcallback()", "Error when sending command to DCM : " + e.toString());
  }
}

} /* mc_naoqi_dcm */

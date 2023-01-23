/// Module to communicate with mc_rtc_naoqi interface for whole-body control via mc_rtc framework
/// Enables connection of a preproccess callback to DCM loop for sending joint commands every 12ms
/// Implemented for both NAO and PEPPER robots.

#include <algorithm>

// #include <qi/anyobject.hpp>
// #include <qi/anyvalue.hpp>

// #include <boost/bind.hpp>

#include "mc_naoqi_dcm.h"
#include "NAORobotModule.h"
#include "PepperRobotModule.h"


namespace mc_naoqi_dcm
{
MCNAOqiDCM::MCNAOqiDCM(qi::SessionPtr session)
    : session_(session)
{
  
  #ifdef PEPPER
    // Create Pepper robot module
    robot_module = PepperRobotModule();
  #else
    // Create NAO robot module
    robot_module = NAORobotModule();
  #endif

  // Get the DCM proxy
  try{
    dcmProxy = session_->service("DCM");
  }catch (const std::exception &e){
    throw std::runtime_error(std::string("MCNAOqiDCM::MCNAOqiDCM(): Impossible to create DCM Proxy : ") + e.what());
  }

  // Get the memoryProxy proxy
  try{
    memoryProxy = session_->service("memoryProxy");
  }catch (const std::exception &e){
    throw std::runtime_error(std::string("MCNAOqiDCM::MCNAOqiDCM(): Impossible to create memoryProxy Proxy : ") + e.what());
  }

  // Check that DCM is running
//   signed long isDCMRunning;
//   try{
//     isDCMRunning = getParentBroker()->getProxy("ALLauncher")->call<bool>("isModulePresent", std::string("DCM"));
//   }catch (AL::ALError &e){
//     throw ALERROR(getName(), "MCNAOqiDCM", "Error when connecting to DCM : " + e.what());
//   }

//   if (!isDCMRunning){
//     throw ALERROR(getName(), "MCNAOqiDCM", "Error no DCM running ");
//   }

  // initialize sensor reading/setting
  init();

  // Save initial sensor values into 'jointPositionCommands'
  for (int i = 0; i < robot_module.actuators.size(); i++){
    jointPositionCommands.push_back(memoryProxy.call<qi::AnyValue>("getData", robot_module.setActuatorKeys[i]));
    sensorValues.push_back(jointPositionCommands[i].toFloat());
  }

  // Send initial command to the actuators
  int DCMtime;
  try
  {
    DCMtime = dcmProxy.call<int>("getTime", 0);
  }
  catch(const std::exception& e)
  {
    throw std::runtime_error(std::string("MCNAOqiDCM::MCNAOqiDCM(): Error on DCM getTime : ") + e.what());
  }

  commands[4][0] = qi::AnyReference::from(DCMtime);
  for (unsigned i = 0; i < robot_module.actuators.size(); i++){
    commands[5][i][0] = qi::AnyReference::from(jointPositionCommands[i]);
  }
  try{
    dcmProxy.call<void>("setAlias", commands);
  }catch(const std::exception& e){
    throw std::runtime_error(std::string("MCNAOqiDCM::MCNAOqiDCM(): Error when sending command to DCM : ") + e.what());
  }
}

// Module destructor
MCNAOqiDCM::~MCNAOqiDCM()
{
  bumperSafetyReflex(false);
  setWheelSpeed(0.0f, 0.0f, 0.0f);
  setStiffness(0.0f);
  setWheelsStiffness(0.0f);
  stopLoop();
}

// Enable/disable mobile base safety reflex
void MCNAOqiDCM::bumperSafetyReflex(bool state)
{
  if(state){
    // Subscribe to events
    memoryProxy.call<void>("subscribeToEvent", "RightBumperPressed", "MCNAOqiDCM", "onBumperPressed");
    memoryProxy.call<void>("subscribeToEvent", "LeftBumperPressed", "MCNAOqiDCM", "onBumperPressed");
    memoryProxy.call<void>("subscribeToEvent", "BackBumperPressed", "MCNAOqiDCM", "onBumperPressed");
  }else{
    // Unsubscribe event callback
    memoryProxy.call<void>("unsubscribeToEvent", "RightBumperPressed", "MCNAOqiDCM");
    memoryProxy.call<void>("unsubscribeToEvent", "LeftBumperPressed", "MCNAOqiDCM");
    memoryProxy.call<void>("unsubscribeToEvent", "BackBumperPressed", "MCNAOqiDCM");
  }
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
  // fDCMPreProcessConnection.disconnect(0);
  preProcessConnected = false;
}

void MCNAOqiDCM::onBumperPressed() {
  // Turn off wheels
  setWheelsStiffness(0.0f);
  setWheelSpeed(0.0f, 0.0f, 0.0f);
}

bool MCNAOqiDCM::isPreProccessConnected(){
  return preProcessConnected;
}

void MCNAOqiDCM::init()
{
  // create 'jointActuator' alias to be used for sending joint possition commands
  createAliasPrepareCommand("jointActuator", robot_module.setActuatorKeys, commands);
  // create 'jointStiffness' alias to be used for setting joint stiffness commands
  createAliasPrepareCommand("jointStiffness", robot_module.setHardnessKeys, jointStiffnessCommands);
  // keep body joints turned off at initialization
  setStiffness(0.0f);
  // prepare commands for all led groups of robot_module
  createLedAliases();

  #ifdef PEPPER
    // create wheels speed and stiffness commands
    for (size_t i = 0; i < robot_module.specialJointGroups.size(); i++) {
      if(robot_module.specialJointGroups[i].groupName == "wheels"){
        const JointGroup& wheels = robot_module.specialJointGroups[i];
        std::string wheelsSpeedAliasName = wheels.groupName+std::string("Speed");
        std::string wheelsStiffnessAliasName = wheels.groupName+std::string("Stiffness");
        createAliasPrepareCommand(wheelsSpeedAliasName, wheels.setActuatorKeys, wheelsCommands);
        createAliasPrepareCommand(wheelsStiffnessAliasName, wheels.setHardnessKeys, wheelsStiffnessCommands);
        // keep wheels turned off at initialization
        setWheelsStiffness(0.0f);
        break;
      }
    }
  #endif
}

// Modified from naoqi_dcm_driver
void MCNAOqiDCM::createAliasPrepareCommand(std::string aliasName,
                const std::vector<std::string> &mem_keys,
                std::vector <qi::AnyValue> &alias_command, std::string updateType){

  // prepare the command
  std::vector <qi::AnyValue> commandAlias;
  commandAlias.reserve(2);
  commandAlias.resize(2);
  commandAlias[0] = qi::AnyReference::from(aliasName);

  // set joints actuators keys
  std::vector <qi::AnyValue> commandAlias_keys;
  commandAlias_keys.resize(mem_keys.size());
  for(int i=0; i<mem_keys.size(); ++i)
    commandAlias_keys[i] = qi::AnyReference::from(mem_keys[i]);
  commandAlias[1] = qi::AnyReference::from(commandAlias_keys);

  qi::AnyValue commandAlias_qi(qi::AnyReference::from(commandAlias));

  // Create alias
  try
  {
    dcmProxy.call<void>("createAlias", commandAlias_qi);
  }
  catch(const std::exception& e)
  {
    throw std::runtime_error(std::string("MCNAOqiDCM::createLedAlias(): Error when creating Alias :") + e.what());
  }
  
  // alias created succesfully
  // prepare command for this alias to be set via DCM 'setAlias' call
  alias_command.reserve(6);
  alias_command.resize(4);
  alias_command[0] = qi::AnyReference::from(aliasName);
  alias_command[1] = qi::AnyReference::from(updateType);
  alias_command[2] = qi::AnyReference::from("time-separate");
  alias_command[3] = qi::AnyReference::from(0); // Importance level. Not yet implemented. Must be set to 0
  // placeholder for command time
  std::vector <qi::AnyValue> commands_time_;
  commands_time_.reserve(1);
  commands_time_.resize(1);
  commands_time_[0] = qi::AnyReference::from(0.);
  alias_command[4] = qi::AnyReference::from(commands_time_);

  // placeholder for command values
  std::vector <std::vector <qi::AnyValue> > commands_values_;
  commands_values_.reserve(mem_keys.size());
  commands_values_.resize(mem_keys.size());
  for(int i=0; i<mem_keys.size(); ++i)
  {
    commands_values_[i].resize(1);
    commands_values_[i][0] = qi::AnyReference::from(0);
  }

  alias_command[5] = qi::AnyReference::from(commands_values_);
}

void MCNAOqiDCM::createLedAliases()
{
  // RGB led groups
  for(int i=0;i<robot_module.rgbLedGroups.size();i++){
    const rgbLedGroup& leds = robot_module.rgbLedGroups[i];
    std::vector<qi::AnyValue> redLedCommands;
    std::vector<qi::AnyValue> greenLedCommands;
    std::vector<qi::AnyValue> blueLedCommands;
    std::string rName = leds.groupName+std::string("Red");
    std::string gName = leds.groupName+std::string("Green");
    std::string bName = leds.groupName+std::string("Blue");
    createAliasPrepareCommand(rName, leds.redLedKeys, redLedCommands, "Merge");
    createAliasPrepareCommand(gName, leds.greenLedKeys, greenLedCommands, "Merge");
    createAliasPrepareCommand(bName, leds.blueLedKeys, blueLedCommands, "Merge");
    // map led group name to led commands
    ledCmdMap[leds.groupName] = {redLedCommands, greenLedCommands, blueLedCommands};
  }

  // Single channel led groups
  for(int i=0;i<robot_module.iLedGroups.size();i++){
    const iLedGroup& leds = robot_module.iLedGroups[i];
    std::vector<qi::AnyValue> intensityLedCommands;
    createAliasPrepareCommand(leds.groupName, leds.intensityLedKeys, intensityLedCommands, "Merge");
    // map led group name to led commands
    ledCmdMap[leds.groupName] = {intensityLedCommands};
  }
}

void MCNAOqiDCM::setWheelsStiffness(const float &stiffnessValue)
{
  int DCMtime;
  // increase stiffness with the "jointStiffness" Alias created at initialisation
  try
  {
    DCMtime = dcmProxy.call<int>("getTime", 0);
  }
  catch(const std::exception& e)
  {
    throw std::runtime_error(std::string("MCNAOqiDCM::setWheelsStiffness(): Error on DCM getTime : ") + e.what());
  }

  wheelsStiffnessCommands[4][0]= qi::AnyReference::from(DCMtime);
  wheelsStiffnessCommands[5][0][0] = qi::AnyReference::from(stiffnessValue);
  wheelsStiffnessCommands[5][1][0] = qi::AnyReference::from(stiffnessValue);
  wheelsStiffnessCommands[5][2][0] = qi::AnyReference::from(stiffnessValue);

  try{
    dcmProxy.call<void>("setAlias", wheelsStiffnessCommands);
  }catch(const std::exception& e){
    throw std::runtime_error(std::string("MCNAOqiDCM::setWheelsStiffness(): Error when sending stiffness to DCM : ") + e.what());
  }
}

void MCNAOqiDCM::setWheelSpeed(const float &speed_fl, const float &speed_fr, const float &speed_b)
{
  int DCMtime;
  try
  {
    DCMtime = dcmProxy.call<int>("getTime", 0);
  }
  catch(const std::exception& e)
  {
    throw std::runtime_error(std::string("MCNAOqiDCM::setWheelSpeed(): Error on DCM getTime : ") + e.what());
  }


  wheelsCommands[4][0]= qi::AnyReference::from(DCMtime);
  wheelsCommands[5][0][0] = qi::AnyReference::from(speed_fl);
  wheelsCommands[5][1][0] = qi::AnyReference::from(speed_fr);
  wheelsCommands[5][2][0] = qi::AnyReference::from(speed_b);

  try{
    dcmProxy.call<void>("setAlias", wheelsCommands);
  }catch(const std::exception& e){
    throw std::runtime_error(std::string("MCNAOqiDCM::setWheelSpeed(): Error when sending command to DCM : ") + e.what());
  }
}

void MCNAOqiDCM::setStiffness(const float &stiffnessValue)
{
  int DCMtime;
  // increase stiffness with the "jointStiffness" Alias created at initialisation
  try
  {
    DCMtime = dcmProxy.call<int>("getTime", 0);
  }
  catch(const std::exception& e)
  {
    throw std::runtime_error(std::string("MCNAOqiDCM::setStiffness(): Error on DCM getTime : ") + e.what());
  }
  
  jointStiffnessCommands[4][0]= qi::AnyReference::from(DCMtime);

  for(int i=0;i<robot_module.actuators.size();i++){
    jointStiffnessCommands[5][i][0] = qi::AnyReference::from(stiffnessValue);
  }

  try{
    dcmProxy.call<void>("setAlias", jointStiffnessCommands);
  }catch(const std::exception& e){
    throw std::runtime_error(std::string("MCNAOqiDCM::setStiffness(): Error when sending stiffness to DCM : ") + e.what());
  }
}

void MCNAOqiDCM::setJointAngles(std::vector<float> jointValues)
{
  // update values in the vector that is used to send joint commands every 12ms
  for(int i=0;i<jointValues.size();i++)
    jointPositionCommands[i] = qi::AnyReference::from(jointValues[i]);
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

std::vector<std::string> MCNAOqiDCM::bumperNames() const
{
  return robot_module.bumpers;
}

std::vector<std::string> MCNAOqiDCM::tactileSensorNames() const
{
  return robot_module.tactile;
}

std::vector<std::string> MCNAOqiDCM::wheelNames() const
{
  std::vector<std::string> wheelNames;
  for (size_t i = 0; i < robot_module.specialJointGroups.size(); i++) {
    if(robot_module.specialJointGroups[i].groupName == "wheels"){
      wheelNames = robot_module.specialJointGroups[i].jointsNames;
      break;
    }
  }
  return wheelNames;
}

// Method is not synchronized with DCM loop
// Better approach might be to call GetValues on postprocess of DCM loop
std::vector<float> MCNAOqiDCM::getSensors()
{
  // Get all values from memoryProxy using fastaccess
  for (int i = 0; i < robot_module.actuators.size(); i++)
    sensorValues[i] = memoryProxy.call<qi::AnyValue>("getData", robot_module.setActuatorKeys[i]).toFloat();
  return sensorValues;
}

void MCNAOqiDCM::connectToDCMloop()
{
  // Connect callback to the DCM pre proccess
  // try{
  //   //  onPreProcess is useful because it’s called just before the computation of orders sent to the chestboard (USB). Sending commands at this level means that you have the shortest delay to your command.
  //   fDCMPreProcessConnection =
  //       dcmProxy.call<qi::AnyObject>("atPreProcess", boost::bind(&MCNAOqiDCM::synchronisedDCMcallback, this));
  //       // what happens if I add it twice? add same callback while it is already added? try in python?
  // }catch (const std::exception &e){
  //   throw std::runtime_error(std::string("MCNAOqiDCM::connectToDCMloop(): Error when connecting to DCM preProccess: ") + e.what());
  // }

}

// using 'jointActuator' alias created 'command'
// that will use data from 'jointPositionCommands' to send it to DCM every 12 ms
void MCNAOqiDCM::synchronisedDCMcallback()
{
  int DCMtime;
  try
  {
    DCMtime = dcmProxy.call<int>("getTime", 0);
  }
  catch(const std::exception& e)
  {
    throw std::runtime_error(std::string("MCNAOqiDCM::synchronisedDCMcallback(): Error on DCM getTime : ") + e.what());
  }

  commands[4][0]= qi::AnyReference::from(DCMtime);

  // XXX make this faster with memcpy?
  for (unsigned i = 0; i < robot_module.actuators.size(); i++){
    // new actuator value = latest values from jointPositionCommands
    commands[5][i][0] = qi::AnyReference::from(jointPositionCommands[i]);
  }

  try{
    dcmProxy.call<void>("setAlias", commands);
  }catch(const std::exception& e){
    throw std::runtime_error(std::string("MCNAOqiDCM::synchronisedDCMcallback(): Error when sending command to DCM : ") + e.what());
  }
}


void MCNAOqiDCM::sayText(const std::string &toSay)
{
  try{
    qi::AnyObject ttsProxy = session_->service("ALTextToSpeech");
    ttsProxy.call<void>("say", toSay);
  }catch(const std::exception& e){
    throw std::runtime_error(std::string("MCNAOqiDCM::sayText(): Could not get proxy to ALTextToSpeech: ") + e.what());
  }
}


void MCNAOqiDCM::setLeds(std::string ledGroupName, const float &r, const float &g, const float &b)
{
  int DCMtime;
  try
  {
    DCMtime = dcmProxy.call<int>("getTime", 0);
  }
  catch(const std::exception& e)
  {
    throw std::runtime_error(std::string("MCNAOqiDCM::setLeds(): Error on DCM getTime : ") + e.what());
  }

  if(ledCmdMap.find(ledGroupName) != ledCmdMap.end()){
    std::vector<std::vector<qi::AnyValue>> &rgbCmnds = ledCmdMap[ledGroupName];

    qiLogInfo("rgbCmnds[0].size()") << rgbCmnds[0].size() << std::endl;
    qiLogInfo("rgbCmnds[0][5].size()") << rgbCmnds[0][5].size() << std::endl;

    rgbCmnds[0][4][0]= qi::AnyReference::from(DCMtime);
    rgbCmnds[1][4][0]= qi::AnyReference::from(DCMtime);
    rgbCmnds[2][4][0]= qi::AnyReference::from(DCMtime);

    // set RGB values for every memory key of this led group
    for (int i = 0; i < rgbCmnds[0][5].size(); i++){
      rgbCmnds[0][5][i][0] = qi::AnyReference::from(r);
      rgbCmnds[1][5][i][0] = qi::AnyReference::from(g);
      rgbCmnds[2][5][i][0] = qi::AnyReference::from(b);
    }

    try{
      dcmProxy.call<void>("setAlias", rgbCmnds[0]);
      dcmProxy.call<void>("setAlias", rgbCmnds[1]);
      dcmProxy.call<void>("setAlias", rgbCmnds[2]);
    }catch(const std::exception& e){
      throw std::runtime_error(std::string("MCNAOqiDCM::setLeds(): Error when sending command to DCM : ") + e.what());
    }
  }
}

void MCNAOqiDCM::setLedsDelay(std::string ledGroupName, const float &r, const float &g, const float &b, const int& delay)
{
  int DCMtime;
  try
  {
    DCMtime = dcmProxy.call<int>("getTime", 0);
  }
  catch(const std::exception& e)
  {
    throw std::runtime_error(std::string("MCNAOqiDCM::setLedsDelay(): Error on DCM getTime : ") + e.what());
  }

  if(ledCmdMap.find(ledGroupName) != ledCmdMap.end()){
    std::vector<std::vector<qi::AnyValue>> &rgbCmnds = ledCmdMap[ledGroupName];

    qiLogInfo("rgbCmnds[0].size()") << rgbCmnds[0].size() << std::endl;
    qiLogInfo("rgbCmnds[0][5].size()") << rgbCmnds[0][5].size() << std::endl;

    rgbCmnds[0][4][0]= qi::AnyReference::from(DCMtime);
    rgbCmnds[1][4][0]= qi::AnyReference::from(DCMtime);
    rgbCmnds[2][4][0]= qi::AnyReference::from(DCMtime);

    // set RGB values for every memory key of this led group
    for (int i = 0; i < rgbCmnds[0][5].size(); i++){
      rgbCmnds[0][5][i][0] = qi::AnyReference::from(r);
      rgbCmnds[1][5][i][0] = qi::AnyReference::from(g);
      rgbCmnds[2][5][i][0] = qi::AnyReference::from(b);
    }

    try{
      dcmProxy.call<void>("setAlias", rgbCmnds[0]);
      dcmProxy.call<void>("setAlias", rgbCmnds[1]);
      dcmProxy.call<void>("setAlias", rgbCmnds[2]);
    }catch(const std::exception& e){
      throw std::runtime_error(std::string("MCNAOqiDCM::setLedsDelay(): Error when sending command to DCM : ") + e.what());
    }
  }
}

void MCNAOqiDCM::isetLeds(std::string ledGroupName, const float &intensity)
{
  int DCMtime;
  try
  {
    DCMtime = dcmProxy.call<int>("getTime", 0);
  }
  catch(const std::exception& e)
  {
    throw std::runtime_error(std::string("MCNAOqiDCM::isetLeds(): Error on DCM getTime : ") + e.what());
  }

  std::vector<std::vector<qi::AnyValue>> &intensityCmnds = ledCmdMap[ledGroupName];
  // assert if intensityCmnds.size()==1, indeed single channel led group

  intensityCmnds[0][4][0]= qi::AnyReference::from(DCMtime);

  // set intensity values for every memory key of this led group
  for (int i = 0; i < intensityCmnds[0][5].size(); i++){
    intensityCmnds[0][5][i][0] = qi::AnyReference::from(intensity);
  }

  try{
      dcmProxy.call<void>("setAlias", intensityCmnds[0]);
  }catch(const std::exception& e){
    throw std::runtime_error(std::string("MCNAOqiDCM::isetLeds(): Error when sending command to DCM : ") + e.what());
  }
}

void MCNAOqiDCM::blink()
{
  // This is possible because led aliases update type is "Merge"
  setLedsDelay("eyesPeripheral", 0.0, 0.0, 0.0, 75);
  setLedsDelay("eyesPeripheral", 0.0, 0.0, 0.0, 225);
  setLedsDelay("eyesPeripheral", 1.0, 1.0, 1.0, 300);
  setLedsDelay("eyesCenter", 0.0, 0.0, 0.0, 150);
  setLedsDelay("eyesCenter", 1.0, 1.0, 1.0, 300);
}

} /* mc_naoqi_dcm */

/// Module to communicate with mc_rtc_naoqi interface for whole-body control via mc_rtc framework
/// Enables connection of a preproccess callback to DCM loop for sending joint commands every 12ms
/// Implemented for both NAO and PEPPER robots.

#include "mc_naoqi_dcm_reader.h"
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

#include <ctime>
#include <iterator>
#include <locale>
#include <string>

namespace mc_naoqi_dcm
{
MCNAOqiDCM::MCNAOqiDCM(boost::shared_ptr<AL::ALBroker> broker, const std::string &name)
    : AL::ALModule(broker, name), fMemoryFastAccess(boost::shared_ptr<AL::ALMemoryFastAccess>(new AL::ALMemoryFastAccess()))
{
  setModuleDescription("Module to communicate with mc_rtc_naoqi interface for whole-body control via mc_rtc framework");

  // Bind methods to make them accessible through proxies
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

  std::time_t time = std::time({});
  char timeString[20];
  std::strftime(timeString, sizeof(timeString), "%F-%T", std::gmtime(&time));
  logfile.open("/home/nao/naoqi/libmc_naoqi_dcm_log"+std::string(timeString)+".txt");
  
  // Get the DCM proxy
  try{
    dcmProxy = getParentBroker()->getDcmProxy();
  }catch (AL::ALError &e){
    logfile << "[MCNAOqiDCM] Impossible to create DCM Proxy :" + e.toString() + "\n";
    throw ALERROR(getName(), "MCNAOqiDCM", "Impossible to create DCM Proxy : " + e.toString());
  }

  // Get the ALMemory proxy
  try{
    memoryProxy = getParentBroker()->getMemoryProxy();
  }catch (AL::ALError &e){
    logfile << "[MCNAOqiDCM] Impossible to create ALMemory Proxy :" + e.toString() + "\n";
    throw ALERROR(getName(), "MCNAOqiDCM", "Impossible to create ALMemory Proxy : " + e.toString());
  }

  // Check that DCM is running
  signed long isDCMRunning;
  try{
    isDCMRunning = getParentBroker()->getProxy("ALLauncher")->call<bool>("isModulePresent", std::string("DCM"));
  }catch (AL::ALError &e){
    logfile << "[MCNAOqiDCM] Error when connecting to DCM :" + e.toString() + "\n";
    throw ALERROR(getName(), "MCNAOqiDCM", "Error when connecting to DCM : " + e.toString());
  }

  if (!isDCMRunning){
    logfile << "[MCNAOqiDCM] Error no DCM running\n";
    throw ALERROR(getName(), "MCNAOqiDCM", "Error no DCM running ");
  }

  // Create the fast memory access to read sensor values
  fMemoryFastAccess->ConnectToVariables(getParentBroker(), robot_module.readSensorKeys, false);
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

} /* mc_naoqi_dcm */

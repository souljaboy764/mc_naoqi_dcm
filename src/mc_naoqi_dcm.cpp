/// Module to communicate with mc_rtc_naoqi interface for whole-body control via mc_rtc framework
/// Enables connection of a preproccess callback to DCM loop for sending joint commands every 12ms
/// Implemented for both NAO and PEPPER robots.

#include <iostream>
#include <algorithm>
#include <chrono>
#include <thread>

#include "mc_naoqi_dcm.h"
#include "PepperRobotModule.h"

std::vector<std::string> colors{"Red", "Green", "Blue"};

namespace mc_naoqi_dcm
{
MCNAOqiDCM::MCNAOqiDCM(qi::SessionPtr session)
		: session_(session)
{
	// Create Pepper robot module
	robot_module = PepperRobotModule();

	// Get the DCM proxy
	try{
		dcmProxy = session_->service("DCM");
	}catch (const std::exception &e){
		throw std::runtime_error(std::string("MCNAOqiDCM::MCNAOqiDCM(): Impossible to create DCM Proxy : ") + e.what());
	}

	// Get the memoryProxy proxy
	try{
		memoryProxy = session_->service("ALMemory");
	}catch (const std::exception &e){
		throw std::runtime_error(std::string("MCNAOqiDCM::MCNAOqiDCM(): Impossible to create memoryProxy Proxy : ") + e.what());
	}

	// initialize sensor reading/setting
	init();
	getSensors();
	setStiffness(0.0f);
	setJointAngles(std::vector<float>(sensorValues.begin(), sensorValues.begin()+robot_module.actuators.size()));
}

// Module destructor
MCNAOqiDCM::~MCNAOqiDCM()
{
	setStiffness(0.0f);
}

void MCNAOqiDCM::init()
{
	// create 'jointActuator' alias to be used for sending joint possition commands
	createAliasPrepareCommand("jointActuator", robot_module.setActuatorKeys, jointPositionCommands);
	// create 'jointStiffness' alias to be used for setting joint stiffness commands
	createAliasPrepareCommand("jointStiffness", robot_module.setHardnessKeys, jointStiffnessCommands);
	sensorValues = std::vector<float>(robot_module.sensors.size());
}

// Modified from naoqi_dcm_driver
void MCNAOqiDCM::createAliasPrepareCommand(std::string aliasName,
								const std::vector<std::string> &mem_keys, 
								std::vector<std::vector<std::vector<qi::AnyValue>>> &alias_command,
								std::string updateType){
	
	// Create alias
	try
	{
		dcmProxy.call<void>("createAlias", qi::AnyValue::from<std::vector<qi::AnyValue>>({
			qi::AnyValue::from<std::string>(aliasName),
			qi::AnyValue::from<std::vector<std::string>>(mem_keys)
		}));
	}
	catch(const std::exception& e)
	{
		throw std::runtime_error(std::string("MCNAOqiDCM::createLedAlias(): Error when creating Alias :") + e.what());
	}
	
	// alias created succesfully
	// placeholder for command values
	// prepare command for this alias to be set via DCM 'setAlias' call
	alias_command = std::vector<std::vector<std::vector<qi::AnyValue>>>(
				mem_keys.size(), 
				{
					{
						qi::AnyValue::from<float>(0.), // Angle Value
						qi::AnyValue::from<int>(0) // Time
					}
				}
			);
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
	
	for(int i=0;i<robot_module.actuators.size();i++)
	{
		jointStiffnessCommands[i][0][0] = qi::AnyValue::from<float>(stiffnessValue);
		jointStiffnessCommands[i][0][1]= qi::AnyValue::from<int>(DCMtime);
	}

	try{
		dcmProxy.call<void>("setAlias", qi::AnyValue::from<std::vector<qi::AnyValue>>({
			qi::AnyValue::from<std::string>("jointStiffness"),
			qi::AnyValue::from<std::string>("ClearAll"),
			qi::AnyValue::from<std::string>("time-mixed"),
			qi::AnyValue::from<std::vector<std::vector <std::vector <qi::AnyValue>>>>(jointStiffnessCommands)
		}));
	}catch(const std::exception& e){
		throw std::runtime_error(std::string("MCNAOqiDCM::setStiffness(): Error when sending stiffness to DCM : ") + e.what());
	}
}

// update values in the vector that is used to send joint commands every 12ms
void MCNAOqiDCM::setJointAngles(std::vector<float> jointValues)
{
	int DCMtime;
	try
	{
		DCMtime = dcmProxy.call<int>("getTime", 0);
	}
	catch(const std::exception& e)
	{
		throw std::runtime_error(std::string("MCNAOqiDCM::setJointAngles(): Error on DCM getTime : ") + e.what());
	}

	for(int i=0;i<robot_module.actuators.size();i++)
	{
		jointPositionCommands[i][0][0] = qi::AnyValue::from<float>(jointValues[i]);
		jointPositionCommands[i][0][1]= qi::AnyValue::from<int>(DCMtime);
	}
	
	try
	{
		dcmProxy.call<void>("setAlias", qi::AnyValue::from<std::vector<qi::AnyValue>>({
			qi::AnyValue::from<std::string>("jointActuator"),
			qi::AnyValue::from<std::string>("ClearAll"),
			qi::AnyValue::from<std::string>("time-mixed"),
			qi::AnyValue::from<std::vector<std::vector <std::vector <qi::AnyValue>>>>(jointPositionCommands)
		}));
	}catch(const std::exception& e){
		throw std::runtime_error(std::string("MCNAOqiDCM::setJointAngles(): Error when sending command to DCM : ") + e.what());
	}
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
	// Get all values from memoryProxy using fastaccess
	for (int i = 0; i < robot_module.sensors.size(); i++)
		sensorValues[i] = memoryProxy.call<qi::AnyValue>("getData", robot_module.readSensorKeys[i]).toFloat();
	return sensorValues;
}

} /* mc_naoqi_dcm */

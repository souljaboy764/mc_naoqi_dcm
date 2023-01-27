/// Module to communicate with mc_rtc_naoqi interface for whole-body control via mc_rtc framework
/// Enables connection of a preproccess callback to DCM loop for sending joint commands every 12ms
/// Implemented for both NAO and PEPPER robots.

#include <iostream>
#include <algorithm>
#include <chrono>
#include <thread>

#include "mc_naoqi_dcm.h"
#include "NAORobotModule.h"
#include "PepperRobotModule.h"

std::vector<std::string> colors{"Red", "Green", "Blue"};

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
		memoryProxy = session_->service("ALMemory");
	}catch (const std::exception &e){
		throw std::runtime_error(std::string("MCNAOqiDCM::MCNAOqiDCM(): Impossible to create memoryProxy Proxy : ") + e.what());
	}

	// initialize sensor reading/setting
	init();
	getSensors();
	setStiffness(1.0f);
	setJointAngles(std::vector<float>(sensorValues.begin(), sensorValues.begin()+robot_module.actuators.size()));
}

// Module destructor
MCNAOqiDCM::~MCNAOqiDCM()
{
	bumperSafetyReflex(false);
	setWheelSpeed(0.0f, 0.0f, 0.0f);
	setStiffness(0.0f);
	setWheelsStiffness(0.0f);
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

void MCNAOqiDCM::onBumperPressed() {
	// Turn off wheels
	setWheelsStiffness(0.0f);
	setWheelSpeed(0.0f, 0.0f, 0.0f);
}

void MCNAOqiDCM::init()
{
	// create 'jointActuator' alias to be used for sending joint possition commands
	createAliasPrepareCommand("jointActuator", robot_module.setActuatorKeys, jointPositionCommands);
	// create 'jointStiffness' alias to be used for setting joint stiffness commands
	createAliasPrepareCommand("jointStiffness", robot_module.setHardnessKeys, jointStiffnessCommands);
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

void MCNAOqiDCM::createLedAliases()
{
	// RGB led groups
	for(int i=0;i<robot_module.rgbLedGroups.size();i++){
		const rgbLedGroup& leds = robot_module.rgbLedGroups[i];
		std::vector<std::vector<std::vector<qi::AnyValue>>> redLedCommands;
		std::vector<std::vector<std::vector<qi::AnyValue>>> greenLedCommands;
		std::vector<std::vector<std::vector<qi::AnyValue>>> blueLedCommands;
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
		std::vector<std::vector<std::vector<qi::AnyValue>>> intensityLedCommands;
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

	wheelsStiffnessCommands[0][0][0] = wheelsStiffnessCommands[1][0][0] = wheelsStiffnessCommands[2][0][0] = qi::AnyValue::from<float>(stiffnessValue);
	wheelsStiffnessCommands[0][0][1] = wheelsStiffnessCommands[1][0][1] = wheelsStiffnessCommands[2][0][1] = qi::AnyValue::from<int>(DCMtime);

	try{
		dcmProxy.call<void>("setAlias", qi::AnyValue::from<std::vector<qi::AnyValue>>({
			qi::AnyValue::from<std::string>(robot_module.specialJointGroups[0].groupName+std::string("Stiffness")),
			qi::AnyValue::from<std::string>("ClearAll"),
			qi::AnyValue::from<std::string>("time-mixed"),
			qi::AnyValue::from<std::vector<std::vector <std::vector <qi::AnyValue>>>>(wheelsStiffnessCommands)
		}));
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


	wheelsCommands[0][0][0] = qi::AnyValue::from<float>(speed_fl);
	wheelsCommands[1][0][0] = qi::AnyValue::from<float>(speed_fr);
	wheelsCommands[2][0][0] = qi::AnyValue::from<float>(speed_b);
	wheelsCommands[0][0][1] = wheelsCommands[1][0][1] = wheelsCommands[2][0][1] = qi::AnyValue::from<int>(DCMtime);

	try{
		dcmProxy.call<void>("setAlias", qi::AnyValue::from<std::vector<qi::AnyValue>>({
			qi::AnyValue::from<std::string>(robot_module.specialJointGroups[0].groupName+std::string("Speed")),
			qi::AnyValue::from<std::string>("ClearAll"),
			qi::AnyValue::from<std::string>("time-mixed"),
			qi::AnyValue::from<std::vector<std::vector <std::vector <qi::AnyValue>>>>(wheelsStiffnessCommands)
		}));
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
	for (int i = 0; i < robot_module.sensors.size(); i++)
		sensorValues[i] = memoryProxy.call<qi::AnyValue>("getData", robot_module.readSensorKeys[i]).toFloat();
	return sensorValues;
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
	setLedsDelay(ledGroupName, r, g, b, 0.);
}

void MCNAOqiDCM::setLedsDelay(std::string ledGroupName, const float &r, const float &g, const float &b, const int& delay)
{
	if(ledCmdMap.find(ledGroupName) != ledCmdMap.end())
	{
		int DCMtime;
		try
		{
			DCMtime = dcmProxy.call<int>("getTime", delay);
		}
		catch(const std::exception& e)
		{
			throw std::runtime_error(std::string("MCNAOqiDCM::setLedsDelay(): Error on DCM getTime : ") + e.what());
		}

		std::vector<std::vector<std::vector<std::vector<qi::AnyValue>>>> &rgbCmnds = ledCmdMap[ledGroupName];

		qiLogInfo("rgbCmnds.size()") << rgbCmnds.size() << std::endl;
		qiLogInfo("rgbCmnds[0].size()") << rgbCmnds[0].size() << std::endl;


		// set RGB values for every memory key of this led group
		for (int i = 0; i < rgbCmnds[0].size(); i++){
			rgbCmnds[0][i][0][0] = qi::AnyValue::from<float>(r);
			rgbCmnds[1][i][0][0] = qi::AnyValue::from<float>(g);
			rgbCmnds[2][i][0][0] = qi::AnyValue::from<float>(b);
			rgbCmnds[0][i][0][1] = rgbCmnds[1][i][0][1] = rgbCmnds[2][i][0][1] = qi::AnyValue::from<int>(DCMtime);
		}

		try{
			for (int i = 0; i < 3; i++)
				dcmProxy.call<void>("setAlias", qi::AnyValue::from<std::vector<qi::AnyValue>>({
					qi::AnyValue::from<std::string>(ledGroupName + colors[i]),
					qi::AnyValue::from<std::string>("Merge"),
					qi::AnyValue::from<std::string>("time-mixed"),
					qi::AnyValue::from<std::vector<std::vector <std::vector <qi::AnyValue>>>>(rgbCmnds[i])
				}));
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

	std::vector<std::vector<std::vector<std::vector<qi::AnyValue>>>> &intensityCmnds = ledCmdMap[ledGroupName];
	// assert if intensityCmnds.size()==1, indeed single channel led group


	// set intensity values for every memory key of this led group
	for (int i = 0; i < intensityCmnds[0].size(); i++)
	{
		intensityCmnds[0][i][0][0] = qi::AnyValue::from<float>(intensity);
		intensityCmnds[0][i][0][1]= qi::AnyValue::from<int>(DCMtime);
	}

	try{
		dcmProxy.call<void>("setAlias", qi::AnyValue::from<std::vector<qi::AnyValue>>({
			qi::AnyValue::from<std::string>(ledGroupName),
			qi::AnyValue::from<std::string>("Merge"),
			qi::AnyValue::from<std::string>("time-mixed"),
			qi::AnyValue::from<std::vector<std::vector <std::vector <qi::AnyValue>>>>(intensityCmnds[0])
		}));
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

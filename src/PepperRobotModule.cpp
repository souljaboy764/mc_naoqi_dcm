#include "PepperRobotModule.h"

namespace mc_naoqi_dcm
{
PepperRobotModule::PepperRobotModule() : RobotModule()
{
  name = "pepper";

  // body joints
  actuators = {"LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand",
   	"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand"};

  // generate memory keys for sending commands to the joints (position/stiffness)
  genMemoryKeys("", actuators, "/Position/Actuator/Value", setActuatorKeys);
  genMemoryKeys("", actuators, "/Hardness/Actuator/Value", setHardnessKeys);

  // generate memory keys for reading sensor values
  // NB! Joint encoders must be in the beginning of the readSensorKeys/sensors
  genMemoryKeys("", actuators, "/Position/Sensor/Value", readSensorKeys, true, "Encoder");
  genMemoryKeys("", actuators, "/ElectricCurrent/Sensor/Value", readSensorKeys, true, "ElectricCurrent");
}

} /* mc_naoqi_dcm */

// Program to test the basic DCM functionality to see whether it is actually working.
// If this program is not working, there is some problem with the Pepper.


#include <vector>
#include <string>

#include <qi/anyobject.hpp>
#include <qi/anyvalue.hpp>
#include <qi/applicationsession.hpp>
#include <qi/session.hpp>

int main(int argc, char* argv[])
{
	// Connect to the Robot
	qi::ApplicationSession app(argc, argv);
	app.start();
	qi::SessionPtr session = app.session();
	
	// Wake the robot up and set stiffness
	qi::AnyObject motion = session->service("ALMotion");
	motion.call<void>("setBreathEnabled", "Body", false);
	if(!motion.call<bool>("robotIsWakeUp"))
	{
		motion.call<void>("wakeUp");
		motion.call<void>("setStiffnesses", "Body", 1.0);
	}

	// Create a DCM Alias
	qi::AnyObject dcm = session->service("DCM");
	dcm.call<void>("createAlias", qi::AnyValue::from<std::vector<qi::AnyValue>>({
		qi::AnyValue::from<std::string>("jointActuator"),
		qi::AnyValue::from<std::vector<std::string>>({
			"Device/SubDeviceList/RWristYaw/Position/Actuator/Value", 
			"Device/SubDeviceList/LWristYaw/Position/Actuator/Value"
		})
	}));

	// Send commands via DCM
	int t = dcm.call<int>("getTime", 0);
	dcm.call<void>("setAlias", qi::AnyValue::from<std::vector<qi::AnyValue>>({
		qi::AnyValue::from<std::string>("jointActuator"),
		qi::AnyValue::from<std::string>("ClearAll"),
		qi::AnyValue::from<std::string>("time-mixed"),
		qi::AnyValue::from<std::vector<std::vector <std::vector <qi::AnyValue>>>>({
			{
				{qi::AnyValue::from<float>(1.6), qi::AnyValue::from<int>(t)}
			},
			{
				{qi::AnyValue::from<float>(-1.6), qi::AnyValue::from<int>(t)}
			}
		})
	}));

	qi::os::sleep(2);

	t = dcm.call<int>("getTime", 0);
	dcm.call<void>("setAlias", qi::AnyValue::from<std::vector<qi::AnyValue>>({
		qi::AnyValue::from<std::string>("jointActuator"),
		qi::AnyValue::from<std::string>("ClearAll"),
		qi::AnyValue::from<std::string>("time-mixed"),
		qi::AnyValue::from<std::vector<std::vector <std::vector <qi::AnyValue>>>>({
			{
				{qi::AnyValue::from<float>(0.), qi::AnyValue::from<int>(t)}
			},
			{
				{qi::AnyValue::from<float>(0.), qi::AnyValue::from<int>(t)}
			}
		})
	}));

	qi::os::sleep(2);

	t = dcm.call<int>("getTime", 0);
	dcm.call<void>("setAlias", qi::AnyValue::from<std::vector<qi::AnyValue>>({
		qi::AnyValue::from<std::string>("jointActuator"),
		qi::AnyValue::from<std::string>("ClearAll"),
		qi::AnyValue::from<std::string>("time-mixed"),
		qi::AnyValue::from<std::vector<std::vector <std::vector <qi::AnyValue>>>>({
			{
				{qi::AnyValue::from<float>(-1.6), qi::AnyValue::from<int>(t)}
			},
			{
				{qi::AnyValue::from<float>(1.6), qi::AnyValue::from<int>(t)}
			}
		})
	}));

	qi::os::sleep(2);

	t = dcm.call<int>("getTime", 0);
	dcm.call<void>("setAlias", qi::AnyValue::from<std::vector<qi::AnyValue>>({
		qi::AnyValue::from<std::string>("jointActuator"),
		qi::AnyValue::from<std::string>("ClearAll"),
		qi::AnyValue::from<std::string>("time-mixed"),
		qi::AnyValue::from<std::vector<std::vector <std::vector <qi::AnyValue>>>>({
			{
				{qi::AnyValue::from<float>(0.), qi::AnyValue::from<int>(t)}
			},
			{
				{qi::AnyValue::from<float>(0.), qi::AnyValue::from<int>(t)}
			}
		})
	}));

	qi::os::sleep(2);
	motion.call<void>("setStiffnesses", "Body", 0.0);
	session->close();
	
	return 0;
}

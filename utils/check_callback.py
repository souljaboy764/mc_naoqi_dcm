# Disactivate safety reflexes
# First, go to http://pepper.local/advanced/#/settings to enable the disactivation

import qi
import sys
import time

# Connect to Naoqi session
session = qi.Session()
try:
    session.connect("tcp://192.168.100.165:9559")
except RuntimeError:
    print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
           "Please check your script arguments. Run with -h option for help.")
    sys.exit(1)


motion  = session.service("ALMotion")
motion.setBreathEnabled("Body", False)
if not motion.robotIsWakeUp():
    motion.wakeUp()
    # motion.setStiffnesses("Body", 1.0)
motion.setStiffnesses("Body", 0.0)
# Access the module
mcnaoqidcm_service  = session.service("MCNAOqiDCM")

# Check if the callback is connected to DCM loop
# mcnaoqidcm_service.startLoop()
# for i in range(50):
#     mcnaoqidcm_service.setStiffness(0.5*i/50)
# time.sleep(1)
# print("Is callback connected to DCM: " + str(mcnaoqidcm_service.isPreProccessConnected()))
print("Sensors: " + str(mcnaoqidcm_service.getJointOrder()))

# Move wrists 
print(mcnaoqidcm_service.getSensors())
# values[4] = -1.6
# values[5] = 1
# values[10] = 1.6
# values[11] = 1
# mcnaoqidcm_service.setJointAngles(values[:12])
# time.sleep(1)


# for i in reversed(range(50)):
#     mcnaoqidcm_service.setStiffness(0.5*i/50)
# mcnaoqidcm_service.stopLoop()
# Disactivate safety reflexes
# First, go to http://pepper.local/advanced/#/settings to enable the disactivation

import qi
import sys

# Connect to Naoqi session
session = qi.Session()
try:
    session.connect("tcp://192.168.100.184:9559")
except RuntimeError:
    print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
           "Please check your script arguments. Run with -h option for help.")
    sys.exit(1)


motion  = session.service("ALMotion")
motion.setBreathEnabled("Body", False)
if not motion.robotIsWakeUp():
    motion.wakeUp()
    motion.setStiffnesses("Body", 1.0)
# Access the module
mcnaoqidcm_service  = session.service("MCNAOqiDCM")

# Check if the callback is connected to DCM loop
print("Is callback connected to DCM: " + str(mcnaoqidcm_service.isPreProccessConnected()))

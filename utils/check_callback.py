# Disactivate safety reflexes
# First, go to http://pepper.local/advanced/#/settings to enable the disactivation

import qi
import sys

# Connect to Naoqi session
session = qi.Session()
try:
    session.connect("tcp://192.168.100.172:9559")
except RuntimeError:
    print ("Can't connect to Naoqi at 192.168.100.172:9559.\n"
           "Please check your script arguments. Run with -h option for help.")
    sys.exit(1)

# Access the module
mcnaoqidcm_service  = session.service("MCNAOqiDCM")
print(session.services())
print(mcnaoqidcm_service)
# Check if the callback is connected to DCM loop
# print("Is callback connected to DCM: " + str(mcnaoqidcm_service.getSensors()))
print("Is callback connected to DCM: ", mcnaoqidcm_service.setJointAngles([-0.0061359405517578125, -0.027611732482910156, -0.007669925689697266, 1.5707, 0.4451, 1.5661944150924683, 0.14266014099121094, -1.2011067867279053, -0.4939417839050293, -0.08901405334472656, 0.6818981170654297, 1.5677282810211182, -0.14266014099121094, 1.2011070251464844, 0.49240779876708984, 0.13034796714782715, 0.6757469177246094, 0.320000022649765, 0.5760000348091125, 0.12800000607967377, 0.09600000083446503, 0.19200000166893005, 0.1600000113248825, 0.0, 0.1600000113248825]))

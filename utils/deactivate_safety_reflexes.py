# Deactivate safety reflexes
# First, go to http://pepper.local/advanced/#/settings to enable the deactivation

import qi
import sys
import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--ip", help="IP Address of Pepper", required=True)
parser.add_argument("--port", help="Port of Pepper (default: 9559)", default="9559")
args = parser.parse_args()

# Connect to Naoqi session
session = qi.Session()
try:
    session.connect("tcp://"+args.ip+":"+args.port)
except RuntimeError:
    print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + args.port +".\n"
           "Please check your script arguments. Run with -h option for help.")
    sys.exit(1)

# Get the service ALAutonomousLife.
life_service  = session.service("ALAutonomousLife")
life_service.setState("disabled")

# Get the service ALMotion.
motion_service  = session.service("ALMotion")

# Deactivate safety reflexes
motion_service.setCollisionProtectionEnabled("RArm", False)
motion_service.setCollisionProtectionEnabled("LArm", False)
motion_service.setDiagnosisEffectEnabled(False)
motion_service.setSmartStiffnessEnabled(False)
motion_service.setExternalCollisionProtectionEnabled("All", False)
motion_service.setFallManagerEnabled(False)
motion_service.setPushRecoveryEnabled(False)

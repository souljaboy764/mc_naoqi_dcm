# Disactivate safety reflexes
# First, go to http://pepper.local/advanced/#/settings to enable the disactivation

import qi
import sys
import time
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

motion  = session.service("ALMotion")
motion.setBreathEnabled("Body", False)
if not motion.robotIsWakeUp():
    motion.wakeUp()
    motion.setStiffnesses("Body", 1.0)

# Access the module
mcnaoqidcm_service  = session.service("MCNAOqiDCM")
mcnaoqidcm_service.setStiffness(1.0)
# Check if the callback is connected to DCM loop
sensorsOrder = mcnaoqidcm_service.getSensorsOrder()
sensorValues = mcnaoqidcm_service.getSensors()
assert len(sensorsOrder) == len(sensorValues)
for i in range(len(sensorsOrder)):
    print(sensorsOrder[i], sensorValues[i])

# print("Is callback connected to DCM: ", mcnaoqidcm_service.setJointAngles([0.0, -0.023009777069091797, -0.009203910827636719, 0., 0.2225, 1.7656118869781494, 0.07363104820251465, -1.7180583477020264, -0.11351466178894043, -1.5907998085021973, 0.6810193061828613, 1.7395341396331787, -0.07363104820251465, 1.699650764465332, 0.10431075096130371, -1.5892658233642578, 0.6757469177246094, 0.7040000557899475, 1.0880000591278076, 0.1600000113248825, 0.1600000113248825, 0.4480000138282776, 0.0, 0.09600000083446503, 0.09600000083446503, 0.0, 0.0, 0.01600000075995922, 0.06400000303983688, 0.06400000303983688, 0.0, 0.06400000303983688, 0.0, 0.03200000151991844, -0.21076172590255737, -0.1341211050748825, -9.92496109008789, 0.0007989483419805765, 0.0007989483419805765, -0.001065264455974102, -0.010737720876932144, -0.016681846231222153, 0.0070946612395346165, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0][:17]))
while True:
    sensorValues = mcnaoqidcm_service.getSensors()
    print(sensorValues[-3])
    time.sleep(0.01)
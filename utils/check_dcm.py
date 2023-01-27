# Program to test the basic DCM functionality to see whether it is actually working.
# If this program is not working, there is some problem with the Pepper.

import qi
import sys
import time

# Connect to the Robot
app = qi.Application(sys.argv)
app.start()
session = app.session

# Wake the robot up and set stiffness
motion  = session.service("ALMotion")
motion.setBreathEnabled("Body", False)
if not motion.robotIsWakeUp():
    motion.wakeUp()
    motion.setStiffnesses("Body", 1.0)

# Create a DCM Alias
dcm  = session.service("DCM")
dcm.createAlias([
    "jointActuator",
    [
        "Device/SubDeviceList/RWristYaw/Position/Actuator/Value",
        "Device/SubDeviceList/LWristYaw/Position/Actuator/Value"
    ]
])

# Send commands via DCM
t = dcm.getTime(0)
dcm.setAlias([
        "jointActuator",
        "ClearAll",
        "time-mixed",
        [
            [
                [1.6, t],
            ],
            [
                [-1.6, t],
            ]
        ]
    ])

time.sleep(2)

t = dcm.getTime(0)
dcm.setAlias([
        "jointActuator",
        "ClearAll",
        "time-mixed",
        [
            [
                [0., t],
            ],
            [
                [0., t],
            ]
        ]
    ])

time.sleep(2)

t = dcm.getTime(0)
dcm.setAlias([
        "jointActuator",
        "ClearAll",
        "time-mixed",
        [
            [
                [-1.6, t],
            ],
            [
                [1.6, t],
            ]
        ]
    ])

time.sleep(2)

t = dcm.getTime(0)
dcm.setAlias([
        "jointActuator",
        "ClearAll",
        "time-mixed",
        [
            [
                [0., t],
            ],
            [
                [0., t],
            ]
        ]
    ])

time.sleep(2)
motion.setStiffnesses("Body", 0.0)
session.close()
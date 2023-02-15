# Disactivate safety reflexes
# First, go to http://pepper.local/advanced/#/settings to enable the disactivation

import qi
import sys
import time
import datetime
import numpy as np

import matplotlib.pyplot as plt
import matplotlib.colors as colors
import matplotlib.cm as cmx

# Connect to Naoqi session
session = qi.Session()
remote = "tcp://192.168.100.184:9559"
try:
    session.connect(remote)
except RuntimeError:
    print ("Can't connect to Naoqi at ip " + remote + "\n"
           "Please check your script arguments. Run with -h option for help.")
    sys.exit(1)


motion  = session.service("ALMotion")
motion.setBreathEnabled("Body", False)
if not motion.robotIsWakeUp():
    motion.wakeUp()
# motion.setStiffnesses("Body", 1.0)
# motion.setStiffnesses("Body", 0.0)
# Access the module
mcnaoqidcm_service  = session.service("MCNAOqiDCM")

# Check if the callback is connected to DCM loop
mcnaoqidcm_service.startLoop()
# for i in range(100):
#     motion.setStiffnesses("Body", (i+1)/100)
#     time.sleep(0.01)
# time.sleep(1)
print("Is callback connected to DCM: " + str(mcnaoqidcm_service.isPreProccessConnected()))
fig = plt.figure()
ax_res = fig.add_subplot(1,2,1)
ax_cur = fig.add_subplot(1,2,2)
plt.ion()
jet = cm = plt.get_cmap('jet') 
cNorm  = colors.Normalize(vmin=0, vmax=6)
scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=jet)


start = datetime.datetime.now()
times = [0.0]
vals = []
currents = []
init_values = np.array(mcnaoqidcm_service.getSensors()[:12])
prev_values = np.array(mcnaoqidcm_service.getSensors()[:12])
target = np.array(mcnaoqidcm_service.getSensors()[:12])
vals = np.zeros_like(init_values)
currents = np.zeros_like(init_values)
z = np.zeros_like(init_values).astype(bool)
o = np.ones_like(init_values).astype(bool)

torque_constants = np.array([27.5, 19.2, 27.5, 19.2, 21.2, 21.2, 27.5, 19.2, 27.5, 19.2, 21.2, 21.2])
joints_names = mcnaoqidcm_service.getJointOrder()
while True:
    sensor_vals = np.array(mcnaoqidcm_service.getSensors())
    values = sensor_vals[:12]
    residual = values - prev_values
    vals = np.vstack([vals, residual])
    currents = np.vstack([currents, sensor_vals[12:]])
    times.append((datetime.datetime.now() - start).total_seconds())
    
    if len(vals)>50:
        vals = vals[1:]
        currents = currents[1:]
        times.pop(0)
    
    ax_res.clear()
    ax_cur.clear()
    for i in range(6):
        ax_res.plot(times, vals[:, 6+i], linestyle='--', linewidth=5, color=scalarMap.to_rgba(i))
        ax_cur.plot(times, torque_constants[6+i]*currents[:, 6+i], linestyle='-', linewidth=5, color=scalarMap.to_rgba(i), label=joints_names[6+i])
    plt.legend()
    plt.pause(0.001)
    if not plt.fignum_exists(1):
        break
    prev_values = values
    target = np.where(np.abs(residual)>0.02, values, target)
    # mcnaoqidcm_service.setJointAngles(target.tolist())
    # motion.setAngles(mcnaoqidcm_service.getJointOrder(), target.tolist(), 0.7)

# for i in reversed(range(100)):
#     motion.setStiffnesses("Body", i/100)
#     time.sleep(0.01)
# motion.setStiffnesses("Body", 0.0)
time.sleep(1)
mcnaoqidcm_service.stopLoop()
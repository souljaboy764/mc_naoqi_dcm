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

import kdl_parser_py.urdf
import PyKDL as kdl

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
motion.setStiffnesses("Arms", 1.0)
# Access the module
mcnaoqidcm_service  = session.service("MCNAOqiDCM")

# Check if the callback is connected to DCM loop
fig = plt.figure()
ax_res = fig.add_subplot(2,2,1)
ax_cur = fig.add_subplot(2,2,2)
ax_val = fig.add_subplot(2,2,3)
ax_grav = fig.add_subplot(2,2,4)
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
residuals = np.zeros_like(init_values)
currents = np.zeros_like(init_values)
q_dot = np.zeros_like(init_values)[:5]
q_ddot = np.zeros_like(init_values)[:5]
gravity_compensating_jt_torques = np.zeros_like(init_values)[:5]
z = np.zeros_like(init_values).astype(bool)
o = np.ones_like(init_values).astype(bool)

torque_constants = np.array([27.5, 19.2, 27.5, 19.2, 21.2, 21.2, 27.5, 19.2, 27.5, 19.2, 21.2, 21.2])
joints_names = mcnaoqidcm_service.getJointOrder()

urdf_file = "/home/laptop/pepper_ws/src/pepper_robot/pepper_description/urdf/pepper1.0_generated_urdf/pepper.urdf"
(ok, tree) = kdl_parser_py.urdf.treeFromFile(urdf_file)
chain = tree.getChain("torso", "r_wrist")
grav_vector = kdl.Vector(0, 0, -9.81)
dyn_kdl = kdl.ChainDynParam(chain, grav_vector)
jt_positions = kdl.JntArray(chain.getNrOfSegments())
jt_positions[0] = 0.0
jt_positions[1] = 0.0
jt_positions[2] = 0.0
grav_matrix = kdl.JntArray(chain.getNrOfSegments())

while True:
    sensor_vals = np.array(mcnaoqidcm_service.getSensors())
    values = sensor_vals[:12]
    residual = values - prev_values
    residuals = np.vstack([residuals, residual])
    vals = np.vstack([vals, values])
    currents = np.vstack([currents, sensor_vals[12:]])
    times.append((datetime.datetime.now() - start).total_seconds())
    
    if len(vals)>50:
        vals = vals[1:]
        residuals = residuals[1:]
        currents = currents[1:]
        gravity_compensating_jt_torques = gravity_compensating_jt_torques[1:]
        times.pop(0)

    for i in range(5):
        jt_positions[i] = values[6+i]
    dyn_kdl.JntToGravity(jt_positions, grav_matrix)
    gravity_compensating_jt_torques = np.vstack([gravity_compensating_jt_torques, [grav_matrix[i] for i in range(grav_matrix.rows())]])
    
    ax_res.clear()
    ax_cur.clear()
    ax_val.clear()
    ax_grav.clear()
    # ax_res.plot(times, np.ones_like(times)*0.02, linestyle='-', linewidth=5, color='r')
    # ax_res.plot(times, np.ones_like(times)*-0.02, linestyle='-', linewidth=5, color='r', )
    for i in range(5):
        ax_res.plot(times, residuals[:, 6+i], linestyle='--', linewidth=5, color=scalarMap.to_rgba(i), label=joints_names[6+i])
        ax_cur.plot(times, currents[:, 6+i], linestyle='-', linewidth=5, color=scalarMap.to_rgba(i), label=joints_names[6+i])
        ax_val.plot(times, vals[:, 6+i], linestyle='--', linewidth=5, color=scalarMap.to_rgba(i), label=joints_names[6+i])
        ax_grav.plot(times, np.abs(gravity_compensating_jt_torques[:, i]), linestyle='--', linewidth=5, color=scalarMap.to_rgba(i), label=joints_names[6+i])
    # plt.legend()
    plt.pause(0.001)
    if not plt.fignum_exists(1):
        break
    prev_values = values #0.7 * prev_values + 0.3 * values
    target = np.where(np.abs(residual)>0.02, values, target)
    # motion.setAngles(mcnaoqidcm_service.getJointOrder(), target.tolist(), 1.0)

time.sleep(1)

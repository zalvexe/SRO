import time
import math
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.require('sim')

sim.startSimulation()
print("Simulation Started")
sim.addLog(1, "Python is tracking velocities...")

p3dx_RW = sim.getObject("/PioneerP3DX/rightMotor")
p3dx_LW = sim.getObject("/PioneerP3DX/leftMotor")
initial_euler = sim.getObjectOrientation(sim.getObject("/PioneerP3DX"), -1)

#parameter Robot
rw = 0.195 / 2  
rb = 0.381 / 2  

t_data = []
wr_data = []
wl_data = []
vx_data = []
wx_data = []
x_odom_abs = []
y_odom_abs = []
x_odom_intg = []
y_odom_intg = []

x_dot_abs_data = []
y_dot_abs_data = []
x_dot_intg_data = []
y_dot_intg_data = []

theta_dot_data = []
dt = 0.01
x_pose = 0
y_pose = 0
gamma_pose = initial_euler[2]
gamma = 0.1

x_pose_abs = 0
y_pose_abs = 0
x_pose_intg = 0
y_pose_intg = 0
SAMPLE_TIME = 0.05
dt = SAMPLE_TIME
try:
    start_time = time.time()
    elapsed_prev = 0
    while (time.time() - start_time) < 20:
        loop_start = time.time()
        current_time = time.time() - start_time #ini jadi elapsed
        # dt = current_time - t_data[-1] if t_data else 0
        # dt = current_time - elapsed_prev
        dt = current_time - elapsed_prev if elapsed_prev > 0 else SAMPLE_TIME
        elapsed_prev = current_time

        wr_vel = sim.getJointVelocity(p3dx_RW)
        wl_vel = sim.getJointVelocity(p3dx_LW)
        
        vx = (wr_vel + wl_vel) * rw / 2 
        wx = (wr_vel - wl_vel) * rw / (2 * rb) 
        euler_angle = sim.getObjectOrientation(sim.getObject("/PioneerP3DX"), -1) 
        gamma_pose = gamma_pose + wx * dt

        #kalo pake sensor orientasi yng absolut:
        x_dot_abs = vx * math.cos(euler_angle[2])  
        y_dot_abs = vx * math.sin(euler_angle[2])  

        x_pose_abs = x_pose_abs + x_dot_abs * dt
        y_pose_abs = y_pose_abs + y_dot_abs * dt

        #untuk posisi diintegralkan
        x_dot_integral = vx * math.cos(gamma_pose)
        y_dot_integral = vx * math.sin(gamma_pose)

        x_pose_intg = x_pose_intg + x_dot_integral * dt
        y_pose_intg = y_pose_intg + y_dot_integral * dt

        x_odom_abs.append(x_pose_abs)
        y_odom_abs.append(y_pose_abs)

        x_odom_intg.append(x_pose_intg)
        y_odom_intg.append(y_pose_intg)
        
        sim.addLog(1, f"RW:{wr_vel:.1f}, LW:{wl_vel:.1f} | Vx:{vx:.1f}m/s, Wx:{wx:.1f}rad/s")
        # sim.addLog(1, f"Body Velocities -> x_dot: {x_dot:.2f} m/s, y_dot: {y_dot:.2f} m/s)")  
                  # #, theta_dot: {y_dot:.2f} rad/s")
        # sim.addLog(1, f"Pose -> x: {x_pose:.2f} m, y: {y_pose:.2f} m, gamma: {gamma_pose:.2f}, dt: {dt:.2f}")  #, theta: {theta_dot:.2f} rad")
        sim.addLog(1, f"Pose Abs -> x: {x_pose_abs:.2f} m, y: {y_pose_abs:.2f} m")  #, theta: {theta_dot:.2f} rad")
        sim.addLog(1, f"Pose Intg -> x: {x_pose_intg:.2f} m, y: {y_pose_intg:.2f} m")  #, theta: {theta_dot:.2f} rad")

        t_data.append(current_time)
        wr_data.append(wr_vel)
        wl_data.append(wl_vel)
        vx_data.append(vx)
        wx_data.append(wx)
        x_dot_abs_data.append(x_dot_abs)
        y_dot_abs_data.append(y_dot_abs)
        x_dot_intg_data.append(x_dot_integral)
        y_dot_intg_data.append(y_dot_integral)
        # theta_dot_data.append(theta_dot)
        elapsed_loop = time.time() - loop_start
        sleep_time = SAMPLE_TIME - elapsed_loop
        if sleep_time > 0:
            time.sleep(sleep_time)

        # time.sleep(0.05) # Sampling rate
        

finally:
    sim.stopSimulation()
    print("\nSimulation Stopped")

# --- PLOTTING ---
plt.figure(figsize=(10, 5))
plt.plot(x_odom_abs, y_odom_abs, 'b-', label='Odometry Path (Absolute)', color='purple')
plt.plot(x_odom_intg, y_odom_intg, 'r-', label='Odometry Path (Integrated)', color='blue')
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.title('P3DX Odometry Path')
plt.legend()
plt.grid(True)
plt.show()

# plt.figure(figsize=(10, 5))
# plt.plot(x_odom_intg, y_odom_intg, 'r-', label='Odometry Path (Integrated)', color='blue')
# plt.xlabel('X Position (m)')
# plt.ylabel('Y Position (m)')
# plt.title('P3DX Odometry Path')
# plt.legend()
# plt.grid(True)
# plt.show()
# # Plot 1: Temporal plot of P3DX joint velocity
# plt.figure(figsize=(10, 5))
# plt.plot(t_data, wr_data, label=r'Right Wheel ($\dot{\phi}_R$)', color='blue')
# plt.plot(t_data, wl_data, label=r'Left Wheel ($\dot{\phi}_L$)', color='red')
# plt.title('Temporal Plot of P3DX Joint Velocity')
# plt.xlabel('Time (sec)')
# plt.ylabel('Angular Velocity (rad/s)')
# plt.legend()
# plt.grid(True)
# plt.savefig('joint_velocity.png') # Otomatis save gambar untuk dilampirkan ke PDF
# plt.show()

# # Plot 2: Temporal plot of P3DX body velocity
# plt.figure(figsize=(10, 5))
# plt.plot(t_data, vx_data, label=r'Linear Velocity ($V_x$)', color='green')
# plt.plot(t_data, wx_data, label=r'Angular Velocity ($\omega$)', color='purple')
# plt.title('Temporal Plot of P3DX Body Velocity')
# plt.xlabel('Time (sec)')
# plt.ylabel('Velocity (m/s & rad/s)')
# plt.legend()
# plt.grid(True)
# plt.savefig('body_velocity.png')
# plt.show()

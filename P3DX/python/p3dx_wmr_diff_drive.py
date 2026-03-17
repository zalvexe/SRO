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

#ambil Handle Motor sesuai hierarchy di CoppeliaSim
p3dx_RW = sim.getObject("/PioneerP3DX/rightMotor")
p3dx_LW = sim.getObject("/PioneerP3DX/leftMotor")

#parameter Robot
rw = 0.195 / 2  #wheel radius
rb = 0.381 / 2  #body radius (setengah jarak antar roda)

#list untuk menyimpan data plotting
t_data = []
wr_data = []
wl_data = []
vx_data = []
wx_data = []

try:
    #main Loop (dijalankan 20 detik agar robot sempat belok menghindari tembok)
    start_time = time.time()
    while (time.time() - start_time) < 20:
        current_time = time.time() - start_time
        
        # Ambil nilai kecepatan joint
        # Catatan: Slide menggunakan getJointTargetVelocity, tapi untuk 
        # membaca kecepatan putaran fisik yang aktual, getJointVelocity lebih akurat.
        wr_vel = sim.getJointVelocity(p3dx_RW)
        wl_vel = sim.getJointVelocity(p3dx_LW)
        
        # Hitung Body Velocity dengan Kinematika Differential Drive standar
        vx = (wr_vel + wl_vel) * rw / 2 
        wx = (wr_vel - wl_vel) * rw / (2 * rb) 
        
        # Tampilkan log di status bar CoppeliaSim
        sim.addLog(1, f"RW:{wr_vel:.1f}, LW:{wl_vel:.1f} | Vx:{vx:.1f}m/s, Wx:{wx:.1f}rad/s")
        
        # Simpan data ke array
        t_data.append(current_time)
        wr_data.append(wr_vel)
        wl_data.append(wl_vel)
        vx_data.append(vx)
        wx_data.append(wx)
        
        time.sleep(0.05) # Sampling rate

finally:
    # 5. Stop Simulation safely
    sim.stopSimulation()
    print("\nSimulation Stopped")

# --- ASSIGNMENT PLOTTING ---

# Plot 1: Temporal plot of P3DX joint velocity
plt.figure(figsize=(10, 5))
plt.plot(t_data, wr_data, label=r'Right Wheel ($\dot{\phi}_R$)', color='blue')
plt.plot(t_data, wl_data, label=r'Left Wheel ($\dot{\phi}_L$)', color='red')
plt.title('Temporal Plot of P3DX Joint Velocity')
plt.xlabel('Time (sec)')
plt.ylabel('Angular Velocity (rad/s)')
plt.legend()
plt.grid(True)
plt.savefig('joint_velocity.png') # Otomatis save gambar untuk dilampirkan ke PDF
plt.show()

# Plot 2: Temporal plot of P3DX body velocity
plt.figure(figsize=(10, 5))
plt.plot(t_data, vx_data, label=r'Linear Velocity ($V_x$)', color='green')
plt.plot(t_data, wx_data, label=r'Angular Velocity ($\omega$)', color='purple')
plt.title('Temporal Plot of P3DX Body Velocity')
plt.xlabel('Time (sec)')
plt.ylabel('Velocity (m/s & rad/s)')
plt.legend()
plt.grid(True)
plt.savefig('body_velocity.png')
plt.show()
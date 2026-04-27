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

robot_penjaga_RW = sim.getObject("/Robot_Pemain/rightMotor")
robot_penjaga_LW = sim.getObject("/Robot_Pemain/leftMotor")
initial_euler_penjaga = sim.getObjectOrientation(sim.getObject("/Robot_Pemain"), -1)

robot_lawan_01_RW = sim.getObject("/Robot_Lawan_01/rightMotor")
robot_lawan_01_LW = sim.getObject("/Robot_Lawan_01/leftMotor")
initial_euler_lawan_01 = sim.getObjectOrientation(sim.getObject("/Robot_Lawan_01"), -1)

robot_lawan_02_RW = sim.getObject("/Robot_Lawan_02/rightMotor")
robot_lawan_02_LW = sim.getObject("/Robot_Lawan_02/leftMotor")
initial_euler_lawan_02 = sim.getObjectOrientation(sim.getObject("/Robot_Lawan_02"), -1)

#parameter Robot
rw = 0.195 / 2
rb = 0.381 / 2

goal_gawang = [[5.125, 0.925],[5.075, -1.125]]
robot_gawang_arena = [[3.1, 2.3], [5.075, 2.272], [5.075, -2.272], [3.1, -2.3]]

arena_x_min = min(point[0] for point in robot_gawang_arena)
arena_x_max = max(point[0] for point in robot_gawang_arena)
arena_y_min = min(point[1] for point in robot_gawang_arena)
arena_y_max = max(point[1] for point in robot_gawang_arena)
penjaga_home_x = arena_x_min + 0.15
blocked_x = arena_x_min - 0.15

gamma_pose_penjaga = initial_euler_penjaga[2]
gamma_pose_lawan_01 = initial_euler_lawan_01[2]
gamma_pose_lawan_02 = initial_euler_lawan_02[2]

SAMPLE_TIME = 0.05
dt = SAMPLE_TIME
prev_pos_lawan_01 = None
stuck_time_lawan_01 = 0.0
unstuck_time_lawan_01 = 0.0

try:
    start_time = time.time()
    elapsed_prev = 0
    while (time.time() - start_time) < 90:
        loop_start = time.time()
        current_time = time.time() - start_time #ini jadi elapsed
        dt = current_time - elapsed_prev if elapsed_prev > 0 else SAMPLE_TIME
        elapsed_prev = current_time

        wr_penjaga_vl = sim.getJointVelocity(robot_penjaga_RW)
        wl_penjaga_vl = sim.getJointVelocity(robot_penjaga_LW)

        vx_penjaga = (wr_penjaga_vl + wl_penjaga_vl) * rw / 2
        wx_penjaga = (wr_penjaga_vl - wl_penjaga_vl) * rw / (2*rb)

        p3dx_pos_penjaga = sim.getObjectPosition(sim.getObject("/Robot_Pemain"), -1)
        p3dx_euler_penjaga = sim.getObjectOrientation(sim.getObject("/Robot_Pemain"), -1)
        gamma_pose_penjaga = gamma_pose_penjaga + wx_penjaga * dt

        wr_lawan_01_vl = sim.getJointVelocity(robot_lawan_01_RW)
        wl_lawan_01_vl = sim.getJointVelocity(robot_lawan_01_LW)

        vx_lawan_01 = (wr_lawan_01_vl + wl_lawan_01_vl) * rw / 2
        wx_lawan_01 = (wr_lawan_01_vl - wl_lawan_01_vl) * rw / (2*rb)

        p3dx_pos_lawan_01 = sim.getObjectPosition(sim.getObject("/Robot_Lawan_01"), -1)
        p3dx_euler_lawan_01 = sim.getObjectOrientation(sim.getObject("/Robot_Lawan_01"), -1)
        gamma_pose_lawan_01 = gamma_pose_lawan_01 + wx_lawan_01 * dt

        wr_lawan_02_vl = sim.getJointVelocity(robot_lawan_02_RW)
        wl_lawan_02_vl = sim.getJointVelocity(robot_lawan_02_LW)

        vx_lawan_02 = (wr_lawan_02_vl + wl_lawan_02_vl) * rw / 2
        wx_lawan_02 = (wr_lawan_02_vl - wl_lawan_02_vl) * rw / (2*rb)

        p3dx_pos_lawan_02 = sim.getObjectPosition(sim.getObject("/Robot_Lawan_02"), -1)
        p3dx_euler_lawan_02 = sim.getObjectOrientation(sim.getObject("/Robot_Lawan_02"), -1)
        gamma_pose_lawan_02 = gamma_pose_lawan_02 + wx_lawan_02 * dt

        bola_merah = sim.getObjectPosition(sim.getObject("/Bola_Merah"), -1)

        t_world_robot_penjaga = np.array([[math.cos(p3dx_euler_penjaga[2]), -math.sin(p3dx_euler_penjaga[2]), p3dx_pos_penjaga[0]],
                                        [math.sin(p3dx_euler_penjaga[2]), math.cos(p3dx_euler_penjaga[2]), p3dx_pos_penjaga[1]],
                                        [0, 0, 1]])
        t_world_robot_lawan_01 = np.array([[math.cos(p3dx_euler_lawan_01[2]), -math.sin(p3dx_euler_lawan_01[2]), p3dx_pos_lawan_01[0]],
                                        [math.sin(p3dx_euler_lawan_01[2]), math.cos(p3dx_euler_lawan_01[2]), p3dx_pos_lawan_01[1]],
                                        [0, 0, 1]])
        t_world_robot_lawan_02 = np.array([[math.cos(p3dx_euler_lawan_02[2]), -math.sin(p3dx_euler_lawan_02[2]), p3dx_pos_lawan_02[0]],
                                        [math.sin(p3dx_euler_lawan_02[2]), math.cos(p3dx_euler_lawan_02[2]), p3dx_pos_lawan_02[1]],
                                        [0, 0, 1]])
        
        sphere_pos_robot_penjaga = np.linalg.inv(t_world_robot_penjaga) @ np.array([[bola_merah[0]], [bola_merah[1]], [1]])
        sphere_pos_robot_lawan_01 = np.linalg.inv(t_world_robot_lawan_01) @ np.array([[bola_merah[0]], [bola_merah[1]], [1]])
        sphere_pos_robot_lawan_02 = np.linalg.inv(t_world_robot_lawan_02) @ np.array([[bola_merah[0]], [bola_merah[1]], [1]])

        t_world_sphere_penjaga = np.array([[math.cos(p3dx_euler_penjaga[2]), -math.sin(p3dx_euler_penjaga[2]), bola_merah[0]],
                                        [math.sin(p3dx_euler_penjaga[2]), math.cos(p3dx_euler_penjaga[2]), bola_merah[1]],
                                        [0, 0, 1]])
        t_world_sphere_lawan_01 = np.array([[math.cos(p3dx_euler_lawan_01[2]), -math.sin(p3dx_euler_lawan_01[2]), bola_merah[0]],
                                        [math.sin(p3dx_euler_lawan_01[2]), math.cos(p3dx_euler_lawan_01[2]), bola_merah[1]],
                                        [0, 0, 1]])
        t_world_sphere_lawan_02 = np.array([[math.cos(p3dx_euler_lawan_02[2]), -math.sin(p3dx_euler_lawan_02[2]), bola_merah[0]],
                                        [math.sin(p3dx_euler_lawan_02[2]), math.cos(p3dx_euler_lawan_02[2]), bola_merah[1]],
                                        [0, 0, 1]])
        
        sim.addLog(1, f"Posisi bola terhadap Robot penjaga: x={sphere_pos_robot_penjaga[0][0]:.2f}, y={sphere_pos_robot_penjaga[1][0]:.2f}")
        sim.addLog(1, f"Posisi bola terhadap Robot Lawan 01: x={sphere_pos_robot_lawan_01[0][0]:.2f}, y={sphere_pos_robot_lawan_01[1][0]:.2f}")
        sim.addLog(1, f"Posisi bola terhadap Robot Lawan 02: x={sphere_pos_robot_lawan_02[0][0]:.2f}, y={sphere_pos_robot_lawan_02[1][0]:.2f}")

        # buat penjaga gawang tetap di sekitar arena gawang
        penjaga_target_x = blocked_x
        penjaga_target_y = max(arena_y_min, min(bola_merah[1], arena_y_max))
        penjaga_error_world_x = penjaga_target_x - p3dx_pos_penjaga[0]
        penjaga_error_world_y = penjaga_target_y - p3dx_pos_penjaga[1]

        cos_theta = math.cos(p3dx_euler_penjaga[2])
        sin_theta = math.sin(p3dx_euler_penjaga[2])
        penjaga_error_robot_x = cos_theta * penjaga_error_world_x + sin_theta * penjaga_error_world_y
        penjaga_error_robot_y = -sin_theta * penjaga_error_world_x + cos_theta * penjaga_error_world_y

        vx_target_penjaga = max(-0.6, min(0.6, 1.2 * penjaga_error_robot_x))
        wx_target_penjaga = max(-1.5, min(1.5, 3.0 * math.atan2(penjaga_error_robot_y, max(0.001, penjaga_error_robot_x))))
        if abs(penjaga_error_robot_x) < 0.15 and abs(penjaga_error_robot_y) < 0.1:
            vx_target_penjaga = 0.0
            wx_target_penjaga = 0.0
        
        # lawan 1: posisikan diri di belakang bola supaya dorongan mengarah ke tengah gawang
        lawan_01_ball_x = sphere_pos_robot_lawan_01[0][0]
        lawan_01_ball_y = sphere_pos_robot_lawan_01[1][0]
        lawan_01_ball_dist = math.hypot(lawan_01_ball_x, lawan_01_ball_y)
        lawan_01_ball_heading = math.atan2(lawan_01_ball_y, max(0.001, lawan_01_ball_x))

        goal_center_x = 0.5 * (goal_gawang[0][0] + goal_gawang[1][0])
        goal_center_y = 0.5 * (goal_gawang[0][1] + goal_gawang[1][1])

        ball_to_goal_x = goal_center_x - bola_merah[0]
        ball_to_goal_y = goal_center_y - bola_merah[1]
        ball_to_goal_norm = max(0.001, math.hypot(ball_to_goal_x, ball_to_goal_y))
        ball_to_goal_unit_x = ball_to_goal_x / ball_to_goal_norm
        ball_to_goal_unit_y = ball_to_goal_y / ball_to_goal_norm

        behind_ball_offset = 0.28
        lawan_01_target_world_x = bola_merah[0] - behind_ball_offset * ball_to_goal_unit_x
        lawan_01_target_world_y = bola_merah[1] - behind_ball_offset * ball_to_goal_unit_y

        lawan_01_error_world_x = lawan_01_target_world_x - p3dx_pos_lawan_01[0]
        lawan_01_error_world_y = lawan_01_target_world_y - p3dx_pos_lawan_01[1]
        lawan_01_cos_theta = math.cos(p3dx_euler_lawan_01[2])
        lawan_01_sin_theta = math.sin(p3dx_euler_lawan_01[2])
        lawan_01_target_robot_x = lawan_01_cos_theta * lawan_01_error_world_x + lawan_01_sin_theta * lawan_01_error_world_y
        lawan_01_target_robot_y = -lawan_01_sin_theta * lawan_01_error_world_x + lawan_01_cos_theta * lawan_01_error_world_y
        lawan_01_target_heading = math.atan2(lawan_01_target_robot_y, max(0.001, lawan_01_target_robot_x))

        vx_target_lawan_01 = max(-0.55, min(0.6, 1.5 * lawan_01_target_robot_x))
        wx_target_lawan_01 = max(-1.6, min(1.6, 3.0 * lawan_01_target_heading))

        # Saat dekat bola, dorong ke arah goal center (bukan sekadar lurus depan robot).
        if lawan_01_ball_dist < 0.42:
            push_heading_world = math.atan2(ball_to_goal_y, max(0.001, ball_to_goal_x))
            push_heading_robot = push_heading_world - p3dx_euler_lawan_01[2]
            push_heading_robot = math.atan2(math.sin(push_heading_robot), math.cos(push_heading_robot))
            vx_target_lawan_01 = max(0.35, min(0.75, 0.55 + 0.6 * lawan_01_ball_x))
            wx_target_lawan_01 = max(-1.2, min(1.2, 2.5 * push_heading_robot))

        move_delta_lawan_01 = 0.0
        if prev_pos_lawan_01 is not None:
            move_delta_lawan_01 = math.hypot(
                p3dx_pos_lawan_01[0] - prev_pos_lawan_01[0],
                p3dx_pos_lawan_01[1] - prev_pos_lawan_01[1]
            )

        if unstuck_time_lawan_01 > 0.0:
            unstuck_time_lawan_01 = max(0.0, unstuck_time_lawan_01 - dt)
            turn_sign = -1.0 if lawan_01_ball_heading > 0 else 1.0
            vx_target_lawan_01 = -0.6
            wx_target_lawan_01 = 1.4 * turn_sign
        else:
            pushing_command = abs(vx_target_lawan_01) > 0.2 or abs(wx_target_lawan_01) > 0.7
            if pushing_command and lawan_01_ball_dist > 0.2 and move_delta_lawan_01 < 0.003:
                stuck_time_lawan_01 += dt
            else:
                stuck_time_lawan_01 = max(0.0, stuck_time_lawan_01 - 0.5 * dt)

            if stuck_time_lawan_01 > 0.45:
                unstuck_time_lawan_01 = 0.6
                stuck_time_lawan_01 = 0.0

        # Sudah ditangani oleh satu recovery anti-stuck di atas.

        #  lawan 2 juga ngejar bola, tapi lebih lambat dari lawan 1
        lawan_02_ball_x = sphere_pos_robot_lawan_02[0][0]
        lawan_02_ball_y = sphere_pos_robot_lawan_02[1][0]
        lawan_02_ball_dist = math.hypot(lawan_02_ball_x, lawan_02_ball_y)
        lawan_02_ball_heading = math.atan2(lawan_02_ball_y, max(0.001, lawan_02_ball_x))

        vx_target_lawan_02 = max(-0.15, min(0.15, 0.5 * lawan_02_ball_x))
        wx_target_lawan_02 = max(-0.325, min(0.325, 1.2 * lawan_02_ball_heading))

        if abs(lawan_02_ball_heading) > 0.8:
            vx_target_lawan_02 = max(-0.3, min(0.3, vx_target_lawan_02))

        if lawan_02_ball_dist < 0.18:
            vx_target_lawan_02 = 0.0
            wx_target_lawan_02 = 0.0

        wr_target_penjaga = (vx_target_penjaga + (rb * wx_target_penjaga)/2) / rw
        wl_target_penjaga = (vx_target_penjaga - (rb * wx_target_penjaga)/2) / rw

        wr_target_lawan_01 = (vx_target_lawan_01 + (rb * wx_target_lawan_01)/2) / rw
        wl_target_lawan_01 = (vx_target_lawan_01 - (rb * wx_target_lawan_01)/2) / rw

        wr_target_lawan_02 = (vx_target_lawan_02 + (rb * wx_target_lawan_02)/2) / rw
        wl_target_lawan_02 = (vx_target_lawan_02 - (rb * wx_target_lawan_02)/2) / rw

        sim.setJointTargetVelocity(robot_penjaga_RW, wr_target_penjaga)
        sim.setJointTargetVelocity(robot_penjaga_LW, wl_target_penjaga)

        sim.setJointTargetVelocity(robot_lawan_01_RW, wr_target_lawan_01)
        sim.setJointTargetVelocity(robot_lawan_01_LW, wl_target_lawan_01)

        sim.setJointTargetVelocity(robot_lawan_02_RW, wr_target_lawan_02)
        sim.setJointTargetVelocity(robot_lawan_02_LW, wl_target_lawan_02)

        prev_pos_lawan_01 = [p3dx_pos_lawan_01[0], p3dx_pos_lawan_01[1]]

finally:
    sim.stopSimulation()
    print("Simulation Stopped")
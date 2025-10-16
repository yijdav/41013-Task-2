import numpy as np
from ir_support import UR3
from spatialmath import SE3
from spatialgeometry import Cuboid, Cylinder, Mesh
from roboticstoolbox import DHLink, DHRobot, jtraj, PrismaticDH
from math import pi
import swift 
from pathlib import Path
import matplotlib.pyplot as plt
import keyboard
import spatialmath.base as spb
import trimesh
import roboticstoolbox as rtb
import os
import time
from kuka_ropbot import Kuka
from abb import abb
#from Cobot280 import myCobot280
from AssessmentTwo import myCobot280
import pygame
# -----------------------------------------------------------------------------------#

    # -----------------------------------------------------
    # ---------------- Robot joint sliders ----------------
    # -----------------------------------------------------
# ---------- Control buttons: also enable joystick on selection ----------
def select_kuka(_=None): 
    active["robot"] = r1
    joystick_enabled["v"] = True
    print("Controlling: Kuka")
def select_abb(_=None):
    active["robot"] = r2
    joystick_enabled["v"] = True
    print("Controlling: ABB")
def select_ur3(_=None):
    active["robot"] = r3
    joystick_enabled["v"] = True
    print("Controlling: UR3")
def select_cobot(_=None):
    active["robot"] = r4.robot
    joystick_enabled["v"] = True
    print("Controlling: myCobot280")

def _set_joint_deg(robot, j, deg): # fetches the robot and sets the jth joint to deg degrees
    q = list(robot.q)
    qmin, qmax = -pi, pi
    q[j] = np.clip(np.deg2rad(float(deg)), qmin, qmax)
    robot.q = q

def slider_cb(value_deg, joint_index): # callback for sliders, sets the joint of the active robot
    rob = active["robot"]
    if joint_index >= getattr(rob, "n", len(getattr(rob, "links", []))):
        return
    _set_joint_deg(rob, joint_index, value_deg)
    env.step(0)

def robot_joint_control():
    # Control buttons for active robot selection
    env.add(swift.Button(desc="Control Kuka", cb=select_kuka))
    env.add(swift.Button(desc="Control ABB", cb=select_abb))
    env.add(swift.Button(desc="Control UR3", cb=select_ur3))
    env.add(swift.Button(desc="Control myCobot", cb=select_cobot))

    shared_sliders = []
    for i in range(6):
        init_deg = 0.0
        try:
            init_deg = float(np.rad2deg(active["robot"].q[i]))
        except Exception:
            pass
        s = swift.Slider(
            cb=lambda v, j=i: slider_cb(v, j),
            min=-180, max=180, step=1,
            value=init_deg,
            desc=f"Joint {i+1} (Active Robot)",
            unit="&#176;",
        )
        env.add(s)
        shared_sliders.append(s)
    return shared_sliders


    # ---------------------------------------------------------------------
    # ---------------- controller manipulation of end effector ------------
    # ---------------------------------------------------------------------

def _clamp_to_qlim(robot, q):
    qq = np.array(q, dtype=float)
    n = len(qq)
    for j in range(n):
        qmin, qmax = -pi, pi
        qq[j] = np.clip(qq[j], qmin, qmax)
    return qq

def _dls_step(robot, dx, lam_=0.1, dq_limit=2.0):
    if not hasattr(robot, "jacob0"):
        return np.zeros(getattr(robot, "n", 6))
    J = robot.jacob0(robot.q)  # 6xN
    JTJ = J.T @ J
    N = JTJ.shape[0]
    dq = np.linalg.solve(JTJ + (lam_**2) * np.eye(N), J.T @ dx)
    return np.clip(dq, -dq_limit, dq_limit)

def _joy_axis(v, th):
    return 0.0 if abs(v) < th else v




if __name__ == "__main__":
    env = swift.Swift()
    env.launch(realtime=True)
    #--------------------------------------------ENVIRONMENT--------------------------------------------#
    sca=0.1
    workshop = Mesh("Environmental_models/workshop.stl", scale=[sca, sca, sca])
    env.add(workshop)
    #--------------------------------------------ROBOTS--------------------------------------------#
    r1 = Kuka()
    r2 = abb()
    r3 = UR3()
    r4pos = SE3(0,1,0)
    r4 = myCobot280(r4pos) 
    r1.base = SE3(0.5, 0.5, 0)
    env.add(r1)
    r2.base = SE3(0, 0, 0)
    env.add(r2)
    r3.base = SE3(0, 2, 0)
    r3.add_to_env(env)
    env.add(r4.robot)
    env.add(r4.base_mesh)
    #--------------------------------------------Tester--------------------------------------------#
    # Create trajectories for three robots
    steps = 50
    q1 = rtb.jtraj(r1.q, [joint - pi/4 for joint in r1.q], steps).q
    q2 = rtb.jtraj(r2.q, [joint - pi/4 for joint in r2.q], steps).q
    q3 = rtb.jtraj(r3.q, [joint - 0.8 for joint in r3.q], steps).q
    q4 = rtb.jtraj(r4.robot.q, [joint - 0.8 for joint in r4.robot.q], steps).q

    for i in range(steps):
        r1.q = q1[i]
        r2.q = q2[i]
        r3.q = q3[i]
        r4.robot.q = q4[i]
        env.step(0.05)


    active = {"robot": r1}  # set Kuka as the default active robot



    
    robot_joint_control()
    """Add flag check to only run the sliders once the robots have completed their tasks"""

    # --- Joystick setup (disabled until a robot is selected) ---
    pygame.init()
    pygame.joystick.init()
    joy = None
    if pygame.joystick.get_count() > 0:
        joy = pygame.joystick.Joystick(0)
        joy.init()
        print(f"Joystick: {joy.get_name()} | Buttons={joy.get_numbuttons()} Axes={joy.get_numaxes()}")
    else:
        print("No joystick")

    joystick_enabled = {"v": False}  # becomes True when a Control button is pressed

    
    # Joystick params
    dt_joy = 0.02
    Kv = 0.3      # linear gain
    Kw = 0.8      # angular gain
    lam = 0.1     # DLS damping
    deadzone = 0.1
    button_gain = 0.5





    t_prev = time.time()
    while True:
        # Joystick-driven end-effector velocity control of the active robot
        now = time.time()
        dt_loop = now - t_prev
        t_prev = now

        if joystick_enabled["v"]:
            pygame.event.pump()
            axes = [ _joy_axis(joy.get_axis(i), deadzone) for i in range(joy.get_numaxes()) ]
            buttons = [ joy.get_button(i) for i in range(joy.get_numbuttons()) ]

            # Map joystick -> spatial velocity [vx, vy, vz, wx, wy, wz]
            vx = Kv * (axes[0] if len(axes) > 0 else 0.0)                       # left X
            vy = -Kv * (axes[1] if len(axes) > 1 else 0.0)                      # left Y (invert)
            vz = Kv * button_gain * ((buttons[3] if len(buttons)>3 else 0) - (buttons[0] if len(buttons)>0 else 0))  # TRIANGLE - CROSS

            wx = Kw * (axes[2] if len(axes) > 2 else 0.0)                       # right X
            wy = Kw * (axes[3] if len(axes) > 3 else 0.0)                       # right Y
            wz = Kw * button_gain * ((buttons[2] if len(buttons)>2 else 0) - (buttons[1] if len(buttons)>1 else 0))  # SQUARE - CIRCLE

            dx = np.array([vx, vy, vz, wx, wy, wz], dtype=float)

            rob = active["robot"]
            dq = _dls_step(rob, dx, lam_=lam)
            q_new = np.array(rob.q, dtype=float) + dq * dt_joy
            q_new = _clamp_to_qlim(rob, q_new)
            rob.q = q_new

        env.step(0)
        time.sleep(0.01)
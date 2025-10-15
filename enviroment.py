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
# -----------------------------------------------------------------------------------#

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
    r4 = myCobot280(r4pos) #Jordan gotta fix his robot cause its fucking shit up
    r1.base = SE3(0.5, 0.5, 0)
    env.add(r1)
    r2.base = SE3(0, 0, 0)
    env.add(r2)
    r3.base = SE3(0, 2, 0)
    r3.add_to_env(env)
    env.add(r4.robot)
    env.add(r4.base_mesh)

    # # Create trajectories for three robots
    # steps = 50
    # q1 = rtb.jtraj(r1.q, [joint - pi/4 for joint in r1.q], steps).q
    # q2 = rtb.jtraj(r2.q, [joint - pi/4 for joint in r2.q], steps).q
    # q3 = rtb.jtraj(r3.q, [joint - 0.8 for joint in r3.q], steps).q
    # q4 = rtb.jtraj(r4.robot.q, [joint - 0.8 for joint in r4.robot.q], steps).q

    # for i in range(steps):
    #     r1.q = q1[i]
    #     r2.q = q2[i]
    #     r3.q = q3[i]
    #     r4.robot.q = q4[i]
    #     env.step(0.05)

# ---------------- Robot joint sliders ----------------
    active = {"robot": r1}  # start controlling Kuka
    print("Sliders are always active. Use the Control buttons to switch the active robot.")

    def select_kuka(_=None):
        active["robot"] = r1
        print("Controlling: Kuka")

    def select_abb(_=None):
        active["robot"] = r2
        print("Controlling: ABB")

    def select_ur3(_=None):
        active["robot"] = r3
        print("Controlling: UR3")

    def select_cobot(_=None):
        active["robot"] = r4.robot
        print("Controlling: myCobot280")


    def _set_joint_deg(robot, j, deg):
        q = list(robot.q)
        qmin, qmax = -pi, pi  # Default limits
        q[j] = np.clip(np.deg2rad(float(deg)), qmin, qmax)
        robot.q = q

    # Slider callback (shared for all robots) - always active
    def slider_cb(value_deg, joint_index):
        rob = active["robot"]
        # Guard joint index if robot has fewer than 6 DoF
        if joint_index >= getattr(rob, "n", len(getattr(rob, "links", []))):
            return
        _set_joint_deg(rob, joint_index, value_deg)
        env.step(0)

    # Control buttons for active robot selection
    env.add(swift.Button(desc="Control Kuka", cb=select_kuka))
    env.add(swift.Button(desc="Control ABB", cb=select_abb))
    env.add(swift.Button(desc="Control UR3", cb=select_ur3))
    env.add(swift.Button(desc="Control myCobot", cb=select_cobot))

    # Build 6 shared sliders (UI min/max use ±180; we clamp in slider_cb)
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

    while True:
        env.step(0)
        time.sleep(0.01)
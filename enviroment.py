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
# -----------------------------------------------------------------------------------#

if __name__ == "__main__":
    r1 = Kuka()
    r2 = abb()
    r3 = UR3()
    # r4 = myCobot280() #Jordan gotta fix his robot cause its fucking shit up
    env = swift.Swift()
    env.launch(realtime=True)


    r1.base = SE3(0.5, 0.5, 0)
    env.add(r1)

    r2.base = SE3(0, -2.5, 0)
    env.add(r2)

    r3.base = SE3(0, 2, 0)
    r3.add_to_env(env)



    # Create trajectories for three robots
    steps = 50
    q1 = rtb.jtraj(r1.q, [joint + pi/4 for joint in r1.q], steps).q
    q2 = rtb.jtraj(r2.q, [joint - pi/4 for joint in r2.q], steps).q
    q3 = rtb.jtraj(r3.q, [joint - 0.8 for joint in r3.q], steps).q

    for i in range(steps):
        r1.q = q1[i]
        r2.q = q2[i]
        r3.q = q3[i]
        env.step(0.05)
    env.hold()
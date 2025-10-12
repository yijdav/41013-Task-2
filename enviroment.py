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
    r = Kuka()
    # r = abb()
    env = swift.Swift()
    env.launch(realtime=True)
    r.base = SE3(0.5, 0.5, 0)
    env.add(r)

    env.hold()
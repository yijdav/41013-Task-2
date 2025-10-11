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

# -----------------------------------------------------------------------------------#
class Kuka(DHRobot):
    #https://github.com/ros-industrial/kuka_experimental
    def __init__(self):
        links = [
            DHLink(a=0,      alpha=0, d=0, offset=0),
            DHLink(a=0.35,  alpha=pi/2,    d=0.74,     offset=0),
            DHLink(a=1.25,  alpha=0,    d=0.05,     offset=0),
            DHLink(a=0,  alpha=pi/2, d=0, offset=0),
            DHLink(a=0,      alpha=-pi/2, d=-0.05,     offset=0),
            DHLink(a=1.1,      alpha=0,    d=0, offset=0)
        ]
        mesh_dir = "kuka150mesh"
        mesh_files = [
            "base_link.stl",
            "link_1.stl",
            "link_2.stl",
            "link_3.stl",
            "link_4.stl",
            "link_5.stl"
        ]
        # Example transforms for each mesh (adjust as needed for your STL alignment)
        mesh_transforms = [
            SE3(),
            SE3().Rx(-pi/2)*SE3(-0.35,-0.1,0),
            SE3().Rx(-pi/2)*SE3(-1.25,0,0),
            SE3(),
            SE3().Ry(-pi/2).Rx(-pi/2),
            SE3()
        ]
        sca = 1.0  # Scale factor for the meshes
        for i, link in enumerate(links):
            mesh_path = f"{mesh_dir}/{mesh_files[i]}"
            link.geometry = [Mesh(mesh_path, scale=[sca, sca, sca], pose=mesh_transforms[i])]
        DHRobot.__init__(self, links, name='KUKA')
        # Set a test joint configuration for visualization
        self.q = [0, -pi/2, 0, 0, 0, 0]
        self._qtest = [0,-pi/2,0,0,0,0]


    # -----------------------------------------------------------------------------------#
    def test(self):
        """
        Test the class by adding 3d objects into a new Swift window and do a simple movement
        """
        env = swift.Swift()
        env.launch(realtime= True)
        self.q = self._qtest
        self.base = SE3(0.5,0,0)
        env.add(self)

        joint = 2  # Change this to test different joints (0 to 5)
        q_goal = self.q.copy()
        q_goal[joint] = self.q[joint] - 6 * pi  

        qtraj = rtb.jtraj(self.q, q_goal, 300).q
        for q in qtraj:
            self.q = q
            env.step(0.02)

            
        env.hold()

# ---------------------------------------------------------------------------------------#
if __name__ == "__main__":
    Kuka().test()

    # env = swift.Swift()
    # env.launch(realtime=True)
    # r = Kuka()
    # r.base = SE3(0, 0, 0)
    # env.add(r)

    # env.hold()
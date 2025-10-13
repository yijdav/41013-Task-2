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

# ------------------------------------------- ----------------------------------------#
class abb(DHRobot):
    #https://github.com/ros-industrial/abb/tree/kinetic-devel/abb_crb15000_support/meshes/crb15000_5_95
    def __init__(self):
        links = [
            DHLink(a=0,      alpha=0, d=0, offset=0),
            DHLink(a=0,  alpha=pi/2,    d=0.26,     offset=0),
            DHLink(a=0,  alpha=-pi,    d=0,     offset=0),
            DHLink(a=0,  alpha=pi/2 , d=0, offset=pi/2),
            DHLink(a=0,      alpha=0, d=0.11,     offset=-pi/2),
            DHLink(a=0.46,      alpha=0,    d=0, offset=0)
        ]
        mesh_dir = "abb_robot_mesh"
        mesh_files = [
            "base_link (1).stl",
            "link_1 (1).stl",
            "link_2 (1).stl",
            "link_3 (1).stl",
            "link_4 (1).stl",
            "link_5 (1).stl"
        ]
        # Example transforms for each mesh (adjust as needed for your STL alignment)
        mesh_transforms = [
            SE3(),
            SE3().Rx(-pi/2),
            SE3().Rx(pi/2)*SE3(0,0,0),
            SE3().Ry(-pi/2)*SE3(0,0,0),
            SE3(),
            SE3()
        ]
        sca = 1.0  # Scale factor for the meshes
        for i, link in enumerate(links):
            mesh_path = f"{mesh_dir}/{mesh_files[i]}"
            print(f"Trying to load mesh: {mesh_path}")
            try:
                link.geometry = [Mesh(mesh_path, scale=[sca, sca, sca], pose=mesh_transforms[i])]
                print(f"Loaded mesh: {mesh_path}")
            except Exception as e:
                print(f"Failed to load mesh {mesh_path}: {e}")
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
        self.base = SE3(0,0,0)
        env.add(self)

        joint = 3  # Change this to test different joints (0 to 5)
        q_goal = self.q.copy()
        q_goal[joint] = self.q[joint] - 6 * pi  

        qtraj = rtb.jtraj(self.q, q_goal, 300).q
        for q in qtraj:
            self.q = q
            env.step(0.02)

            
        env.hold()

    def testAllJoints(self):
        """
        Test the class by adding 3d objects into a new Swift window and do a simple movement
        """
        env = swift.Swift()
        env.launch(realtime= True)
        self.q = self._qtest
        self.base = SE3(0.5,0.5,0)
        env.add(self)

        q_goal = [self.q[i]-pi/3 for i in range(self.n)]
        qtraj = rtb.jtraj(self.q, q_goal, 50).q
        # fig = self.plot(self.q)
        for q in qtraj:
            self.q = q
            env.step(0.02)
            # fig.step(0.01)
        time.sleep(3)
        env.hold()

# ---------------------------------------------------------------------------------------#
if __name__ == "__main__":
    env = swift.Swift()
    env.launch(realtime=True)
    r = abb()
    r.base = SE3(0, 0, 0)
    env.add(r)

    env.hold()

# ABB Robot class
import numpy as np
from spatialmath import SE3
from spatialgeometry import Mesh
from roboticstoolbox import DHLink, DHRobot, jtraj, PrismaticDH
from math import pi
import swift 
from pathlib import Path
import matplotlib.pyplot as plt
import keyboard
import spatialmath.base as spb
import roboticstoolbox as rtb
import time
from swift import Button

# ------------------------------------------- ----------------------------------------#
class abb(DHRobot):
    #https://github.com/ros-industrial/abb/tree/kinetic-devel/abb_crb15000_support/meshes/crb15000_5_95
    def __init__(self):
        sca = 1  # Scale factor for the meshes
        links = [
            DHLink(a=0,      alpha=0, d=0, offset=0),
            DHLink(a=0,  alpha=pi/2,    d=0.266*sca,     offset=0),
            DHLink(a=0.44*sca,  alpha=-pi,    d=0,     offset=pi/2),
            DHLink(a=0.11*sca,  alpha=-pi/2 , d=0, offset=0),
            DHLink(a=0,      alpha=pi/2, d=0.47*sca,     offset=0),
            DHLink(a=0,      alpha=0,    d=0, offset=0)
        ]
        mesh_dir = "abbMeshesUpdated"
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
            SE3().Rx(-pi/2)*SE3(0,0,-0.079*sca),
            SE3().Rx(pi/2)*SE3().Ry(pi/2)*SE3(0,-0.08*sca,-0.44*sca),
            SE3().Ry(-pi/2)*SE3().Rx(pi)*SE3(0,-0.08*sca,-0.11*sca),
            SE3().Rz(pi/2)*SE3().Rx(pi/2)*SE3(-0.37*sca,0,0),
            SE3().Rx(pi/2)*SE3().Ry(pi/2)*SE3(0,0.08*sca,0)
        ]

        abb_colors = [
            [0.35, 0.35, 0.35, 1.0],   # Darker gray base
            [0.42, 0.43, 0.45, 1.0],   # Slightly bluish gray
            [0.48, 0.48, 0.48, 1.0],   # Medium gray
            [0.38, 0.39, 0.42, 1.0],   # Darker bluish gray
            [0.45, 0.45, 0.45, 1.0],   # Medium-light gray
            [0.32, 0.32, 0.32, 1.0]    # Dark gray
        ]

        # Add joint limits for each link
        links[0].qlim = [-pi, pi]      # Joint 1: ±180°
        links[1].qlim = [-pi, pi]      # Joint 2: ±180°
        links[2].qlim = [-pi/2, pi/2]      # Joint 3: ±90°
        links[3].qlim = [-pi, pi]      # Joint 4: ±180°
        links[4].qlim = [-pi, pi]      # Joint 5: ±180°
        links[5].qlim = [-pi, pi]      # Joint 6: ±180°
        
        for i, link in enumerate(links):
            mesh_path = f"{mesh_dir}/{mesh_files[i]}"
            link.geometry = [Mesh(mesh_path, scale=[sca, sca, sca], pose=mesh_transforms[i], color = abb_colors[i])]

        DHRobot.__init__(self, links, name='KUKA')
        # Set a test joint configuration for visualization
        self.q = [0, -pi/2, 0, 0, 0, 0]
        self._qtest = [0,-pi/2,0,0,0,0]

    def set_joint(self, j, value):
        """Set joint value from slider (value in degrees)"""
        q = list(self.q)  # Create a copy of current joint values
        q[j] = np.deg2rad(float(value))  # Convert degrees to radians
        self.q = q

    def add_sliders(self, env):
        """Add interactive sliders for joint control"""
        j = 0
        for link in self.links:
            if link.isjoint:
                env.add(
                    swift.Slider(
                        lambda x, j=j: self.set_joint(j, x),
                        min=np.round(np.rad2deg(link.qlim[0]), 2),
                        max=np.round(np.rad2deg(link.qlim[1]), 2),
                        step=1,
                        value=np.round(np.rad2deg(self.q[j]), 2),
                        desc=" Joint " + str(j),
                        unit="&#176;",
                    )
                )
                j += 1

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

        joint = 4  # Change this to test different joints (0 to 5)
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

    def partFinding(self, part):
        """
        Function to find a part in the environment and move the robot to it
        """
        pass

# ---------------------------------------------------------------------------------------#
if __name__ == "__main__":
    env = swift.Swift()
    env.launch(realtime=True)
    r = abb()
    r.base = SE3(0, 0, 0)
    env.add(r)

    bolt = Mesh("Environmental_models\ImageToStl.com_M9x12+screw+without+head.stl", scale=[2, 2, 2])
    env.add(bolt)
    
    r.add_sliders(env)

    button_pressed = {'value': False}
    # Define the callback
    def on_button_press(_):
        button_pressed['value'] = True
        print("Button pressed")
    # Create and add the button
    test_button = swift.Button(cb=on_button_press, desc="Select Robot")
    env.add(test_button)
    
    while True:
        env.step(0)
        time.sleep(0.01)
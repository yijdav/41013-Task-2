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
import pygame
# -----------------------------------------------------------------------------------#
class Kuka(DHRobot):
    #https://github.com/ros-industrial/kuka_experimental
    def __init__(self):
        sca = 0.3
        links = [
            DHLink(a=0,      alpha=0, d=0, offset=0),
            DHLink(a=0.35*sca,  alpha=-pi/2,    d=0.74*sca,     offset=0),
            DHLink(a=1.25*sca,  alpha=pi,    d=0.05*sca,     offset=0),
            DHLink(a=-0.07*sca,  alpha=pi/2, d=0, offset=pi/2),
            DHLink(a=0,      alpha=-pi/2, d=1.1*sca,     offset=0),
            DHLink(a=0,      alpha=0,    d=0, offset=0)
        ]
        mesh_dir = "kuka150mesh"
        mesh_files = [
            "base_link.stl",
            "link_1.stl",
            "link_2.stl",
            "link_3.stl",
            "link_4_fixed_origin.stl",
            "link_5.stl"
        ]

        # Add joint limits for each link
        links[0].qlim = [-pi, pi]      # Joint 1: ±180°
        links[1].qlim = [-pi, pi]      # Joint 2: ±180°
        links[2].qlim = [-pi, 0]      # Joint 3: ±180°
        links[3].qlim = [-pi, pi]      # Joint 4: ±180°
        links[4].qlim = [-pi, pi]      # Joint 5: ±180°
        links[5].qlim = [-pi, pi]      # Joint 6: ±180°
    

        yellow_colors = [
            [0.75, 0.45, 0.35, 1.0],  # Darker pale peachy orange
            [0.72, 0.42, 0.30, 1.0],  # Darker muted pale orange
            [0.78, 0.48, 0.38, 1.0],  # Darker light pale orange
            [0.70, 0.40, 0.28, 1.0],  # Darker deeper pale orange
            [0.77, 0.47, 0.37, 1.0],  # Darker soft pale orange
            [0.73, 0.43, 0.32, 1.0]   # Darker medium pale orange
        ]
        # Example transforms for each mesh (adjust as needed for your STL alignment)
        mesh_transforms = [
            SE3(),
            SE3().Rx(pi/2)*SE3(-0.35*sca,0,0),
            SE3().Rx(-pi/2)*SE3(-1.25*sca,0,0),
            SE3().Ry(-pi/2)*SE3().Rx(pi)*SE3(0,0,0.07*sca),
            SE3().Ry(-pi/2)*SE3().Rz(-pi/2)*SE3.Rx(pi)*SE3(-0.35*sca,0,0),
            SE3().Rx(-pi/2)*SE3().Ry(pi/2)*SE3(0,0,0)
        ]
        for i, link in enumerate(links):
            mesh_path = f"{mesh_dir}/{mesh_files[i]}"
            link.geometry = [Mesh(mesh_path, scale=[sca, sca, sca], pose=mesh_transforms[i],color=yellow_colors[i])]
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
        self.base = SE3(0.5,0,0)
        env.add(self)

        joint = 5  # Change this to test different joints (0 to 5)
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
        self.q = self._qtest
        q_goal = [self.q[i]-pi/3 for i in range(self.n)]
        qtraj = rtb.jtraj(self.q, q_goal, 50).q
        # fig = self.plot(self.q)
        for q in qtraj:
            self.q = q
            env.step(0.02)
            # fig.step(0.01)
        time.sleep(3)

    def joystickSetup(self):
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            print("No joystick detected.")
            exit()

        joy = pygame.joystick.Joystick(0)
        joy.init()

        print(f"Joystick: {joy.get_name()}")
        print(f" - {joy.get_numbuttons()} buttons")
        print(f" - {joy.get_numaxes()} axes")
        return joy

    # --- helpers for joystick control ---
    def _clamp_to_qlim(self, q):
        q = np.array(q, dtype=float)
        for j, link in enumerate(self.links):
            lim = getattr(link, "qlim", None)
            if lim is None:
                continue
            q[j] = np.clip(q[j], float(lim[0]), float(lim[1]))
        return q

    def _dls_step(self, dx, lam=0.1, dq_limit=2.0):
        J = self.jacob0(self.q)            # 6xN
        JTJ = J.T @ J
        N = JTJ.shape[0]
        dq = np.linalg.solve(JTJ + (lam**2) * np.eye(N), J.T @ dx)
        return np.clip(dq, -dq_limit, dq_limit)

    def run_joystick_control(self, env, joy=None, dt=0.02, Kv=0.3, Kw=0.8,
                             lam=0.1, deadzone=0.1, button_gain=0.5):
        if joy is None:
            joy = self.joystickSetup()

        print("Joystick control running. Press ESC (keyboard) to quit, 'r' to reset pose.")

        while True:
            pygame.event.pump()

            axes_raw = [joy.get_axis(i) for i in range(joy.get_numaxes())]
            axes = [0.0 if abs(a) < deadzone else a for a in axes_raw]
            buttons = [joy.get_button(i) for i in range(joy.get_numbuttons())]

            # Map joystick -> end-effector spatial velocity [vx, vy, vz, wx, wy, wz]
            vx = Kv * axes[0]                              # left stick X
            vy = -Kv * axes[1]                             # left stick Y (invert)
            vz = Kv * button_gain * (buttons[3] - buttons[0])   # TRIANGLE - CROSS
            wx = Kw * axes[2]                              # right stick X
            wy = Kw * axes[3]                              # right stick Y
            wz = Kw * button_gain * (buttons[2] - buttons[1])   # SQUARE - CIRCLE

            dx = np.array([vx, vy, vz, wx, wy, wz], dtype=float)

            dq = self._dls_step(dx, lam=lam)
            q_new = np.array(self.q, dtype=float) + dq * dt
            q_new = self._clamp_to_qlim(q_new)
            self.q = q_new

            # Reset to zero with 'r'
            try:
                if keyboard.is_pressed('r'):
                    self.q = np.zeros(self.n)
            except Exception:
                pass

            # Quit with ESC
            try:
                if keyboard.is_pressed('esc'):
                    break
            except Exception:
                pass

            env.step(dt)
            time.sleep(dt)

# ---------------------------------------------------------------------------------------#
if __name__ == "__main__":
    env = swift.Swift()
    env.launch(realtime=True)
    r = Kuka()
    r.base = SE3(0, 0, 0)
    env.add(r)

    # Optional: keep sliders too
    r.add_sliders(env)

    # Start joystick control
    
    button_pressed = {'value': False}

    def on_button_press(_):
        button_pressed['value'] = True
        print("Button pressed")
        joy = r.joystickSetup()
        r.run_joystick_control(env, joy)
        
    test_button = swift.Button(cb=on_button_press, desc="Select Robot")
    env.add(test_button)
    # After joystick loop ends, keep window alive
    while True:
        env.step(0)
        time.sleep(0.01)



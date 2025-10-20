import numpy as np
from ir_support import UR3
from spatialmath import SE3
from spatialgeometry import Cuboid, Cylinder, Mesh, Sphere
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
from AssessmentTwo import myCobot280, Assignment2
import pygame


class guiAndControl:
    def __init__(self, env, r1, r2, r3, r4):
        self.env = env
        self.r1, self.r2, self.r3, self.r4 = r1, r2, r3, r4
        self.active = {"robot": r1}
        self.joystick_enabled = {"v": False}
        self._sliders = []

    # -------- robot selection buttons --------
    def select_kuka(self, _=None):
        self.active["robot"] = self.r1
        self.joystick_enabled["v"] = True
        print("Controlling: Kuka")

    def select_abb(self, _=None):
        self.active["robot"] = self.r2
        self.joystick_enabled["v"] = True
        print("Controlling: ABB")

    def select_ur3(self, _=None):
        self.active["robot"] = self.r3
        self.joystick_enabled["v"] = True
        print("Controlling: UR3")

    def select_cobot(self, _=None):
        self.active["robot"] = self.r4.robot
        self.joystick_enabled["v"] = True
        print("Controlling: myCobot280")

    # -------- sliders --------
    def _set_joint_deg(self, robot, j, deg):
        q = list(robot.q)
        q[j] = np.clip(np.deg2rad(float(deg)), -pi, pi)
        robot.q = q

    def slider_cb(self, value_deg, joint_index):
        rob = self.active["robot"]
        if joint_index >= getattr(rob, "n", len(getattr(rob, "links", []))):
            return
        self._set_joint_deg(rob, joint_index, value_deg)
        self.env.step(0)

    def robot_joint_control(self):
        self.env.add(swift.Button(desc="Control Kuka", cb=self.select_kuka))
        self.env.add(swift.Button(desc="Control ABB", cb=self.select_abb))
        self.env.add(swift.Button(desc="Control UR3", cb=self.select_ur3))
        self.env.add(swift.Button(desc="Control myCobot", cb=self.select_cobot))

        for i in range(6):
            init_deg = 0.0
            try:
                init_deg = float(np.rad2deg(self.active["robot"].q[i]))
            except Exception:
                pass
            s = swift.Slider(
                cb=lambda v, j=i: self.slider_cb(v, j),
                min=-180, max=180, step=1,
                value=init_deg,
                desc=f"Joint {i+1} (Active Robot)",
                unit="&#176;",
            )
            self.env.add(s)
            self._sliders.append(s)

    # -------- joystick helpers --------
    def _clamp_to_qlim(self, robot, q):
        qq = np.array(q, dtype=float)
        n = min(len(qq), getattr(robot, "n", len(qq)))
        for j in range(n):
            qq[j] = np.clip(qq[j], -pi, pi)
        return qq

    def _dls_step(self, robot, dx, lam_=0.1, dq_limit=2.0):
        J = robot.jacob0(robot.q)
        JTJ = J.T @ J
        N = JTJ.shape[0]
        dq = np.linalg.solve(JTJ + (lam_**2) * np.eye(N), J.T @ dx)
        return np.clip(dq, -dq_limit, dq_limit)

    @staticmethod
    def _joy_axis(v, th):
        return 0.0 if abs(v) < th else v

    def joystick_tick(self, joy, dt_joy, Kv, Kw, lam, deadzone, button_gain):
        if not (self.joystick_enabled["v"] and joy is not None):
            return
        pygame.event.pump()
        axes = [self._joy_axis(joy.get_axis(i), deadzone) for i in range(joy.get_numaxes())]
        buttons = [joy.get_button(i) for i in range(joy.get_numbuttons())]

        vx = Kv * (axes[0] if len(axes) > 0 else 0.0)
        vy = -Kv * (axes[1] if len(axes) > 1 else 0.0)
        vz = Kv * button_gain * ((buttons[3] if len(buttons)>3 else 0) - (buttons[0] if len(buttons)>0 else 0))
        wx = Kw * (axes[2] if len(axes) > 2 else 0.0)
        wy = Kw * (axes[3] if len(axes) > 3 else 0.0)
        wz = Kw * button_gain * ((buttons[2] if len(buttons)>2 else 0) - (buttons[1] if len(buttons)>1 else 0))

        dx = np.array([vx, vy, vz, wx, wy, wz], dtype=float)
        rob = self.active["robot"]
        dq = self._dls_step(rob, dx, lam_=lam)
        q_new = np.array(rob.q, dtype=float) + dq * dt_joy
        rob.q = self._clamp_to_qlim(rob, q_new)
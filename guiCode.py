import swift
import numpy as np
from math import pi
import pygame
from ir_support import UR3
from spatialmath import SE3
from spatialgeometry import Cuboid, Cylinder, Mesh, Sphere
from roboticstoolbox import DHLink, DHRobot, jtraj, PrismaticDH
import matplotlib.pyplot as plt
import keyboard
import spatialmath.base as spb
import trimesh
import roboticstoolbox as rtb
import os
import time
import serial
import threading
from kuka_ropbot import Kuka
from abb import abb
#from Cobot280 import myCobot280
from AssessmentTwo import myCobot280, Assignment2


class EStop:
    def __init__(self, on_engage=None, on_reset=None):
        self.engaged = False       # true while e-stop is active
        self.run_enabled = True    # true when motion is allowed
        self.on_engage = on_engage
        self.on_reset = on_reset
        self.arduino = None
        self._hw_thread = None

    def engage(self):
        self.engaged = True
        self.run_enabled = False
        if callable(self.on_engage):
            try: self.on_engage()
            except Exception: pass
        print("[E-STOP] engaged")

    def reset(self):
        # clears the fault but does NOT resume motion
        self.engaged = False
        self.run_enabled = False
        if callable(self.on_reset):
            try: self.on_reset()
            except Exception: pass
        print("[E-STOP] reset (press Resume to move)")

    def resume(self):
        if not self.engaged:
            self.run_enabled = True
            print("[E-STOP] resumed")

    def should_run(self) -> bool:
        # Only move when not engaged AND explicitly resumed
        return (not self.engaged) and self.run_enabled
    
    def start_hardware_estop_listener(self, port='COM4', baud=9600, timeout=0.1):
        """Opens the serial port and starts a background listener thread."""
        try:
            self.arduino = serial.Serial(port=port, baudrate=baud, timeout=timeout)
            print(f"[HW E-STOP] Connected to {port} @ {baud}")
        except Exception as e:
            print(f"[HW E-STOP] Could not open {port}: {e}")
            self.arduino = None
            return

        if self._hw_thread is None or not self._hw_thread.is_alive():
            self._hw_thread = threading.Thread(target=self._hardware_estop_loop, daemon=True)
            self._hw_thread.start()

    def _hardware_estop_loop(self):
        """Background serial loop. Accepts lines: ESTOP_Rn, CLEAR_Rn, CONFIRM_Rn."""
        while True:
            if self.arduino is None:
                time.sleep(0.25)
                continue
            try:
                line = self.arduino.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    time.sleep(0.02)
                    continue

                # Parse robot id (last number in the line)
                rid = None
                for token in line.split('_'):
                    if token and token[-1].isdigit():
                        rid = int(token[-1])

                if line.startswith("ESTOP_R"):
                    print(f"🚨 Hardware E-STOP pressed: Robot {rid}")
                    self.engage()         # engage immediately
                elif line.startswith("CLEAR_R"):
                    print(f"✅ Hardware E-STOP cleared: Robot {rid}")
                    self.reset()          # clear fault but DO NOT resume
                elif line.startswith("CONFIRM_R"):
                    print(f"⚪ Confirm release requested: Robot {rid}")
                    self.resume()         # resume motion if possible
                    # Optional: update a UI label here if you have one

            except Exception as e:
                print("[HW E-STOP] Read error:", e)
                time.sleep(0.1)


class guiAndControl:
    def __init__(self, env, r1, r2, r3, r4, on_estop=None):
        self.env = env
        self.r1, self.r2, self.r3, self.r4 = r1, r2, r3, r4
        self.active = {"robot": r1}
        self.joystick_enabled = {"v": False}
        self._sliders = []
        self.estop = EStop(on_engage=on_estop, on_reset=None)
        # UI elements
        self.prox_label = None

    # --- selection ---
    def select_kuka(self, _=None):
        self.active["robot"] = self.r1; self.joystick_enabled["v"] = True
        print("Controlling: Kuka")

    def select_abb(self, _=None):
        self.active["robot"] = self.r2; self.joystick_enabled["v"] = True
        print("Controlling: ABB")

    def select_ur3(self, _=None):
        self.active["robot"] = self.r3; self.joystick_enabled["v"] = True
        print("Controlling: UR3")

    def select_cobot(self, _=None):
        self.active["robot"] = self.r4.robot; self.joystick_enabled["v"] = True
        print("Controlling: myCobot280")

    # --- e-stop UI ---
    def estop_engage_ui(self, _=None): self.estop.engage()
    def estop_reset_ui(self, _=None): self.estop.reset()
    def estop_resume_ui(self, _=None): self.estop.resume()

    # --- sliders ---
    def _set_joint_deg(self, robot, j, deg):
        q = list(robot.q)
        q[j] = np.clip(np.deg2rad(float(deg)), -pi, pi)
        robot.q = q

    def slider_cb(self, value_deg, joint_index):
        if not self.estop.should_run():
            return
        rob = self.active["robot"]
        if joint_index >= getattr(rob, "n", len(getattr(rob, "links", []))):
            return
        self._set_joint_deg(rob, joint_index, value_deg)
        self.env.step(0)

    def robot_joint_control(self):
        # E-STOP buttons
        self.env.add(swift.Button(desc="!!! E‑STOP !!!", cb=self.estop_engage_ui))
        self.env.add(swift.Button(desc="↩ Reset E‑STOP", cb=self.estop_reset_ui))
        self.env.add(swift.Button(desc="▶ Resume sim", cb=self.estop_resume_ui))

        # Robot selection
        self.env.add(swift.Button(desc="Control Kuka", cb=self.select_kuka))
        self.env.add(swift.Button(desc="Control ABB", cb=self.select_abb))
        self.env.add(swift.Button(desc="Control UR3", cb=self.select_ur3))
        self.env.add(swift.Button(desc="Control myCobot", cb=self.select_cobot))

        # Six shared sliders
        for i in range(6):
            init_deg = 0.0
            try: init_deg = float(np.rad2deg(self.active["robot"].q[i]))
            except Exception: pass
            s = swift.Slider(
                cb=lambda v, j=i: self.slider_cb(v, j),
                min=-180, max=180, step=1,
                value=init_deg,
                desc=f"Joint {i+1} (Active Robot)",
                unit="&#176;",
            )
            self.env.add(s)
            self._sliders.append(s)

        # Proximity status label (non-interactive button used as label)
        try:
            self.prox_label = swift.Button(desc="[Proximity] --", cb=lambda _: None)
            self.env.add(self.prox_label)
        except Exception:
            # If UI element creation fails, keep going silently
            self.prox_label = None

    # --- joystick ---
    @staticmethod
    def _joy_axis(v, th): return 0.0 if abs(v) < th else v

    def _dls_step(self, robot, dx, lam_=0.1, dq_limit=2.0):
        J = robot.jacob0(robot.q)
        JTJ = J.T @ J
        N = JTJ.shape[0]
        dq = np.linalg.solve(JTJ + (lam_**2) * np.eye(N), J.T @ dx)
        return np.clip(dq, -dq_limit, dq_limit)

    def _clamp(self, robot, q):
        q = np.array(q, dtype=float)
        for j in range(min(len(q), getattr(robot, "n", len(q)))):
            q[j] = np.clip(q[j], -pi, pi)
        return q

    def joystick_tick(self, joy, dt_joy, Kv, Kw, lam, deadzone, button_gain):
        if not (self.joystick_enabled["v"] and joy is not None and self.estop.should_run()):
            return
        pygame.event.pump()
        axes = [self._joy_axis(joy.get_axis(i), deadzone) for i in range(joy.get_numaxes())]
        buttons = [joy.get_button(i) for i in range(joy.get_numbuttons())]
        if not (any(abs(a) > deadzone for a in axes) or any(buttons)):
            return

        vx = Kv * (axes[0] if len(axes) > 0 else 0.0)
        vy = -Kv * (axes[1] if len(axes) > 1 else 0.0)
        vz = Kv * button_gain * ((buttons[3] if len(buttons) > 3 else 0) - (buttons[0] if len(buttons) > 0 else 0))
        wx = Kw * (axes[2] if len(axes) > 2 else 0.0)
        wy = Kw * (axes[3] if len(axes) > 3 else 0.0)
        wz = Kw * button_gain * ((buttons[2] if len(buttons) > 2 else 0) - (buttons[1] if len(buttons) > 1 else 0))
        dx = np.array([vx, vy, vz, wx, wy, wz], dtype=float)

        rob = self.active["robot"]
        dq = self._dls_step(rob, dx, lam_=lam)
        rob.q = self._clamp(rob, np.array(rob.q, dtype=float) + dq * dt_joy)



    def move_ob(self,dx=0.0, dy=0.0):
        # Respect e-stop if present
        if hasattr(self, "estop") and not self.estop.should_run():
            return
        # World-frame translate then update Swift
        try:
            self.obstruction.T = SE3(float(dx), float(dy), 0.0) * self.obstruction.T
        except Exception:
            # Fallback if your Mesh uses a different transform prop
            try:
                self.obstruction.pose = SE3(float(dx), float(dy), 0.0) * self.obstruction.pose
            except Exception:
                pass
        self.env.step(0)

    def obstructionMovement(self):
        sca = 0.005
        self.obstruction = Mesh(
            'Environmental_models/snapchat-dancing-hotdog-meme-whole-hotdog.stl',
            scale=(sca, sca, sca * 1.1),
            pose=SE3(1.5, 0.2, 0.0),
        )
        self.env.add(self.obstruction)
        Stride = 0.067  # metres per press
        self.env.add(swift.Button(desc="⬅ Obstruction Right",  cb=lambda _: self.move_ob(-Stride, 0.0)))
        self.env.add(swift.Button(desc="➡ Obstruction Left", cb=lambda _: self.move_ob( Stride, 0.0)))
        self.env.add(swift.Button(desc="⬆ Obstruction Down",    cb=lambda _: self.move_ob(0.0,  Stride)))
        self.env.add(swift.Button(desc="⬇ Obstruction Up",  cb=lambda _: self.move_ob(0.0, -Stride)))

    # --- proximity UI update ---
    def set_proximity_status(self, is_near: bool, dist: float | None, thresh: float | None):
        """Update UI label with proximity state; safe no-op if UI unavailable."""
        if self.prox_label is None:
            return
        try:
            if dist is None:
                txt = "[Proximity] --"
            else:
                if is_near:
                    txt = f"[Proximity] 🔴 NEAR: {dist:.2f} m (< {thresh:.2f} m)"
                else:
                    txt = f"[Proximity] 🟢 SAFE: {dist:.2f} m (≥ {thresh:.2f} m)"
            # Attempt to update the button's description
            self.prox_label.desc = txt
        except Exception:
            pass
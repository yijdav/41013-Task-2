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
from guiCode import guiAndControl
#hi

class core:
    def __init__(self):
        self.penDots = []
        self.m5_screw_mesh = None
        self.m5_nut_mesh = None
        self.m5_screws = []
        self.m5_nuts = []
        self.min_z = 0.05  # meters
        self._followers = []  # (obj, robot, offset)

    def add_m5_screw_and_nut(self, env,
                              screw_pose: SE3 | None = None,
                              nut_pose: SE3 | None = None,
                              scale_mm_to_m: bool = True):

        root = Path(__file__).parent
        assets = root / "Environmental_models"
        screw_path = (assets / "M5-Screw-95.stl").resolve()
        nut_path = (assets / "M5-Nut-105.stl").resolve()

        if not screw_path.exists():
            raise FileNotFoundError(f"Missing STL: {screw_path}")
        if not nut_path.exists():
            raise FileNotFoundError(f"Missing STL: {nut_path}")

        scale = [0.001, 0.001, 0.001] if scale_mm_to_m else [1.0, 1.0, 1.0]

        # Default poses: near ABB base at (2.5, 1, 0), placed in front at y ~ 0.8
        if screw_pose is None:
            screw_pose = SE3(2.50, 0.80, 0.030) * SE3.Rx(-pi)  # vertical orientation
        if nut_pose is None:
            nut_pose = SE3(2.60, 0.80, 0.030)  # flat on ground

        screw_mesh = Mesh(str(screw_path), scale=scale, color=(0.75, 0.75, 0.75, 1.0))
        nut_mesh = Mesh(str(nut_path), scale=scale, color=(0.6, 0.6, 0.6, 1.0))

        screw_mesh.T = screw_pose
        nut_mesh.T = nut_pose

        env.add(screw_mesh)
        env.add(nut_mesh)

        # Store references for later sorting/pick routines
        self.m5_screw_mesh = screw_mesh
        self.m5_nut_mesh = nut_mesh

        return screw_mesh, nut_mesh

    def add_m5_screws_and_nuts(self, env,
                                count: int = 5,
                                screws_center: tuple[float, float] = (2.35, 0.65),
                                nuts_center: tuple[float, float] = (2.55, 0.65),
                                ring_radius: float = 0.10,
                                z_height: float = 0.030,
                                scale_mm_to_m: bool = True):

        root = Path(__file__).parent
        assets = root / "Environmental_models"
        screw_path = (assets / "M5-Screw-95.stl").resolve()
        nut_path = (assets / "M5-Nut-105.stl").resolve()

        if not screw_path.exists():
            raise FileNotFoundError(f"Missing STL: {screw_path}")
        if not nut_path.exists():
            raise FileNotFoundError(f"Missing STL: {nut_path}")

        scale = [0.001, 0.001, 0.001] if scale_mm_to_m else [1.0, 1.0, 1.0]

        screws = []
        nuts = []

        # Place screws in a small ring around screws_center
        for i in range(count):
            theta = 2 * np.pi * i / max(count, 1)
            x = screws_center[0] + ring_radius * np.cos(theta)
            y = screws_center[1] + ring_radius * np.sin(theta)
            m = Mesh(str(screw_path), scale=scale, color=(0.75, 0.75, 0.75, 1.0))
            m.T = SE3(float(x), float(y), float(z_height)) * SE3.Rx(-pi)
            env.add(m)
            screws.append(m)

        # Place nuts in a small ring around nuts_center
        for i in range(count):
            theta = 2 * np.pi * i / max(count, 1)
            x = nuts_center[0] + ring_radius * np.cos(theta)
            y = nuts_center[1] + ring_radius * np.sin(theta)
            m = Mesh(str(nut_path), scale=scale, color=(0.6, 0.6, 0.6, 1.0))
            m.T = SE3(float(x), float(y), float(z_height))
            env.add(m)
            nuts.append(m)

        # Save references
        self.m5_screws = screws
        self.m5_nuts = nuts

        return screws, nuts

    # ---------------- sorting helpers ---------------- #
    def _solve_ik(self, robot, T_goal, q0=None):
        q0 = np.array(robot.q if q0 is None else q0, dtype=float)
        try:
            sol = robot.ikine_LM(T_goal, q0=q0, mask=[1, 1, 1, 1, 1, 1], joint_limits=True)
            return np.array(sol.q, dtype=float)
        except Exception:
            # fallback: try without joint limits
            sol = robot.ikine_LM(T_goal, q0=q0, mask=[1, 1, 1, 1, 1, 1])
            return np.array(sol.q, dtype=float)

    def _exec_traj(self, robot, env, q_start, q_end, steps=80, held=None, hold_offset=SE3(0, 0, -0.06)):
        qs = rtb.jtraj(np.array(q_start, dtype=float), np.array(q_end, dtype=float), steps).q
        for q in qs:
            robot.q = np.array(q, dtype=float)
            if held is not None:
                ee = robot.fkine(robot.q)
                held.T = ee * hold_offset
            env.step(0.01)

    @staticmethod
    def _rt_from(T_se3):
        return T_se3.R, np.ravel(T_se3.t)

    def _clamp_pose_z(self, T_goal, min_z):
        """Clamp the Z component of a pose to be at least min_z, preserving rotation."""
        try:
            R, t = self._rt_from(T_goal)
        except Exception:
            T_goal = SE3(T_goal)
            R, t = self._rt_from(T_goal)
        t = np.array(t, dtype=float)
        t[2] = max(float(t[2]), float(min_z))
        return SE3.Rt(R, t)

    def _pick_and_pile(self, robot, env, items, pile_xy, approach_h=0.15, pick_h_default=0.030, pile_h=0.030, orient_down=True, should_run=lambda: True):
        for i, mesh in enumerate(items):
            if not should_run(): 
                    self.wait_until_run(should_run, env)

            T_item = mesh.T
            # Ensure SE3 type (Mesh.T may be a 4x4 ndarray)
            try:
                _ = T_item.t
            except Exception:
                T_item = SE3(T_item)
            xyz = np.ravel(T_item.t)
            x, y, z = float(xyz[0]), float(xyz[1]), float(xyz[2])
            z_pick = z if not np.isnan(z) else pick_h_default
            z_pick = max(z_pick, self.min_z)
            R_down = SE3.Rx(-pi) if orient_down else SE3()

            # Approach above item
            T_approach = SE3(x, y, max(approach_h, z_pick + 0.08)) * R_down
            T_approach = self._clamp_pose_z(T_approach, self.min_z + 0.02)
            q_a = self._solve_ik(robot, T_approach, q0=robot.q)
            self._exec_traj(robot, env, robot.q, q_a, steps=90)

            # Descend to pick
            T_pick = SE3(x, y, z_pick) * R_down
            T_pick = self._clamp_pose_z(T_pick, self.min_z)
            q_p = self._solve_ik(robot, T_pick, q0=q_a)
            self._exec_traj(robot, env, robot.q, q_p, steps=60)

            # Attach/follow
            held = mesh

            # Retreat
            self._exec_traj(robot, env, robot.q, q_a, steps=60, held=held)

            # Move above pile
            T_pile_a = SE3(float(pile_xy[0]), float(pile_xy[1]), max(approach_h, pile_h + 0.02 * i + 0.08)) * R_down
            T_pile_a = self._clamp_pose_z(T_pile_a, self.min_z + 0.02)
            q_pa = self._solve_ik(robot, T_pile_a, q0=robot.q)
            self._exec_traj(robot, env, robot.q, q_pa, steps=100, held=held)

            # Lower to pile
            T_pile = SE3(float(pile_xy[0]), float(pile_xy[1]), pile_h + 0.01 * i) * R_down
            T_pile = self._clamp_pose_z(T_pile, self.min_z)
            q_pd = self._solve_ik(robot, T_pile, q0=q_pa)
            self._exec_traj(robot, env, robot.q, q_pd, steps=60, held=held)

            # Release (stop following)
            held.T = T_pile
            held = None

            # Retract
            self._exec_traj(robot, env, robot.q, q_pa, steps=60)

    def sort_screws_and_nuts(self, env, ur3_robot, abb_robot,
                              screw_pile_xy=(2.30, 0.60), nut_pile_xy=(2.70, 0.60),
                              approach_h=0.15, pick_h=0.030, pile_h=0.030,
                              should_run=lambda: True):  # <- added

        # Set neutral poses (simple defaults)
        try:
            ur3_robot.q = np.array([0, -pi/2, pi/2, -pi/2, -pi/2, 0], dtype=float)
        except Exception:
            pass
        try:
            abb_robot.q = np.array([0, -pi/2, 0, 0, 0, 0], dtype=float)
        except Exception:
            pass
        env.step(0.05)

        # UR3 handles screws
        if getattr(self, 'm5_screws', None):
            self._pick_and_pile(
                ur3_robot, env, self.m5_screws, screw_pile_xy,
                approach_h=approach_h, pick_h_default=pick_h, pile_h=pile_h,
                orient_down=True, should_run=should_run  # <- pass through
            )

        # ABB handles nuts
        if getattr(self, 'm5_nuts', None):
            self._pick_and_pile(
                abb_robot, env, self.m5_nuts, nut_pile_xy,
                approach_h=approach_h, pick_h_default=pick_h, pile_h=pile_h,
                orient_down=True, should_run=should_run  # <- pass through
            )



    def rmrc_draw_square(self, robot, env, origin, side_length, steps_per_side, dt, should_run=lambda: True):
        #Defines the corners of the square, using origin as a starting point
        corners = [
            origin,
            origin * SE3(side_length, 0, 0),
            origin * SE3(side_length, side_length, 0),
            origin * SE3(0, side_length, 0),
            origin  # return to start
        ]

        # Try IK, but don't block if it fails
        try:
            q = robot.ikine_LM(corners[0], q0=np.array(robot.q, dtype=float),
                               mask=[1,1,1,1,1,1], joint_limits=True, ilimit=100, tol=1e-3).q
            robot.q = np.array(q, dtype=float)
        except Exception:
            pass

        #Repeat section for each edge of the square
        for i in range(len(corners)-1):
            if not should_run(): 
                self.wait_until_run(should_run, env)
            start_pose = corners[i]
            end_pose = corners[i+1]

            for s in np.linspace(0, 1, steps_per_side):
                if not should_run(): 
                    self.wait_until_run(should_run, env)
                #Interpolate in Cartesian space
                desired_pose = start_pose.interp(end_pose, s)

                #Compute Jacobian
                J = robot.jacob0(robot.q)

                #Computes xdot, that is the change in translation during a timestep dt
                current_pose = robot.fkine(robot.q)
                xdot = (desired_pose.t - current_pose.t) / dt

                #Computes the rotational component of xdot using the skew symmetric matrix over dt
                R_current = current_pose.R
                R_desired = desired_pose.R
                R_diff = R_desired @ R_current.T
                ang_diff = np.array([
                    R_diff[2,1]-R_diff[1,2],
                    R_diff[0,2]-R_diff[2,0],
                    R_diff[1,0]-R_diff[0,1]
                ]) / 2 / dt

                #Combines rotational and translational components of xdot
                xdot_full = np.hstack((xdot.astype(float), ang_diff.astype(float)))

                #Compute joint velocities
                _lambda = 0.1  #damping factor, tweak between 0.01 and 0.5
                JT = J.T
                qdot = JT @ np.linalg.inv(J @ JT + (_lambda**2) * np.eye(6)) @ xdot_full

                #Update joint positions
                robot.q = (robot.q + qdot * dt).astype(float)

                #Draw line using pen
                penDot = Sphere(radius=0.01, color=[1.0, 0.0, 0.0, 1.0])
                fk = robot.fkine(robot.q)
                _offset = SE3(0,0,-0.06)
                penDot.T = SE3(fk.t.flatten().astype(float)) *_offset
                env.add(penDot)
                self.penDots.append(penDot)
                env.step(float(dt))

    def rmrc_move_line(self, robot, env, target_pose: SE3, steps: int = 60, dt: float = 0.05,
                        lam: float = 0.1, should_run=lambda: True):
        """Move end-effector from current pose to target_pose using RMRC (DLS)."""
        try:
            current_pose = robot.fkine(robot.q)
        except Exception:
            current_pose = SE3(robot.fkine(robot.q))
        target_pose = self._clamp_pose_z(target_pose, self.min_z)

        for s in np.linspace(0.0, 1.0, steps):
            if not should_run():
                self.wait_until_run(should_run, env)

            desired_pose = current_pose.interp(target_pose, s)
            desired_pose = self._clamp_pose_z(desired_pose, self.min_z)

            J = robot.jacob0(robot.q)
            now_pose = robot.fkine(robot.q)
            try:
                R_current = now_pose.R
            except Exception:
                now_pose = SE3(now_pose); R_current = now_pose.R
            xdot = (desired_pose.t - now_pose.t) / dt

            R_desired = desired_pose.R
            R_diff = R_desired @ R_current.T
            ang_diff = np.array([
                R_diff[2,1]-R_diff[1,2],
                R_diff[0,2]-R_diff[2,0],
                R_diff[1,0]-R_diff[0,1]
            ]) / 2.0 / dt

            xdot_full = np.hstack((np.asarray(xdot, dtype=float).flatten(), ang_diff.astype(float)))
            JT = J.T
            JJt = J @ JT
            dq = JT @ np.linalg.inv(JJt + (lam**2) * np.eye(6)) @ xdot_full

            robot.q = (np.asarray(robot.q, dtype=float) + dq * dt).astype(float)
            # update any followers while moving
            if self._followers:
                self.update_followers()
            env.step(float(dt))

    def rmrc_move_offset(self, robot, env, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0,
                         steps: int = 60, dt: float = 0.05, lam: float = 0.1, should_run=lambda: True):
        """RMRC move by a translational offset from current pose (keeps orientation)."""
        fk = robot.fkine(robot.q)
        try:
            start_pose = fk
        except Exception:
            start_pose = SE3(fk)
        target_pose = start_pose * SE3(dx, dy, dz)
        target_pose = self._clamp_pose_z(target_pose, self.min_z)
        self.rmrc_move_line(robot, env, target_pose, steps=steps, dt=dt, lam=lam, should_run=should_run)

    def wait_until_run(self, should_run, env, sleep=0.01):
        while not should_run():
            env.step(0)
            time.sleep(sleep)


    def Animating(self, robot, should_run=lambda: True):
        #DRAWING FIRST BOX
        box_dir = "Environmental_models/centeredBox.stl"

        sideLength = 0.2
        dt=0.05
        steps_per_side=30      
        laps = 1
        origin = SE3(2.3,-1,0)* SE3(0.17,0.41,0.1) * SE3.Rx(-pi) #FIRST SE3 IS ROBOTS BASE SECOND IS OFFSET FROM ROBOT POS
        for i in range(laps):        
            if not should_run(): break
            self.rmrc_draw_square(robot, env, origin*SE3(0,0,-i*0.01), sideLength, steps_per_side, dt, should_run=should_run)
        box1 = Mesh(box_dir, pose=SE3(2.3,-1,0)* SE3(0.27,0.31,0), scale = (1,1,1), color = (0.7,0.2,0.2))
        env.add(box1)

        #DRAWING SECOND BOX
        origin = SE3(2.3,-1,0)* SE3(-0.26,0.41,0.1) * SE3.Rx(-pi) 
        self.rmrc_draw_square(robot, env, origin*SE3(0,0,-i*0.01), sideLength, steps_per_side, dt, should_run=should_run)
        box2 = Mesh(box_dir, pose=SE3(2.3,-1,0)* SE3(-0.16,0.31,0), scale = (1,1,1), color = (0.7,0.2,0.2))
        env.add(box2)

        env.step(0.05)
        return box1, box2
        

    def move_to_joint_positions(self, robot, env, q_target, steps=50, dt=0.05, should_run=lambda: True, held=None, hold_offset=SE3(0, 0, -0.06)):
        q0 = np.asarray(robot.q, dtype=float)
        qt = np.asarray(q_target, dtype=float)

        qs = rtb.jtraj(q0, qt, int(steps)).q
        for q in qs:
            if not should_run():
                self.wait_until_run(should_run, env)
            robot.q = np.asarray(q, dtype=float)
            if held is not None:
                ee = robot.fkine(robot.q)
                held.T = ee * hold_offset
            if self._followers:
                self.update_followers()
            env.step(float(dt))
        return np.asarray(robot.q, dtype=float)


    def attach_follow(self, obj, robot, offset=SE3(0, 0, -0.06)):
        for o, r, _ in self._followers:
            if o is obj and r is robot:
                return
        self._followers.append((obj, robot, offset))

    def detach_follow(self, obj):
        self._followers = [(o, r, off) for (o, r, off) in self._followers if o is not obj]

    def update_followers(self, env=None):
        for (obj, robot, offset) in list(self._followers):
            ee = robot.fkine(robot.q)
            T = ee * offset
            obj.T = T





if __name__ == "__main__":
    env = swift.Swift()
    env.launch(realtime=True)
    c = core()
       
    #--------------------------------------------ENVIRONMENT--------------------------------------------#
    sca=0.1
    workshop = Mesh("Environmental_models/workshop.stl", scale=[sca, sca, sca])
    env.add(workshop)

    sca = 0.002
    env.add(Mesh("Environmental_models/stop_button_base.stl", scale=[sca, sca, sca], pose=SE3(1.19, -1.28, 0.64)*SE3.Rx(-pi/2)*SE3.Rz(pi), color=(1, 0, 0.2, 0.5)))
    env.add(Mesh("Environmental_models/stop_button.stl", scale=[sca, sca, sca], pose=SE3(1.19, -1.28, 0.64)*SE3.Rx(-pi/2)*SE3.Rz(pi), color=(0.8, 0.8, 0.8, 1.0)))

    sca = 0.1
    env.add(Mesh("Environmental_models/safety_text.stl", scale=[sca, sca, sca], pose=SE3(2.13, -1.22, 0.89)*SE3.Rx(-pi/2)*SE3.Rz(pi), color=(0.9, 0.9, 0.9, 1.0)))

    sca = 0.005
    env.add(Mesh("Environmental_models/firetop.stl", scale=[sca, sca, sca], pose=SE3(1, -1.19, 0), color=(0.2, 0.2, 0.2, 1.0)))
    env.add(Mesh("Environmental_models/firebottom.stl", scale=[sca, sca, sca], pose=SE3(1, -1.19, 0), color=(0.6, 0, 0.1, 1.0)))
    #--------------------------------------------ROBOTS--------------------------------------------#
    r1 = Kuka()
    r2 = abb()
    r3 = UR3()
    r4pos = SE3(2.3,-1,0)
    r4 = myCobot280(r4pos) 
    r1.base = SE3(2.6, 0, 0)
    env.add(r1)
    r2.base = SE3(2.5, 1, 0)
    env.add(r2)
    r3.base = SE3(2.1, 1, 0)
    r3.add_to_env(env)
    env.add(r4.robot)
    env.add(r4.base_mesh)

    # Build GUI early so we can pass should_run into animations
    e = guiAndControl(env, r1, r2, r3, r4)
    e.robot_joint_control()  # buttons, sliders, e-stop
    e.estop.start_hardware_estop_listener(port="COM4", baud=9600)  # adjust as needed

    e.obstructionMovement()

    c.add_m5_screws_and_nuts(env, count=5)


# Moving Cobot --------------------------------------------------------------------
    # c.Animating(r4.robot, should_run=e.estop.should_run)
    box1, box2 = c.Animating(r4.robot, should_run=e.estop.should_run)

# Moving Kuka --------------------------------------------------------------------
    desired_q_r1 = np.deg2rad([0, -90, -30, -90, 0, -30])
    c.move_to_joint_positions(r1, env, desired_q_r1, steps=50, dt=0.05, should_run=e.estop.should_run)

    c.attach_follow(box1, r1, offset=SE3(0, -0.133, 0)*SE3.Rx(-pi/2))  # tune offset to your TCP

    desired_q_r1 = np.deg2rad([0, 75, -40, -115, 0, -20])
    c.move_to_joint_positions(r1, env, desired_q_r1, steps=50, dt=0.05, should_run=e.estop.should_run)

    c.detach_follow(box1)

    desired_q_r1 = np.deg2rad([0, -125, 0, -10, 0, -70])
    c.move_to_joint_positions(r1, env, desired_q_r1, steps=50, dt=0.05, should_run=e.estop.should_run)

    c.attach_follow(box2, r1, offset=SE3(0, -0.133, 0)*SE3.Rx(-pi/2))

    desired_q_r1 = np.deg2rad([0, 120, -35, -90, 0, -30])
    c.move_to_joint_positions(r1, env, desired_q_r1, steps=50, dt=0.05, should_run=e.estop.should_run)

    c.detach_follow(box2)

    desired_q_r1 = np.deg2rad([0, 120, -35, 140, 0, -30])
    c.move_to_joint_positions(r1, env, desired_q_r1, steps=50, dt=0.05, should_run=e.estop.should_run)

# Moving ur3 and abb --------------------------------------------------------------------
    # c.rmrc_move_offset(r3, env, dx=0.00, dy=-0.10, dz=0.05, steps=60, dt=0.05, should_run=e.estop.should_run)
    # c.rmrc_move_offset(r2, env, dx=0.00, dy=0.10, dz=0.05, steps=60, dt=0.05, should_run=e.estop.should_run)
    c.sort_screws_and_nuts(env, ur3_robot=r3, abb_robot=r2,
                            screw_pile_xy=(2.30, 0.50),
                            nut_pile_xy=(2.70, 0.50),
                            approach_h=0.15, pick_h=0.030, pile_h=0.030, should_run=e.estop.should_run)

    




    # Joystick setup
    pygame.init(); pygame.joystick.init()
    joy = pygame.joystick.Joystick(0) if pygame.joystick.get_count() > 0 else None
    if joy: joy.init()

    # # --- proximity alert: hot dog near KUKA ---
    # HOTDOG_NEAR_THRESH = 0.35  # metres (XY-plane distance)
    # hotdog_near_flag = False

    # def _xy_from_pose(T_like):
    #     try:
    #         # T_like might be an SE3 or a 4x4 ndarray
    #         T_se3 = T_like if hasattr(T_like, 't') else SE3(T_like)
    #         t = np.ravel(T_se3.t).astype(float)
    #         return float(t[0]), float(t[1])
    #     except Exception:
    #         return None

    while True:
        e.joystick_tick(joy, 0.02, 0.3, 0.8, 0.1, 0.1, 0.5)  # gated by e.stop

        # # Proximity check each tick (non-blocking)
        # try:
        #     # Hot dog pose
        #     T_hotdog = getattr(e, 'obstruction', None)
        #     T_hotdog = getattr(T_hotdog, 'T', None) if T_hotdog is not None else None
        #     # KUKA base pose
        #     T_kuka_base = getattr(r1, 'base', None)

        #     hotdog_xy = _xy_from_pose(T_hotdog) if T_hotdog is not None else None
        #     kuka_xy = _xy_from_pose(T_kuka_base) if T_kuka_base is not None else None

        #     if hotdog_xy is not None and kuka_xy is not None:
        #         dx = hotdog_xy[0] - kuka_xy[0]
        #         dy = hotdog_xy[1] - kuka_xy[1]
        #         dist = float(np.hypot(dx, dy))

        #         if dist < HOTDOG_NEAR_THRESH and not hotdog_near_flag:
        #             print(f"[ALERT] Hot dog near KUKA: {dist:.2f} m (threshold {HOTDOG_NEAR_THRESH:.2f} m)")
        #             hotdog_near_flag = True
        #         elif dist >= HOTDOG_NEAR_THRESH and hotdog_near_flag:
        #             # Reset flag when it moves away so we can alert again on next approach
        #             hotdog_near_flag = False

        #         # Update UI label (if available)
        #         try:
        #             e.set_proximity_status(is_near=(dist < HOTDOG_NEAR_THRESH), dist=dist, thresh=HOTDOG_NEAR_THRESH)
        #         except Exception:
        #             pass
        # except Exception:
        #     pass

        env.step(0)
        time.sleep(0.01)

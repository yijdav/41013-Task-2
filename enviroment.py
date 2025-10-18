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


class core:
    def rmrc_draw_square(self, robot, env, origin, side_length, steps_per_side, dt):
        
        #Defines the corners of the square, using origin as a starting point
        corners = [
            origin,
            origin * SE3(side_length, 0, 0),
            origin * SE3(side_length, side_length, 0),
            origin * SE3(0, side_length, 0),
            origin  # return to start
        ]

        #Compute initial joint configuration using IK
        q = robot.ikine_LM(corners[0], q0=np.array(robot.q, dtype=float), mask=[1,1,1,1,1,1], joint_limits=True).q
        robot.q = np.array(q, dtype=float)  # Ensure standard Python float array

        #Repeat section for each edge of the square
        for i in range(len(corners)-1):
            start_pose = corners[i]
            end_pose = corners[i+1]

            for s in np.linspace(0, 1, steps_per_side):
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
    r4 = myCobot280(SE3(2.3,-1,0)) 
    r1.base = SE3(2.3, 0, 0)
    env.add(r1)
    r2.base = SE3(2.5, 1, 0)
    env.add(r2)
    r3.base = SE3(2.1, 1, 0)
    r3.add_to_env(env)
    env.add(r4.robot)
    env.add(r4.base_mesh)

    #--------------------------------------------Tester--------------------------------------------#
    # Create trajectories for three robots
    steps = 50
    q1 = rtb.jtraj(r1.q, [joint - pi/4 for joint in r1.q], steps).q
    q2 = rtb.jtraj(r2.q, [joint - pi/4 for joint in r2.q], steps).q
    q3 = rtb.jtraj(r3.q, [joint - 0.8 for joint in r3.q], steps).q
    q4 = rtb.jtraj(r4.robot.q, [joint - 0.8 for joint in r4.robot.q], steps).q

    for i in range(steps):
        r1.q = q1[i]
        r2.q = q2[i]
        r3.q = q3[i]
        r4.robot.q = q4[i]
        env.step(0.05)

    #--------------------------------------------GUI--------------------------------------------#
    e = guiAndControl(env, r1, r2, r3, r4)
    # builds the robot control buttons and sliders
    e.robot_joint_control() #I keep deleting this by accident lol

    # --- Joystick setup (disabled until a robot is selected) ---
    pygame.init()
    pygame.joystick.init()
    joy = pygame.joystick.Joystick(0) if pygame.joystick.get_count() > 0 else None
    if joy: joy.init()
    print(f"Joystick: {joy.get_name()} | Buttons={joy.get_numbuttons()} Axes={joy.get_numaxes()}") if joy else print("No joystick connected")


    while True:
        e.joystick_tick(joy, 0.02, 0.3, 0.8, 0.1, 0.1, 0.5)
        env.step(0)
        time.sleep(0.01)

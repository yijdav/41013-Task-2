import numpy as np
from ir_support import UR3
from spatialmath import SE3
from spatialmath.base import trinterp
from spatialgeometry import Cuboid, Cylinder, Mesh, Sphere
from roboticstoolbox import DHLink, DHRobot, jtraj, PrismaticDH
from math import pi
import swift 
from pathlib import Path
import matplotlib.pyplot as plt
import keyboard

env = None
class myCobot280:
    def __init__(self,base_pos):
        #SET DH PARAMETERS FOR LINKS
        links = [
        DHLink(d=0.15, a=0, alpha=1.5708, offset=0),
        DHLink(d=0, a=-0.2, alpha=0, offset=-1.5708),
        DHLink(d=0, a=-0.2, alpha=0, offset=0),
        DHLink(d=-0.15, a=0, alpha=1.5708, offset=-1.5708),
        DHLink(d=0.1505, a=0, alpha=-1.5708, offset=1.5708),
        DHLink(d=0.0456, a=0, alpha=0, offset=0)]
        #SET MESH DIRECTORIES
        base_dir = "mycobot280_meshes\Base.stl"
        shoulder_dir = "mycobot280_meshes\Shoulder.stl"
        elbow_dir = "mycobot280_meshes\Elbow.stl"
        wrist_dir = "mycobot280_meshes\Wrist.stl"
        ee_dir = "mycobot280_meshes\End Effector.stl"

        #CREATE MESHES
        scale = [0.001,0.001,0.001]
        self.base_mesh = Mesh(base_dir,pose=SE3.Rx(pi/2),color = (0.6,0.5,0.2,1), scale = scale)
        shoulder_mesh = Mesh(shoulder_dir,pose = SE3(0,-0.15,0)*SE3.Ry(pi/2),color = (0.2,0.2,0.2,1), scale = scale)
        elbow1_mesh = Mesh(elbow_dir,pose = (SE3.Ry(-pi/2)*SE3.Rx(pi/2)*SE3(-0.151,-0.1,0)),color = (0.2,0.2,0.6,1), scale = scale)
        elbow2_mesh = Mesh(elbow_dir,pose = (SE3.Ry(pi/2)*SE3.Rx(pi/2)*SE3(0,0.1,0)),color = (0.2,0.6,0.3,1), scale = scale)
        wrist_mesh = Mesh(wrist_dir,pose=SE3.Rx(pi)*SE3.Ry(pi/2)*SE3(0,-0.0744,0),color = (0.2,0.2,0.2,1), scale = scale)
        ee_mesh = Mesh(ee_dir,pose=SE3.Rx(-pi/2)*SE3(0,-0.1,0),color = (0.2,0.2,0.2,1), scale = scale)

        
        # links1 = DHLink(d=0.13122, a=0, alpha=1.5708, offset=0)
        # links2 = DHLink(d=0, a=-0.1104, alpha=0, offset=-1.5708)
        # links3 = DHLink(d=0, a=-0.96, alpha=0, offset=0)
        # links4 = DHLink(d=0.634, a=0, alpha=1.5708, offset=-1.5708)
        # links5 = DHLink(d=0.7505, a=0, alpha=-1.5708, offset=1.5708)
        # links6 = DHLink(d=0.456, a=0, alpha=0, offset=0)

        #ATTACH MESHES TO LINKS
        links[0].geometry = [shoulder_mesh]
        links[1].geometry = [elbow1_mesh]
        links[2].geometry = [elbow2_mesh]
        links[3].geometry = [wrist_mesh]
        links[4].geometry = [ee_mesh]
        self.robot = DHRobot(links)
        
        self.robot.links[1].qlim = [-1.745,1.745]
        self.robot.links[2].qlim = [-2.7,2.7]
        self.robot.links[4].qlim = [-3/2*pi,-pi/2]

        #ADD CYLINDERS TO REPRESENT AXES
        # axis1 = self.robot.A(0,self.robot.q)
        # axis2 = self.robot.A(1,self.robot.q)
        # axis3 = self.robot.A(2,self.robot.q)
        # axis4 = self.robot.A(3,self.robot.q)        
        # axis5 = self.robot.A(4,self.robot.q)
        # axis6 = self.robot.A(5,self.robot.q)
        # axslength = 0.2
        # drawaxis1 = Cylinder(0.01, axslength, color = [1.0,0.0,0.0,1.0], pose = axis1)
        # drawaxis2 = Cylinder(0.01, axslength, color = [1.0,0.0,0.0,1.0], pose = axis2)
        # drawaxis3 = Cylinder(0.01, axslength, color = [1.0,0.0,0.0,1.0], pose = axis3)
        # drawaxis4 = Cylinder(0.01, axslength, color = [1.0,0.0,0.0,1.0], pose = axis4)
        # drawaxis5 = Cylinder(0.01, axslength, color = [1.0,0.0,0.0,1.0], pose = axis5)
        # drawaxis6 = Cylinder(0.01, axslength, color = [1.0,0.0,0.0,1.0], pose = axis6) 
        # env.add(drawaxis1)
        # env.add(drawaxis2)
        # env.add(drawaxis3)
        # env.add(drawaxis4)
        # env.add(drawaxis5)
        # env.add(drawaxis6)



        #THIS SETS THE ROBOTS INITIAL POSITION WITH THE BASE IN CORRECT POS
        offset = SE3(0,0,0.14)
        self.base_mesh.T = base_pos*self.base_mesh.T
        self.robot.base = self.robot.base*base_pos*offset

class Assignment2:
    
    def CreateEnvironment(self):
        #STEUP SWIFT ENV
        global env
        env = swift.Swift()
        env.launch(realtime=True)
        
        #ADD COBOT AND SET POSITION
        initial_pos = SE3(0,0,0)
        cobotHolder = myCobot280(initial_pos)

        self.mycobot280 = cobotHolder.robot
        env.add(self.mycobot280)
        env.add(cobotHolder.base_mesh)
        
        

        env.step(0)
        
        
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




    def AnimateCobot280(self):
        
        sideLength = 0.2
        steps_per_side=30
        dt=0.05
        laps = 3
        origin = SE3(0.3,0.2,0.07)* SE3.Rx(-pi)
        box_dir = "Box.stl"
        box_mesh = Mesh(box_dir, pose = SE3(0.3,0.2,0)*SE3.Rx(pi/2), scale = (1,1,1), color = (0.7,0.2,0.2))
        self.penDots = []
        # Call the RMRC function where laps is how tall to make the box
        for i in range(laps):        
            self.rmrc_draw_square(self.mycobot280, env, origin*SE3(0,0,-i*0.01), sideLength, steps_per_side, dt)
        for dot in self.penDots:
            env.remove(dot)
        env.add(box_mesh)

        #initialq = self.mycobot280.ikine_LM(origin,q0=self.mycobot280.q,mask=[1,1,1,1,1,1],joint_limits=True).q
        #self.mycobot280.q = initialq
        #env.step(0.05)
        



        # traj = jtraj([0,self.mycobot280.qlim[0,1],0,0,0,0],[0,self.mycobot280.qlim[1,1],0,0,0,0],30)
        # for q in traj.q:
        #     self.mycobot280.q = q
        #     env.step(0.05)

        # steps = 30
        # poses = [SE3(trinterp(T1.A, T2.A, s)) for s in np.linspace(0, 1, steps)]
        # newq = self.mycobot280.q
        # #print(poses)
        # for i in range(len(poses)):
        #     self.mycobot280.q = self.mycobot280.ikine_LM(poses[i],q0=newq,mask=[1,1,1,1,1,1],joint_limits=True).q
        #     newq = self.mycobot280.q
        #     env.step(0.05)


            # endq = self.mycobot280.ikine_LM(squarePoses[i],q0=self.mycobot280.q,mask=[1,1,1,1,1,1],joint_limits=True).q
            # endq = self.shortest_path(self.mycobot280.q,endq)
            # traj = jtraj(self.mycobot280.q,endq,30)
            # penDot = Sphere(radius=0.025, color=[1.0, 0.0, 0.0, 1.0])
            # penDot.T = self.mycobot280.fkine(self.mycobot280.q)
            # env.add(penDot)
            # for q in traj.q:
            #     self.mycobot280.q = q
                
            #     env.step(0.05)
        env.hold()


    #THIS FUNCTION NORMALISED DISTANCES SO INSTEAD OF GOING LONG WAY JOINTS GO SHORTWAY IN JTRAJ        
    def shortest_path(self, q_start, q_goal):
        q_start = np.asarray(q_start)
        q_goal = np.asarray(q_goal)
        dq = q_goal - q_start
        # Wrap difference into [-pi, pi]
        dq = (dq + pi) % (2 * pi) - pi
        return q_start + dq
if __name__ == "__main__":
    a2 = Assignment2()

    a2.CreateEnvironment()
    a2.AnimateCobot280()

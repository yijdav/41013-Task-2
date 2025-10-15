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
        #0.0634->-0.13 

        examplemodel_dir = (r"c:\Python Scripts\AssessmentTwo\mycobot280meshes\cobot280.stl")
        #CREATE MESHES
        scale = [0.001,0.001,0.001]
        base_mesh = Mesh(base_dir,pose=SE3.Rx(pi/2),color = (0.6,0.5,0.2,1), scale = scale)
        shoulder_mesh = Mesh(shoulder_dir,pose = SE3(0,-0.15,0)*SE3.Ry(pi/2),color = (0.2,0.2,0.2,1), scale = scale)
        elbow1_mesh = Mesh(elbow_dir,pose = (SE3.Ry(-pi/2)*SE3.Rx(pi/2)*SE3(-0.151,-0.1,0)),color = (0.2,0.2,0.6,1), scale = scale)
        elbow2_mesh = Mesh(elbow_dir,pose = (SE3.Ry(pi/2)*SE3.Rx(pi/2)*SE3(0,0.1,0)),color = (0.2,0.6,0.3,1), scale = scale)
        wrist_mesh = Mesh(wrist_dir,pose=SE3.Rx(pi)*SE3.Ry(pi/2)*SE3(0,-0.0744,0),color = (0.2,0.2,0.2,1), scale = scale)
        ee_mesh = Mesh(ee_dir,pose=SE3.Rx(-pi/2)*SE3(0,-0.1,0),color = (0.2,0.2,0.2,1), scale = scale)
        #base_mesh = Cuboid(scale=[0.05,0.05,0.13122],color = [1,0,0,1])
        #shoulder_mesh = Cuboid(scale = [0.1104,0.05,0.05],color=[0,1,0,1])
        #elbow1_mesh = Cuboid(scale = [0.096,0.05,0.05],pose=SE3(0,0,0),color=[0,0,1,1])
        #elbow2_mesh = Cuboid(scale = [0.05,0.05,0.0634],pose=SE3(0,0,0),color=[0.2,0.6,0.2,1])
        #wrist_mesh = Cuboid(scale = [0.05,0.05,0.07505],color=[0.3,0.3,0.3,1])
        #ee_mesh = Cuboid(scale = [0.03,0.03,0.0456],color=[0.8,0.8,0.8,1])

        examplemodel_mesh = Mesh(examplemodel_dir,color=[0.7,0.2,0.2,1],scale=scale,pose = SE3(0.8,0,0)*SE3.Rx(pi/2))
        # verts = np.array(base_mesh.)   # access underlying trimesh object
        # print("Min:", verts.min(axis=0))
        # print("Max:", verts.max(axis=0))
        # print("Center:", verts.mean(axis=0))

        # links1 = DHLink(d=0.13122, a=0, alpha=1.5708, offset=0)
        # links2 = DHLink(d=0, a=-0.1104, alpha=0, offset=-1.5708)
        # links3 = DHLink(d=0, a=-0.96, alpha=0, offset=0)
        # links4 = DHLink(d=0.634, a=0, alpha=1.5708, offset=-1.5708)
        # links5 = DHLink(d=0.7505, a=0, alpha=-1.5708, offset=1.5708)
        # links6 = DHLink(d=0.456, a=0, alpha=0, offset=0)
        #ATTACH MESHES TO LINKS
        #links[0].geometry = [base_mesh]
        links[0].geometry = [shoulder_mesh]
        links[1].geometry = [elbow1_mesh]
        links[2].geometry = [elbow2_mesh]
        links[3].geometry = [wrist_mesh]
        links[4].geometry = [ee_mesh]
        self.robot = DHRobot(links)
        
        #self.robot.links[1].qlim = [-1.745,1.745]
        #self.robot.links[2].qlim = [-2.7,2.7]
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


        #base_pos = SE3(0.5,1,0.2)
        #THIS SETS THE ROBOTS INITIAL POSITION WITH THE BASE IN CORRECT POS
        offset = SE3(0,0,0.14)
        base_mesh.T = base_pos*base_mesh.T
        self.robot.base = self.robot.base*base_pos*offset

        env.add(base_mesh)
        env.add(self.robot)        
        #env.add(examplemodel_mesh)
        env.step(0)


        #USE THIS TO TEST ROBOT IN PYTHON
        q = [0,0,0,0,0,0]
        plt.close()
        fig = self.robot.teach(q, block=False)
        while not keyboard.is_pressed('enter'):
            fig.step(0.05)
        fig.close()


    def get_robot(self):
        return self.robot
        
        #return self.robot
        

class Assignment2:
    
    def CreateEnvironment(self):
        #STEUP SWIFT ENV
        global env
        env = swift.Swift()
        env.launch(realtime=True)
        
        #ADD COBOT AND SET POSITION
        initial_pos = SE3(0,0,0)
        cobotHolder = myCobot280(initial_pos)

        self.mycobot280 = cobotHolder.get_robot()


        env.step(0)
        
        
    def rmrc_draw_square(self, robot, env, origin=SE3(0.3,0,0)*SE3.Rz(-pi), side_length=0.1, steps_per_side=50, dt=0.1):
        """
        Draw a square using RMRC with full 6x6 Jacobians.
        Ensures all joint values are Python floats to avoid Swift JSON errors.
        """
        # 1️⃣ Define square corners
        corners = [
            origin,
            origin * SE3(side_length, 0, 0),
            origin * SE3(side_length, side_length, 0),
            origin * SE3(0, side_length, 0),
            origin  # return to start
        ]

        # 2️⃣ Compute initial joint configuration via IK
        q = robot.ikine_LM(corners[0], q0=np.array(robot.q, dtype=float), mask=[1,1,1,1,1,1], joint_limits=True).q
        robot.q = np.array(q, dtype=float)  # Ensure standard Python float array

        # 3️⃣ Loop over edges
        for i in range(len(corners)-1):
            start_pose = corners[i]
            end_pose = corners[i+1]

            for s in np.linspace(0, 1, steps_per_side):
                # 4️⃣ Interpolate in Cartesian space
                desired_pose = start_pose.interp(end_pose, s)

                # 5️⃣ Compute Jacobian
                J = robot.jacob0(robot.q)

                # 6️⃣ Translation error
                current_pose = robot.fkine(robot.q)
                xdot = (desired_pose.t - current_pose.t) / dt

                # 7️⃣ Rotation error
                R_current = current_pose.R
                R_desired = desired_pose.R
                R_err = R_desired @ R_current.T
                ang_err = np.array([
                    R_err[2,1]-R_err[1,2],
                    R_err[0,2]-R_err[2,0],
                    R_err[1,0]-R_err[0,1]
                ]) / 2 / dt

                xdot_full = np.hstack((xdot.astype(float), ang_err.astype(float)))

                # 8️⃣ Compute joint velocities
                λ = 0.1  # damping factor, tweak between 0.01 and 0.5
                JT = J.T
                qdot = JT @ np.linalg.inv(J @ JT + (λ**2) * np.eye(6)) @ xdot_full



                # 9️⃣ Update joint positions (convert to standard float)
                robot.q = (robot.q + qdot * dt).astype(float)


                # 🔟 Visualize pen
                penDot = Sphere(radius=0.01, color=[1.0, 0.0, 0.0, 1.0])
                fk = robot.fkine(robot.q)
                penDot.T = SE3(fk.t.flatten().astype(float))  # Ensure float
                env.add(penDot)
                env.step(float(dt))  # Ensure dt is standard float



    def AnimateCobot280(self):
        origin = SE3(0.2,-0.1,0.1)
        sideLength = 0.2
        # Call the RMRC function
        self.rmrc_draw_square(self.mycobot280, env, origin, sideLength, steps_per_side=80, dt=0.05)
        env.hold()

        #ARBITRARY TRAJ TO TEST ANIMATION
        # traj = jtraj([0,0,0,0,0,0],[1,1,1,1,1,1],50)
        # for q in traj.q:
        #     self.mycobot280.q = q
        #     penDot = Sphere(radius=0.025, color=[1.0, 0.0, 0.0, 1.0])
        #     penDot.T = self.mycobot280.fkine(self.mycobot280.q)
        #     env.add(penDot)
        #     env.step(0.05)
        # traj = jtraj(self.mycobot280.q,[-1,-1,-1,-1,-1,-1],50)
        # for q in traj.q:
        #     self.mycobot280.q = q
        #     penDot = Sphere(radius=0.025, color=[1.0, 0.0, 0.0, 1.0])
        #     penDot.T = self.mycobot280.fkine(self.mycobot280.q)
        #     env.add(penDot)
        #     env.step(0.05)


        # traj = jtraj([0,0,0,0,self.mycobot280.qlim[0,4],0],[0,0,0,0,self.mycobot280.qlim[1,4],0],30)
        # for q in traj.q:
        #     self.mycobot280.q = q
        #     env.step(0.05)

        #DRAW A SQUARE
        sideLength = 0.3
        origin = SE3(0.3,0,0)*SE3.Rz(pi)#self.mycobot280.fkine(self.mycobot280.q)
        initialq = self.mycobot280.ikine_LM(origin,q0=self.mycobot280.q,mask=[1,1,1,1,1,1],joint_limits=True).q
        self.mycobot280.q = initialq
        squarePoses = [
            origin * SE3(0,0,0),
            origin * SE3(sideLength,0,0),
            origin * SE3(sideLength,sideLength,0),
            origin * SE3(0,sideLength,0),
            origin * SE3(0,0,0)
        ]
        # for i in np.arange(0,len(squarePoses)+1):
        #     interpspots = [SE3(trinterp(squarePoses[i], squarePoses[i+1], s)) for s in np.linspace(0, 1, 50)]
        #     for i in np.arange(0,len(interpspots)+1):
        #         self.mycobot280.q = self.mycobot280.ikine_LM(squarePoses[i],q0=self.mycobot280.q,mask=[1,1,1,1,1,1],joint_limits=True).q
        #         penDot = Sphere(radius=0.025, color=[1.0, 0.0, 0.0, 1.0])
        #         penDot.T = self.mycobot280.fkine(self.mycobot280.q)
        #         env.add(penDot)
            

        # T1 = SE3(0.2, -0.1, 0)
        # T2 = SE3(0.2, 0.1, 0)

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


    #THIS FUNCTION NORMALISED DISTANCES SO INSTEAD OF GOING LONG WAY JOINTS GO SHORTWAY IN JTRAJ        
    def shortest_path(self, q_start, q_goal):
        q_start = np.asarray(q_start)
        q_goal = np.asarray(q_goal)
        dq = q_goal - q_start
        # Wrap difference into [-pi, pi]
        dq = (dq + pi) % (2 * pi) - pi
        return q_start + dq

a2 = Assignment2()

a2.CreateEnvironment()
a2.AnimateCobot280()

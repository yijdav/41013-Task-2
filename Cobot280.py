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

class myCobot280:
    def __init__(self):
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
        base_mesh = Mesh(base_dir,pose=SE3.Rx(pi/2),color = (0.6,0.5,0.2,1), scale = scale)
        shoulder_mesh = Mesh(shoulder_dir,pose = SE3(0,-0.15,0)*SE3.Ry(pi/2),color = (0.2,0.2,0.2,1), scale = scale)
        elbow1_mesh = Mesh(elbow_dir,pose = (SE3.Ry(-pi/2)*SE3.Rx(pi/2)*SE3(-0.151,-0.1,0)),color = (0.2,0.2,0.6,1), scale = scale)
        elbow2_mesh = Mesh(elbow_dir,pose = (SE3.Ry(pi/2)*SE3.Rx(pi/2)*SE3(0,0.1,0)),color = (0.2,0.6,0.3,1), scale = scale)
        wrist_mesh = Mesh(wrist_dir,pose=SE3.Rx(pi)*SE3.Ry(pi/2)*SE3(0,-0.0744,0),color = (0.2,0.2,0.2,1), scale = scale)
        ee_mesh = Mesh(ee_dir,pose=SE3.Rx(-pi/2)*SE3(0,-0.1,0),color = (0.2,0.2,0.2,1), scale = scale)

        #ATTACH MESHES TO LINKS
        #links[0].geometry = [base_mesh]
        links[0].geometry = [shoulder_mesh]
        links[1].geometry = [elbow1_mesh]
        links[2].geometry = [elbow2_mesh]
        links[3].geometry = [wrist_mesh]
        links[4].geometry = [ee_mesh]
        self.robot = DHRobot(links)
        
        self.robot.links[1].qlim = [-1.745,1.745]
        self.robot.links[2].qlim = [-2.7,2.7]


if __name__ == "__main__":
    env = swift.Swift()
    env.launch(realtime=True)
    r = myCobot280()
    r.robot.base = SE3(0,0,0)
    env.add(r.robot)
    env.hold()


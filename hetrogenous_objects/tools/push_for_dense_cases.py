import pybullet as p
import time
import os
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt
import math
from tools.util import poly_disc, poly_stick, polysCollide
from tools.general_objects import get_poly
from tools.show_arrangement import show_hetrogenous_arrangement
from shapely.geometry import Polygon

MAX_ITERATIONS = 10000


class compress_pushing(object):
    '''
    similator to compress the scene to push objects
    objects objID: ( [x, y, theta, 'cuboid', Length, Width)
    '''
    def __init__(self, objects, num_obstacles, Height=1000, Width=1000,offset=5) -> None:
        self.objects = objects
        self.num_obstacles = num_obstacles
        self.Height = Height
        self.Width = Width
        self.offset = offset
        self.scalar = 1/1000.0 # scalar from workspace size to meter
        p.connect(p.DIRECT)
        self.create_sim()
        # input("Enter to start.")

    def create_sim(self):
        '''
        create pybullet scene
        ----------
        |      <-| 
        |      <-|
        ----------
        '''
        p.setTimeStep(1./240)
        p.setGravity(0, 0, -9.8)
        # Load plane contained in pybullet_data
        planeId = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"))

        # add three fixed sticks
        self.thickness = 100
        self.z_size = 100
        # W_Wall
        visualShapeId = p.createVisualShape(shapeType=p.GEOM_BOX,
            rgbaColor=[1, 1, 1, 1],
            specularColor=[0.4, .4, 0],
            halfExtents=np.array([self.thickness, (self.Height-2*self.offset), self.z_size])/2*self.scalar,
            )
        collisionShapeId = p.createCollisionShape(
            shapeType=p.GEOM_BOX,
            halfExtents=np.array([self.thickness, (self.Height-2*self.offset), self.z_size])/2*self.scalar,
            )
        p.createMultiBody(baseMass=0,
                      baseInertialFramePosition=[0, 0, 0],
                      baseCollisionShapeIndex=collisionShapeId,
                      baseVisualShapeIndex=visualShapeId,
                      basePosition=np.array([-self.thickness/2+self.offset, self.Height/2, self.z_size/2])*self.scalar,
                      useMaximalCoordinates=True,
                      )
        
        # S_Wall
        visualShapeId = p.createVisualShape(shapeType=p.GEOM_BOX,
            rgbaColor=[1, 1, 1, 1],
            specularColor=[0.4, .4, 0],
            halfExtents=np.array([2*self.Width, self.thickness, self.z_size])/2*self.scalar,
            )
        collisionShapeId = p.createCollisionShape(
            shapeType=p.GEOM_BOX,
            halfExtents=np.array([2*self.Width, self.thickness, self.z_size])/2*self.scalar,
            )
        p.createMultiBody(baseMass=0,
                      baseInertialFramePosition=[0, 0, 0],
                      baseCollisionShapeIndex=collisionShapeId,
                      baseVisualShapeIndex=visualShapeId,
                      basePosition=np.array([self.Width, -self.thickness/2+self.offset, self.z_size/2])*self.scalar,
                      useMaximalCoordinates=True
                      )

        # N_Wall
        visualShapeId = p.createVisualShape(shapeType=p.GEOM_BOX,
            rgbaColor=[1, 1, 1, 1],
            specularColor=[0.4, .4, 0],
            halfExtents=np.array([2*self.Width, self.thickness, self.z_size])/2*self.scalar,
            )
        collisionShapeId = p.createCollisionShape(
            shapeType=p.GEOM_BOX,
            halfExtents=np.array([2*self.Width, self.thickness, self.z_size])/2*self.scalar,
            )
        p.createMultiBody(baseMass=0,
                      baseInertialFramePosition=[0, 0, 0],
                      baseCollisionShapeIndex=collisionShapeId,
                      baseVisualShapeIndex=visualShapeId,
                      basePosition=np.array([self.Width, self.Height+self.thickness/2-self.offset, self.z_size/2])*self.scalar,
                      useMaximalCoordinates=True
                      )

        # E_Wall, the moving one
        visualShapeId = p.createVisualShape(shapeType=p.GEOM_BOX,
            rgbaColor=[1, 1, 1, 1],
            specularColor=[0.4, .4, 0],
            halfExtents=np.array([self.thickness, (self.Height-2*self.offset), self.z_size])/2*self.scalar,
            )
        collisionShapeId = p.createCollisionShape(
            shapeType=p.GEOM_BOX,
            halfExtents=np.array([self.thickness, (self.Height-2*self.offset), self.z_size])/2*self.scalar,
            )
        self.moving_stick = p.createMultiBody(baseMass=10000,
                      baseInertialFramePosition=[0, 0, 0],
                      baseCollisionShapeIndex=collisionShapeId,
                      baseVisualShapeIndex=visualShapeId,
                      basePosition=np.array([2*self.Width+self.thickness/2-self.offset, self.Height/2, self.z_size/2])*self.scalar,
                      useMaximalCoordinates=True,
                      )



    def add_objects(self):
        '''
        add objects into the scene
        '''
        curr_dir = os.path.dirname(os.path.abspath(__file__))
        coll_offset__scalar = 1.05
        self.bodyID2objID = {}
        for objID, pose in self.objects_n_obstacles.items():
            if pose[3] == 'disc':
                visualShapeId = p.createVisualShape(
                    shapeType=p.GEOM_MESH,
                    fileName = os.path.join(curr_dir, "modified_cylinder.obj"),
                    rgbaColor=[1., 1., 1., 1],
                    specularColor=[0.4, .4, 0],
                    meshScale=np.array([pose[4]/4.3, pose[5]/4.3, self.z_size/5])*self.scalar,
                    )
                collisionShapeId = p.createCollisionShape(
                    shapeType=p.GEOM_MESH,
                    fileName = os.path.join(curr_dir, "modified_cylinder.obj"),
                    meshScale=np.array([pose[4]/4.3, pose[5]/4.3, self.z_size/5])*self.scalar*coll_offset__scalar,
                    )
                bodyID = p.createMultiBody(baseMass=1.0,
                            baseInertialFramePosition=np.array([0, 0, -self.z_size/2])*self.scalar,
                            baseCollisionShapeIndex=collisionShapeId,
                            baseVisualShapeIndex=visualShapeId,
                            basePosition=np.array([pose[0], pose[1], self.z_size/2])*self.scalar,
                            baseOrientation = p.getQuaternionFromEuler(np.array([0,0,pose[2]])),
                            useMaximalCoordinates=True
                            )
                p.changeDynamics(bodyID, -1, lateralFriction=0.05)
                if objID in self.objects:
                    self.bodyID2objID[bodyID] = objID
            elif pose[3] == 'cuboid':
                visualShapeId = p.createVisualShape(shapeType=p.GEOM_BOX,
                    rgbaColor=[1.0, 1.0, 1.0, 1],
                    specularColor=[0.4, .4, 0],
                    halfExtents=np.array([pose[4], pose[5], self.z_size])/2*self.scalar,
                    )
                collisionShapeId = p.createCollisionShape(
                    shapeType=p.GEOM_BOX,
                    halfExtents=np.array([pose[4], pose[5], self.z_size])/2*self.scalar*coll_offset__scalar,
                    )
                bodyID = p.createMultiBody(baseMass=1.0,
                            baseInertialFramePosition=np.array([0, 0, -self.z_size/2])*self.scalar,
                            baseCollisionShapeIndex=collisionShapeId,
                            baseVisualShapeIndex=visualShapeId,
                            basePosition=np.array([pose[0], pose[1], self.z_size/2])*self.scalar,
                            baseOrientation = p.getQuaternionFromEuler(np.array([0,0,pose[2]])),
                            useMaximalCoordinates=True,
                            )
                p.changeDynamics(bodyID, -1, lateralFriction=0.05)
                if objID in self.objects:
                    self.bodyID2objID[bodyID] = objID
        
        # show_hetrogenous_arrangement(
        #         len(self.objects_n_obstacles), 0, self.objects_n_obstacles, self.objects_n_obstacles, WIDTH=2000,
        #         show_arrangements='start', show_text=False
        #     )

    def maintain(self):
        '''
        keep GUI
        '''
        while 1:
            p.stepSimulation()
            time.sleep(1./240)
        p.disconnect()

    def random_sparse(self):
        '''
        create random sparse scene
        '''        
        # get avg area
        areas = {}
        for k in self.objects.keys():
            if self.objects[k][3] == 'disc':
                areas[k] = math.pi/4*self.objects[k][4]*self.objects[k][5]
            elif self.objects[k][3] == 'cuboid':
                areas[k] = self.objects[k][4]*self.objects[k][5]
        avg_area = np.average(list(areas.values()))/4.0
        radius = math.sqrt(avg_area/math.pi)
        self.objects_n_obstacles = self.objects.copy()
        obsID = len(self.objects)
        for _ in range(self.num_obstacles):
            self.objects_n_obstacles[obsID] = [None, None, None]+['disc']+[radius*2,radius*2]
            obsID += 1
        while 1:
            try:
                self.objects_n_obstacles = repeated_sampling(2*self.Width, self.Height, self.objects_n_obstacles)
            except Exception as e:
                print(e)
            else:
                return


    def push_for_dense(self):
        '''
        push the stick to generte dense cases
        '''
        goal_pos = np.array([self.Width+self.thickness/2-self.offset, self.Height/2,self.z_size/2])*self.scalar
        while 1:
            curr_pos,curr_qua = p.getBasePositionAndOrientation(self.moving_stick)
            if curr_pos[0] > goal_pos[0]:
                p.resetBasePositionAndOrientation(
                    bodyUniqueId = self.moving_stick,
                    posObj = [curr_pos[0]-0.002, curr_pos[1], curr_pos[2]],
                    ornObj = curr_qua,
                )
            else:
                for _ in range(300):
                    p.stepSimulation()
                    time.sleep(1/240.0)
                return
            p.stepSimulation()
            time.sleep(1/240.0)


    def record_arrangement(self):
        for bodyID, objID in self.bodyID2objID.items():
            curr_pos,curr_qua = p.getBasePositionAndOrientation(bodyID)
            x,y = curr_pos[0]/self.scalar, curr_pos[1]/self.scalar
            _,_,theta = p.getEulerFromQuaternion(curr_qua)
            new_obj = [x,y,theta]+list(self.objects[objID])[3:]
            self.objects[objID] = new_obj
        # show_hetrogenous_arrangement(
        #         len(self.objects), 0, self.objects, self.objects, 
        #         show_arrangements='start', show_text=False
        #     )
        

    def reset_simulation(self,num_obstacles):
        p.resetSimulation()
        self.num_obstacles = num_obstacles
        self.create_sim()
        for _ in range(50):
            p.stepSimulation()

def get_points(obj):
    """
    Given an object tuple (x, y, theta, type, l, h) it returns the set of points used to 
    approximate the shape
    """
    if obj[3] == 'cuboid':
        return poly_stick((obj[0], obj[1]), obj[2], obj[4], obj[5])
    elif obj[3] == 'disc':
        return poly_disc((obj[0], obj[1]), obj[2], obj[4], obj[5])
    else:
        return get_poly(obj+[0])



def sample(obj, obstacles, L, H):
    """
    Find a safe pose for obj given some obstacles - repeated sampling
    """
    workspace = poly_stick((L/2, H/2), 0, L, H)
    for i in range(MAX_ITERATIONS):
        x = np.random.uniform(0, L, 1)[0]
        y  = np.random.uniform(0, H, 1)[0]
        theta = np.random.uniform(0, 6.28, 1)[0]
        new_obj = [x, y, theta, obj[3], obj[4], obj[5]]
        obj_points = get_points(new_obj)

        if valid(obj_points, workspace, obstacles):
            return new_obj

    return -1

def valid(obj, workspace, placed_objects):
    #Check if obj overlaps with any other placed object
    for obs in placed_objects:
        if polysCollide(obj, obs):
            return False
    
    #Check if obj lies within the workspace or not
    workspace_polygon = Polygon(workspace)
    obj_polygon = Polygon(obj)
    if not workspace_polygon.contains(obj_polygon):
        return False

    return True

def repeated_sampling(L, H, objects):
    """
    Generate poses for all objects by repeatedly sampling till valid poses are found
    """
    for curr_idx in objects:
        obstacles = [get_points(objects[t]) for t in range(0, curr_idx)]
        obj_with_valid_pose = sample(objects[curr_idx], obstacles, L, H)
        
        if obj_with_valid_pose == -1:
            raise ValueError("Repeated Sampling failed")

        objects[curr_idx] = obj_with_valid_pose
    
    return objects

def generate_dense_arrangement(objects):
    '''
    generate dense arrangement
    '''
    num_obstacles = int(len(objects))
    simulator = compress_pushing(objects.copy(),num_obstacles)
    while 1:
        simulator.random_sparse()
        simulator.add_objects()
        simulator.push_for_dense()
        simulator.record_arrangement()
        workspace = poly_stick((1000/2, 1000/2), 0, 1000, 1000)
        for i in range(len(simulator.objects)):
            obj_points = get_points(simulator.objects[i])
            obstacles = [get_points(simulator.objects[j]) for j in range(i)]
            if not valid(obj_points, workspace, obstacles):
                print("collision detected")
                num_obstacles = max(0, num_obstacles-5)
                simulator.reset_simulation(num_obstacles)
                break
        else:
            p.disconnect()
            return simulator.objects
        


if __name__ == '__main__':
    simulator = compress_pushing()
    simulator.push_for_dense()
    simulator.maintain()
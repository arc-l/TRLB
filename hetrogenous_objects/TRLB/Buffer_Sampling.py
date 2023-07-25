from math import pi
from random import random, shuffle
from tools.util import poly_disc
from tools.util import poly_stick, polysCollide, polysWithin
from tools.general_objects import get_poly,get_radius

from numpy import array
from numpy.random import uniform

class Buffer_Sampling(object):
    def __init__(self, partial_action_sequence, partial_buffers, partial_schedule, start_arr, goal_arr, Height, Width, Length, ratio):
        self.partial_action_sequence = partial_action_sequence
        self.partial_buffers = partial_buffers
        self.partial_schedule = partial_schedule
        self.start_arr = start_arr
        self.goal_arr = goal_arr
        self.Height = Height
        self.Width = Width
        self.Length = Length
        self.ratio = ratio

        # debug
        self.num_collision = 0

        # Increamentally add constraints
        self.buffer_locations, self.trouble_action_index = self.partial_optimization()

    def partial_optimization(self):
        current_buffers = []
        current_at_start = []
        current_at_goal = []
        obs_dict = {}
        buffer_locations = {}

        current_at_start = list(self.start_arr.keys())

        for action in self.partial_action_sequence:
            current_objID = action[0]
            current_primitive_action = action[1]
            if current_primitive_action == 'b': # from start to buffer
                current_buffers.append(current_objID)
                # buffer_locations[current_objID] = self.start_arr[current_objID]
                direction = uniform(low=0.0, high=pi)
                if self.start_arr[current_objID][3] == 'cuboid':
                    polygon = array(
                        poly_stick([0,0], direction, 
                        self.start_arr[current_objID][4], self.start_arr[current_objID][5])
                    )
                elif self.start_arr[current_objID][3] == 'disc':
                    polygon = array(
                        poly_disc([0,0], direction, 
                        self.start_arr[current_objID][4], self.start_arr[current_objID][5])
                    )
                else:
                    new_pose = tuple([0,0,direction]+list(self.start_arr[current_objID][3:]))
                    polygon = array(get_poly(new_pose))
                # polygon = array(poly_stick([0,0], direction, self.Length, self.ratio*self.Length))
                buffer_locations[current_objID] = (
                    uniform(0 - min(polygon[:, 0]), self.Width - max(polygon[:, 0])), 
                    uniform(0 - min(polygon[:, 1]), self.Height - max(polygon[:, 1])),
                    direction,
                    *self.start_arr[current_objID][3:]
                    )
                obs_dict[current_objID] = []
                for obj_id in current_at_start:
                    if obj_id == current_objID:
                        continue
                    else:
                        obs_dict[current_objID].append(self.start_arr[obj_id])
                for obj_id in current_at_goal:
                    obs_dict[current_objID].append(self.goal_arr[obj_id])
            elif current_objID in self.partial_buffers: # from buffer to goal
                current_at_goal.append(current_objID)
                for obj_id in current_buffers:
                    if obj_id == current_objID:
                        continue
                    else:
                        obs_dict[obj_id].append(self.goal_arr[current_objID])
            else: # from start to goal
                current_at_goal.append(current_objID)
                for obj_id in current_buffers:
                    obs_dict[obj_id].append(self.goal_arr[current_objID])
            
            # check feasibility
            isfeasible, new_buffer_location_dict = self.buffer_sampling(current_buffers, buffer_locations, obs_dict)
            
            if not isfeasible:
                return buffer_locations, self.partial_action_sequence.index(action)
            else:
                # update buffer locations
                for buffer in new_buffer_location_dict.keys():
                    buffer_locations[buffer] = new_buffer_location_dict[buffer]
                # update obj status
                if current_primitive_action == 'b': # from start to buffer
                    current_at_start.remove(current_objID)
                elif current_objID in self.partial_buffers: # from buffer to goal
                    current_buffers.remove(current_objID)
                    del obs_dict[current_objID]
                    for obj_id in current_buffers:
                        obs_dict[obj_id].append(buffer_locations[current_objID])
                else: # from start to goal
                    current_at_start.remove(current_objID)
        return buffer_locations, float('inf')

    def buffer_sampling(self, buffers, buffer_current_location_dict, obs_dict):
        '''
        buffer_current_location_dict[objID] = (x,y,theta, 'type', length, width)
        '''
        
        isfeasible = False

        # # check previous locations
        # new_buffer_dict = {}
        # for bufferID in buffers:
        #     obs_list = obs_dict[bufferID] + list(new_buffer_dict.values())
        #     if not self.disk_collision_check(buffer_current_location_dict[bufferID], obs_list):
        #         new_buffer_dict[bufferID] = buffer_current_location_dict[bufferID]
        #     else:
        #         break
        # else:
        #     isfeasible = True

        timeout = 100
        while (timeout > 0) and (not isfeasible):
            timeout -= 1
            previous_buffer_feasible = True
            new_buffer_dict = {}
            shuffle(buffers)
            for bufferID in buffers:
                if not previous_buffer_feasible: # previous buffer is not placed
                    break
                previous_buffer_feasible = False
                inner_timeout = 10
                obs_list = obs_dict[bufferID] + list(new_buffer_dict.values())
                
                # check the current position
                if not self.collision_check(buffer_current_location_dict[bufferID], obs_list):
                    previous_buffer_feasible = True 
                    new_buffer_dict[bufferID] = buffer_current_location_dict[bufferID]
                    continue
                # generate new positions
                while inner_timeout >= 0:
                    inner_timeout -= 1
                    direction = uniform(low=0.0, high=pi)
                    if self.start_arr[bufferID][3] == 'cuboid':
                        polygon = array(
                            poly_stick([0,0], direction, 
                            self.start_arr[bufferID][4], self.start_arr[bufferID][5])
                        )
                    elif self.start_arr[bufferID][3] == 'disc':
                        polygon = array(
                            poly_disc([0,0], direction, 
                            self.start_arr[bufferID][4], self.start_arr[bufferID][5])
                        )
                    else:
                        new_pose = tuple([0,0,direction]+list(self.start_arr[bufferID][3:]))
                        polygon = array(get_poly(new_pose))
                    # polygon = array(poly_stick([0,0], direction, self.Length, self.ratio*self.Length))
                    new_location = (
                        uniform(0 - min(polygon[:, 0]), self.Width - max(polygon[:, 0])), 
                        uniform(0 - min(polygon[:, 1]), self.Height - max(polygon[:, 1])),
                        direction,
                        *self.start_arr[bufferID][3:]
                        )
                    if not self.collision_check(new_location, obs_list):
                        previous_buffer_feasible = True 
                        new_buffer_dict[bufferID] = new_location
                        break
            isfeasible = previous_buffer_feasible # the last buffer is feasible
        
        return isfeasible, new_buffer_dict
    
    def collision_check(self, obj, obs_list):
        L=self.Width
        H=self.Height
        isColliding = False
        if obj[3] == 'cuboid':
            obj_poly = array(
                poly_stick(
                    [obj[0], obj[1]], obj[2], obj[4], obj[5] 
                )
            )
        elif obj[3] == 'disc':
            obj_poly = array(
                poly_disc(
                    [obj[0], obj[1]], obj[2], obj[4], obj[5]
                )
            )
        else:
            obj_poly = array(
                get_poly(obj)
            )
        # making sure that obj is not at the boundary
        workspace = poly_stick((L/2, H/2), 0, L, H)

        if polysWithin(obj_poly, workspace):
            isColliding = True
            return isColliding

        for obs in obs_list:
            if obs[3] == 'cuboid':
                obs_poly = array(
                    poly_stick(
                        [obs[0], obs[1]], obs[2], obs[4], obs[5] 
                    )
                )
            elif obs[3] == 'disc':
                obs_poly = array(
                    poly_disc(
                        [obs[0], obs[1]], obs[2], obs[4], obs[5] 
                 )
                )
            else:
                obs_poly = array(get_poly(obs))
            if polysCollide(obj_poly, obs_poly):
                isColliding = True
                break
        return isColliding


    def disk_collision_check(self, obj, obs_list):
        isColliding = False
        for obs in obs_list:
            self.num_collision += 1
            if ((obj[0]-obs[0])**2+(obj[1]-obs[1])**2<=4*self.radius**2):
                isColliding = True 
                break
        return isColliding

from random import random, shuffle

class Buffer_Sampling(object):
    def __init__(self, partial_action_sequence, partial_buffers, partial_schedule, start_arr, goal_arr, Height, Width, radius):
        self.partial_action_sequence = partial_action_sequence
        self.partial_buffers = partial_buffers
        self.partial_schedule = partial_schedule
        self.start_arr = start_arr
        self.goal_arr = goal_arr
        self.Height = Height
        self.Width = Width
        self.radius = radius

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
                buffer_locations[current_objID] = (
                        random()*(self.Width-2*self.radius)+self.radius,
                        random()*(self.Height-2*self.radius)+self.radius
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
                if not self.disk_collision_check(buffer_current_location_dict[bufferID], obs_list):
                    previous_buffer_feasible = True 
                    new_buffer_dict[bufferID] = buffer_current_location_dict[bufferID]
                    continue
                # generate new positions
                while inner_timeout >= 0:
                    inner_timeout -= 1
                    new_location = (
                        random()*(self.Width-2*self.radius)+self.radius,
                        random()*(self.Height-2*self.radius)+self.radius
                    )
                    if not self.disk_collision_check(new_location, obs_list):
                        previous_buffer_feasible = True 
                        new_buffer_dict[bufferID] = new_location
                        break
            isfeasible = previous_buffer_feasible # the last buffer is feasible
        
        return isfeasible, new_buffer_dict
    
    def disk_collision_check(self, obj, obs_list):
        isColliding = False
        for obs in obs_list:
            self.num_collision += 1
            if ((obj[0]-obs[0])**2+(obj[1]-obs[1])**2<=4*self.radius**2):
                isColliding = True 
                break
        return isColliding

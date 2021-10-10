import gurobipy as gp

class Buffer_Generation(object):
    def __init__(self, partial_action_sequence, partial_buffers, partial_schedule, start_arr, goal_arr, Height, Width, radius):
        self.partial_action_sequence = partial_action_sequence
        self.partial_buffers = partial_buffers
        self.partial_schedule = partial_schedule
        self.start_arr = start_arr
        self.goal_arr = goal_arr
        self.Height = Height
        self.Width = Width
        self.radius = radius

        # Increamentally add constraints
        self.buffer_locations, self.trouble_action_index = self.partial_optimization()

    def partial_optimization(self):
        # solution
        buffer_locations = {}
        
        m = gp.Model()
        m.setParam('OutputFlag', 0)
        m.setParam('Threads', 1)
        m.setParam('NonConvex', 2)
        m.setParam('Seed', 1)
        m.setParam('TimeLimit', 500)

        b = len(self.partial_buffers)

        ### variables
        x = m.addVars(b, vtype=gp.GRB.CONTINUOUS, lb=self.radius, ub=self.Width-self.radius)
        y = m.addVars(b, vtype=gp.GRB.CONTINUOUS, lb=self.radius, ub=self.Height-self.radius)

        current_buffers = []
        current_at_start = []
        current_at_goal = []

        current_at_start = list(self.start_arr.keys())

        for action in self.partial_action_sequence:
            current_objID = action[0]
            current_primitive_action = action[1]
            if current_primitive_action == 'b': # from start to buffer
                buffer_index = self.partial_buffers.index(current_objID)
                current_at_start.remove(current_objID)
                for obj_id in current_at_start:
                    m.addQConstr(
                        (x[buffer_index]-self.start_arr[obj_id][0])*(x[buffer_index]-self.start_arr[obj_id][0]) + 
                        (y[buffer_index]-self.start_arr[obj_id][1])*(y[buffer_index]-self.start_arr[obj_id][1]) >= 
                        4*self.radius**2 
                    )
                for obj_id in current_at_goal:
                    m.addQConstr(
                        (x[buffer_index]-self.goal_arr[obj_id][0])*(x[buffer_index]-self.goal_arr[obj_id][0]) + 
                        (y[buffer_index]-self.goal_arr[obj_id][1])*(y[buffer_index]-self.goal_arr[obj_id][1]) >= 
                        4*self.radius**2 
                    )
                for obj_id in current_buffers:
                    old_buffer_index = self.partial_buffers.index(obj_id)
                    m.addQConstr(
                        (x[buffer_index]-x[old_buffer_index])*(x[buffer_index]-x[old_buffer_index]) + 
                        (y[buffer_index]-y[old_buffer_index])*(y[buffer_index]-y[old_buffer_index]) >= 
                        4*self.radius**2
                    )
                current_buffers.append(current_objID)
            elif current_objID in self.partial_buffers: # from buffer to goal
                current_buffers.remove(current_objID)
                for obj_id in current_buffers:
                    old_buffer_index = self.partial_buffers.index(obj_id)
                    m.addQConstr(
                        (x[old_buffer_index]-self.goal_arr[current_objID][0])*(x[old_buffer_index]-self.goal_arr[current_objID][0]) + 
                        (y[old_buffer_index]-self.goal_arr[current_objID][1])*(y[old_buffer_index]-self.goal_arr[current_objID][1]) >= 
                        4*self.radius**2 
                    )
                current_at_goal.append(current_objID)
            else: # from start to goal
                current_at_start.remove(current_objID)
                for obj_id in current_buffers:
                    old_buffer_index = self.partial_buffers.index(obj_id)
                    m.addQConstr(
                        (x[old_buffer_index]-self.goal_arr[current_objID][0])*(x[old_buffer_index]-self.goal_arr[current_objID][0]) + 
                        (y[old_buffer_index]-self.goal_arr[current_objID][1])*(y[old_buffer_index]-self.goal_arr[current_objID][1]) >= 
                        4*self.radius**2 
                    )
                current_at_goal.append(current_objID)
            
            # check feasibility
            m.optimize()

            try:
                for buffer_id, buffer_obj_id in enumerate(self.partial_buffers):
                    test = (x[buffer_id].x, y[buffer_id].x)
                    buffer_locations[buffer_obj_id] = test
                isfeasible = True
            except Exception:
                isfeasible = False
            
            if not isfeasible:
                return buffer_locations, self.partial_action_sequence.index(action)
        return buffer_locations, float('inf')


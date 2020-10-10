## Project: 3D Motion Planning
![](3Dmotionplanning)

---


# Required Steps for a Passing Submission:
#### Load the 2.5D map in the colliders.csv file describing the environment and Discretize the environment into a grid or graph representation.

# 
        north, east, att = global_to_local(self.global_position, self.global_home)
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
#### Define the start and goal locations.

        grid_start = (int(north - north_offset),
                      int(east - east_offset))
​        global_goal = [0,0,0]
​        global_goal[0] = lat # Green area
​        global_goal[1] = long
​        global_position = self.global_position
​        global_goal[2] = global_position[2]
​        local_goal = global_to_local(global_goal,self.global_home)

        # Set goal as some arbitrary position on the grid
        grid_goal = (int(local_goal[0]-north_offset),int(local_goal[1]-east_offset))


#### Perform a search using A* or other search algorithm.



path, _ = a_star(grid, heuristic, grid_start, grid_goal)





#### Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.



def collinearity(self,point_1, point_2, point_3):
        matrix = np.concatenate((point_1, point_2, point_3), 0)
        det = np.linalg.det(matrix)
        return abs(det) < 1e-6

to_remove=[]
        for  i in range(len(pruned_path) - 2):
            point_1 = np.array([pruned_path[i][0],pruned_path[i][1], 1.]).reshape(1, -1)
            point_2 = np.array([pruned_path[i+1][0],pruned_path[i+1][1], 1.]).reshape(1, -1)
            point_3 = np.array([pruned_path[i+2][0],pruned_path[i+2][1], 1.]).reshape(1, -1)


            if self.collinearity(point_1, point_2, point_3):
                to_remove.append(pruned_path[i + 1])


#### Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].





​         waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        # 
​        self.waypoints = waypoints
        # 
​        self.send_waypoints()







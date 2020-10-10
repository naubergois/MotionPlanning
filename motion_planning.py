import argparse
import time
import msgpack
from enum import Enum, auto
import re

import numpy as np

from planning_utils import a_star, heuristic, create_grid
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection,args,points,init_lat,init_long,alts,set_home):
        super().__init__(connection)
        self.args=args
        self.set_home=set_home
        self.init_lat=init_lat
        self.init_long=init_long
        self.points=points
        self.alts=alts
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        print(self.local_velocity[0:2])
        print('state ',self.flight_state)
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0 and np.linalg.norm(self.local_velocity[0:2]) > 0.0:
                        print('landing')
                        self.landing_transition()
        elif self.flight_state == States.LANDING:
            print('landing state')
            if np.linalg.norm(self.local_velocity[0:2]) == 0.0:
                        print('landed')
                        self.disarming_transition()
                        time.sleep(10)

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)


    def collinearity(self,point_1, point_2, point_3):
        matrix = np.concatenate((point_1, point_2, point_3), 0)
        det = np.linalg.det(matrix)
        return abs(det) < 1e-6

    def sub_plan(self,start,lat,long,alt,data):

        TARGET_ALTITUDE = alt
        SAFETY_DISTANCE = 10
        # TODO: retrieve current global position
        north, east, att = global_to_local(self.global_position, self.global_home)

        # TODO: convert to current local position using global_to_local()

        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))


        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)

        print(grid.shape)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        grid_start = start

        global_goal = [0,0,0]
        global_goal[0] = lat # Green area
        global_goal[1] = long
        global_position = self.global_position
        global_goal[2] = global_position[2]
        local_goal = global_to_local(global_goal,self.global_home)

        # Set goal as some arbitrary position on the grid
        grid_goal = (int(local_goal[0]-north_offset),int(local_goal[1]-east_offset))
        # TODO: adapt to set goal as latitude / longitude position and convert

        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        print('Local Start and Goal: ', grid_start, grid_goal)
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        # TODO: prune path to minimize number of waypoints
        # TODO (if you're feeling ambitious): Try a different approach altogether!

        with open('path.txt', 'w') as f:
            for item in path:
                print(item)
                f.write("%s\n" % str(item))




        pruned_path = [p for p in path]
        to_remove=[]
        for  i in range(len(pruned_path) - 2):
            point_1 = np.array([pruned_path[i][0],pruned_path[i][1], 1.]).reshape(1, -1)
            point_2 = np.array([pruned_path[i+1][0],pruned_path[i+1][1], 1.]).reshape(1, -1)
            point_3 = np.array([pruned_path[i+2][0],pruned_path[i+2][1], 1.]).reshape(1, -1)


            if self.collinearity(point_1, point_2, point_3):
                to_remove.append(pruned_path[i + 1])

        for  i in range(len(to_remove)):
            #print('removing ')
            pruned_path.remove(to_remove[i])
        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

        return pruned_path[len(pruned_path)-1]

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values

        # TODO: set home position to (lon0, lat0, 0)
        if (self.set_home):
            self.set_home_position(self.init_long,self.init_lat,0)
        # TODO: retrieve current global position
        global_position = self.global_position

        # TODO: retrieve current global position
        north, east, att = global_to_local(self.global_position, self.global_home)
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)

        print(grid.shape)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        grid_start = (-north_offset, -east_offset)


        # TODO: convert start position to current position rather than map center
        grid_start = (int(north - north_offset),
                      int(east - east_offset))

        #new_start=self.sub_plan(grid_start, -122.397632,37.793108,2,data)
        #point2=self.sub_plan(new_start, -122.398017,37.795503,100,data)
        #point3=self.sub_plan(point2, -122.394965,37.796663,200,data)
        old_start=grid_start
        alt=2
        for i in range(len(self.points)):
            new_start=self.sub_plan(old_start, self.points[i][0],self.points[i][1],self.alts[i],data)
            alt=alt+100
            old_start=new_start
        #new_start=self.sub_plan(grid_start, self.points[0][0],self.points[0][1],2,data)
        #point2=self.sub_plan(new_start, self.points[1][0],self.points[1][1],100,data)
        #point3=self.sub_plan(point2, self.points[2][0],self.points[2][1],200,data)

        self.old_start=old_start

        # TODO: set home position to (lon0, lat0, 0)


    def get_old_start(self):
        return self.old_start


    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")


    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)

    alts=[2,100,200]
    points=[(-122.397632,37.793108),(-122.398017,37.795503),(-122.394965,37.796663)]
    file = 'colliders.csv'

    with open(file) as f:
        first_line = f.readline()
    match = re.match(r'^lat0 (.*), lon0 (.*)$', first_line)
    lat0 = match.group(1)
    lon0 = match.group(2)

    lat0=float(lat0)
    lon0=float(lon0)

    print('lat0: ',lat0)
    print('lon0',lon0)


    alts=[2,100,200]



    drone = MotionPlanning(conn, args,points,lat0,lon0,alts,True)
    time.sleep(1)

    drone.start()



    points=[(-122.398017,37.795503),(-122.398778,37.797214),(-122.395567,37.796858)]

    alts=[200,200,200]

    time.sleep(10)
    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn, args,points,lat0,lon0,alts,False)
    time.sleep(2)

    drone.start()

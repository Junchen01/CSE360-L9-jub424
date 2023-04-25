import socket
import time
import sys
import time
import math
import numpy as np
import networkx as nx

from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1

positions = {}
rotations = {}
target = [1,0.1]
IP_ADDRESS = '192.168.0.203'
clientAddress = "192.168.0.35"
optitrackServerAddress = "192.168.0.4"
robot_id = 303
v = 0
omega = 0
t = 0
r = 1
distance = 10000

coordinates = [(-4.5, 2.6), 
               (-3.712652561385562, 1.927262157731573), 
               (-0.07986378935238658, 2.1339701394649433), 
               (-1.9652179979683462, 0.6115308258438046), 
               (-3.671134015160076, 0.5689270634686083), 
               (-4.399200342482153, -0.2107989267820498), 
               (-1.772093359572139, 2.166299185667051), 
               (-0.9836137770032711, -0.15125786891311427), 
               (-3.1014768626823788, 1.8797638321847696), 
               (-0.4904194009508043, 1.0319365038940584), 
               (-1.1749874651354664, 2.090140404481539), 
               (-3.5497436892919176, 0.4680222178178347), 
               (-3.8800891077833666, 1.5464695049173875), 
               (-3.394590473310317, 1.9503822521124334), 
               (0, -0.5)]

def find_path():
    G = nx.Graph()
    # Calculate the distance between each pair of points
    for i in range(len(coordinates)):
        for j in range(len(coordinates)):
            distance_now = math.dist(coordinates[i], coordinates[j])
            if distance_now < 1.5:
                G.add_edge(i, j, weight=distance_now)
    # Find the shortest path
    # Find the shortest path between two nodes
    shortest_path = nx.shortest_path(G, 0, 2, weight='weight')

    shortest_path = [coordinates[i] for i in shortest_path]
    return shortest_path

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles

    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)

    rotations[robot_id] = rotz




def find_distance(positions, target):
    x_diff = target[0] - positions[0]
    y_diff = target[1] - positions[1]
    dis = x_diff * x_diff + y_diff * y_diff
    return math.sqrt(dis)

def find_orientation(positions, target):
    x_diff = target[0] - positions[0]
    y_diff = target[1] - positions[1]
    orientation = np.arctan2(y_diff, x_diff)
    return orientation

def send_speed(v, omega):
    u = np.array([v - omega, v + omega])
    u[u > 1500] = 1500
    u[u < -1500] = -1500
    # Send control input to the motors
    command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])
    s.send(command.encode('utf-8'))

def find_orientation_error(desired_orientation, rotations_now):
    sin_tmp = math.sin(desired_orientation - math.radians(rotations_now))
    cos_tmp = math.cos(desired_orientation - math.radians(rotations_now))
    orientation_err = np.arctan2(sin_tmp, cos_tmp)
    orientation_err = math.degrees(orientation_err)
    return orientation_err

def points_in_circle(a,b,r):
    #The lower this value the higher quality the circle is with more points generated
    stepSize = 0.1
    #Generated vertices
    positions = []
    t = 0
    while t < 2* math.pi:
        positions.append((r * math.cos(t) + a, r * math.sin(t) + b))
        t += stepSize
    return positions



# Connect to the robot
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((IP_ADDRESS, 5000))
print('Connected')
# This will create a new NatNet client
streaming_client = NatNetClient()
streaming_client.set_client_address(clientAddress)
streaming_client.set_server_address(optitrackServerAddress)
streaming_client.set_use_multicast(True)
# Configure the streaming client to call our rigid body handler on the emulator to send data out.
streaming_client.rigid_body_listener = receive_rigid_body_frame

# Start up the streaming client now that the callbacks are set up.
# This will run perpetually, and operate on a separate thread.
is_running = streaming_client.run()


target_list = find_path()
# target_list = [(0,-0.5)]
target_list_len = len(target_list)
target_list_at = 0
target = target_list[target_list_at]
time_index = 0

try:
    while is_running:
        if robot_id in positions:
            print("target: " + str(target))
            # last position
            # print('Last position', positions[robot_id], ' rotation', rotations[robot_id])
            time_index = time_index + 1
            # print("time: ",time_index)

            distance = find_distance(positions[robot_id], target)
            print('distance: ', distance)

            desired_orientation = find_orientation(positions[robot_id], target)
            # print('desired orientation: ', desired_orientation)
            # print('current orientation: ', rotations[robot_id])

            orientation_error = find_orientation_error(desired_orientation, rotations[robot_id])
            print('orientation error: ', orientation_error)


            print('Last position', positions[robot_id])

            
            print('-----------------------------')
            
            v = 1500 * distance
            omega = 200 * orientation_error
            send_speed(v, omega)
            
            x_tmp = target[0]
            y_tmp = target[1]
            # target =( r * math.cos(time_index) + x_tmp,  r * math.sin(time_index) + y_tmp)
            

            time.sleep(0.1)

            if distance < 0.6 and target_list_at < target_list_len:
                target = target_list[target_list_at]
                target_list_at += 1
                # print('point change to: ' + target)
            elif  distance < 0.6 and target_list_at == target_list_len:
                break
            
except KeyboardInterrupt:
    # STOP
    command = 'CMD_MOTOR#00#00#00#00\n'
    s.send(command.encode('utf-8'))

# Close the connection
command = 'CMD_MOTOR#00#00#00#00\n'
s.send(command.encode('utf-8'))
s.shutdown(2)
s.close()
streaming_client.shutdown()
# 1. Import libraries

import read_moves
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
import rospkg
from picamera import PiCamera
import time
import socket
import numpy
import os

from dotenv import load_dotenv
load_dotenv()

counter = 0
oldX = 0.0
oldY = 0.0
PI = 3.14
rotation = None

# Getting odometry data
def PositionCB(data):
    global oldY, oldX, rotation
    oldX = data.pose.pose.position.x
    oldY = data.pose.pose.position.y
    rotation = data.pose.pose.orientation


# 2. Connect to Maestro4

print("Connecting to server Maestro...")

HOST = os.getenv('HOST_ADDRESS')
PORT = int(os.getenv('PORT'))
BUFFER_SIZE = 2048

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect((HOST, PORT))

camera = PiCamera()
time.sleep(1)

movement = read_moves.RobotMovement()
rospy.Subscriber("/odom", Odometry, PositionCB)

f3 = open("route_adagrad.txt", "w")		
coords = '({oldX}, {oldY})\n'.format(oldX=oldX, oldY=oldY)
f3.write(coords)				

counter = 1
while True:

    # 3. Snap photo

    sending_image = 'send_images/send{counter}.jpeg'.format(counter=counter)
    camera.capture(sending_image)
    print("Snapshot taken.")

    # 4. Send photo to Maestro4

    file = open(sending_image, 'rb')
    file_data = file.read(BUFFER_SIZE)

    while file_data:
        client.send(file_data)
        file_data = file.read(BUFFER_SIZE)
    client.send(b"edima")

    file.close()

    # 5. Receive result from Maestro4
    recv_data = client.recv(BUFFER_SIZE)
    print(recv_data)


    # 6. Move according to class


    if recv_data == b"0":
        print("Received zero. Breaking...")
        movement.stop()
        coords = '({oldX}, {oldY})\n'.format(oldX=oldX, oldY=oldY)
        f3.write(coords)			
        break
    elif recv_data == b"1":
        print("Received one.")
        movement.move_forward(-0.5)
        time.sleep(1)
        movement.move_backwards(+0.5)
        time.sleep(1)
    elif recv_data == b"2":
        print("Received two.")
        movement.move_forward(+0.6)
        time.sleep(1)
        movement.move_forward(+0.6)
        time.sleep(1)
        movement.move_forward(+0.6)
        time.sleep(1)
        movement.move_forward(+0.6)
        time.sleep(1)
    elif recv_data == b"3":
        print("Received three.")
        movement.move_forward(+0.5)
        time.sleep(1)
        movement.move_forward(+0.5)
        time.sleep(1)
        movement.move_forward(+0.5)
        time.sleep(1)
        movement.move_forward(+0.5)
        time.sleep(1)
        coords = '({oldX}, {oldY})\n'.format(oldX=oldX, oldY=oldY)		
        f3.write(coords)			
        movement.rotate(False)
        time.sleep(5)
        movement.move_forward(+0.6)
        time.sleep(1)
    elif recv_data == b"4":
        print("Received four.")
        movement.move_forward(+0.5)
        time.sleep(1)
        movement.move_forward(+0.5)
        time.sleep(1)
        movement.move_forward(+0.5)
        time.sleep(1)
        movement.move_forward(+0.5)
        time.sleep(1)
        coords = '({oldX}, {oldY})\n'.format(oldX=oldX, oldY=oldY)		
        f3.write(coords)		
        movement.rotate()
        time.sleep(5)
        movement.move_forward(+0.6)
        time.sleep(1)
        
    rospy.sleep(1)
    
    coords = '({oldX}, {oldY})\n'.format(oldX=oldX, oldY=oldY)		
    f3.write(coords)			

    counter += 1


print('Images sent: {counter}.'.format(counter=counter))
print("Closing socket and exiting...")
f3.close()					
client.close()

import rospy
import numpy as np
import baxter  # here we are importing the baxter.py interface. (cause it's in this same folder, but in your project please clone the repo as submodule and import the interface as described in the readme)
import os
import speckleRobotServer

"""
Usage: Execute the following command in the terminal:
python DataCrawler.py

Input: 
move (direction) (distance)
rotate (direction) (angle)
move_sensor (object_index)
grip (object_index)
reverse
create_dataset
exit


direction: up, down, left, right, forward, backward (for move) and wrist_cw, wrist_ccw, flex_arm, extend_arm (for rotate)
angle: angle in degrees
distance: distance in cm
"""

rospy.init_node("testing")
rospy.sleep(2.0)
robot_left = baxter.BaxterRobot(rate=100, arm="left")
rospy.sleep(2.0)
robot_right = baxter.BaxterRobot(rate=100, arm="right")

robot_left.set_robot_state(True)
robot_right.set_robot_state(True)

server, connection, address = speckleRobotServer.startServer()


robot_right.gripper_calibrate()
robot_right.gripper_prepare_to_grip()
print(robot_left._endpoint_state.pose.position)
robot_right.move_to_neutral()
robot_left.move_to_neutral()

reverse_operation = None

object_positions = [[0.7202440738293987, 0.30346615561828016, 0.1914454910919908]]


def rotation(direction, angle):
    global reverse_operation
    angles = robot_left.joint_angle()
    angle = np.deg2rad(angle)
    if direction == "wrist_cw":
        angles["left_w2"] += angle
        reverse_operation = ("rotate", "wrist_ccw", np.rad2deg(angle))
    elif direction == "wrist_ccw":
        angles["left_w2"] -= angle
        reverse_operation = ("rotate", "wrist_cw", np.rad2deg(angle))
    elif direction == "flex_arm":
        angles["left_w1"] -= angle
        reverse_operation = ("rotate", "extend_arm", np.rad2deg(angle))
    elif direction == "extend_arm":
        angles["left_w1"] += angle
        reverse_operation = ("rotate", "flex_arm", np.rad2deg(angle))
    else:
        print("Invalid direction")
        return
    for i in range(5): # maximum rotation angle is 60 degrees for 5
        robot_left.set_joint_position(angles)
        rospy.sleep(0.2)


def movement(direction, distance):
    global reverse_operation
    p = robot_left._endpoint_state.pose.position
    q = robot_left._endpoint_state.pose.orientation
    distance = distance / 100
    if direction == "up":
        p.z += distance
        reverse_operation = ("move", "down", distance * 100)
    elif direction == "down":
        p.z -= distance
        reverse_operation = ("move", "up", distance * 100)
    elif direction == "left":
        p.y += distance
        reverse_operation = ("move", "right", distance * 100)
    elif direction == "right":
        p.y -= distance
        reverse_operation = ("move", "left", distance * 100)
    elif direction == "forward":
        p.x += distance
        reverse_operation = ("move", "backward", distance * 100)
    elif direction == "backward":
        p.x -= distance
        reverse_operation = ("move", "forward", distance * 100)
    else:
        print("Invalid direction")
        return
    robot_left.set_cartesian_position([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])


def grip(object_position):
    global robot_right, robot_left
    robot_left.move_to_neutral()
    go_to_object(object_position, "right")
    robot_right.set_cartesian_position([object_position[0], object_position[1], object_position[2] - 0.4], [0,1,0,0])
    rospy.sleep(2.0)
    robot_right.gripper_grip()
    rospy.sleep(2.0)
    robot_right.set_cartesian_position([object_position[0], object_position[1], object_position[2]], [0,1,0,0])
    rospy.sleep(2.0)
    robot_right.set_cartesian_position([object_position[0], object_position[1], object_position[2] - 0.4], [0,1,0,0])
    rospy.sleep(2.0)
    robot_right.gripper_release()
    rospy.sleep(2.0)
    robot_right.set_cartesian_position([object_position[0], object_position[1], object_position[2]], [0,1,0,0])
    rospy.sleep(2.0)
    robot_right.move_to_neutral()



def go_to_object(object_position, arm):
    global robot_right, robot_left
    if arm == "left":
        robot = robot_left
    else:
        robot = robot_right
    robot.set_cartesian_position(object_position, [0, 1, 0, 0])
    rospy.sleep(2.0)


while True:
    command = input("Enter command: ")
    if command == "exit":
        break
    command = command.split()
    move = command[0]
    if move == "rotate":
        direction = command[1]
        angle = float(command[2])
        rotation(direction, angle)
    elif move == "move":
        direction = command[1]
        distance = float(command[2])
        movement(direction, distance)
    elif move == "reverse":
        if reverse_operation[0] == "rotate":
            rotation(reverse_operation[1], reverse_operation[2])
        elif reverse_operation[0] == "move":
            movement(reverse_operation[1], reverse_operation[2])
    elif move == "grip":
        grip(object_positions[int(command[1])])
    elif move == "move_sensor":
        go_to_object(object_positions[int(command[1])], "left")

    elif move == "create_dataset":
        start_time = rospy.get_time()
        dataset_folder = 'SpeckleRobotDataset'
        material_type = command[1] #'real_lemon'

        
        currentShotCount = 1

        fileName = f'{dataset_folder}/{material_type}/{material_type}_{currentShotCount}.jpg'
        if not os.path.exists(f'{dataset_folder}'):
            os.makedirs(f'{dataset_folder}')
        if not os.path.exists(f'{dataset_folder}/{material_type}'):
            os.makedirs(f'{dataset_folder}/{material_type}')

        speckleRobotServer.receiveShot(connection, fileName)
        currentShotCount += 1
        fileName = f'{dataset_folder}/{material_type}/{material_type}_{currentShotCount}.jpg'

        for i in range(9):
            rotation("wrist_cw", 5)
            speckleRobotServer.receiveShot(connection, fileName)
            currentShotCount += 1
            fileName = f'{dataset_folder}/{material_type}/{material_type}_{currentShotCount}.jpg'

        rotation("wrist_ccw", 45)

        for i in range(9):
            rotation("wrist_ccw", 5)
            speckleRobotServer.receiveShot(connection, fileName)
            currentShotCount += 1
            fileName = f'{dataset_folder}/{material_type}/{material_type}_{currentShotCount}.jpg'

        rotation("wrist_cw", 45)

        for i in range(4):
            movement("left", 0.5)
            speckleRobotServer.receiveShot(connection, fileName)
            currentShotCount += 1
            fileName = f'{dataset_folder}/{material_type}/{material_type}_{currentShotCount}.jpg'

            for i in range(9):
                rotation("wrist_cw", 5)
                speckleRobotServer.receiveShot(connection, fileName)
                currentShotCount += 1
                fileName = f'{dataset_folder}/{material_type}/{material_type}_{currentShotCount}.jpg'

            rotation("wrist_ccw", 45)

        movement("right", 2)

        for i in range(4):
            movement("right", 0.5)
            speckleRobotServer.receiveShot(connection, fileName)
            currentShotCount += 1
            fileName = f'{dataset_folder}/{material_type}/{material_type}_{currentShotCount}.jpg'

            for i in range(9):
                rotation("wrist_ccw", 5)
                speckleRobotServer.receiveShot(connection, fileName)
                currentShotCount += 1
                fileName = f'{dataset_folder}/{material_type}/{material_type}_{currentShotCount}.jpg'

            rotation("wrist_cw", 45)

        movement("left", 2)

        for i in range(4):
            movement("forward", 0.25)
            speckleRobotServer.receiveShot(connection, fileName)
            currentShotCount += 1
            fileName = f'{dataset_folder}/{material_type}/{material_type}_{currentShotCount}.jpg'
            for i in range(5):
                rotation("extend_arm", 0.2)
                speckleRobotServer.receiveShot(connection, fileName)
                currentShotCount += 1
                fileName = f'{dataset_folder}/{material_type}/{material_type}_{currentShotCount}.jpg'

            rotation("flex_arm", 1)

        movement("backward", 1)

        for i in range(4):
            movement("backward", 0.25)
            speckleRobotServer.receiveShot(connection, fileName)
            currentShotCount += 1
            fileName = f'{dataset_folder}/{material_type}/{material_type}_{currentShotCount}.jpg'
            for i in range(5):
                rotation("flex_arm", 0.2)
                speckleRobotServer.receiveShot(connection, fileName)
                currentShotCount += 1
                fileName = f'{dataset_folder}/{material_type}/{material_type}_{currentShotCount}.jpg'

            rotation("extend_arm", 1)

        movement("forward", 1)

        print(f"DATASET FINISHED IN {(rospy.get_time() - start_time) / 60} MINUTES")
    
    
    
    
    elif move == "dataset_test":
        start_time = rospy.get_time()
        dataset_folder = 'SpeckleRobotDataset'
        material_type = command[1] #'real_apple'

        
        currentShotCount = 1

        fileName = f'{dataset_folder}/{material_type}/{material_type}_{currentShotCount}.jpg'
        if not os.path.exists(f'{dataset_folder}'):
            os.makedirs(f'{dataset_folder}')
        if not os.path.exists(f'{dataset_folder}/{material_type}'):
            os.makedirs(f'{dataset_folder}/{material_type}')

        speckleRobotServer.receiveShot(connection, fileName)
        currentShotCount += 1
        fileName = f'{dataset_folder}/{material_type}/{material_type}_{currentShotCount}.jpg'

        for i in range(2):
            rotation("wrist_cw", 15)
            speckleRobotServer.receiveShot(connection, fileName)
            currentShotCount += 1
            fileName = f'{dataset_folder}/{material_type}/{material_type}_{currentShotCount}.jpg'

        rotation("wrist_ccw", 30)

        for i in range(2):
            rotation("wrist_ccw", 15)
            speckleRobotServer.receiveShot(connection, fileName)
            currentShotCount += 1
            fileName = f'{dataset_folder}/{material_type}/{material_type}_{currentShotCount}.jpg'

        rotation("wrist_cw", 30)

        for i in range(2):
            movement("left", 1)
            speckleRobotServer.receiveShot(connection, fileName)
            currentShotCount += 1
            fileName = f'{dataset_folder}/{material_type}/{material_type}_{currentShotCount}.jpg'

            for i in range(2):
                rotation("wrist_cw", 15)
                speckleRobotServer.receiveShot(connection, fileName)
                currentShotCount += 1
                fileName = f'{dataset_folder}/{material_type}/{material_type}_{currentShotCount}.jpg'

            rotation("wrist_ccw", 30)

        movement("right", 2)

        for i in range(2):
            movement("right", 1)
            speckleRobotServer.receiveShot(connection, fileName)
            currentShotCount += 1
            fileName = f'{dataset_folder}/{material_type}/{material_type}_{currentShotCount}.jpg'

            for i in range(2):
                rotation("wrist_ccw", 15)
                speckleRobotServer.receiveShot(connection, fileName)
                currentShotCount += 1
                fileName = f'{dataset_folder}/{material_type}/{material_type}_{currentShotCount}.jpg'

            rotation("wrist_cw", 30)

        movement("left", 2)

        for i in range(1):
            movement("forward", 1)
            speckleRobotServer.receiveShot(connection, fileName)
            currentShotCount += 1
            fileName = f'{dataset_folder}/{material_type}/{material_type}_{currentShotCount}.jpg'
            for i in range(1):
                rotation("extend_arm", 1)
                speckleRobotServer.receiveShot(connection, fileName)
                currentShotCount += 1
                fileName = f'{dataset_folder}/{material_type}/{material_type}_{currentShotCount}.jpg'

            rotation("flex_arm", 1)

        movement("backward", 1)

        for i in range(1):
            movement("backward", 1)
            speckleRobotServer.receiveShot(connection, fileName)
            currentShotCount += 1
            fileName = f'{dataset_folder}/{material_type}/{material_type}_{currentShotCount}.jpg'
            for i in range(1):
                rotation("flex_arm", 1)
                speckleRobotServer.receiveShot(connection, fileName)
                currentShotCount += 1
                fileName = f'{dataset_folder}/{material_type}/{material_type}_{currentShotCount}.jpg'

            rotation("extend_arm", 1)

        movement("forward", 1)

        print(f"DATASET FINISHED IN {(rospy.get_time() - start_time) / 60} MINUTES")
    else:
        print("Invalid command")
        continue

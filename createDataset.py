import speckleRobotServer
import keyboard
import os

dataset_folder = 'SpeckleRobotDataset'
material_type = 'pencilcase_cloth'

server, connection, address = speckleRobotServer.startServer()

currentShotCount = 1
while True:
    # If space is pressed, receive a shot
    fileName = f'{dataset_folder}/{material_type}/{material_type}_{currentShotCount}.jpg'

    # if folder does not exist, create it
    if not os.path.exists(f'{dataset_folder}'):
        os.makedirs(f'{dataset_folder}')
    if not os.path.exists(f'{dataset_folder}/{material_type}'):
        os.makedirs(f'{dataset_folder}/{material_type}')

    if keyboard.is_pressed('space'):
        speckleRobotServer.receiveShot(connection, fileName)
        currentShotCount += 1

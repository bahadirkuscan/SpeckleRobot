import socket
import os
from enum import Enum
from PIL import Image
 
image_file = 'image.jpg'

class State(Enum):
    WAIT = 0
    EXIT = 1
    TAKE_SHOT = 2
    

state = State.WAIT


def takeShot():
    os.system(f'rpicam-jpeg -t 1 -o {image_file}')
    with Image.open(image_file) as img:
        img.save(image_file, "JPEG", quality=50)


def sendShot(server):
    if os.path.exists('image.jpg'):
        with open('image.jpg', 'rb') as f:
            bytesToSend = f.read(8192)
            while bytesToSend:
                server.send(bytesToSend)
                bytesToSend = f.read(8192)
            server.sendall(b'EOF')  # Signal the end of file transmission  # Signal that the file transfer is complete
            print("Image sent successfully.")
    else:
        print("No image file found.")


def processCommand(command):
    global state

    if command == 'wait':
        state = State.WAIT
    if command == 'exit':
        state = State.EXIT
    if command == 'take_shot':
        state = State.TAKE_SHOT


def processState(server):
    global state

    if state == State.TAKE_SHOT:
        takeShot()
        sendShot(server)
        state = State.WAIT


def client():
    host = '192.168.148.190'  # The server's hostname or IP address
    port = 12345          # The port used by the server

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((host, port))
        try:
            while True:
                data = s.recv(1024)
                if not data:
                    break
                command = data.decode()
                
                processCommand(command.lower())

                if state == State.EXIT:
                    break
                
                processState(s)

        except KeyboardInterrupt:
            print("Client shutting down.")
        finally:
            s.close()

if __name__ == "__main__":
    client()

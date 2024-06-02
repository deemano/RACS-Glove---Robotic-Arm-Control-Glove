import socket
from xarm.wrapper import XArmAPI
import time
import signal
import sys

# Replace with the actual IP address of your xArm
xarm_ip = '192.168.1.242'

# Global variables for the arm and socket
arm = None
client_socket = None

# Define home position (replace with your actual home position values)
home_position = {'x': 206, 'y': 0, 'z': -51.5, 'roll': 180, 'pitch': 0, 'yaw': 0} # taken from UFactory Suit

def signal_handler(sig, frame):
    print('You pressed Ctrl+C! Disconnecting the arm and closing the socket...')
    if arm:
        arm.set_state(state=4)  # Ensure the arm is stopped
        arm.disconnect()
    if client_socket:
        client_socket.close()
    sys.exit(0)

def main():
    global arm, client_socket

    # Initialize the xArm
    arm = XArmAPI(xarm_ip)
    arm.connect()

    if arm.connected:
        print("Successfully connected to the xArm 7.\n")

        # Additional initialization steps
        arm.motion_enable(enable=True)
        arm.set_mode(0)  # Set to position mode
        arm.set_state(0)  # Set state to ready

        # Setup the socket client
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect(('10.0.243.26', 5000))  # Replace with the actual IP address of the Pi
        client_socket.settimeout(1.0)  # Set a timeout for the socket

        current_position = arm.position
        current_position = {'x': current_position[0], 'y': current_position[1], 'z': current_position[2]}

        # Retrieve the current orientation
        current_orientation = arm.get_position(is_radian=False)
        print(f"Current orientation: {current_orientation}")
        if len(current_orientation) < 6:
            print("Error: Unexpected current_orientation length. Using default rotation angles.")
            rotation_angles = {'roll': 180, 'pitch': 0, 'yaw': 0}
        else:
            rotation_angles = {'roll': current_orientation[3], 'pitch': current_orientation[4], 'yaw': current_orientation[5]}

        current_direction = None
        stop_count = 0

        # Initialize variables to track rotation
        is_rotating = False
        stored_position = None

        # Setup signal handler for Ctrl+C
        signal.signal(signal.SIGINT, signal_handler)

        try:
            while True:
                try:
                    data = client_socket.recv(1024).decode('utf-8').strip()
                    print(f"Received data: {data}")  # Debug statement

                    if data == "STOP":
                        stop_count += 1
                        if stop_count == 2:
                            # Reset the robot position and rotation on double STOP
                            arm.set_state(state=4)  # Stop the robot
                            current_position = arm.position
                            current_position = {'x': current_position[0], 'y': current_position[1], 'z': current_position[2]}
                            current_orientation = arm.get_position(is_radian=False)
                            if len(current_orientation) < 6:
                                print("Error: Unexpected current_orientation length. Using default rotation angles.")
                                rotation_angles = {'roll': 180, 'pitch': 0, 'yaw': 0}
                            else:
                                rotation_angles = {'roll': current_orientation[3], 'pitch': current_orientation[4], 'yaw': current_orientation[5]}
                            arm.set_position(
                                x=current_position['x'],
                                y=current_position['y'],
                                z=current_position['z'],
                                roll=rotation_angles['roll'],
                                pitch=rotation_angles['pitch'],
                                yaw=rotation_angles['yaw'],
                                speed=70,
                                mvacc=500,
                                wait=True  # Ensure the command is executed immediately
                            )
                            arm.set_state(0)  # Set state to ready
                            current_direction = None
                            print("RESET received. Robot reset to current position and rotation.")
                        continue
                    elif data == "HOME":
                        # Move to home position
                        print("Moving to home position...")
                        arm.set_state(state=4)  # Stop the robot
                        arm.set_position(
                            x=home_position['x'],
                            y=home_position['y'],
                            z=home_position['z'],
                            roll=home_position['roll'],
                            pitch=home_position['pitch'],
                            yaw=home_position['yaw'],
                            speed=70,
                            mvacc=500,
                            wait=True  # Ensure the command is executed immediately
                        )
                        arm.set_state(0)  # Set state to ready
                        print("Robot moved to home position.")
                        continue
                    else:
                        stop_count = 0  # Reset stop count if any other command is received

                    # Stop the robot before changing direction
                    if current_direction and data != current_direction:
                        arm.set_state(state=4)  # Stop the robot
                        arm.set_position(
                            x=current_position['x'],
                            y=current_position['y'],
                            z=current_position['z'],
                            roll=rotation_angles['roll'],
                            pitch=rotation_angles['pitch'],
                            yaw=rotation_angles['yaw'],
                            speed=70,
                            mvacc=500,
                            wait=True  # Ensure the command is executed immediately
                        )
                        arm.set_state(state=0)  # Set state to ready

                    current_direction = data

                    if current_direction.startswith(("+rot", "-rot")):
                        is_rotating = True
                        # Apply rotation commands
                        if current_direction == "+rotX":
                            rotation_angles['roll'] += 30
                        elif current_direction == "-rotX":
                            rotation_angles['roll'] -= 30
                        elif current_direction == "+rotY":
                            rotation_angles['pitch'] += 30
                        elif current_direction == "-rotY":
                            rotation_angles['pitch'] -= 30
                        elif current_direction == "+rotZ":
                            rotation_angles['yaw'] += 30
                        elif current_direction == "-rotZ":
                            rotation_angles['yaw'] -= 30

                        # Ensure the rotations are within the joint limits
                        rotation_angles['roll'] = max(min(rotation_angles['roll'], 360), 0)
                        rotation_angles['pitch'] = max(min(rotation_angles['pitch'], 360), 0)
                        rotation_angles['yaw'] = max(min(rotation_angles['yaw'], 360), 0)

                        print(f"Rotating to angles: {rotation_angles}")

                    else:
                        is_rotating = False
                        # Apply translation commands
                        if current_direction == "right":
                            current_position['y'] += 10
                        elif current_direction == "left":
                            current_position['y'] -= 10
                        elif current_direction == "forward":
                            current_position['x'] += 10
                        elif current_direction == "back":
                            current_position['x'] -= 10
                        elif current_direction == "up":
                            current_position['z'] += 10
                        elif current_direction == "down":
                            current_position['z'] -= 10

                        # Ensure the positions are within the robot's range
                        current_position['x'] = max(min(current_position['x'], 800), -800)
                        current_position['y'] = max(min(current_position['y'], 800), -800)
                        current_position['z'] = max(min(current_position['z'], 800), 0)

                        print(f"Moving to position: {current_position} with locked rotation {rotation_angles}")

                    # Move the robot arm with locked orientation
                    code = arm.set_position(
                        x=current_position['x'],
                        y=current_position['y'],
                        z=current_position['z'],
                        roll=rotation_angles['roll'],  # Maintain roll
                        pitch=rotation_angles['pitch'],  # Maintain pitch
                        yaw=rotation_angles['yaw'],  # Maintain yaw
                        speed=70,
                        mvacc=500,
                        wait=False  # Allowing for continuous movement without waiting for completion
                    )
                    if code != 0:
                        print(f"Error moving to position: {current_position}, code={code}")

                    time.sleep(0.1)  # Add a small delay to ensure smooth continuous movement

                except ValueError:
                    print(f"Unexpected data format: {data}")
                except socket.timeout:
                    print("Socket timed out. Waiting for new data...")
                    continue
        except Exception as e:
            print(f"An error occurred: {e}")
        finally:
            arm.set_state(state=4)  # Ensure the arm is stopped
            arm.disconnect()
            client_socket.close()
    else:
        print("Failed to connect to the xArm 7.")

if __name__ == "__main__":
    main()

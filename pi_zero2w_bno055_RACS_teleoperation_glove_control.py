import smbus
import time
import math
import socket
import RPi.GPIO as GPIO

# I2C bus setup
bus = smbus.SMBus(1)
BNO055_ADDRESS = 0x28

# Register addresses
QUATERNION_DATA_W_LSB = 0x20

# Operation modes
OPR_MODE_CONFIG = 0x00
OPR_MODE_NDOF = 0x0C

# Threshold for detecting significant changes
THRESHOLD = 10.0  # Adjust this value as needed

# GPIO setup for buttons
BUTTON_PIN1 = 37  # GPIO 26 (Pin 37)
BUTTON_PIN2 = 35  # GPIO 19 (Pin 35)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(BUTTON_PIN1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BUTTON_PIN2, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Function to read a byte
def read_byte(addr, reg):
    return bus.read_byte_data(addr, reg)

# Function to write a byte
def write_byte(addr, reg, value):
    bus.write_byte_data(addr, reg, value)

# Function to read multiple bytes
def read_bytes(addr, reg, length):
    return bus.read_i2c_block_data(addr, reg, length)

# Function to compute two's complement
def twos_complement(val, bits):
    if val & (1 << (bits - 1)):
        val -= 1 << bits
    return val

# Initialize the sensor
def wake_up_bno055():
    try:
        # Read chip ID
        chip_id = read_byte(BNO055_ADDRESS, 0x00)
        if chip_id != 0xA0:
            raise Exception("Invalid chip ID: {}".format(chip_id))

        # Switch to config mode
        write_byte(BNO055_ADDRESS, 0x3D, OPR_MODE_CONFIG)
        time.sleep(0.025)

        # Reset the sensor
        write_byte(BNO055_ADDRESS, 0x3F, 0x20)
        time.sleep(0.65)

        # Set power mode to normal
        write_byte(BNO055_ADDRESS, 0x3E, 0x00)
        time.sleep(0.025)

        # Set to page 0
        write_byte(BNO055_ADDRESS, 0x07, 0x00)

        # Set operation mode to NDOF
        write_byte(BNO055_ADDRESS, 0x3D, OPR_MODE_NDOF)
        time.sleep(0.025)

        print("BNO055 is awake and ready.")
    except Exception as e:
        print(f"Error during BNO055 wake-up: {e}")

def read_quaternions():
    data = read_bytes(BNO055_ADDRESS, QUATERNION_DATA_W_LSB, 8)
    qw = (data[0] | (data[1] << 8)) & 0xFFFF
    qx = (data[2] | (data[3] << 8)) & 0xFFFF
    qy = (data[4] | (data[5] << 8)) & 0xFFFF
    qz = (data[6] | (data[7] << 8)) & 0xFFFF

    if qw >= 32768: qw -= 65536
    if qx >= 32768: qx -= 65536
    if qy >= 32768: qy -= 65536
    if qz >= 32768: qz -= 65536

    scale = (1.0 / (1 << 14))
    qw *= scale
    qx *= scale
    qy *= scale
    qz *= scale

    return qw, qx, qy, qz

def quaternion_to_euler(qw, qx, qy, qz):
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if norm == 0: norm = 1e-10  # Ensure norm is non-zero to prevent division by zero
    qw /= norm
    qx /= norm
    qy /= norm
    qz /= norm

    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)

    return yaw, roll, pitch

def main():
    wake_up_bno055()
    previous_yaw, previous_roll, previous_pitch = 0, 0, 0
    button1_pressed = False
    button2_pressed = False
    current_direction = "STOP"
    rotation_direction = "STOP"

    # Setup the socket server
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('0.0.0.0', 5000))
    server_socket.listen(1)
    print("Waiting for a connection...")
    connection, client_address = server_socket.accept()
    print("Connection from", client_address)

    try:
        while True:
            if GPIO.input(BUTTON_PIN1) == GPIO.LOW:
                button1_pressed = True
                qw, qx, qy, qz = read_quaternions()
                yaw, roll, pitch = quaternion_to_euler(qw, qx, qy, qz)

                if abs(yaw - previous_yaw) > THRESHOLD:
                    if yaw > previous_yaw:
                        current_direction = "back"
                    else:
                        current_direction = "forward"

                if abs(roll - previous_roll) > THRESHOLD:
                    if roll > previous_roll:
                        current_direction = "right"
                    else:
                        current_direction = "left"

                if abs(pitch - previous_pitch) > THRESHOLD:
                    if pitch > previous_pitch:
                        current_direction = "down"
                    else:
                        current_direction = "up"

                print(current_direction)
                connection.sendall(current_direction.encode('utf-8'))
                previous_yaw, previous_roll, previous_pitch = yaw, roll, pitch
            else:
                if button1_pressed:
                    button1_pressed = False
                    current_direction = "STOP"
                    print("STOP")
                    connection.sendall("STOP".encode('utf-8'))

            if GPIO.input(BUTTON_PIN2) == GPIO.LOW:
                button2_pressed = True
                qw, qx, qy, qz = read_quaternions()
                yaw, roll, pitch = quaternion_to_euler(qw, qx, qy, qz)

                if yaw > previous_yaw + THRESHOLD:
                    rotation_direction = "+rotZ"
                elif yaw < previous_yaw - THRESHOLD:
                    rotation_direction = "-rotZ"

                if roll > previous_roll + THRESHOLD:
                    rotation_direction = "+rotX"
                elif roll < previous_roll - THRESHOLD:
                    rotation_direction = "-rotX"

                if pitch > previous_pitch + THRESHOLD:
                    rotation_direction = "+rotY"
                elif pitch < previous_pitch - THRESHOLD:
                    rotation_direction = "-rotY"

                print(rotation_direction)
                connection.sendall(rotation_direction.encode('utf-8'))
                previous_yaw, previous_roll, previous_pitch = yaw, roll, pitch
            else:
                if button2_pressed:
                    button2_pressed = False
                    rotation_direction = "STOP"
                    print("Stopped printing rotations")
                    connection.sendall("STOP".encode('utf-8'))

            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Program stopped")
    finally:
        connection.close()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
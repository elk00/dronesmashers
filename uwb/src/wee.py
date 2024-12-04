import socket
import struct
STRUCT_FORMAT = '<IHIfff'  # Little-endian: I for uint32_t, Q for uint64_t, f for float
STRUCT_SIZE = struct.calcsize(STRUCT_FORMAT)

# Function to decode structured UDP data
def decode_c_struct(data):
    global prev_freq
    unpacked_data = struct.unpack(STRUCT_FORMAT, data)
    message_length, message_type, robot_id, x, y, z = unpacked_data

    print(f"Robot ID: {robot_id}, robot.x: {x}, robot.y: {y}, robot.z: {z}\n")

# Function to run UDP server to receive data
def run_udp():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', 8000))
    sock.setblocking(False)  # Set socket to non-blocking mode
    print("Running UDP server...")

    while True:
        try:
            data, _ = sock.recvfrom(STRUCT_SIZE)
            if len(data) == STRUCT_SIZE:
                decode_c_struct(data)
            else:
                print("Length mismatch")
        except socket.error:
            pass  # Handle the case where no data is available to read


if __name__ == "__main__":
    run_udp()

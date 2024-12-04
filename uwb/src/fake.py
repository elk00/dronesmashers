import socket
import struct
import time

# Define the message structure format for packing data
STRUCT_FORMAT = '<IHIfff'  # I for uint32_t, H for uint16_t, and f for float

# Constant values for the message
MESSAGE_TYPE = 1            # UWB_TYPE (example)
ROBOT_ID = 1                # Example robot ID
X, Y, Z = 1.0, 1.0, 0.0     # Constant positions for testing
MESSAGE_LENGTH = struct.calcsize(STRUCT_FORMAT)

# Destination server address
SERVER_ADDRESS = ('127.0.0.1', 8000)

def send_fake_data():
    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Pack the data according to the defined structure
    message = struct.pack(
        STRUCT_FORMAT,
        MESSAGE_LENGTH,
        MESSAGE_TYPE,
        ROBOT_ID,
        X,
        Y,
        Z
    )

    # Continuously send the message every second
    while True:
        sock.sendto(message, SERVER_ADDRESS)
        print(f"Sent: Message Length: {MESSAGE_LENGTH}, Message Type: {MESSAGE_TYPE}, "
              f"Robot ID: {ROBOT_ID}, x: {X}, y: {Y}, z: {Z}")
        time.sleep(1)  # Adjust the frequency as needed

if __name__ == "__main__":
    send_fake_data()


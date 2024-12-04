import serial

def print_serial_config(ser):
    print(f"Baudrate: {ser.baudrate}")
    print(f"Parity: {ser.parity}")
    print(f"Stop bits: {ser.stopbits}")
    print(f"Byte size: {ser.bytesize}")
    print(f"Timeout: {ser.timeout}")

def read_serial_data(port, baudrate=9600, timeout=1):
    try:
        with serial.Serial(port, baudrate, timeout=timeout) as ser:
            print_serial_config(ser)  # Print serial settings
            
            while True:
                if ser.in_waiting > 0:
                    raw_data = ser.read(ser.in_waiting)
                    if raw_data:
                        print(f"Raw Data: {raw_data}")
    except serial.SerialException as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        print("Program terminated by user.")

if __name__ == "__main__":
    serial_port = '/dev/ttyUSB0'  # Adjust this
    baud_rate = 921600
    
    read_serial_data(serial_port, baud_rate)

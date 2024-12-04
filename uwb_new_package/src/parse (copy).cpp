#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <stdint.h>

#include <iostream>
#include <ostream>

#include "nlink_linktrack_nodeframe0.h"
#include "nlink_linktrack_nodeframe1.h"
#include "nlink_linktrack_nodeframe2.h"
#include "nlink_linktrack_nodeframe3.h"
#include "nlink_utils.h"

#include "MessageProtocol.hpp"
#include "UDPClient.hpp"

#define FRAME_HEADER 0x55  // NLink frame header value
#define MAX_FRAME_SIZE 1024

UDPClient client("127.0.0.1", 8000);

int configure_port(int fd) {
    struct termios tty;

    if (tcgetattr(fd, &tty) != 0) {
        perror("Error from tcgetattr");
        return -1;
    }

    // Set Baud Rate to 921600
    cfsetospeed(&tty, B921600); 
    cfsetispeed(&tty, B921600); 

    // Parity: None (disable parity)
    tty.c_cflag &= ~PARENB;

    // Stop bits: 1
    tty.c_cflag &= ~CSTOPB;

    // Byte size: 8
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    // Hardware flow control: Disable
    tty.c_cflag &= ~CRTSCTS;

    // Enable the receiver and set local mode
    tty.c_cflag |= (CLOCAL | CREAD);

    // Disable software flow control (XON/XOFF)
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);

    // Raw input mode (disable canonical mode, echo, etc.)
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // Raw output mode (disable post-processing)
    tty.c_oflag &= ~OPOST;

    // Set read timeout (timeout of 1 second as in Python)
    tty.c_cc[VMIN]  = 0;  // Minimum number of characters for non-blocking read
    tty.c_cc[VTIME] = 10; // Timeout in deciseconds (1 second)

    // Apply the settings
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("Error from tcsetattr");
        return -1;
    }

    // Flush the serial port to remove any data already in the buffer
    tcflush(fd, TCIOFLUSH);

    return 0;
}


// Function to enable low-latency mode
int set_low_latency(int fd) {
    struct serial_struct ser_info;

    if (ioctl(fd, TIOCGSERIAL, &ser_info) < 0) {
        perror("Error getting serial information");
        return -1;
    }

    ser_info.flags |= ASYNC_LOW_LATENCY;  // Enable low-latency mode

    if (ioctl(fd, TIOCSSERIAL, &ser_info) < 0) {
        perror("Error setting low-latency mode");
        return -1;
    }

    printf("Low-latency mode enabled.\n");
    return 0;
}

// Function to process and parse the full frame data
void process_frame_data(uint8_t *data, size_t data_length) {
   if (data[1] == 0x04) {
        if (g_nlt_nodeframe2.UnpackData(data, data_length)) {
            nlt_nodeframe2_result_t *result = &g_nlt_nodeframe2.result;
            printf("LinkTrack NodeFrame2 data unpack successfully:\r\n");
            printf("id:%d, system_time:%d, valid_node_count:%d\r\n", result->id,
                   result->system_time, result->valid_node_count);
            
            // Print position data (pos_3d) from result
            printf("Position data: x = %f, y = %f, z = %f\n",
                   result->pos_3d[0], result->pos_3d[1], result->pos_3d[2]);
            
            for (int i = 0; i < result->valid_node_count; ++i) {
                nlt_nodeframe2_node_t *node = result->nodes[i];
                printf("id:%d, distance:%f\r\n", 
                       node->id, node->dis);
            }

                // Prepare data for StructTypeOne
            UWBStruct data;
            //data.robot_id = htonl(1);  // Convert to network byte order
            data.x = result->pos_3d[0];  // Assuming IEEE 754 format; no conversion needed
            data.y = result->pos_3d[1];  // Assuming IEEE 754 format; no conversion needed
            data.z = result->pos_3d[2];  // Assuming IEEE 754 format; no conversion needed

            if (!client.sendMessage(UWB_TYPE, &data, sizeof(data))) {
            std::cerr << "Failed to send message from Client One" << std::endl;
        }

        }
    } else {
        printf("Unknown function mark: 0x%02X\n", data[1]);
        printf("Full frame data: ");
        for (int i = 0; i < data_length; i++) {
            printf("%02X ", data[i]);
        }
        printf("\n");
    }
}

int main() {
    const char *portname = "/dev/ttyUSB0";  // Adjust for your USB device

    // Open the serial port
    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("Error opening serial port");
        return -1;
    }

    // Configure the port
    if (configure_port(fd) < 0) {
        close(fd);
        return -1;
    }
    
    tcflush(fd, TCIOFLUSH);  // Flush both input and output buffers


    // Enable low-latency mode
    if (set_low_latency(fd) < 0) {
        close(fd);
        return -1;
    }
    if (!client.connectToServer()) {
        std::cerr << "Error creating UDP socket" << std::endl;
        return -1;
    }

    uint8_t byte_buffer[MAX_FRAME_SIZE];  // Buffer for storing frame data
    size_t frame_length = 0;
    int n;
    int header_found = 0;
    size_t bytes_received = 0;

    printf("Starting serial read...\n");

    while (1) {
        // Read data
        n = read(fd, byte_buffer + bytes_received, MAX_FRAME_SIZE - bytes_received);

        if (n > 0) {
            bytes_received += n;

            // Look for frame header (0x55)
            if (!header_found) {
                for (size_t i = 0; i < bytes_received; i++) {
                    if (byte_buffer[i] == FRAME_HEADER) {
                        header_found = 1;
                        frame_length = 0;  // Reset for new frame
                        bytes_received = bytes_received - i; // Adjust for valid frame bytes
                        memmove(byte_buffer, byte_buffer + i, bytes_received);  // Shift the frame
                        break;
                    }
                }
            }

            // If the header is found, accumulate bytes until a full frame is received
            if (header_found && bytes_received >= 6) {
                // Get frame length from the 4th and 5th bytes
                frame_length = (byte_buffer[4] << 8) | byte_buffer[5];
                if (bytes_received >= frame_length) {
                    // Process the full frame
                    process_frame_data(byte_buffer, frame_length);

                    // Reset for the next frame
                    header_found = 0;
                    bytes_received = 0;
                }
            }
        } else {
            usleep(1000);  // Sleep to avoid busy waiting
        }
    }

    close(fd);
    client.closeConnection();
    return 0;
}


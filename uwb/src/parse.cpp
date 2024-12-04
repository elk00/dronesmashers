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

#include <rclcpp/rclcpp.hpp> 
#include <geometry_msgs/msg/pose_stamped.hpp>

#define FRAME_HEADER 0x55  // NLink frame header value
#define MAX_FRAME_SIZE 1024
#define SERIAL_READ_TIMEOUT_MS 1000  // Timeout for serial reads

class UwbPosePublisher : public rclcpp::Node {
public:
    UwbPosePublisher() : Node("uwb_pose_publisher") {
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/vision_pose/pose", 10);
    }

    void publishPose(float x, float y, float z) {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->get_clock()->now();
        pose_msg.header.frame_id = "map";
        pose_msg.pose.position.x = x;
        pose_msg.pose.position.y = y;
        pose_msg.pose.position.z = z;
        pose_msg.pose.orientation.x = 0.0;
        pose_msg.pose.orientation.y = 0.0;
        pose_msg.pose.orientation.z = 0.0;
        pose_msg.pose.orientation.w = 1.0;

        pose_publisher_->publish(pose_msg);
        RCLCPP_INFO(this->get_logger(), "Published pose: x=%f, y=%f, z=%f", x, y, z);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
};

int configure_port(int fd) {
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        perror("Error from tcgetattr");
        return -1;
    }
    cfsetospeed(&tty, B921600);
    cfsetispeed(&tty, B921600);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= (CLOCAL | CREAD);

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("Error from tcsetattr");
        return -1;
    }
    tcflush(fd, TCIOFLUSH);
    return 0;
}

int configure_port_tof(int fd) {
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        perror("Error from tcgetattr");
        return -1;
    }
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= (CLOCAL | CREAD);

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("Error from tcsetattr");
        return -1;
    }
    tcflush(fd, TCIOFLUSH);
    return 0;
}


int set_low_latency(int fd) {
    struct serial_struct ser_info;
    if (ioctl(fd, TIOCGSERIAL, &ser_info) < 0) {
        perror("Error getting serial information");
        return -1;
    }
    ser_info.flags |= ASYNC_LOW_LATENCY;
    if (ioctl(fd, TIOCSSERIAL, &ser_info) < 0) {
        perror("Error setting low-latency mode");
        return -1;
    }
    printf("Low-latency mode enabled.\n");
    return 0;
}

void process_frame_data(std::shared_ptr<UwbPosePublisher> publisher, uint8_t *data, size_t data_length, float tof_z) {
    if (data[1] == 0x04) {
        if (g_nlt_nodeframe2.UnpackData(data, data_length)) {
            nlt_nodeframe2_result_t *result = &g_nlt_nodeframe2.result;
            float x = result->pos_3d[0];
            float y = result->pos_3d[1];
            printf("Unpacked: x=%f, y=%f, z=%f\n", x, y, tof_z);
            publisher->publishPose(x, y, tof_z); // Pass ToF data as z-coordinate
        } else {
            printf("Error unpacking frame data.\n");
        }
    } else {
        printf("Unknown mark: 0x%02X\n", data[1]);
    }
}
void process_tof_data(int tof_fd, float &tof_z) {
    char buffer[32];
    memset(buffer, 0, sizeof(buffer));
    int n = read(tof_fd, buffer, sizeof(buffer) - 1);
    if (n > 0) {
        buffer[n] = '\0';
        try {
            tof_z = std::stof(buffer); // Convert string to float
            //printf("Received ToF data: %f\n", tof_z);
        } catch (const std::exception &e) {
            printf("Error parsing ToF data: %s\n", e.what());
        }
    }
}
void reset_state(uint8_t *buffer, size_t &bytes_received, int &header_found) {
    printf("Resetting state due to invalid frame or timeout.\n");
    memset(buffer, 0, MAX_FRAME_SIZE);
    bytes_received = 0;
    header_found = 0;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto publisher = std::make_shared<UwbPosePublisher>();

    const char *portname = "/dev/ttyUSB0";
    const char *tof_port = "/dev/ttyUSB1";

    int uwb_fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    int tof_fd = open(tof_port, O_RDWR | O_NOCTTY | O_SYNC);
    if (uwb_fd < 0 || tof_fd < 0) {
        perror("Error opening serial port");
        return -1;
    }

    if (configure_port(uwb_fd) < 0 || set_low_latency(uwb_fd) < 0 || 
        configure_port_tof(tof_fd) < 0 || set_low_latency(tof_fd) < 0) {
        close(uwb_fd);
        close(tof_fd);
        return -1;
    }

    uint8_t byte_buffer[MAX_FRAME_SIZE];
    size_t bytes_received = 0;
    int n, header_found = 0;
    size_t frame_length = 0;
    struct timespec last_read_time, current_time;
    float tof_z = 0.0; // Store the latest ToF data

    printf("Starting serial read...\n");

    clock_gettime(CLOCK_MONOTONIC, &last_read_time);

    while (rclcpp::ok()) {
        // Process ToF data
        process_tof_data(tof_fd, tof_z);

        // Process UWB data
        n = read(uwb_fd, byte_buffer + bytes_received, MAX_FRAME_SIZE - bytes_received);
        clock_gettime(CLOCK_MONOTONIC, &current_time);

        if (n > 0) {
            bytes_received += n;
            if (!header_found) {
                for (size_t i = 0; i < bytes_received; i++) {
                    if (byte_buffer[i] == FRAME_HEADER) {
                        header_found = 1;
                        frame_length = 0;
                        bytes_received -= i;
                        memmove(byte_buffer, byte_buffer + i, bytes_received);
                        break;
                    }
                }
            }

            if (header_found && bytes_received >= 6) {
                frame_length = (byte_buffer[4] << 8) | byte_buffer[5];
                if (bytes_received >= frame_length) {
                    process_frame_data(publisher, byte_buffer, frame_length, tof_z);
                    header_found = 0;
                    bytes_received = 0;
                }
            }

            clock_gettime(CLOCK_MONOTONIC, &last_read_time);
        } else if ((current_time.tv_sec - last_read_time.tv_sec) * 1000 +
                       (current_time.tv_nsec - last_read_time.tv_nsec) / 1000000 >
                   SERIAL_READ_TIMEOUT_MS) {
            reset_state(byte_buffer, bytes_received, header_found);
        }

        usleep(1000);
    }

    close(uwb_fd);
    close(tof_fd);
    rclcpp::shutdown();
    return 0;
}


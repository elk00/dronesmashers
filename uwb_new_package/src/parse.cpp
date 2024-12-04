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

UDPClient client("127.0.0.1", 8000);

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

void process_frame_data(std::shared_ptr<UwbPosePublisher> publisher, uint8_t *data, size_t data_length) {
    if (data[1] == 0x04) {
        if (g_nlt_nodeframe2.UnpackData(data, data_length)) {
            nlt_nodeframe2_result_t *result = &g_nlt_nodeframe2.result;
            printf("Unpacked: x=%f, y=%f, z=%f\n", result->pos_3d[0], result->pos_3d[1], result->pos_3d[2]);
            publisher->publishPose(result->pos_3d[0], result->pos_3d[1], result->pos_3d[2]);
        }
    } else {
        printf("Unknown mark: 0x%02X\n", data[1]);
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto publisher = std::make_shared<UwbPosePublisher>();

    const char *portname = "/dev/ttyUSB0";
    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("Error opening serial port");
        return -1;
    }

    if (configure_port(fd) < 0 || set_low_latency(fd) < 0) {
        close(fd);
        return -1;
    }

    uint8_t byte_buffer[MAX_FRAME_SIZE];
    size_t bytes_received = 0;
    int n, header_found = 0;
    size_t frame_length = 0;

    printf("Starting serial read...\n");

    while (rclcpp::ok()) {
        n = read(fd, byte_buffer + bytes_received, MAX_FRAME_SIZE - bytes_received);
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
                    process_frame_data(publisher, byte_buffer, frame_length);
                    header_found = 0;
                    bytes_received = 0;
                }
            }
        } else {
            usleep(1000);
        }
    }

    close(fd);
    rclcpp::shutdown();
    return 0;
}


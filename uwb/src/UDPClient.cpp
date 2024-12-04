#include "UDPClient.hpp"
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>



// Constructor
UDPClient::UDPClient(const std::string& server_ip, uint16_t server_port)
    : sockfd_(-1), server_ip_(server_ip), server_port_(server_port) {}

// Destructor
UDPClient::~UDPClient() {
    closeConnection();
}

// Initialize the UDP socket if it's not already open
bool UDPClient::connectToServer() {
    if (sockfd_ == -1) {  // Only create a socket if it's not already open
        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);  // Use SOCK_DGRAM for UDP
        if (sockfd_ < 0) {
            perror("Socket creation error");
            return false;
        }
    }
    return true;
}

// Send a message without encryption
bool UDPClient::sendMessage(MessageType msg_type, const void* data, size_t data_size) {
    if (!connectToServer()) {  // Ensure the socket is open before sending
        return false;
    }

    // Prepare the message header
    MessageHeader header;
    header.length = htonl(static_cast<uint32_t>(sizeof(MessageHeader) + data_size));
    header.type = htons(msg_type);

    // Allocate buffer for the entire message
    size_t total_size = sizeof(MessageHeader) + data_size;
    char* buffer = new char[total_size];

    // Copy header and payload into buffer
    memcpy(buffer, &header, sizeof(MessageHeader));
    memcpy(buffer + sizeof(MessageHeader), data, data_size);

    // Set up server address for sending
    struct sockaddr_in server_addr {};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(server_port_);

    if (inet_pton(AF_INET, server_ip_.c_str(), &server_addr.sin_addr) <= 0) {
        perror("Invalid address");
        delete[] buffer;
        return false;
    }

    // Send the message using sendto()
    ssize_t bytes_sent = sendto(sockfd_, buffer, total_size, 0,
                                (struct sockaddr*)&server_addr, sizeof(server_addr));

    if (bytes_sent < 0) {
        perror("Sendto error");
        delete[] buffer;
        return false;
    }

    delete[] buffer;
    return true;
}


// Close the connection (for UDP, it's just closing the socket)
void UDPClient::closeConnection() {
    if (sockfd_ != -1) {
        close(sockfd_);
        sockfd_ = -1;
    }
}

// Check if the socket is valid
bool UDPClient::isConnected() const {
    return sockfd_ != -1;
}

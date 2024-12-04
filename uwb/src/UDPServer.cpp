#include "UDPServer.hpp"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>


// Constructor
UDPServer::UDPServer(uint16_t port)
    : port_(port), server_fd_(-1), running_(false), message_handler_(nullptr) {}

// Destructor
UDPServer::~UDPServer() {
    stop();
}

// Start the server by initializing the UDP socket and binding it
bool UDPServer::start() {
    if ((server_fd_ = socket(AF_INET, SOCK_DGRAM, 0)) == 0) {  // Use SOCK_DGRAM for UDP
        perror("Server socket creation failed");
        return false;
    }

    struct sockaddr_in address {};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;  // Listen on all interfaces
    address.sin_port = htons(port_);

    if (bind(server_fd_, (struct sockaddr*)&address, sizeof(address)) < 0) {
        perror("Server bind failed");
        close(server_fd_);
        server_fd_ = -1;
        return false;
    }

    running_ = true;
    std::cout << "Server listening on port " << port_ << std::endl;

    while (running_) {
        struct sockaddr_in client_addr {};
        socklen_t client_addr_len = sizeof(client_addr);
        char buffer[4096] = {0};

        // Use recvfrom for UDP to get client data and address
        ssize_t bytes_received = recvfrom(server_fd_, buffer, sizeof(buffer), 0,
                                          (struct sockaddr*)&client_addr, &client_addr_len);

        if (bytes_received < 0) {
            perror("Receive error");
            return false;
        }
        
            // Receive data without decryption (e.g., from local clients)
        receiveData(buffer, bytes_received, client_addr);

        
    }

    return true;
}

// Receive data from clients using recvfrom()
void UDPServer::receiveData(char* buffer, ssize_t bytes_received, struct sockaddr_in& client_addr) {
    // Process the received message
    
    if (message_handler_) {
        if (bytes_received >= sizeof(MessageHeader)) {
            MessageHeader* header = reinterpret_cast<MessageHeader*>(buffer);
            uint16_t msg_type = ntohs(header->type);
            size_t payload_size = ntohl(header->length) - sizeof(MessageHeader);
            const char* payload = buffer + sizeof(MessageHeader);

            message_handler_(server_fd_, static_cast<MessageType>(msg_type), payload, payload_size);
        } else {
            std::cerr << "Received data is too small to contain a valid header" << std::endl;
        }
    }
}

// Stop the server
void UDPServer::stop() {
    running_ = false;

    if (server_fd_ != -1) {
        close(server_fd_);
        server_fd_ = -1;
    }
}

void UDPServer::setMessageHandler(std::function<void(int, MessageType, const void*, size_t)> handler) {
    message_handler_ = handler;
}

// Check if it's an external host
bool UDPServer::isExternalHost(struct sockaddr_in& client_addr) {
    // Example logic to identify external hosts
    char client_ip[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &(client_addr.sin_addr), client_ip, INET_ADDRSTRLEN);
    // If the message is not coming from localhost, assume it's an external host
    return strcmp(client_ip, "127.0.0.1") != 0;
}

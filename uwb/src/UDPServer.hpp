// include/UDPServer.hpp

#ifndef UDPSERVER_HPP
#define UDPSERVER_HPP

#include <string>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>
#include <arpa/inet.h>
#include "MessageProtocol.hpp"  // Common message protocol

class UDPServer {
public:
    UDPServer(uint16_t port);
    ~UDPServer();

    bool start();  // Start listening for incoming UDP messages
    void stop();   // Stop the server

    // Set a callback function to handle received messages
    void setMessageHandler(std::function<void(int client_fd, MessageType msg_type, const void* data, size_t data_size)> handler);

private:
    uint16_t port_;
    int server_fd_;  // Socket file descriptor for UDP
    bool running_;
    std::function<void(int client_fd, MessageType msg_type, const void* data, size_t data_size)> message_handler_;

    // Helper methods
    void receiveData(char* buffer, ssize_t bytes_received, struct sockaddr_in& client_addr);  // Method for receiving data from clients

    //check if the received message comes from external ip or locally
    bool isExternalHost(struct sockaddr_in& client_addr);

    // Non-copyable
    UDPServer(const UDPServer&) = delete;
    UDPServer& operator=(const UDPServer&) = delete;
    
};

#endif // UDPSERVER_HPP


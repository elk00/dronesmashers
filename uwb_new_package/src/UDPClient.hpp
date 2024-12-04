// include/UDPClient.hpp

#ifndef UDPCLIENT_HPP
#define UDPCLIENT_HPP

#include <string>
#include <cstdint>
#include "MessageProtocol.hpp"
#include <cstring>
#include <arpa/inet.h>

class UDPClient {
public:
    // Constructor to initialize the UDP client with server IP and port
    UDPClient(const std::string& server_ip, uint16_t server_port);

    // Destructor to clean up resources
    ~UDPClient();

    // Open socket for conneciton
    bool connectToServer();

    // Send a message without encrpytion
    bool sendMessage(MessageType msg_type, const void* data, size_t data_size);

    // Close the UDP socket
    void closeConnection();

    // Check if the client is connected (i.e., if the socket is valid)
    bool isConnected() const;

private:
    int sockfd_;                // Socket file descriptor (must be initialized first)
    std::string server_ip_;      // Server IP address
    uint16_t server_port_;       // Server port number


};

#endif // UDPCLIENT_HPP


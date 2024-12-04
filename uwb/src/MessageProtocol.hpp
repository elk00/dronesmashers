// include/MessageProtocol.hpp

#ifndef MESSAGE_PROTOCOL_HPP
#define MESSAGE_PROTOCOL_HPP

#include <cstdint>
#include <vector>

// Message types (shared between client and server)
enum MessageType : uint16_t {
    UWB_TYPE = 1,
    ROBOT_DATA_TYPE = 2 // Add more message types as needed
};

// Ensure packed structure without padding
#pragma pack(push, 1)
struct MessageHeader {
    uint32_t length;     // Total message length (header + payload)
    uint16_t type;       // Message type identifier
};
#pragma pack(pop)

// Define your data structs
#pragma pack(push, 1)
struct UWBStruct{
    uint32_t robot_id;        // Unique identifier for the UWB node
    float x;                 // X position (in meters)
    float y;                 // Y position (in meters)
    float z;                 // Z position (optional, in meters)
};



struct RobotData{
	UWBStruct uwb_data;
	
	//flags to indicate what has been updated
	bool uwb_update = false;
	
};


#pragma pack(pop)

//Define QueueMessage


#endif // MESSAGE_PROTOCOL_HPP


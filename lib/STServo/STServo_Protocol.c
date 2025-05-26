/**
 * @file STServo_Protocol.c
 * @brief ST Servo Communication Protocol Implementation
 * @author Adapted for PlatformIO/STM32F401RE
 * @date 2025
 */

#include "STServo_Protocol.h"
#include <string.h>

/**
 * @brief Calculate checksum for packet
 * @param packet Packet data (excluding checksum)
 * @param length Packet length (excluding checksum)
 * @return Calculated checksum
 */
uint8_t STServo_CalculateChecksum(const uint8_t *packet, uint8_t length)
{
    if (!packet || length < 4) {
        return 0;
    }
    
    uint8_t checksum = 0;
    // Skip header bytes (0xFF, 0xFF), start from ID
    for (uint8_t i = 2; i < length; i++) {
        checksum += packet[i];
    }
    
    return ~checksum;  // Bitwise NOT
}

/**
 * @brief Validate packet checksum
 * @param packet Complete packet including checksum
 * @param length Total packet length
 * @return true if checksum is valid, false otherwise
 */
bool STServo_ValidateChecksum(const uint8_t *packet, uint8_t length)
{
    if (!packet || length < 5) {
        return false;
    }
    
    uint8_t calculated_checksum = STServo_CalculateChecksum(packet, length - 1);
    uint8_t received_checksum = packet[length - 1];
    
    return calculated_checksum == received_checksum;
}

/**
 * @brief Build a complete packet
 * @param packet Pointer to packet structure
 * @param id Servo ID
 * @param instruction Instruction code
 * @param parameters Parameter data
 * @param param_length Parameter length
 * @return Total packet length
 */
uint8_t STServo_BuildPacket(STServo_Packet_t *packet, uint8_t id, uint8_t instruction, 
                           const uint8_t *parameters, uint8_t param_length)
{
    if (!packet) {
        return 0;
    }
    
    // Build packet
    packet->header[0] = ST_SERVO_FRAME_HEADER;
    packet->header[1] = ST_SERVO_FRAME_HEADER2;
    packet->id = id;
    packet->length = param_length + 2;  // instruction + checksum
    packet->instruction = instruction;
    
    // Copy parameters if provided
    if (parameters && param_length > 0) {
        memcpy(packet->parameters, parameters, param_length);
    }
    
    // Calculate and set checksum
    uint8_t total_length = 4 + param_length + 1;  // header + id + length + instruction + params
    packet->checksum = STServo_CalculateChecksum((uint8_t*)packet, total_length);
    
    return total_length + 1;  // Include checksum in total length
} 
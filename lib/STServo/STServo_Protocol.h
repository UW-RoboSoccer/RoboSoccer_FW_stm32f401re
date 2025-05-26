/**
 * @file STServo_Protocol.h
 * @brief ST Servo Communication Protocol Definitions
 * @author Adapted for PlatformIO/STM32F401RE
 * @date 2025
 */

#ifndef _ST_SERVO_PROTOCOL_H
#define _ST_SERVO_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Protocol constants
#define ST_SERVO_FRAME_HEADER       0xFF
#define ST_SERVO_FRAME_HEADER2      0xFF
#define ST_SERVO_BROADCAST_ID       0xFE
#define ST_SERVO_BUFFER_SIZE        32

// Instruction set
#define ST_INST_PING                0x01
#define ST_INST_READ                0x02
#define ST_INST_WRITE               0x03
#define ST_INST_REG_WRITE           0x04
#define ST_INST_ACTION              0x05
#define ST_INST_SYNC_READ           0x82
#define ST_INST_SYNC_WRITE          0x83

// Error flags
#define ST_ERROR_NONE               0x00
#define ST_ERROR_VOLTAGE            0x01
#define ST_ERROR_ANGLE_LIMIT        0x02
#define ST_ERROR_OVERHEATING        0x04
#define ST_ERROR_RANGE              0x08
#define ST_ERROR_CHECKSUM           0x10
#define ST_ERROR_OVERLOAD           0x20
#define ST_ERROR_INSTRUCTION        0x40

// Packet structure
typedef struct {
    uint8_t header[2];      // 0xFF, 0xFF
    uint8_t id;             // Servo ID
    uint8_t length;         // Length of parameters + 2
    uint8_t instruction;    // Instruction code
    uint8_t parameters[ST_SERVO_BUFFER_SIZE - 6];  // Parameters
    uint8_t checksum;       // Checksum
} __attribute__((packed)) STServo_Packet_t;

// Communication functions
uint8_t STServo_CalculateChecksum(const uint8_t *packet, uint8_t length);
bool STServo_ValidateChecksum(const uint8_t *packet, uint8_t length);
uint8_t STServo_BuildPacket(STServo_Packet_t *packet, uint8_t id, uint8_t instruction, 
                           const uint8_t *parameters, uint8_t param_length);

#ifdef __cplusplus
}
#endif

#endif // _ST_SERVO_PROTOCOL_H 
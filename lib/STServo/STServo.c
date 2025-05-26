/**
 * @file STServo.c
 * @brief ST Series Serial Servo Control Library Implementation
 * @author Adapted for PlatformIO/STM32F401RE
 * @date 2025
 */

#include "STServo.h"
#include "STServo_Protocol.h"
#include <string.h>

// Private variables
static STServo_Error_t last_error = STSERVO_OK;

// Private function prototypes
static bool STServo_SendPacket(STServo_Handle_t *handle, const uint8_t *packet, uint8_t length);
static bool STServo_ReceivePacket(STServo_Handle_t *handle, uint8_t *packet, uint8_t *length);
static bool STServo_WriteData(STServo_Handle_t *handle, uint8_t id, uint8_t address, const uint8_t *data, uint8_t length);
static bool STServo_ReadData(STServo_Handle_t *handle, uint8_t id, uint8_t address, uint8_t *data, uint8_t length);

/**
 * @brief Initialize the ST Servo handle
 * @param handle Pointer to servo handle
 * @param huart Pointer to UART handle
 * @return true if successful, false otherwise
 */
bool STServo_Init(STServo_Handle_t *handle, UART_HandleTypeDef *huart)
{
    if (!handle || !huart) {
        last_error = STSERVO_ERROR_INVALID_PARAM;
        return false;
    }
    
    handle->huart = huart;
    handle->timeout_ms = 100;  // Default 100ms timeout
    handle->initialized = true;
    last_error = STSERVO_OK;
    
    return true;
}

/**
 * @brief Deinitialize the ST Servo handle
 * @param handle Pointer to servo handle
 */
void STServo_DeInit(STServo_Handle_t *handle)
{
    if (handle) {
        handle->initialized = false;
        handle->huart = NULL;
    }
}

/**
 * @brief Write position command to servo
 * @param handle Servo handle
 * @param id Servo ID (1-253)
 * @param position Target position (0-4095)
 * @param speed Movement speed (0-4095)
 * @param acceleration Acceleration (0-255)
 * @return true if successful, false otherwise
 */
bool STServo_WritePosition(STServo_Handle_t *handle, uint8_t id, int16_t position, uint16_t speed, uint8_t acceleration)
{
    if (!handle || !handle->initialized) {
        last_error = STSERVO_ERROR_INVALID_PARAM;
        return false;
    }
    
    uint8_t params[7];
    params[0] = SMS_STS_GOAL_POSITION_L;  // Starting address
    params[1] = position & 0xFF;          // Position low byte
    params[2] = (position >> 8) & 0xFF;   // Position high byte
    params[3] = 0;                        // Time low byte (0 = max speed)
    params[4] = 0;                        // Time high byte
    params[5] = speed & 0xFF;             // Speed low byte
    params[6] = (speed >> 8) & 0xFF;      // Speed high byte
    
    return STServo_WriteData(handle, id, SMS_STS_ACC, &acceleration, 1) &&
           STServo_WriteData(handle, id, SMS_STS_GOAL_POSITION_L, &params[1], 6);
}

/**
 * @brief Write speed command to servo (wheel mode)
 * @param handle Servo handle
 * @param id Servo ID
 * @param speed Speed (-4095 to 4095, negative = reverse)
 * @param acceleration Acceleration (0-255)
 * @return true if successful, false otherwise
 */
bool STServo_WriteSpeed(STServo_Handle_t *handle, uint8_t id, int16_t speed, uint8_t acceleration)
{
    if (!handle || !handle->initialized) {
        last_error = STSERVO_ERROR_INVALID_PARAM;
        return false;
    }
    
    uint8_t speed_data[2];
    speed_data[0] = speed & 0xFF;
    speed_data[1] = (speed >> 8) & 0xFF;
    
    return STServo_WriteData(handle, id, SMS_STS_ACC, &acceleration, 1) &&
           STServo_WriteData(handle, id, SMS_STS_GOAL_SPEED_L, speed_data, 2);
}

/**
 * @brief Enable or disable servo torque
 * @param handle Servo handle
 * @param id Servo ID
 * @param enable true to enable torque, false to disable
 * @return true if successful, false otherwise
 */
bool STServo_EnableTorque(STServo_Handle_t *handle, uint8_t id, bool enable)
{
    if (!handle || !handle->initialized) {
        last_error = STSERVO_ERROR_INVALID_PARAM;
        return false;
    }
    
    uint8_t torque_enable = enable ? 1 : 0;
    return STServo_WriteData(handle, id, SMS_STS_TORQUE_ENABLE, &torque_enable, 1);
}

/**
 * @brief Read current position from servo
 * @param handle Servo handle
 * @param id Servo ID
 * @return Current position (0-4095) or -1 on error
 */
int16_t STServo_ReadPosition(STServo_Handle_t *handle, uint8_t id)
{
    if (!handle || !handle->initialized) {
        last_error = STSERVO_ERROR_INVALID_PARAM;
        return -1;
    }
    
    uint8_t data[2];
    if (STServo_ReadData(handle, id, SMS_STS_PRESENT_POSITION_L, data, 2)) {
        return (int16_t)(data[0] | (data[1] << 8));
    }
    return -1;
}

/**
 * @brief Read current speed from servo
 * @param handle Servo handle
 * @param id Servo ID
 * @return Current speed or -1 on error
 */
int16_t STServo_ReadSpeed(STServo_Handle_t *handle, uint8_t id)
{
    if (!handle || !handle->initialized) {
        last_error = STSERVO_ERROR_INVALID_PARAM;
        return -1;
    }
    
    uint8_t data[2];
    if (STServo_ReadData(handle, id, SMS_STS_PRESENT_SPEED_L, data, 2)) {
        return (int16_t)(data[0] | (data[1] << 8));
    }
    return -1;
}

/**
 * @brief Read current load from servo
 * @param handle Servo handle
 * @param id Servo ID
 * @return Current load or -1 on error
 */
int16_t STServo_ReadLoad(STServo_Handle_t *handle, uint8_t id)
{
    if (!handle || !handle->initialized) {
        last_error = STSERVO_ERROR_INVALID_PARAM;
        return -1;
    }
    
    uint8_t data[2];
    if (STServo_ReadData(handle, id, SMS_STS_PRESENT_LOAD_L, data, 2)) {
        return (int16_t)(data[0] | (data[1] << 8));
    }
    return -1;
}

/**
 * @brief Read voltage from servo
 * @param handle Servo handle
 * @param id Servo ID
 * @return Voltage in 0.1V units or -1 on error
 */
int16_t STServo_ReadVoltage(STServo_Handle_t *handle, uint8_t id)
{
    if (!handle || !handle->initialized) {
        last_error = STSERVO_ERROR_INVALID_PARAM;
        return -1;
    }
    
    uint8_t data;
    if (STServo_ReadData(handle, id, SMS_STS_PRESENT_VOLTAGE, &data, 1)) {
        return (int16_t)data;
    }
    return -1;
}

/**
 * @brief Read temperature from servo
 * @param handle Servo handle
 * @param id Servo ID
 * @return Temperature in Celsius or -1 on error
 */
int16_t STServo_ReadTemperature(STServo_Handle_t *handle, uint8_t id)
{
    if (!handle || !handle->initialized) {
        last_error = STSERVO_ERROR_INVALID_PARAM;
        return -1;
    }
    
    uint8_t data;
    if (STServo_ReadData(handle, id, SMS_STS_PRESENT_TEMPERATURE, &data, 1)) {
        return (int16_t)data;
    }
    return -1;
}

/**
 * @brief Read current from servo
 * @param handle Servo handle
 * @param id Servo ID
 * @return Current in mA or -1 on error
 */
int16_t STServo_ReadCurrent(STServo_Handle_t *handle, uint8_t id)
{
    if (!handle || !handle->initialized) {
        last_error = STSERVO_ERROR_INVALID_PARAM;
        return -1;
    }
    
    uint8_t data[2];
    if (STServo_ReadData(handle, id, SMS_STS_PRESENT_CURRENT_L, data, 2)) {
        return (int16_t)(data[0] | (data[1] << 8));
    }
    return -1;
}

/**
 * @brief Check if servo is moving
 * @param handle Servo handle
 * @param id Servo ID
 * @return true if moving, false if stopped or error
 */
bool STServo_IsMoving(STServo_Handle_t *handle, uint8_t id)
{
    if (!handle || !handle->initialized) {
        last_error = STSERVO_ERROR_INVALID_PARAM;
        return false;
    }
    
    uint8_t data;
    if (STServo_ReadData(handle, id, SMS_STS_MOVING, &data, 1)) {
        return data != 0;
    }
    return false;
}

/**
 * @brief Ping servo to check if it's responding
 * @param handle Servo handle
 * @param id Servo ID
 * @return true if servo responds, false otherwise
 */
bool STServo_Ping(STServo_Handle_t *handle, uint8_t id)
{
    if (!handle || !handle->initialized) {
        last_error = STSERVO_ERROR_INVALID_PARAM;
        return false;
    }
    
    STServo_Packet_t packet;
    uint8_t packet_length = STServo_BuildPacket(&packet, id, ST_INST_PING, NULL, 0);
    
    if (!STServo_SendPacket(handle, (uint8_t*)&packet, packet_length)) {
        return false;
    }
    
    uint8_t response[8];
    uint8_t response_length;
    return STServo_ReceivePacket(handle, response, &response_length);
}

/**
 * @brief Get last error
 * @param handle Servo handle
 * @return Last error code
 */
STServo_Error_t STServo_GetLastError(STServo_Handle_t *handle)
{
    (void)handle;  // Unused parameter
    return last_error;
}

/**
 * @brief Get error string
 * @param error Error code
 * @return Error description string
 */
const char* STServo_GetErrorString(STServo_Error_t error)
{
    switch (error) {
        case STSERVO_OK: return "No error";
        case STSERVO_ERROR_TIMEOUT: return "Communication timeout";
        case STSERVO_ERROR_CHECKSUM: return "Checksum error";
        case STSERVO_ERROR_INVALID_PARAM: return "Invalid parameter";
        case STSERVO_ERROR_COMM_FAILED: return "Communication failed";
        default: return "Unknown error";
    }
}

// Private function implementations

/**
 * @brief Send packet via UART
 * @param handle Servo handle
 * @param packet Packet data
 * @param length Packet length
 * @return true if successful, false otherwise
 */
static bool STServo_SendPacket(STServo_Handle_t *handle, const uint8_t *packet, uint8_t length)
{
    if (!handle || !packet) {
        last_error = STSERVO_ERROR_INVALID_PARAM;
        return false;
    }
    
    HAL_StatusTypeDef status = HAL_UART_Transmit(handle->huart, (uint8_t*)packet, length, handle->timeout_ms);
    if (status != HAL_OK) {
        last_error = STSERVO_ERROR_COMM_FAILED;
        return false;
    }
    
    return true;
}

/**
 * @brief Receive packet via UART
 * @param handle Servo handle
 * @param packet Buffer for received packet
 * @param length Pointer to received length
 * @return true if successful, false otherwise
 */
static bool STServo_ReceivePacket(STServo_Handle_t *handle, uint8_t *packet, uint8_t *length)
{
    if (!handle || !packet || !length) {
        last_error = STSERVO_ERROR_INVALID_PARAM;
        return false;
    }
    
    // Wait for header bytes
    uint8_t header[2];
    HAL_StatusTypeDef status = HAL_UART_Receive(handle->huart, header, 2, handle->timeout_ms);
    if (status != HAL_OK || header[0] != 0xFF || header[1] != 0xFF) {
        last_error = STSERVO_ERROR_TIMEOUT;
        return false;
    }
    
    // Read ID and length
    uint8_t id_length[2];
    status = HAL_UART_Receive(handle->huart, id_length, 2, handle->timeout_ms);
    if (status != HAL_OK) {
        last_error = STSERVO_ERROR_TIMEOUT;
        return false;
    }
    
    uint8_t packet_length = id_length[1];
    if (packet_length > ST_SERVO_BUFFER_SIZE - 4) {
        last_error = STSERVO_ERROR_INVALID_PARAM;
        return false;
    }
    
    // Read remaining data
    uint8_t remaining_data[ST_SERVO_BUFFER_SIZE];
    status = HAL_UART_Receive(handle->huart, remaining_data, packet_length, handle->timeout_ms);
    if (status != HAL_OK) {
        last_error = STSERVO_ERROR_TIMEOUT;
        return false;
    }
    
    // Build complete packet
    packet[0] = header[0];
    packet[1] = header[1];
    packet[2] = id_length[0];
    packet[3] = id_length[1];
    memcpy(&packet[4], remaining_data, packet_length);
    
    *length = packet_length + 4;
    
    // Validate checksum
    if (!STServo_ValidateChecksum(packet, *length)) {
        last_error = STSERVO_ERROR_CHECKSUM;
        return false;
    }
    
    return true;
}

/**
 * @brief Write data to servo register
 * @param handle Servo handle
 * @param id Servo ID
 * @param address Register address
 * @param data Data to write
 * @param length Data length
 * @return true if successful, false otherwise
 */
static bool STServo_WriteData(STServo_Handle_t *handle, uint8_t id, uint8_t address, const uint8_t *data, uint8_t length)
{
    if (!handle || !data || length == 0) {
        last_error = STSERVO_ERROR_INVALID_PARAM;
        return false;
    }
    
    uint8_t params[ST_SERVO_BUFFER_SIZE - 6];
    params[0] = address;
    memcpy(&params[1], data, length);
    
    STServo_Packet_t packet;
    uint8_t packet_length = STServo_BuildPacket(&packet, id, ST_INST_WRITE, params, length + 1);
    
    return STServo_SendPacket(handle, (uint8_t*)&packet, packet_length);
}

/**
 * @brief Read data from servo register
 * @param handle Servo handle
 * @param id Servo ID
 * @param address Register address
 * @param data Buffer for read data
 * @param length Data length to read
 * @return true if successful, false otherwise
 */
static bool STServo_ReadData(STServo_Handle_t *handle, uint8_t id, uint8_t address, uint8_t *data, uint8_t length)
{
    if (!handle || !data || length == 0) {
        last_error = STSERVO_ERROR_INVALID_PARAM;
        return false;
    }
    
    uint8_t params[2];
    params[0] = address;
    params[1] = length;
    
    STServo_Packet_t packet;
    uint8_t packet_length = STServo_BuildPacket(&packet, id, ST_INST_READ, params, 2);
    
    if (!STServo_SendPacket(handle, (uint8_t*)&packet, packet_length)) {
        return false;
    }
    
    uint8_t response[ST_SERVO_BUFFER_SIZE];
    uint8_t response_length;
    if (!STServo_ReceivePacket(handle, response, &response_length)) {
        return false;
    }
    
    // Extract data from response (skip header, ID, length, error, checksum)
    if (response_length >= length + 6) {
        memcpy(data, &response[5], length);
        return true;
    }
    
    last_error = STSERVO_ERROR_INVALID_PARAM;
    return false;
} 
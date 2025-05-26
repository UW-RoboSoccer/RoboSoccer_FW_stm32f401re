/**
 * @file STServo.h
 * @brief ST Series Serial Servo Control Library for STM32F401RE
 * @author Adapted for PlatformIO/STM32F401RE
 * @date 2025
 * 
 * This library provides control for STS3215 and other ST series servos
 * using UART communication on STM32F401RE with HAL drivers.
 */

#ifndef _ST_SERVO_H
#define _ST_SERVO_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Baud rate definitions
#define SMS_STS_1M      0
#define SMS_STS_0_5M    1
#define SMS_STS_250K    2
#define SMS_STS_128K    3
#define SMS_STS_115200  4
#define SMS_STS_76800   5
#define SMS_STS_57600   6
#define SMS_STS_38400   7

// Memory table definitions
// -------EPROM(Read Only)--------
#define SMS_STS_MODEL_L 3
#define SMS_STS_MODEL_H 4

// -------EPROM(Read/Write)--------
#define SMS_STS_ID 5
#define SMS_STS_BAUD_RATE 6
#define SMS_STS_MIN_ANGLE_LIMIT_L 9
#define SMS_STS_MIN_ANGLE_LIMIT_H 10
#define SMS_STS_MAX_ANGLE_LIMIT_L 11
#define SMS_STS_MAX_ANGLE_LIMIT_H 12
#define SMS_STS_CW_DEAD 26
#define SMS_STS_CCW_DEAD 27
#define SMS_STS_OFS_L 31
#define SMS_STS_OFS_H 32
#define SMS_STS_MODE 33

// -------SRAM(Read/Write)--------
#define SMS_STS_TORQUE_ENABLE 40
#define SMS_STS_ACC 41
#define SMS_STS_GOAL_POSITION_L 42
#define SMS_STS_GOAL_POSITION_H 43
#define SMS_STS_GOAL_TIME_L 44
#define SMS_STS_GOAL_TIME_H 45
#define SMS_STS_GOAL_SPEED_L 46
#define SMS_STS_GOAL_SPEED_H 47
#define SMS_STS_LOCK 55

// -------SRAM(Read Only)--------
#define SMS_STS_PRESENT_POSITION_L 56
#define SMS_STS_PRESENT_POSITION_H 57
#define SMS_STS_PRESENT_SPEED_L 58
#define SMS_STS_PRESENT_SPEED_H 59
#define SMS_STS_PRESENT_LOAD_L 60
#define SMS_STS_PRESENT_LOAD_H 61
#define SMS_STS_PRESENT_VOLTAGE 62
#define SMS_STS_PRESENT_TEMPERATURE 63
#define SMS_STS_MOVING 66
#define SMS_STS_PRESENT_CURRENT_L 69
#define SMS_STS_PRESENT_CURRENT_H 70

// Servo control structure
typedef struct {
    UART_HandleTypeDef *huart;
    uint32_t timeout_ms;
    bool initialized;
} STServo_Handle_t;

// Function prototypes
// Initialization
bool STServo_Init(STServo_Handle_t *handle, UART_HandleTypeDef *huart);
void STServo_DeInit(STServo_Handle_t *handle);

// Basic servo control
bool STServo_WritePosition(STServo_Handle_t *handle, uint8_t id, int16_t position, uint16_t speed, uint8_t acceleration);
bool STServo_WriteSpeed(STServo_Handle_t *handle, uint8_t id, int16_t speed, uint8_t acceleration);
bool STServo_EnableTorque(STServo_Handle_t *handle, uint8_t id, bool enable);

// Advanced control
bool STServo_RegWritePosition(STServo_Handle_t *handle, uint8_t id, int16_t position, uint16_t speed, uint8_t acceleration);
void STServo_RegWriteAction(STServo_Handle_t *handle);
bool STServo_SyncWritePosition(STServo_Handle_t *handle, uint8_t ids[], uint8_t count, int16_t positions[], uint16_t speeds[], uint8_t accelerations[]);

// Feedback functions
int16_t STServo_ReadPosition(STServo_Handle_t *handle, uint8_t id);
int16_t STServo_ReadSpeed(STServo_Handle_t *handle, uint8_t id);
int16_t STServo_ReadLoad(STServo_Handle_t *handle, uint8_t id);
int16_t STServo_ReadVoltage(STServo_Handle_t *handle, uint8_t id);
int16_t STServo_ReadTemperature(STServo_Handle_t *handle, uint8_t id);
int16_t STServo_ReadCurrent(STServo_Handle_t *handle, uint8_t id);
bool STServo_IsMoving(STServo_Handle_t *handle, uint8_t id);

// Utility functions
bool STServo_Ping(STServo_Handle_t *handle, uint8_t id);
bool STServo_SetWheelMode(STServo_Handle_t *handle, uint8_t id);
bool STServo_CalibrationOffset(STServo_Handle_t *handle, uint8_t id);
bool STServo_UnlockEprom(STServo_Handle_t *handle, uint8_t id);
bool STServo_LockEprom(STServo_Handle_t *handle, uint8_t id);

// Error handling
typedef enum {
    STSERVO_OK = 0,
    STSERVO_ERROR_TIMEOUT,
    STSERVO_ERROR_CHECKSUM,
    STSERVO_ERROR_INVALID_PARAM,
    STSERVO_ERROR_COMM_FAILED
} STServo_Error_t;

STServo_Error_t STServo_GetLastError(STServo_Handle_t *handle);
const char* STServo_GetErrorString(STServo_Error_t error);

#ifdef __cplusplus
}
#endif

#endif // _ST_SERVO_H 
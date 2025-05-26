/**
 * @file main.c
 * @brief STM32 Servo Communication Test - Dual UART Version
 * @author PlatformIO/STM32F401RE Implementation
 * @date 2025
 */

#include "stm32f4xx.h"
#include <string.h>

// Simple delay function
void simple_delay(volatile uint32_t count) {
    while(count--);
}

// UART2 transmit function (for debug output to USB)
void uart2_transmit(const char* str) {
    while (*str) {
        // Wait for transmit data register to be empty
        while (!(USART2->SR & USART_SR_TXE));
        // Send character
        USART2->DR = *str++;
    }
}

// UART1 transmit bytes (for servo communication)
void uart1_transmit_bytes(const uint8_t* data, int len) {
    for (int i = 0; i < len; i++) {
        while (!(USART1->SR & USART_SR_TXE));
        USART1->DR = data[i];
    }
}

// UART1 receive with timeout (for servo responses)
int uart1_receive_bytes(uint8_t* buffer, int max_len, int timeout_ms) {
    int received = 0;
    int timeout_count = timeout_ms * 1000; // Rough timeout conversion
    
    while (received < max_len && timeout_count > 0) {
        if (USART1->SR & USART_SR_RXNE) {
            buffer[received++] = USART1->DR;
            timeout_count = timeout_ms * 1000; // Reset timeout on data
        } else {
            timeout_count--;
            simple_delay(10);
        }
    }
    return received;
}

// Initialize USART2 for debug (115200 baud) - USB Serial
void init_uart2_debug(void) {
    // Enable USART2 clock
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    
    // Configure PA2 (TX) and PA3 (RX) for UART
    GPIOA->MODER &= ~((3 << (2*2)) | (3 << (3*2)));  // Clear mode bits
    GPIOA->MODER |= (2 << (2*2)) | (2 << (3*2));     // Set alternate function mode
    
    // Set alternate function to AF7 (USART2)
    GPIOA->AFR[0] &= ~((0xF << (2*4)) | (0xF << (3*4)));
    GPIOA->AFR[0] |= (7 << (2*4)) | (7 << (3*4));
    
    // Configure USART2 for 115200 baud at 16MHz
    USART2->BRR = 139;  // 16000000 / 115200 ≈ 139
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;  // Enable TX, RX, and USART
}

// Initialize USART1 for servo (1M baud) - PA9/PA10
void init_uart1_servo(void) {
    // Enable USART1 clock
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    
    // Configure PA9 (TX) and PA10 (RX) for UART
    GPIOA->MODER &= ~((3 << (9*2)) | (3 << (10*2)));  // Clear mode bits
    GPIOA->MODER |= (2 << (9*2)) | (2 << (10*2));     // Set alternate function mode
    
    // Set alternate function to AF7 (USART1)
    GPIOA->AFR[1] &= ~((0xF << ((9-8)*4)) | (0xF << ((10-8)*4)));
    GPIOA->AFR[1] |= (7 << ((9-8)*4)) | (7 << ((10-8)*4));
    
    // Configure USART1 for 1M baud at 16MHz: BRR = 16000000 / 1000000 = 16
    USART1->BRR = 16;
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;  // Enable TX, RX, and USART
}

// Send servo ping command
int ping_servo(uint8_t servo_id) {
    // STS Ping packet: [0xFF, 0xFF, ID, LEN, CMD, CHECKSUM]
    uint8_t cmd = 0x01;  // Ping command
    uint8_t len = 0x02;  // Length
    uint8_t checksum = ~(servo_id + len + cmd) & 0xFF;
    
    uint8_t packet[] = {0xFF, 0xFF, servo_id, len, cmd, checksum};
    
    // Send ping packet via UART1
    uart1_transmit_bytes(packet, 6);
    
    // Try to receive response via UART1
    uint8_t response[32];
    int received = uart1_receive_bytes(response, sizeof(response), 100);
    
    // Check if we got a valid response
    if (received >= 6 && response[0] == 0xFF && response[1] == 0xFF && response[2] == servo_id) {
        return 1; // Success
    }
    
    return 0; // No response or invalid
}

// Send servo position command (WritePosEx)
int move_servo(uint8_t servo_id, uint16_t position, uint16_t time, uint16_t speed) {
    // WritePosEx uses INST_WRITE (0x03) to memory address SMS_STS_ACC (41)
    // Data format: [ACC, POS_L, POS_H, TIME_L, TIME_H, SPEED_L, SPEED_H]
    uint8_t cmd = 0x03;  // INST_WRITE command
    uint8_t mem_addr = 41;  // SMS_STS_ACC memory address
    uint8_t data_len = 7;   // 7 bytes of data
    uint8_t packet_len = 3 + data_len;  // CMD + MEM_ADDR + DATA_LEN + DATA + CHECKSUM
    
    // Prepare the 7-byte data payload
    uint8_t acc = 50;  // Acceleration (same as working Python code)
    uint8_t pos_l = position & 0xFF;
    uint8_t pos_h = (position >> 8) & 0xFF;
    uint8_t time_l = time & 0xFF;
    uint8_t time_h = (time >> 8) & 0xFF;
    uint8_t speed_l = speed & 0xFF;
    uint8_t speed_h = (speed >> 8) & 0xFF;
    
    // Calculate checksum: ~(ID + LEN + CMD + MEM_ADDR + DATA...)
    uint8_t checksum = servo_id + packet_len + cmd + mem_addr + acc + pos_l + pos_h + time_l + time_h + speed_l + speed_h;
    checksum = ~checksum & 0xFF;
    
    // Build complete packet: [0xFF, 0xFF, ID, LEN, CMD, MEM_ADDR, DATA..., CHECKSUM]
    uint8_t packet[] = {
        0xFF, 0xFF,           // Header
        servo_id,             // Servo ID
        packet_len,           // Packet length
        cmd,                  // INST_WRITE command
        mem_addr,             // SMS_STS_ACC memory address
        acc,                  // Acceleration
        pos_l, pos_h,         // Position (low, high)
        time_l, time_h,       // Time (low, high) 
        speed_l, speed_h,     // Speed (low, high)
        checksum              // Checksum
    };
    
    // Send position packet via UART1
    uart1_transmit_bytes(packet, sizeof(packet));
    
    // Try to receive response via UART1
    uint8_t response[32];
    int received = uart1_receive_bytes(response, sizeof(response), 100);
    
    return (received > 0) ? 1 : 0;
}

int main(void)
{
    // Enable GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    
    // Configure PA5 as output (onboard LED)
    GPIOA->MODER &= ~(3 << (5 * 2));
    GPIOA->MODER |= (1 << (5 * 2));
    
    // Blink LED 3 times to show startup
    for (int i = 0; i < 3; i++) {
        GPIOA->BSRR = GPIO_BSRR_BS5;
        simple_delay(300000);
        GPIOA->BSRR = GPIO_BSRR_BR5;
        simple_delay(300000);
    }
    
    // Initialize both UARTs
    init_uart2_debug();   // USART2 for debug output to USB
    init_uart1_servo();   // USART1 for servo communication
    simple_delay(100000);
    
    uart2_transmit("\r\n🤖 STM32 Dual UART Servo Test\r\n");
    uart2_transmit("📡 Debug: USART2 (PA2/PA3) → USB Serial\r\n");
    uart2_transmit("🔧 Servo: USART1 (PA9/PA10) → Bus Adapter\r\n");
    uart2_transmit("🎯 BIG MOVEMENT TEST!\r\n\r\n");
    
    // Main loop - test servo communication
    int loop_count = 0;
    while (1)
    {
        uart2_transmit("\r\n--- Testing Servo ID 2 with BIG MOVEMENTS ---\r\n");
        
        // Test servo ID 2 directly (we know it works)
        int found = ping_servo(2);
        
        if (found) {
            uart2_transmit("✅ SERVO ID 2 FOUND!\r\n");
            
            // Blink LED rapidly to show success
            for (int j = 0; j < 5; j++) {
                GPIOA->BSRR = GPIO_BSRR_BS5;
                simple_delay(100000);
                GPIOA->BSRR = GPIO_BSRR_BR5;
                simple_delay(100000);
            }
            
            // BIG MOVEMENT SEQUENCE
            uart2_transmit("🚀 MOVEMENT 1: Going to FULL LEFT (512)\r\n");
            move_servo(2, 512, 2000, 300);   // Far left, 2 second movement, slow speed
            simple_delay(3000000);  // Wait 3 seconds
            
            uart2_transmit("🚀 MOVEMENT 2: Going to FULL RIGHT (3584)\r\n");
            move_servo(2, 3584, 2000, 300);  // Far right, 2 second movement, slow speed
            simple_delay(3000000);  // Wait 3 seconds
            
            uart2_transmit("🚀 MOVEMENT 3: Going to CENTER (2048)\r\n");
            move_servo(2, 2048, 2000, 300);  // Center, 2 second movement, slow speed
            simple_delay(3000000);  // Wait 3 seconds
            
            uart2_transmit("🚀 MOVEMENT 4: Quick LEFT-RIGHT-CENTER sequence\r\n");
            move_servo(2, 1024, 1000, 500);  // Left
            simple_delay(1500000);
            move_servo(2, 3072, 1000, 500);  // Right  
            simple_delay(1500000);
            move_servo(2, 2048, 1000, 500);  // Center
            simple_delay(1500000);
            
            uart2_transmit("🎉 MOVEMENT SEQUENCE COMPLETE!\r\n");
            
        } else {
            uart2_transmit("❌ Servo ID 2 not responding\r\n");
            
            // Single slow blink for no response
            GPIOA->BSRR = GPIO_BSRR_BS5;
            simple_delay(1000000);
            GPIOA->BSRR = GPIO_BSRR_BR5;
        }
        
        // Show loop count
        char msg[32];
        msg[0] = 'T';
        msg[1] = 'e';
        msg[2] = 's';
        msg[3] = 't';
        msg[4] = ' ';
        msg[5] = 'C';
        msg[6] = 'y';
        msg[7] = 'c';
        msg[8] = 'l';
        msg[9] = 'e';
        msg[10] = ' ';
        msg[11] = '0' + (loop_count % 10);
        msg[12] = '\r';
        msg[13] = '\n';
        msg[14] = '\0';
        uart2_transmit(msg);
        
        loop_count++;
        
        // Wait 5 seconds before next test cycle
        uart2_transmit("⏳ Waiting 5 seconds before next test...\r\n");
        simple_delay(5000000);
    }
} 
#include "serial_dd_protocol.h"
#include <stdio.h>
#include <stdint.h>
#include "main.h"  // Ensure this is included for type definitions
extern UART_HandleTypeDef huart1;

volatile DD_Uart_Message received_message;  // Structure to hold the decoded message
volatile uint8_t new_message_ready = 0;

// Function to validate the received message (start byte, length, checksum, end byte)
int validate_message(uint8_t *message, uint16_t length) {
    // Check start and end bytes
    if (message[0] != START_BYTE || message[length - 1] != END_BYTE) {
        return 0; // Invalid start or end byte
    }

    // Validate length
    uint16_t received_length = (message[1] << 8) | message[2];
    if (received_length != length) {
        return 0; // Length mismatch
    }

    // Calculate checksum
    uint8_t checksum = 0;
    for (uint16_t i = 1; i < length - 2; i++) {
        checksum ^= message[i];
    }

    // Validate checksum
    if (checksum != message[length - 2]) {
        return 0; // Invalid checksum
    }

    return 1; // Message is valid
}


void process_message(uint8_t *message, uint16_t length) {

    if (!validate_message(message, length)) {
        printf("Invalid message!\n");
        return;
    }

    // Decode the message into the structure
    DD_Uart_Message decoded_msg;
    decoded_msg.length = (message[1] << 8) | message[2];
    decoded_msg.command = (message[3] << 8) | message[4];
    memcpy(decoded_msg.data, &message[5], decoded_msg.length - 7); // Exclude header and footer
    decoded_msg.checksum = message[length - 2]; // Checksum field

    // Store the decoded message for further processing in main loop
    received_message = decoded_msg;
    new_message_ready = 1;  // Set flag indicating new message is available

//    // Print received message for debugging
//    printf("Received command: 0x%04X\n", decoded_msg.command);
//    printf("Received data length: %d\n", decoded_msg.length);
//    printf("Received data: ");
//    for (int i = 0; i < decoded_msg.length - 7; i++) {
//        printf("%02X ", decoded_msg.data[i]);
//    }
//    printf("\n");
}


void send_response(uint16_t command, const char *data) {
    uint8_t message[100];
    uint16_t length;

    // Prepare the message with the command and data as a string
    length = snprintf((char *)message + 5, sizeof(message) - 7, "%s", data) + 7;

    // Set up the message header
    message[0] = START_BYTE;
    message[1] = (length >> 8) & 0xFF;  // High byte of length
    message[2] = length & 0xFF;         // Low byte of length
    message[3] = (command >> 8) & 0xFF;
    message[4] = command & 0xFF;

    // Calculate checksum
    uint8_t checksum = 0;
    for (uint16_t i = 1; i < length - 2; i++) {
        checksum ^= message[i];
    }

    message[length - 2] = checksum;
    message[length - 1] = END_BYTE;

    // Send the message via UART
    HAL_UART_Transmit(&huart1, message, length, HAL_MAX_DELAY);
}

void send_response_len(uint16_t command, const char *data, uint16_t data_len) {
    uint16_t total_len = 7 + data_len;
    if (total_len > 1024) return;  // Safety check

    uint8_t message[1024];  // Buffer large enough for full response

    message[0] = START_BYTE;
    message[1] = (total_len >> 8) & 0xFF;
    message[2] = total_len & 0xFF;
    message[3] = (command >> 8) & 0xFF;
    message[4] = command & 0xFF;

    memcpy(&message[5], data, data_len);

    // Calculate checksum
    uint8_t checksum = 0;
    for (uint16_t i = 1; i < 5 + data_len; i++) {
        checksum ^= message[i];
    }

    message[5 + data_len] = checksum;
    message[6 + data_len] = END_BYTE;

    HAL_UART_Transmit(&huart1, message, total_len, HAL_MAX_DELAY);
}



void DD_USART1_IRQHandler(void) {
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) {
        static uint8_t buffer[100];  // Buffer to accumulate the received bytes
        static uint16_t buffer_index = 0;

        uint8_t byte_received = (uint8_t)(huart1.Instance->DR & 0xFF); // Read received byte

        buffer[buffer_index++] = byte_received;

        // Check if we've received the end byte
        if (byte_received == END_BYTE) {
            // Process the message when the end byte is received
            process_message(buffer, buffer_index);

            // Reset the buffer index for the next message
            buffer_index = 0;
        }
    }


}

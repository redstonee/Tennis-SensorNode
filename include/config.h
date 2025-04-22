#pragma once
#include <Arduino.h>

constexpr auto LORA_M0_PIN = PD2;
constexpr auto LORA_M1_PIN = PD1;
constexpr auto UART1_TX_PIN = PB6;
constexpr auto UART1_RX_PIN = PB7;

// USB Serial
constexpr auto UART2_TX_PIN = PA2;
constexpr auto UART2_RX_PIN = PA3;

// Accelerometer
constexpr auto I2C2_SCL_PIN = PB10;
constexpr auto I2C2_SDA_PIN = PB11;

// Battery
constexpr auto VSENS_PIN = PA1;
constexpr auto SAMPLE_FACTOR = 1.458f; // Voltage divider factor

constexpr auto ACC2_THRESHOLD = 3e5; // Threshold for collision detection

// Lora addresses, 0x00 for dongle, 0x10 - 0x1F for sensor nodes, 0xFF for light node
constexpr uint8_t LORA_DONGLE_ADDRESS = 0x00;      // Address of the dongle
constexpr uint8_t LORA_LIGHT_NODE_ADDRESS = 0xFF;  // Address of the light node
constexpr uint8_t LORA_SENSOR_ADDRESS_BASE = 0x10; // Default address of the sensor node

constexpr uint8_t LORA_CHANNEL = 45;           // Channel of the nodes
constexpr auto LORA_DATA_MIN_INTERVAL = 1000;  // Minimum interval between data packets in ms
constexpr auto LORA_HEARTBEAT_INTERVAL = 3000; // Heartbeat interval in ms

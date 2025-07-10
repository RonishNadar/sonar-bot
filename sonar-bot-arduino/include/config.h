#ifndef CONFIG_H
#define CONFIG_H

#define LEFT_SERVO_PIN 9
#define RIGHT_SERVO_PIN 10
#define LIDAR_SERVO_PIN 11

#define TRIG_PIN 7
#define ECHO_PIN 8

#define BATTERY_PIN A0

#define MPU_ADDRESS 0x68
#define WHEEL_RADIUS 0.03
#define WHEEL_BASE 0.15

#define UART_RX 2   // ESP32 TX -> Arduino RX
#define UART_TX 3   // ESP32 RX <- Arduino TX

#endif

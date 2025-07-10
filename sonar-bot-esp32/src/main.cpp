#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <micro_ros_arduino_udp_transport.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/twist.h>

// Wi-Fi credentials
const char* ssid = "NYU_1143#2";
const char* password = "ChennaiBoizz";

// IP address of your ROS 2 PC (running micro-ROS agent)
IPAddress agent_ip(192, 168, 1, 100);  // <-- Set this to match your PC
uint16_t agent_port = 8888;

// UART to Arduino
#define TX_PIN 17
#define RX_PIN 16
#define BAUD_RATE 115200
HardwareSerial SerialBot(1);

// Micro-ROS handles
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;

std_msgs__msg__Float32MultiArray sensor_msg;
geometry_msgs__msg__Twist cmd_msg;

// Handle incoming cmd_vel messages
void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  SerialBot.print(msg->linear.x, 3);
  SerialBot.print(",");
  SerialBot.println(msg->angular.z, 3);
}

void setup() {
  Serial.begin(115200);
  SerialBot.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  // Set up UDP transport for micro-ROS
  set_microros_udp_transports(agent_ip, agent_port);

  // micro-ROS initialization
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_bot_node", "", &support);

  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "bot_status_topic"
  );

  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"
  );

  sensor_msg.data.capacity = 6;
  sensor_msg.data.size = 6;
  sensor_msg.data.data = (float *)malloc(6 * sizeof(float));

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &cmd_msg, &cmd_vel_callback, ON_NEW_DATA);
}

void loop() {
  if (SerialBot.available()) {
    String line = SerialBot.readStringUntil('\n');
    int count = 0;
    char *ptr = strtok((char *)line.c_str(), ",");
    while (ptr != nullptr && count < 6) {
      sensor_msg.data.data[count++] = atof(ptr);
      ptr = strtok(nullptr, ",");
    }

    if (count == 6) {
      rcl_publish(&publisher, &sensor_msg, NULL);
    }
  }

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
}

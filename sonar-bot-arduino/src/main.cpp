#include <Servo.h>
#include <Wire.h>
#include <AS5600.h>
#include <MPU6050.h>
#include <SoftwareSerial.h>
#include <config.h>
#include "kalman.h"

Kalman1D theta_kf(0.005, 0.1);  // Q=0.005, R=0.1
Servo leftServo, rightServo, lidarServo;
AS5600 leftEnc, rightEnc, lidarEnc;
MPU6050 mpu;
SoftwareSerial espSerial(UART_RX, UART_TX); // RX, TX

float x = 0, y = 0, theta_odom = 0, theta_imu = 0;
float last_left = 0, last_right = 0;
unsigned long last_time = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  espSerial.begin(115200);

  leftServo.attach(LEFT_SERVO_PIN);
  rightServo.attach(RIGHT_SERVO_PIN);
  lidarServo.attach(LIDAR_SERVO_PIN);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  mpu.initialize();
  mpu.setDLPFMode(MPU6050_DLPF_BW_20);

  delay(1000);
  last_left = leftEnc.readAngle();
  last_right = rightEnc.readAngle();
  last_time = millis();
}

float readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  return duration * 0.0343 / 2.0;
}

float readBatteryVoltage() {
  return analogRead(BATTERY_PIN) * (5.0 / 1023.0) * 2;
}

float readLidarAngle() {
  return lidarEnc.readAngle() * 360.0;
}

float getIMUAngle(float dt) {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float accel_theta = atan2(ay, az) * 180 / PI;
  float gyro_theta = gx / 131.0;
  theta_imu = 0.75 * (theta_imu + gyro_theta * dt) + 0.25 * accel_theta;
  return theta_imu;
}

void updateOdometry(float dt) {
  float l_now = leftEnc.readAngle();
  float r_now = rightEnc.readAngle();

  float delta_l = (l_now - last_left) * 2 * PI * WHEEL_RADIUS;
  float delta_r = (r_now - last_right) * 2 * PI * WHEEL_RADIUS;

  last_left = l_now;
  last_right = r_now;

  float v = (delta_r + delta_l) / 2.0;
  float w = (delta_r - delta_l) / WHEEL_BASE;

  theta_odom += w;
  x += v * cos(theta_odom);
  y += v * sin(theta_odom);
}

void setMotorPWM(float v, float w) {
  float v_l = v - w * WHEEL_BASE / 2.0;
  float v_r = v + w * WHEEL_BASE / 2.0;

  int pwm_l = constrain(map(v_l * 100, -100, 100, 0, 180), 0, 180);
  int pwm_r = constrain(map(v_r * 100, -100, 100, 180, 0), 0, 180);

  leftServo.write(pwm_l);
  rightServo.write(pwm_r);
}

void receiveVelocityCmd() {
  if (espSerial.available()) {
    String data = espSerial.readStringUntil('\n');
    float v = data.substring(0, data.indexOf(',')).toFloat();
    float w = data.substring(data.indexOf(',') + 1).toFloat();
    setMotorPWM(v, w);
  }
}

void loop() {
  unsigned long now = millis();
  float dt = (now - last_time) / 1000.0;
  if (dt < 0.05) return;
  last_time = now;

  updateOdometry(dt);
  float imu_theta = getIMUAngle(dt);
  static float last_theta_odom = 0;
  float delta_odom = theta_odom - last_theta_odom;
  last_theta_odom = theta_odom;
  theta_kf.predict(delta_odom);
  theta_kf.update(imu_theta);
  float fused_theta = theta_kf.get();

  float dist = readDistanceCM();
  float lidar_angle = readLidarAngle();
  float batt = readBatteryVoltage();

  espSerial.print(x); espSerial.print(",");
  espSerial.print(y); espSerial.print(",");
  espSerial.print(fused_theta); espSerial.print(",");
  espSerial.print(lidar_angle); espSerial.print(",");
  espSerial.print(dist); espSerial.print(",");
  espSerial.println(batt);

  receiveVelocityCmd();
}

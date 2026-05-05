#include <Arduino.h>
#include "nrf24_interface.h"
#include "motor_control.h"
#include "mpu6050_interface.h"

// Define a variable to hold the incoming data
RF24_Packet myPacket;

void setup() {
  Serial.begin(115200);
  motor_init();
  mpu6050_init();
  // 1. Initialize the radio
  if (nrf24_init()) {
    Serial.println("System Ready.");
  } else {
    while (1){
      Serial.println("Radio Hardware Error.");
    } // Halt if radio fails
  }

  mpu6050_calibrate_gyro();
  mpu6050_reset_attitude();
}


void loop() {
  // Manually ramp up the brightness to test PWM
  // motor_arm(); // Set armed to 1 so motor_write_all would work
  // for (int val = 1000; val <= 2000; val++) {
  //   motor_set_speed(0, val);
  //   motor_set_speed(1, val);
  //   motor_set_speed(2, val);
  //   motor_set_speed(3, val);
  //   delay(1);
  // }

  // 1. Read raw data and compute attitude first
  IMU_Data imu;
  Attitude att;
  float dt = 0.01; // Assuming 100Hz loop (0.01s)

  if (mpu6050_read_raw(&imu)) {
    mpu6050_compute_attitude(&imu, &att, dt);

    // 2. Print the values
    Serial.print("Roll: ");
    Serial.print(mpu6050_get_roll() / 10.0f); // Convert 0.1° to degrees
    Serial.print(" | Pitch: ");
    Serial.print(mpu6050_get_pitch() / 10.0f);
    Serial.print(" | Yaw: ");
    Serial.println(mpu6050_get_yaw());
  } else {
    mpu6050_init();
    Serial.println("read failed");
  }
  // 2. Poll for new packets
  if (nrf24_read_commands(&myPacket)) {

    // 3. Process the data
    Serial.println("Packet Received! Data: ");
    // Example: accessing members of your struct
    Serial.println(myPacket.throttle); 
    Serial.println(myPacket.yaw); 
    Serial.println(myPacket.roll); 
    Serial.println(myPacket.pitch); 
    Serial.println(myPacket.AUX1); 
    Serial.println(myPacket.AUX2); 

  }

  // 4. Optional: Check for signal loss (failsafe)
  if (millis() - nrf24_get_last_packet_ms() > 1000) {
    // No packets for 1 second - trigger landing/failsafe code
  }
}

// main.ino
#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"
#include "SERVO.h"
#include "PID.h"

#define SERVO_PIN 18

MPU6050 mpu;
SERVO servo;
PID pid; // object pid

// complementary filter parameter
const float alpha = 0.98;
float angle_cf = 0.0;

// control timing
unsigned long last_time_us = 0;
const unsigned long control_interval_us = 10000; // 10 ms

// servo mapping limits (degree offset from mid)
const float servo_mid = 90.0;
const float servo_output_limit = 60.0; // limit PID output before mapping to servo

void initServo()
{
  servo.attachPin(SERVO_PIN);
  servo.begin();
  servo.writeAngle((int)servo_mid);
  delay(200);
}

float accelYToAngleDeg(float ay, float az)
{
  return atan2(ay, az) * 180.0 / PI;
}

void setup()
{
  Serial.begin(115200);
  delay(100);
  Wire.begin();
  mpu.initMPU();
  initServo();

  // Setup PID: initial guess values (tune later)
  float Kp = 35.0;
  float Ki = 0.0; // start with 0
  float Kd = 0.8;
  pid.setTunings(Kp, Ki, Kd);
  pid.setOutputLimits(-servo_output_limit, servo_output_limit); // PID output in deg offset
  pid.setIntegratorLimits(-200.0, 200.0);                       // anti-windup integrator clamp

  last_time_us = micros();
  Serial.println("Inverted pendulum (Y-axis) with separate PID module");
  Serial.println("Use Serial prints to tune (Kp/Kd/ Ki). Start with Ki=0.");
}

void loop()
{
  unsigned long now_us = micros();
  if (now_us - last_time_us >= control_interval_us)
  {
    float dt = (now_us - last_time_us) / 1000000.0;
    last_time_us = now_us;

    // read sensor
    mpu.read_raw_data();
    mpu.convert_data();

    // compute angles & rates
    float accel_angle = accelYToAngleDeg(mpu.get_accel_y_g(), mpu.get_accel_z_g());
    float gyro_rate = mpu.get_gyro_x_dps(); // deg/s (tilt around X)

    // combine with complementary filter
    angle_cf = alpha * (angle_cf + gyro_rate * dt) + (1.0 - alpha) * accel_angle;

    // PID compute (setpoint = 0 deg upright)
    float setpoint = 90;
    float pid_output = pid.compute(setpoint, angle_cf, dt); // in deg offset

    // map PID output to servo angle
    float servoAngle = servo_mid + pid_output;
    servoAngle = constrain(servoAngle, 0.0, 180.0);
    servo.writeAngle((int)servoAngle);

    // debug print (tampilkan nilai penting untuk tuning)
    Serial.print("dt:");
    Serial.print(dt * 1000, 2);
    Serial.print("ms ");
    Serial.print("accelAng:");
    Serial.print(accel_angle, 2);
    Serial.print(" cf:");
    Serial.print(angle_cf, 2);
    Serial.print(" gyroX:");
    Serial.print(gyro_rate, 2);
    Serial.print(" pid_out:");
    Serial.print(pid_output, 2);
    Serial.print(" servo:");
    Serial.println(servoAngle, 2);
  }
}

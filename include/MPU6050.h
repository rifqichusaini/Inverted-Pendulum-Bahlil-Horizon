// MPU6050.h
#ifndef MPU6050_h
#define MPU6050_h

#include <Arduino.h>
#include <Wire.h>

#define MPU_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B

class MPU6050
{
private:
  int16_t accel_x, accel_y, accel_z;
  int16_t temp;
  int16_t gyro_x, gyro_y, gyro_z;

  float accel_x_g, accel_y_g, accel_z_g;
  float temp_c;
  float gyro_x_dps, gyro_y_dps, gyro_z_dps;

public:
  MPU6050() {}

  void initMPU()
  {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(PWR_MGMT_1);
    Wire.write(0); // wake up
    Wire.endTransmission(true);
    delay(50);
  }

  void read_raw_data()
  {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);

    accel_x = (int16_t)(Wire.read() << 8 | Wire.read());
    accel_y = (int16_t)(Wire.read() << 8 | Wire.read());
    accel_z = (int16_t)(Wire.read() << 8 | Wire.read());

    temp = (int16_t)(Wire.read() << 8 | Wire.read());

    gyro_x = (int16_t)(Wire.read() << 8 | Wire.read());
    gyro_y = (int16_t)(Wire.read() << 8 | Wire.read());
    gyro_z = (int16_t)(Wire.read() << 8 | Wire.read());
  }

  void convert_data()
  {
    accel_x_g = (float)accel_x / 16384.0;
    accel_y_g = (float)accel_y / 16384.0;
    accel_z_g = (float)accel_z / 16384.0;

    temp_c = (float)temp / 340.0 + 36.53;

    gyro_x_dps = (float)gyro_x / 131.0;
    gyro_y_dps = (float)gyro_y / 131.0;
    gyro_z_dps = (float)gyro_z / 131.0;
  }

  float get_accel_x_g() { return accel_x_g; }
  float get_accel_y_g() { return accel_y_g; }
  float get_accel_z_g() { return accel_z_g; }

  float get_gyro_x_dps() { return gyro_x_dps; }
  float get_gyro_y_dps() { return gyro_y_dps; }
  float get_gyro_z_dps() { return gyro_z_dps; }

  float get_temp_c() { return temp_c; }

  void print_data()
  {
    Serial.print("Ay: ");
    Serial.print(accel_y_g, 4);
    Serial.print(" Az: ");
    Serial.print(accel_z_g, 4);
    Serial.print(" | Gx: ");
    Serial.print(gyro_x_dps, 3);
    Serial.print(" | T: ");
    Serial.println(temp_c, 2);
  }
};

#endif

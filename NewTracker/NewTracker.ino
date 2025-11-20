#include <Wire.h>

#define MPU_ADDR 0x68     // Alamat I2C MPU6050
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define servo_pin 18

// Raw variable
int16_t ax_raw, ay_raw, az_raw;
int16_t temp_raw;
int16_t gx_raw, gy_raw, gz_raw;

// Konversi
float ax, ay, az;
float gx, gy, gz;
float temperatureC;

void setup() {
  Serial.begin(115200);
  Wire.begin();                // SDA = 21, SCL = 22 pada ESP32

  // Wake up MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0);
  Wire.endTransmission(true);

  Serial.println("MPU6050 Ready (No Library)");
}

void loop() {
  bacaMPU();
  konversiData();
  printData();
  run_servo();

  delay(100);  // ubah sesuai kebutuhan
}

void bacaMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);   // Register awal
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, 14, true);  // minta 14 byte

  ax_raw = (Wire.read() << 8) | Wire.read();
  ay_raw = (Wire.read() << 8) | Wire.read();
  az_raw = (Wire.read() << 8) | Wire.read();

  temp_raw = (Wire.read() << 8) | Wire.read();

  gx_raw = (Wire.read() << 8) | Wire.read();
  gy_raw = (Wire.read() << 8) | Wire.read();
  gz_raw = (Wire.read() << 8) | Wire.read();
}

void konversiData() {
  // accelerometer (1 g = 16384 LSB)
  ax = ax_raw / 16384.0;
  ay = ay_raw / 16384.0;
  az = az_raw / 16384.0;

  // gyroscope (1 °/s = 131 LSB)
  gx = gx_raw / 131.0;
  gy = gy_raw / 131.0;
  gz = gz_raw / 131.0;

  // suhu
  temperatureC = (temp_raw / 340.0) + 36.53;
}

void printData() {
  Serial.print("Accel[g]  X: "); Serial.print(ax);
  Serial.print("  Y: "); Serial.print(ay);
  Serial.print("  Z: "); Serial.println(az);

  Serial.print("Gyro[deg/s]  X: "); Serial.print(gx);
  Serial.print("  Y: "); Serial.print(gy);
  Serial.print("  Z: "); Serial.println(gz);

  Serial.print("Temp: ");
  Serial.print(temperatureC);
  Serial.println(" *C");

  Serial.println("---------------------------");
}

void run_servo() {
  float angle_f = ay * 90;
  int angle = map(angle_f, -90, 90, 0, 180);
  angle = constrain(angle, 0, 180);

  servoWriteDeg(angle);  // panggil PWM manual

  Serial.print("Servo: ");
  Serial.println(angle);
}

void servoWriteDeg(int deg) {
  int pulse = map(deg, 0, 180, 500, 2400);
  digitalWrite(servo_pin, HIGH);
  delayMicroseconds(pulse);
  digitalWrite(servo_pin, LOW);
  delayMicroseconds(20000 - pulse);
}

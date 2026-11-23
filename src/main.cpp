#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <PID_v1.h>

// I2C Pin Definitions
#define I2C_SDA 1
#define I2C_SCL 2

// L298N Pin Definitions
#define enable_A 47
#define enable_B 21
#define in_1 42
#define in_2 41
#define in_3 48
#define in_4 14

Adafruit_MPU6050 mpu;

// PID Variables
double setpoint = 0.0; 
double input = 0.0;    
double output = 0.0;   

// ลด Kp ลงมาเริ่มที่ 35.0 ก่อน เพราะปลดล็อกไฟเต็ม 255 แล้ว
double Kp = 40.0;
double Ki = 0.8;
double Kd = 15.0;

PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Complementary Filter & Timing Variables
float currentAngle = 0.0;
unsigned long prevTime = 0;
float gyroY_offset = 0.0; // เปลี่ยนชื่อให้ตรงแกน Y
int printCounter = 0;

void calibrateGyro() {
  Serial.println("Calibrating gyro... Keep robot completely still!");
  float sum = 0;
  int samples = 500;
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sum += g.gyro.y; // แก้เป็นแกน Y ให้ตรงกับใน loop()
    delay(3);
  }
  gyroY_offset = sum / samples;
  Serial.println("Gyro calibration complete!");
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  pinMode(enable_A, OUTPUT);
  pinMode(enable_B, OUTPUT);
  pinMode(in_1, OUTPUT);
  pinMode(in_2, OUTPUT);
  pinMode(in_3, OUTPUT);
  pinMode(in_4, OUTPUT);

  Wire.begin(I2C_SDA, I2C_SCL);

  if (!mpu.begin(0x68, &Wire)) {
    if (!mpu.begin(0x69, &Wire)) {
      Serial.println("Failed to find MPU6050 chip!");
      while (1) { delay(10); }
    }
  }

  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  calibrateGyro();

  // Configure PID Controller
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10);           // 10ms (100Hz)
  myPID.SetOutputLimits(-225, 225);  // ปลดล็อกกำลังมอเตอร์เต็ม 255!

  prevTime = micros();
}

void moveMotors(double pidOutput) {
  bool forward = (pidOutput > 0);
  
  int pwmVal = (int)abs(pidOutput);
  if (pwmVal > 0 && pwmVal < 45) {
    pwmVal = 45; // ค่าชดเชยแรงเสียดทานเริ่มต้นสำหรับไฟ 12V
  }
  if (pwmVal > 255) pwmVal = 255;

  digitalWrite(in_1, forward ? HIGH : LOW);
  digitalWrite(in_2, forward ? LOW : HIGH);
  digitalWrite(in_3, forward ? HIGH : LOW);
  digitalWrite(in_4, forward ? LOW : HIGH);

  analogWrite(enable_A, pwmVal);
  analogWrite(enable_B, pwmVal);
}

void loop() {
  unsigned long currentTime = micros();
  float dt = (currentTime - prevTime) / 1000000.0;

  if (dt >= 0.001) { // ปรับให้ตรงกับ 100Hz (0.01 วินาที)
    prevTime = currentTime;

    // 1. Read MPU6050 Data
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // 2. Calculate Pitch Angle & Gyro Rate (แกน Y)
    float accelAngle = atan2(a.acceleration.x, a.acceleration.z) * 180.0 / PI;
    float gyroRate = (g.gyro.y - gyroY_offset) * 180.0 / PI;

    // 3. Complementary Filter
    currentAngle = 0.98 * (currentAngle + gyroRate * dt) + 0.02 * accelAngle;
    input = currentAngle;

    // 4. Compute PID Output
    myPID.Compute();

    // 5. Drive Motors
    moveMotors(output);

    // 6. Print Telemetry ทุกๆ 100ms กันลูปกระตุก
    printCounter++;
    if (printCounter >= 10) {
      Serial.print("Angle: ");
      Serial.print(input);
      Serial.print(" | PID Output: ");
      Serial.println(output);
      printCounter = 0;
    }
  }
}

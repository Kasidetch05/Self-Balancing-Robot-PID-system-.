#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>

MPU6050 mpu(0x68);

// Sensor offsets
float ax_offset = 0, ay_offset = 0, az_offset = 0;
float gx_offset = 0, gy_offset = 0, gz_offset = 0;

float pitch_gyro_angle = 0.0;
unsigned long timer_us = 0;
float dt = 0.0f; // Global variable for time difference

// Motor pins
constexpr int IN1_A = 5, IN2_A = 6, ENA_A = 4;
constexpr int IN4_B = 9, IN3_B = 8, ENA_B = 10;

//==== Tune Kp first, if the Kp is correct slowly tune Kd and Ki ===//
constexpr float Kp = 40;
constexpr float Ki = 0.2;
constexpr float Kd = 0.9;

// === CONTROL VARIABLES ===
float speedControlInput = 0.0f;
constexpr float maxLeanAngle = 5.0f;

float integral = 0.0f;
float lastError = 0.0f;
float pitchSetpoint = 0.0f;

// Motor correction factors
constexpr float motorA_correction = 1.05f;
constexpr float motorB_correction = 1.05f;

// Safety limits
constexpr int maxPWM = 200;
constexpr float deadZoneForward = 3.0f; // Separate deadzone for leaning forward (positive error)
constexpr float deadZoneBackward = 8.0f; // Separate deadzone for leaning backward (negative error)
constexpr int calibrationSamples = 500;

// === IMPORTANT: TUNE THIS VALUE FIRST ===
float pitch_trim_offset = 0.0f; // Set this to the value you found in Pitch values

// Function declarations
void calibrateSensors();
float readPitch();
void moveForward(int pwm);
void moveBackward(int pwm);
void stopMotors();
void setMotor1(int pwm, bool forward);
void setMotor2(int pwm, bool forward);

void setup() {
  // Use a higher baud rate for faster communication
  Serial.begin(115200);
  Wire.begin();

  pinMode(IN1_A, OUTPUT); pinMode(IN2_A, OUTPUT); pinMode(ENA_A, OUTPUT);
  pinMode(IN4_B, OUTPUT); pinMode(IN3_B, OUTPUT); pinMode(ENA_B, OUTPUT);

  Serial.println("Initializing MPU6050...");
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (true) {}
  }

  Serial.println("MPU6050 connected.");
  calibrateSensors();

  delay(1000); // Wait for MPU to stabilize

  timer_us = micros();
  pitchSetpoint = 0.0f;
}

void loop() {
  // This static variable only runs once and holds its value
  static unsigned long startTime = millis();
  if (millis() - startTime < 2000) {
    stopMotors();
    integral = 0;
    lastError = 0;
    Serial.println("Waiting for stabilization...");
    return;
  }

  // Set the desired speed and calculate the target angle (setpoint)
  speedControlInput = 0.0f; // Set to 0.0 to stand still
  pitchSetpoint = speedControlInput * maxLeanAngle;

  float pitch = readPitch();
  float pitchError = pitch - pitchSetpoint;

  float output = 0.0f;

  // Use separate deadzones for positive and negative errors
  bool isErrorSignificant = false;
  if (pitchError > deadZoneForward) {
    isErrorSignificant = true;
  } else if (pitchError < -deadZoneBackward) {
    isErrorSignificant = true;
  }
  
  if (isErrorSignificant) {
    integral += pitchError * dt;
    integral = constrain(integral, -30.0f, 30.0f);

    float derivative = (pitchError - lastError) / dt;
    derivative = constrain(derivative, -150.0f, 150.0f);

    lastError = pitchError;

    output = Kp * pitchError + Ki * integral + Kd * derivative;
    output = constrain(output, -maxPWM, maxPWM);
  } else {
    integral = 0.0f;
    lastError = 0.0f;
    output = 0.0f;
  }

  int motorPWM = (int)output;

  // Motor control is now active
  if (motorPWM > 0) {
    moveForward(motorPWM);
  } else if (motorPWM < 0) {
    moveBackward(-motorPWM);
  } else {
    stopMotors();
  }

  // Debugging
  Serial.print("Pitch: ");
  Serial.print(pitch, 2);
  Serial.print(" | Setpoint: ");
  Serial.print(pitchSetpoint, 2);
  Serial.print(" | Error: ");
  Serial.print(pitchError, 2);
  Serial.print(" | PWM: ");
  Serial.println(motorPWM);
}

void calibrateSensors() {
  Serial.println("Calibrating MPU6050 offsets...");
  long ax_sum = 0, ay_sum = 0, az_sum = 0;
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;

  for (int i = 0; i < calibrationSamples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getAcceleration(&ax, &ay, &az);
    mpu.getRotation(&gx, &gy, &gz);
    ax_sum += ax; ay_sum += ay; az_sum += az;
    gx_sum += gx; gy_sum += gy; gz_sum += gz;
    delay(3);
  }

  ax_offset = ax_sum / (float)calibrationSamples;
  ay_offset = ay_sum / (float)calibrationSamples;
  az_offset = az_sum / (float)calibrationSamples;
  gx_offset = gx_sum / (float)calibrationSamples;
  gy_offset = gy_sum / (float)calibrationSamples;
  gz_offset = gz_sum / (float)calibrationSamples;

  Serial.println("Calibration complete.");
}

float readPitch() {
  int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
  mpu.getAcceleration(&ax_raw, &ay_raw, &az_raw);
  mpu.getRotation(&gx_raw, &gy_raw, &gz_raw);

  // Apply offsets
  float ax = ax_raw - ax_offset;
  float ay = ay_raw - ay_offset;
  float az = az_raw - az_offset;
  float gy = gy_raw - gy_offset;

  // Convert raw values to physical units
  float axg = ax / 16384.0f;
  float ayg = ay / 16384.0f;
  float azg = az / 16384.0f;
  float gy_deg_per_sec = gy / 131.0f;

  // Calculate pitch from accelerometer
  float pitch_acc = atan2(-ayg, azg) * 180.0f / PI;

  // Calculate time difference (dt)
  unsigned long now_us = micros();
  dt = (now_us - timer_us) / 1000000.0f;
  timer_us = now_us;
  
  // Sanity check for dt to prevent instability
  if (dt <= 0 || dt > 0.1f) {
    dt = 0.01f;
  }

  // Complementary filter
  constexpr float alpha = 1.0f;
  pitch_gyro_angle = alpha * (pitch_gyro_angle + gy_deg_per_sec * dt) + (1.0f - alpha) * pitch_acc;
  
  // Apply inversion and manual trim offset
  float pitch = -pitch_gyro_angle + pitch_trim_offset;

  return pitch;
}

void moveForward(int pwm) {
  int pwmA = constrain(pwm * motorA_correction, 0, 255);
  int pwmB = constrain(pwm * motorB_correction, 0, 255);
  setMotor1(pwmA, true);
  setMotor2(pwmB, true);
}

void moveBackward(int pwm) {
  int pwmA = constrain(pwm * motorA_correction, 0, 255);
  int pwmB = constrain(pwm * motorB_correction, 0, 255);
  setMotor1(pwmA, false);
  setMotor2(pwmB, false);
}

void stopMotors() {
  analogWrite(ENA_A, 0);
  analogWrite(ENA_B, 0);
  digitalWrite(IN1_A, LOW);
  digitalWrite(IN2_A, LOW);
  digitalWrite(IN3_B, LOW);
  digitalWrite(IN4_B, LOW);
}

void setMotor1(int pwm, bool forward) {
  digitalWrite(IN1_A, forward ? LOW : HIGH);
  digitalWrite(IN2_A, forward ? HIGH : LOW);
  analogWrite(ENA_A, pwm);
}

void setMotor2(int pwm, bool forward) {
  digitalWrite(IN4_B, forward ? LOW : HIGH);
  digitalWrite(IN3_B, forward ? HIGH : LOW);
  analogWrite(ENA_B, pwm);
}
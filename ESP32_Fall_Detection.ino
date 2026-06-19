// ============================================
// ESP32 FALL DETECTION SYSTEM
// Complete code with ML model
// ============================================

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SoftwareSerial.h>

// Include your generated model files
#include "fall_detection_model.h"
#include "scaler_params.h"

// ============================================
// PIN DEFINITIONS
// ============================================
#define I2C_SDA 21
#define I2C_SCL 22
#define BUZZER_PIN 4
#define VIBRATION_PIN 5
#define CANCEL_BUTTON_PIN 15
#define PULSE_SENSOR_PIN 34

// GSM Module pins
#define GSM_RX 16
#define GSM_TX 17

// ============================================
// CONSTANTS
// ============================================
const float GRAVITY = 9.81;
const int FALL_DETECTION_THRESHOLD = 5;     // Number of consecutive fall detections
const int SMS_DELAY_MS = 5000;              // Wait 5 seconds before sending SMS
const int BUTTON_DEBOUNCE_MS = 50;          // Button debounce time

// ============================================
// GLOBAL VARIABLES
// ============================================
Adafruit_MPU6050 mpu;
SoftwareSerial gsm(GSM_RX, GSM_TX);  // RX, TX

bool fallDetected = false;
unsigned long fallDetectedTime = 0;
bool smsSent = false;
int fallVoteCount = 0;
int normalVoteCount = 0;

// Moving average buffers for smoothing
float accelXBuffer[5] = {0};
float accelYBuffer[5] = {0};
float accelZBuffer[5] = {0};
float gyroXBuffer[5] = {0};
float gyroYBuffer[5] = {0};
float gyroZBuffer[5] = {0};
int bufferIndex = 0;

// ============================================
// FUNCTION DECLARATIONS
// ============================================
void setupMPU6050();
void readSensors(float features[]);
void applyMovingAverage(float features[]);
void normalizeFeatures(float raw[], float normalized[]);
int detectFall(float features[]);
void triggerFallAlert();
void sendSMS();
bool cancelAlert();
void buzzAlert();
void setupGSM();

// ============================================
// SETUP FUNCTION
// ============================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n==========================================");
  Serial.println("ESP32 FALL DETECTION SYSTEM");
  Serial.println("==========================================\n");
  
  // Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  
  // Initialize MPU6050
  setupMPU6050();
  
  // Initialize pins
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(VIBRATION_PIN, OUTPUT);
  pinMode(CANCEL_BUTTON_PIN, INPUT_PULLUP);
  
  // Turn off buzzer and vibration initially
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(VIBRATION_PIN, LOW);
  
  // Initialize GSM (optional - comment if not using)
  // setupGSM();
  
  Serial.println("\n✅ System ready! Monitoring for falls...\n");
  delay(1000);
}

// ============================================
// MPU6050 INITIALIZATION
// ============================================
void setupMPU6050() {
  Serial.print("Initializing MPU6050... ");
  
  if (!mpu.begin()) {
    Serial.println("FAILED!");
    Serial.println("Check wiring: SDA->21, SCL->22, VCC->3.3V, GND->GND");
    while (1) {
      delay(100);
      digitalWrite(BUZZER_PIN, HIGH);
      delay(100);
      digitalWrite(BUZZER_PIN, LOW);
      delay(100);
    }
  }
  
  // Configure MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  Serial.println("OK!");
  Serial.println("  Accelerometer Range: ±8G");
  Serial.println("  Gyroscope Range: ±500°/s");
}

// ============================================
// READ SENSORS AND EXTRACT FEATURES
// ============================================
void readSensors(float features[]) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Store raw readings
  features[0] = a.acceleration.x / GRAVITY;  // Accel X (in G)
  features[1] = a.acceleration.y / GRAVITY;  // Accel Y (in G)
  features[2] = a.acceleration.z / GRAVITY;  // Accel Z (in G)
  features[3] = g.gyro.x;                    // Gyro X (deg/s)
  features[4] = g.gyro.y;                    // Gyro Y (deg/s)
  features[5] = g.gyro.z;                    // Gyro Z (deg/s)
  
  // Optional: Read pulse sensor if connected
  // int pulseRaw = analogRead(PULSE_SENSOR_PIN);
}

// ============================================
// APPLY MOVING AVERAGE FILTER
// ============================================
void applyMovingAverage(float features[]) {
  // Store new values in buffer
  accelXBuffer[bufferIndex] = features[0];
  accelYBuffer[bufferIndex] = features[1];
  accelZBuffer[bufferIndex] = features[2];
  gyroXBuffer[bufferIndex] = features[3];
  gyroYBuffer[bufferIndex] = features[4];
  gyroZBuffer[bufferIndex] = features[5];
  
  // Calculate averages
  float sumX = 0, sumY = 0, sumZ = 0;
  float sumGX = 0, sumGY = 0, sumGZ = 0;
  
  for (int i = 0; i < 5; i++) {
    sumX += accelXBuffer[i];
    sumY += accelYBuffer[i];
    sumZ += accelZBuffer[i];
    sumGX += gyroXBuffer[i];
    sumGY += gyroYBuffer[i];
    sumGZ += gyroZBuffer[i];
  }
  
  features[0] = sumX / 5.0;
  features[1] = sumY / 5.0;
  features[2] = sumZ / 5.0;
  features[3] = sumGX / 5.0;
  features[4] = sumGY / 5.0;
  features[5] = sumGZ / 5.0;
  
  bufferIndex = (bufferIndex + 1) % 5;
}

// ============================================
// NORMALIZE FEATURES USING SCALER
// ============================================
void normalizeFeatures(float raw[], float normalized[]) {
  for (int i = 0; i < ScalerParams::NUM_FEATURES; i++) {
    normalized[i] = (raw[i] - ScalerParams::MEAN[i]) / ScalerParams::STD[i];
  }
}

// ============================================
// DETECT FALL USING RANDOM FOREST
// ============================================
int detectFall(float features[]) {
  float normalized[ScalerParams::NUM_FEATURES];
  normalizeFeatures(features, normalized);
  return FallDetectionModel::predict(normalized);
}

// ============================================
// BUZZER ALERT PATTERN
// ============================================
void buzzAlert() {
  // Fast beeping pattern for fall alert
  for (int i = 0; i < 3; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    digitalWrite(VIBRATION_PIN, HIGH);
    delay(200);
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(VIBRATION_PIN, LOW);
    delay(200);
  }
  // Continuous slower beep after initial alert
  for (int i = 0; i < 10; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(500);
    digitalWrite(BUZZER_PIN, LOW);
    delay(500);
  }
}

// ============================================
// CANCEL ALERT (Button Press)
// ============================================
void cancelAlert() {
  if (digitalRead(CANCEL_BUTTON_PIN) == LOW) {
    delay(BUTTON_DEBOUNCE_MS);
    if (digitalRead(CANCEL_BUTTON_PIN) == LOW) {
      fallDetected = false;
      smsSent = false;
      fallVoteCount = 0;
      normalVoteCount = 0;
      digitalWrite(BUZZER_PIN, LOW);
      digitalWrite(VIBRATION_PIN, LOW);
      Serial.println("❌ Alert cancelled by user");
      return true;
    }
  }
  return false;
}

// ============================================
// SETUP GSM MODULE (for SMS)
// ============================================
void setupGSM() {
  Serial.print("Initializing GSM module... ");
  gsm.begin(9600);
  delay(2000);
  
  gsm.println("AT");
  delay(500);
  gsm.println("AT+CMGF=1");  // Text mode
  delay(500);
  gsm.println("AT+CNMI=2,2,0,0,0");
  delay(500);
  Serial.println("OK");
}

// ============================================
// TRIGGER FALL ALERT
// ============================================
void triggerFallAlert() {
  Serial.println("⚠️ Fall alert triggered");
  buzzAlert();
  digitalWrite(VIBRATION_PIN, HIGH);
}

// ============================================
// SEND SMS ALERT
// ============================================
void sendSMS() {
  Serial.println("📩 Sending SMS alert...");
  gsm.println("AT+CMGF=1");
  delay(500);
  gsm.println("AT+CMGS=\"+1234567890\"");
  delay(500);
  gsm.print("Fall detected! Please check immediately.");
  gsm.write(0x1A); // Ctrl+Z to send SMS
  delay(1000);
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
  float features[6];
  bool cancelled = cancelAlert();

  readSensors(features);
  applyMovingAverage(features);

  int prediction = detectFall(features);
  if (prediction == 1) {
    fallVoteCount++;
    normalVoteCount = 0;
  } else {
    normalVoteCount++;
    if (normalVoteCount > 3) {
      fallVoteCount = 0;
    }
  }

  if (!fallDetected && fallVoteCount >= FALL_DETECTION_THRESHOLD) {
    fallDetected = true;
    fallDetectedTime = millis();
    Serial.println("⚠️ Fall detected!");
    triggerFallAlert();
  }

  if (fallDetected && !smsSent && !cancelled) {
    if (millis() - fallDetectedTime >= SMS_DELAY_MS) {
      sendSMS();
      smsSent = true;
    }
  }

  delay(100);
}

// ============================================
// SEND SMS ALERT
// ============================================
void sendSMS() {
  Serial.println("📱 Sending SMS alert...");
  
  // Replace with your phone number (international format)
  String phoneNumber = "+8801XXXXXXXXX";
  
  gsm.println("AT+CMGS=\"" + phoneNumber + "\"");
  delay(500);
  gsm.print("FALL DETECTED! Please check immediately.");
  delay(500);
  gsm.write(26);  // CTRL+Z to send
  delay(3000);
  
  Serial.println("✅ SMS sent!");
}

// ============================================
// TRIGGER FALL ALERT SEQUENCE
// ============================================
void triggerFallAlert() {
  Serial.println("🚨 FALL DETECTED! 🚨");
  buzzAlert();
  
  // Check if user cancels within 5 seconds
  unsigned long startTime = millis();
  while (millis() - startTime < SMS_DELAY_MS) {
    if (cancelAlert()) {
      return;
    }
    delay(50);
  }
  
  // Send SMS if not cancelled
  if (!smsSent) {
    sendSMS();
    smsSent = true;
  }
  
  // Keep alerting until reset
  while (true) {
    if (cancelAlert()) {
      break;
    }
    digitalWrite(BUZZER_PIN, HIGH);
    delay(1000);
    digitalWrite(BUZZER_PIN, LOW);
    delay(1000);
  }
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
  // Read sensor data
  float features[ScalerParams::NUM_FEATURES];
  readSensors(features);
  
  // Apply smoothing filter
  applyMovingAverage(features);
  
  // Detect fall using ML model
  int result = detectFall(features);
  
  // Voting mechanism (consecutive detections)
  if (result == 1) {
    fallVoteCount++;
    normalVoteCount = 0;
  } else {
    normalVoteCount++;
    fallVoteCount = 0;
  }
  
  // Print debug information
  Serial.print("Accel: ");
  Serial.print(features[0], 2); Serial.print(", ");
  Serial.print(features[1], 2); Serial.print(", ");
  Serial.print(features[2], 2);
  Serial.print(" | Gyro: ");
  Serial.print(features[3], 1); Serial.print(", ");
  Serial.print(features[4], 1); Serial.print(", ");
  Serial.print(features[5], 1);
  Serial.print(" | Prediction: ");
  Serial.print(result == 1 ? "FALL" : "NORMAL");
  Serial.print(" | Votes: ");
  Serial.print(fallVoteCount);
  Serial.print("/");
  Serial.println(FALL_DETECTION_THRESHOLD);
  
  // Trigger fall alert if threshold reached
  if (fallVoteCount >= FALL_DETECTION_THRESHOLD && !fallDetected) {
    fallDetected = true;
    triggerFallAlert();
  }
  
  // Reset detection after normal activity
  if (normalVoteCount >= 10 && fallDetected) {
    fallDetected = false;
    smsSent = false;
    Serial.println("✅ System reset - back to normal monitoring");
  }
  
  delay(50);  // 20Hz sampling rate (50ms delay)
}
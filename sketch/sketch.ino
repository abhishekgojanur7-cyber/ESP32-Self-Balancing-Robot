#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SSD1306.h>

// --- OLED Configuration ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// --- IMU Configuration ---
Adafruit_MPU6050 mpu;

// --- Motor Pins Configuration ---
const int LEFT_STEP = 12;
const int LEFT_DIR  = 14;
const int RIGHT_STEP = 27;
const int RIGHT_DIR  = 26;

// --- Control Variables ---
float angleVal = 0.0;ESP32-Self-Balancing-Robot
float targetAngle = 0.0; // The perfectly upright balance point
unsigned long lastTime;

// PID constants (These will need real-world tuning!)
float Kp = 40.0;
float Ki = 0.0;
float Kd = 1.0;

float integral = 0.0;
float lastError = 0.0;

void setup() {
  Serial.begin(115200);

  // Initialize Motor Pins
  pinMode(LEFT_STEP, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_STEP, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);

  // Initialize I2C and Display
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED allocation failed"));
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Initialize MPU6050 Sensor
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  lastTime = millis();
}

void loop() {
  // 1. Read Sensor Data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate elapsed time
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  // 2. Sensor Fusion (Complementary Filter to calculate Pitch Angle)
  float accAngle = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  float gyroRate = g.gyro.x * 180.0 / PI;
  angleVal = 0.98 * (angleVal + gyroRate * dt) + 0.02 * accAngle;

  // 3. Compute PID Controller Output
  float error = targetAngle - angleVal;
  integral += error * dt;
  float derivative = (error - lastError) / dt;
  float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  lastError = error;

  // 4. Update OLED Display with state
  display.clearDisplay();
  display.setCursor(0,0);
  display.print("Angle: "); display.println(angleVal);
  display.print("Output: "); display.println(output);
  display.display();

  // 5. Direct Motor Speeds based on PID Output
  if (abs(error) < 0.5) {
    // If perfectly balanced, don't step
    return;
  }

  // Set direction based on lean
  if (output > 0) {
    digitalWrite(LEFT_DIR, HIGH);
    digitalWrite(RIGHT_DIR, LOW);
  } else {
    digitalWrite(LEFT_DIR, LOW);
    digitalWrite(RIGHT_DIR, HIGH);
  }

  // Generate step pulse (Speed is inversely proportional to PID output magnitude)
  int pulseDelay = map(constrain(abs(output), 0, 400), 0, 400, 4000, 500);
  
  digitalWrite(LEFT_STEP, HIGH);
  digitalWrite(RIGHT_STEP, HIGH);
  delayMicroseconds(5);
  digitalWrite(LEFT_STEP, LOW);
  digitalWrite(RIGHT_STEP, LOW);
  
  delayMicroseconds(pulseDelay);
}


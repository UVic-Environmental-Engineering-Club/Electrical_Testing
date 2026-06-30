#include <Arduino.h>
#include <TMCStepper.h>

// Pin mapping for a basic TMC2209 UART test.
// Adjust these to match your breakout and ESP32-C3 wiring.
#define EN_PIN 0
#define STEP_PIN 1
#define DIR_PIN 2
#define RX_PIN 18
#define TX_PIN 19
#define DRIVER_ADDRESS 0b00
#define R_SENSE 0.11f

HardwareSerial TMCSerial(1);
TMC2209Stepper driver(&TMCSerial, R_SENSE, DRIVER_ADDRESS);

// Move a number of steps at a given speed (steps per second).
void moveSteps(int steps, bool dir, uint32_t stepsPerSec) {
  digitalWrite(DIR_PIN, dir ? HIGH : LOW);
  delay(2);

  if (stepsPerSec == 0) return;
  uint32_t intervalUs = 1000000UL / stepsPerSec; // time between step starts
  uint32_t pulseUs = intervalUs / 4; // pulse width (25% of interval)
  if (pulseUs < 2) pulseUs = 2;

  for (int i = 0; i < steps; ++i) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(pulseUs);
    digitalWrite(STEP_PIN, LOW);
    uint32_t rest = intervalUs > pulseUs ? intervalUs - pulseUs : 0;
    delayMicroseconds(rest);
  }
}

void setup() {
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(EN_PIN, LOW);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);

  Serial.begin(115200);
  delay(1000);
  Serial.println("TMC2209 UART test starting...");

  TMCSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(100);

  driver.begin();
  driver.pdn_disable(true);
  driver.I_scale_analog(false);
  driver.rms_current(600);
  driver.microsteps(16);
  driver.toff(5);
  driver.en_spreadCycle(false);

  Serial.print("Connection test: ");
  if (driver.test_connection()) {
    Serial.println("OK");
  } else {
    Serial.println("FAILED");
  }

  
}

void loop() {
  Serial.println("Driver configured. Moving a few steps at variable speeds...");
  // Change these values to adjust speed:
  // stepsPerSec = RPM * stepsPerRev / 60
  moveSteps(6000, true, 4000);  // 400 steps/sec
  delay(500);
  moveSteps(6000, false, 4000); // 200 steps/sec
  delay(500);
}

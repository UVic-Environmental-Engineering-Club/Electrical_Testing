#include <Arduino.h>

// STEP/DIR pins for the TMC2209 breakout.
// Adjust these to match your wiring.
const int STEP_PIN = 1;
const int DIR_PIN = 2;
const int ENABLE_PIN = 0; // usually active LOW for TMC2209

const int STEPS_PER_BURST = 5000;
const unsigned int STEP_PULSE_US = 200; // pulse width and delay in microseconds
const unsigned long PAUSE_MS = 1000;

void stepBurst(int steps, bool direction) {
  digitalWrite(DIR_PIN, direction ? HIGH : LOW);

  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_PULSE_US);
  }
}

void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(ENABLE_PIN, LOW); // pull low to enable driver
}

void loop() {
  stepBurst(STEPS_PER_BURST, true);
  delay(PAUSE_MS);

  stepBurst(STEPS_PER_BURST, false);
  delay(PAUSE_MS);
}

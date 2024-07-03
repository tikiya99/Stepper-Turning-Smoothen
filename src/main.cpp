#include <Arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

#define PPM_PIN 15 

const int stepsPerRevolution = 32000;  
const int stepsPerDegree = stepsPerRevolution / 360;  

const int LstepPin = 22;
const int LdirPin = 18;
const int LenablePin = 25;

const int RstepPin = 23;
const int RdirPin = 19;
const int RenablePin = 26;

volatile uint32_t lastTime = 0;
volatile uint32_t currentTime = 0;
volatile uint16_t ppmValues[8]; 
volatile uint8_t channel = 0;

AccelStepper stepperLeft(AccelStepper::DRIVER, LstepPin, LdirPin);
AccelStepper stepperRight(AccelStepper::DRIVER, RstepPin, RdirPin);

MultiStepper steppers;

int calculateAngle(uint16_t ppmValue) {
  if (ppmValue >= 1550 && ppmValue <= 2000) {
    return map(ppmValue, 1550, 2000, -40, 0); 
  } else if (ppmValue >= 1000 && ppmValue <= 1450) {
    return map(ppmValue, 1000, 1450, 0, 40); 
  } else if (ppmValue > 1450 && ppmValue < 1550) {
    return 0;
  } else {
    return 0;
  }
}

void IRAM_ATTR handlePPMInterrupt() {
  currentTime = micros(); 
  uint32_t pulseLength = currentTime - lastTime;
  lastTime = currentTime;

  if (pulseLength > 3000) {
    channel = 0;
  } else {
    if (channel < 8) {
      ppmValues[channel] = pulseLength;
      channel++;
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), handlePPMInterrupt, FALLING);

  pinMode(LenablePin, OUTPUT);
  pinMode(RenablePin, OUTPUT);

  digitalWrite(LenablePin, LOW);
  digitalWrite(RenablePin, LOW);

  stepperLeft.setMaxSpeed(1500);
  stepperLeft.setAcceleration(500);

  stepperRight.setMaxSpeed(1500);
  stepperRight.setAcceleration(500);

  steppers.addStepper(stepperLeft);
  steppers.addStepper(stepperRight);
}

void moveStepperMotor(int angle) {
  int stepsToMove = angle * stepsPerDegree;
  long positions[2];

  positions[0] = stepsToMove;
  positions[1] = stepsToMove;

  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); 
}

void loop() {
  uint16_t ppmValue = ppmValues[0];

  int angle = calculateAngle(ppmValue);

  Serial.print("PPM Value: ");
  Serial.print(ppmValue);
  Serial.print(", Calculated Angle: ");
  Serial.println(angle);

  moveStepperMotor(angle);

  delay(10); 
}
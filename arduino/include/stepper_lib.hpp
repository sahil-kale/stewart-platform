#ifndef STEPPER_LIB_HPP
#define STEPPER_LIB_HPP

#include <stdint.h>
#include <Arduino.h>

#define MAX_NUM_STEPPERS 3U
#define STEP_DELAY_US 5U

constexpr float HOMING_TIME_DELAY = 0.01;
constexpr float NORMAL_OPERATIONS_TIME_DELAY = 0.0006;

class MultiStepper {
public:
  class IndividualStepper {
  public:
    IndividualStepper()
        : stepPin(0), dirPin(0), limitSwitchPin(0) {}
    IndividualStepper(uint8_t stepPin, uint8_t dirPin, uint8_t limitSwitchPin, bool ccw_for_homing=true) {
      this->stepPin = stepPin;
      this->dirPin = dirPin;
      this->targetStepCount = 0;
      this->currentStepCount = 0;
      this->limitSwitchPin = limitSwitchPin;
      this->homed = false;
      this->ccw_for_homing = ccw_for_homing;
    }

    void init() {
      pinMode(stepPin, OUTPUT);
      pinMode(dirPin, OUTPUT);
      pinMode(limitSwitchPin, INPUT_PULLUP);
    }

    void updateTargetStepCount(int newTargetStepCount) {
      if (this->homed) {
        this->targetStepCount = newTargetStepCount;
      }
    }

    int targetStepCount;
    int currentStepCount;
    uint32_t insideHomingCount = 0;

    uint8_t stepPin;
    uint8_t dirPin;
    uint8_t limitSwitchPin;
    bool homed = false;
    bool ccw_for_homing;

    float time_since_last_step = 0.0;
  };

  IndividualStepper steppers[MAX_NUM_STEPPERS];

  MultiStepper() {}

  void addStepper(IndividualStepper &stepper) {
    if (num_steppers < MAX_NUM_STEPPERS) {
      steppers[num_steppers] = stepper;
    }
    num_steppers++;
  }

  void home() {
    for (uint8_t i = 0; i < num_steppers; i++) {
      steppers[i].targetStepCount = -9999;
    }
  }

  void step(float dt) {
    for (uint8_t i = 0; i < num_steppers; i++) {
      IndividualStepper &stepper = steppers[i];
      if (stepper.targetStepCount != stepper.currentStepCount) {
        float stepCountMultiplier = 1.0;

        // HIGH is CW, LOW is CCW - so setting the directions as such
        if (stepper.targetStepCount > stepper.currentStepCount) {
          digitalWrite(stepper.dirPin, stepper.ccw_for_homing);
          stepCountMultiplier = 1.0;
        } else {
          digitalWrite(stepper.dirPin, !stepper.ccw_for_homing);
          stepCountMultiplier = -1.0;
        }

        if (stepper.homed) {
          stepper.time_since_last_step += dt;
          if (stepper.time_since_last_step >= NORMAL_OPERATIONS_TIME_DELAY) {
            stepper.time_since_last_step = 0.0;
            stepper.currentStepCount += stepCountMultiplier;
            digitalWrite(stepper.stepPin, HIGH);
            delayMicroseconds(STEP_DELAY_US);
            digitalWrite(stepper.stepPin, LOW);
          }

        } else {
          if (digitalRead(stepper.limitSwitchPin) == LOW) {
            stepper.homed = true;
            stepper.currentStepCount = 0;
            if (stepper.targetStepCount < 0) {
              stepper.targetStepCount = 0;
            }
          } else {
            stepper.time_since_last_step += dt;
            if (stepper.time_since_last_step >= HOMING_TIME_DELAY) {
              stepper.time_since_last_step = 0.0;
              stepper.currentStepCount += stepCountMultiplier;
              digitalWrite(stepper.stepPin, HIGH);
              delayMicroseconds(STEP_DELAY_US);
              digitalWrite(stepper.stepPin, LOW);
            }
          }
        }
      }
    }
  }

  uint8_t num_steppers = 0;
};

#endif
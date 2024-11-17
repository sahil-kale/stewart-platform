#ifndef STEPPER_LIB_HPP
#define STEPPER_LIB_HPP

#include <stdint.h>
#include <Arduino.h>

#define NUM_STEPPERS 3U
#define STEP_DELAY_US 5U

class MultiStepper
{
    public:
    class IndividualStepper {
        public:
        IndividualStepper() : stepPin(0), dirPin(0), limitSwitchPin(0), steps_per_second(0) {}
        IndividualStepper(uint8_t stepPin, uint8_t dirPin, uint8_t limitSwitchPin, float stepsPerSecond)
        {
            this->stepPin = stepPin;
            this->dirPin = dirPin;
            this->targetStepCount = 0;
            this->currentStepCount = 0;
            this->limitSwitchPin = limitSwitchPin;
            this->homed = true;
            this->steps_per_second = stepsPerSecond;
        }

        void init()
        {
            pinMode(stepPin, OUTPUT);
            pinMode(dirPin, OUTPUT);
            pinMode(limitSwitchPin, INPUT_PULLUP);
        }

        void updateTargetStepCount(int newTargetStepCount) {
          this->targetStepCount = newTargetStepCount;
        }

        int targetStepCount;
        int currentStepCount;

        uint8_t stepPin;
        uint8_t dirPin;
        uint8_t limitSwitchPin;
        bool homed = false;
        float steps_per_second;

        float time_since_last_step = 0.0;
    };

    IndividualStepper steppers[NUM_STEPPERS];
    
    MultiStepper(IndividualStepper steppers[NUM_STEPPERS])
    {
        for (uint8_t i = 0; i < NUM_STEPPERS; i++)
        {
            this->steppers[i] = steppers[i];
            this->steppers[i].init();
        }
    }

    MultiStepper() {}

    void addStepper(IndividualStepper& stepper) {
        if (num_steppers < NUM_STEPPERS) {
            steppers[num_steppers] = stepper;
        }
        num_steppers++;
    }

    void home()
    {
        for (uint8_t i = 0; i < NUM_STEPPERS; i++)
        {
            steppers[i].targetStepCount = -9999;
        }
    }

    void step(float dt)
    {
        IndividualStepper &stepper = steppers[0];
        // if (stepper.targetStepCount != stepper.currentStepCount) {
        //   digitalWrite(steppers[0].stepPin, HIGH);
        //   delayMicroseconds(10);
        //   digitalWrite(steppers[0].stepPin, LOW);
        //   delayMicroseconds(600);
        // }
        for (uint8_t i = 0; i < 1; i++) {
          IndividualStepper& stepper = steppers[i];
          if (stepper.targetStepCount != stepper.currentStepCount)
          {
              float stepCountMultiplier = 1.0;
              if (stepper.targetStepCount > stepper.currentStepCount)
              {
                  digitalWrite(stepper.dirPin, HIGH);
                  stepCountMultiplier = 1.0;
              }
              else
              {
                  digitalWrite(stepper.dirPin, LOW);
                  stepCountMultiplier = -1.0;
              }

              if (stepper.homed)
              {
                  stepper.time_since_last_step += dt;
                  if (stepper.time_since_last_step >= 0.0006)
                  {
                      stepper.time_since_last_step = 0.0;
                      stepper.currentStepCount += stepCountMultiplier;
                      digitalWrite(stepper.stepPin, HIGH);
                      delayMicroseconds(STEP_DELAY_US);
                      digitalWrite(stepper.stepPin, LOW);
                  }
                  if (digitalRead(stepper.limitSwitchPin) == LOW) {
                    stepper.homed = true;
                    stepper.currentStepCount = 0;
                    if (stepper.targetStepCount < 0) {
                        stepper.targetStepCount = 0;
                    }
                  }
              }
              else
              {
                  if (digitalRead(stepper.limitSwitchPin) == LOW)
                  {
                      stepper.homed = true;
                      stepper.currentStepCount = 0;
                      if (stepper.targetStepCount < 0) {
                        stepper.targetStepCount = 0;
                      }
                  }
                  else
                  {
                    if (stepper.time_since_last_step >= 0.0006) {
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
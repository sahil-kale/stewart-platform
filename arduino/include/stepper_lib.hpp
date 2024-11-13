#ifndef STEPPER_LIB_HPP
#define STEPPER_LIB_HPP

#include <stdint.h>
#include <Arduino.h>

#define NUM_STEPPERS 3U
#define STEP_DELAY_US 1U

class MultiStepper
{
    public:
    class IndividualStepper {
        public:
        IndividualStepper() : stepPin(0), dirPin(0), limitSwitchPin(0), steps_per_second(0) {}
        IndividualStepper(uint8_t stepPin, uint8_t dirPin, uint8_t limitSwitchPin, uint16_t stepsPerRevolution, float rpm)
        {
            this->stepPin = stepPin;
            this->dirPin = dirPin;
            this->targetStepCount = 0;
            this->currentStepCount = 0;
            this->limitSwitchPin = limitSwitchPin;
            this->homed = false;
            this->steps_per_second = (rpm / 60.0) * stepsPerRevolution;
        }

        void init()
        {
            pinMode(stepPin, OUTPUT);
            pinMode(dirPin, OUTPUT);
            pinMode(limitSwitchPin, INPUT);
        }

        void updateTargetStepCount(uint32_t newTargetStepCount) {
          this->targetStepCount = newTargetStepCount;
        }

        uint32_t targetStepCount;
        uint32_t currentStepCount;

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
            steppers[i].targetStepCount = -99999;
        }
    }

    void step(float dt)
    {
        for (uint8_t i = 0; i < NUM_STEPPERS; i++)
        {
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
                    if (stepper.time_since_last_step >= 1.0 / stepper.steps_per_second)
                    {
                        stepper.time_since_last_step = 0.0;
                        stepper.currentStepCount += stepCountMultiplier;
                        digitalWrite(stepper.stepPin, HIGH);
                        delayMicroseconds(STEP_DELAY_US);
                        digitalWrite(stepper.stepPin, LOW);
                    }
                }
                else
                {
                    if (digitalRead(stepper.limitSwitchPin) == HIGH)
                    {
                        stepper.homed = true;
                        stepper.currentStepCount = 0;
                    }
                    else
                    {
                        stepper.currentStepCount += stepCountMultiplier;
                        digitalWrite(stepper.stepPin, HIGH);
                        delayMicroseconds(STEP_DELAY_US);
                        digitalWrite(stepper.stepPin, LOW);
                    }
                }
            }
        }
    }

    uint8_t num_steppers = 0;


};


#endif
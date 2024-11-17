#include <Arduino.h>
#include <Servo.h>
#include "serial_decryption.hpp"
#include "stepper_lib.hpp"

//#define DEBUG

// These values are all placeholders for now
uint8_t stepPins[] = {3, 10, 11};
uint8_t dirPins[] = {4, 2, 1};
uint8_t limitSwitchPin[] = {7, 5, 6};

constexpr float US_PER_SECOND = 1000000.0;
constexpr uint32_t stepsPerSecond = (20000/4.0) / 0.03;
constexpr float rpm = 1 / (0.03 * 4);

MultiStepper::IndividualStepper steppers[NUM_STEPPERS];
MultiStepper multiStepper;

uint32_t time_of_last_loop_micros = 0;

void setup() {
    // Initialize the serial communication
    Serial.begin(115200);

    // pinMode(3, OUTPUT);
    // pinMode(4, OUTPUT);

    // Instantiate stepper objects at once
    for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
        steppers[i] = MultiStepper::IndividualStepper(stepPins[i], dirPins[i], limitSwitchPin[i], stepsPerSecond);
        multiStepper.addStepper(steppers[i]);
    } 

    multiStepper.home();
}

void loop() {
    // If there is data available to read, read it and decrypt it.
    PlatformControlRequest request;
    request.valid = false;
    
    if (Serial.available() >= NUM_BYTES_PER_REQUEST) {
        char serialData[NUM_BYTES_PER_REQUEST];
        Serial.readBytes(serialData, NUM_BYTES_PER_REQUEST);
        request = decryptSerialData(serialData, sizeof(serialData));

        // clear the serial buffer
        while (Serial.available() > 0) {
            Serial.read();
        }
        
#ifdef DEBUG
        if (request.valid) {
            Serial.print("Steps: ");
            for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
                Serial.print(request.steps[i]);
                Serial.print(" ");
            }
            Serial.println();
        } else {
            Serial.println("Invalid data received.");
        }
#endif
    }

    // If the data is valid, update the servo positions.
    if (request.valid) {
        for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
            // multiStepper.steppers[i].updateTargetStepCount(request.steps[i]);
            multiStepper.steppers[i].updateTargetStepCount(99999);
        }
    }
    
    uint32_t current_time = micros();
    float dt = (current_time - time_of_last_loop_micros);
    time_of_last_loop_micros = current_time;
    // Run the step function
    dt = dt / US_PER_SECOND;
    multiStepper.step(dt);
    Serial.println(digitalRead(multiStepper.steppers[0].limitSwitchPin));
    // Serial.println(multiStepper.steppers[0].targetStepCount);
    // Serial.println(multiStepper.steppers[0].currentStepCount);
    // digitalWrite(steppers[0].stepPin, HIGH);
    // delayMicroseconds(10);
    // digitalWrite(steppers[0].stepPin, LOW);
    // delayMicroseconds(600);

    // digitalWrite(3, HIGH);
    // digitalWrite(3, LOW);
    // delayMicroseconds(600);
}

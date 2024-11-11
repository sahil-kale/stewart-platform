#include <Arduino.h>
#include <Servo.h>
#include "serial_decryption.hpp"
#include "stepper_lib.hpp"

//#define DEBUG

// These values are all placeholders for now
uint8_t stepPins[] = {9, 10, 11};
uint8_t dirPins[] = {1, 2, 3};
uint8_t limitSwitchPin[] = {4, 5, 6};

constexpr stepsPerRevolution = 20000; // Arbitrary rn

IndividualStepper steppers[NUM_STEPPERS];
MultiStepper multiStepper;

void setup() {
    // Initialize the serial communication
    Serial.begin(9600);

    // Instantiate stepper objects at once
    for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
        steppers[i] = IndividualStepper(stepPins[i], dirPins[i], stepsPerRevolution);
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
            Serial.print("Duty cycles: ");
            for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
                Serial.print(request.dutyCyclesUs[i]);
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
            if (request.dutyCyclesUs[i] > 0) {
                multiStepper.step()
                servos[i].writeMicroseconds(request.dutyCyclesUs[i]);  // Update servo positions without detaching
            }
        }
    }
}

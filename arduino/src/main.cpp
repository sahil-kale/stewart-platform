#include <Arduino.h>
#include <Servo.h>
#include "serial_decryption.hpp"
#include "stepper_lib.hpp"

//#define DEBUG

// These values are all placeholders for now
uint8_t stepPins[] = {2, 3, 4};
uint8_t dirPins[] = {5, 6, 7};
uint8_t limitSwitchPin[] = {9, 10, 11};
// All the motors are currently set as CW to home (0 = CW to home, 1 = CCW to home)
bool homingOrientation[] = {0, 0, 0};

constexpr float US_PER_SECOND = 1000000.0;

constexpr uint8_t enable_pin = 8;

MultiStepper multiStepper;

uint32_t time_of_last_loop_micros = 0;

bool serial_received = false;

void setup() {
    // Initialize the serial communication
    Serial.begin(921600);

    // Initialize the enable pin
    pinMode(enable_pin, OUTPUT);
    digitalWrite(enable_pin, LOW);
    
    // Instantiate stepper objects at once - all our motors are oriented as CW to home
    for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
      MultiStepper::IndividualStepper stepper = MultiStepper::IndividualStepper(
        stepPins[i], 
        dirPins[i], 
        limitSwitchPin[i], 
        homingOrientation[i]
      );

        stepper.init();

      multiStepper.addStepper(stepper);
    } 

#ifdef TRAP_STEPPER_SPIN
    while (true)
    {

         for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
            digitalWrite(stepPins[i], HIGH);
            delayMicroseconds(10);
            digitalWrite(stepPins[i], LOW);
            delayMicroseconds(500);
         }
    }
#endif

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
            serial_received = true;
            for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
                Serial.print(request.steps[i]);
                Serial.print(" ");
            }
            Serial.println();
        } else {
            Serial.println("Invalid data received.");
        }
#endif

        if (request.valid) {
            serial_received = true;
            for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
                multiStepper.steppers[i].updateTargetStepCount(request.steps[i]);
            }
        }
    }

    uint32_t current_time = micros();
    float dt = (current_time - time_of_last_loop_micros);
    time_of_last_loop_micros = current_time;
    // Run the step function
    dt = dt / US_PER_SECOND;
    multiStepper.step(dt);
}

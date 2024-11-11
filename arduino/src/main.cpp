#include <Arduino.h>
#include <Servo.h>
#include "serial_decryption.hpp"
#include "stepper_lib.hpp"

//#define DEBUG

uint8_t servoPins[] = {9, 10, 11};
Servo servos[NUM_SERVOS];  // Array to hold servo objects

void setup() {
    // Initialize the serial communication
    Serial.begin(9600);

    // Attach servos to the appropriate pins once during setup
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        servos[i].attach(servoPins[i]);
        servos[i].writeMicroseconds(1400); 
    }
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
            for (uint8_t i = 0; i < NUM_SERVOS; i++) {
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
        for (uint8_t i = 0; i < NUM_SERVOS; i++) {
            if (request.dutyCyclesUs[i] > 0) {
                servos[i].writeMicroseconds(request.dutyCyclesUs[i]);  // Update servo positions without detaching
            }
        }
    }
}

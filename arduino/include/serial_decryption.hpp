#ifndef SERIAL_DECRYPTION_HPP
#define SERIAL_DECRYPTION_HPP

#include <stdint.h>
#include <stddef.h>

#define NUM_SERVOS 3U

typedef struct {
    uint16_t dutyCyclesUs[NUM_SERVOS];
    bool valid;
} PlatformControlRequest;

PlatformControlRequest decryptSerialData(const char* serialData, size_t serialDataLength);


#endif // SERIAL_DECRYPTION_HPP
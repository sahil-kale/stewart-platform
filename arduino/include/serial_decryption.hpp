#ifndef SERIAL_DECRYPTION_HPP
#define SERIAL_DECRYPTION_HPP

#include <stdint.h>
#include <stddef.h>

#define NUM_STEPPERS 3U
#define NUM_BYTES_PER_SERVO 2U
#define NUM_BYTES_CHECKSUM 4U
#define NUM_BYTES_PER_REQUEST (NUM_SERVOS * NUM_BYTES_PER_SERVO + NUM_BYTES_CHECKSUM)

typedef struct {
    uint16_t steps[NUM_STEPPERS];
    bool valid;
} PlatformControlRequest;

PlatformControlRequest decryptSerialData(const char* serialData, size_t serialDataLength);


#endif // SERIAL_DECRYPTION_HPP
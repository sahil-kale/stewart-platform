#include "serial_decryption.hpp"

PlatformControlRequest decryptSerialData(const char* serialData, size_t serialDataLength)
{
    PlatformControlRequest request;
    request.valid = true;

    // Check if serial data length is valid or if the serialData pointer is null
    if (serialData == nullptr || serialDataLength != 6)
    {
        request.valid = false;
        return request;  // Early exit if invalid
    }

    // If valid, process the data
    for (int i = 0; i < NUM_SERVOS; i++)
    {
        // Format expected is little endian uint16_t (2 bytes per duty cycle)
        request.dutyCyclesUs[i] = (uint16_t)(unsigned char)serialData[i * 2] | ((uint16_t)(unsigned char)serialData[i * 2 + 1] << 8);
    }

    return request;
}

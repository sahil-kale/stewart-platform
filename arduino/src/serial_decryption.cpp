#include "serial_decryption.hpp"

PlatformControlRequest decryptSerialData(const char* serialData, size_t serialDataLength)
{
    PlatformControlRequest request;
    request.valid = true;

    // Check if serial data length is valid or if the serialData pointer is null
    if (serialData == nullptr || serialDataLength != NUM_BYTES_PER_REQUEST)
    {
        request.valid = false;
        return request;  // Early exit if invalid
    }

    uint32_t checksum = 0;

    // If valid, process the data
    for (size_t i = 0; i < NUM_STEPPERS; i++)
    {
        // Format expected is little endian uint16_t (2 bytes per duty cycle)
        request.dutyCyclesUs[i] = (uint16_t)(unsigned char)serialData[i * 2] | ((uint16_t)(unsigned char)serialData[i * 2 + 1] << 8);
        checksum += request.dutyCyclesUs[i];
    }

    // Extract the checksum - little endian uint32_t (4 bytes)
    uint32_t receivedChecksum = (uint32_t)(unsigned char)serialData[NUM_BYTES_PER_REQUEST - 4] | ((uint32_t)(unsigned char)serialData[NUM_BYTES_PER_REQUEST - 3] << 8) | ((uint32_t)(unsigned char)serialData[NUM_BYTES_PER_REQUEST - 2] << 16) | ((uint32_t)(unsigned char)serialData[NUM_BYTES_PER_REQUEST - 1] << 24);

    // Compare the calculated checksum with the received checksum
    if (checksum != receivedChecksum)
    {
        request.valid = false;
    }

    return request;
}

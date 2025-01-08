#include <Arduino.h>
#include <HardwareSerial.h>

#ifndef RS485_TX_PIN
#define RS485_TX_PIN 6 // TX Pin (DI - Data Out)
#endif

#ifndef RS485_RX_PIN
#define RS485_RX_PIN 7 // RX Pin (RO - Data In)
#endif

#ifndef RS485_DE_PIN
#define RS485_DE_PIN 5 // DE Pin (Driver Enable)
#endif

#ifndef RS485_RE_PIN
#define RS485_RE_PIN 4 // RE Pin (Receiver Enable)
#endif

#ifndef RESPONSE_TIMEOUT
#define RESPONSE_TIMEOUT 1000 // Timeout for response (in milliseconds)
#endif

#ifndef DEBUG
#define DEBUG true // Enable debug output
#endif

uint8_t request[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x60, 0x45, 0xE2};

/**
 * @brief Initializes the HY-WDC6E anemometer module.
 *
 * This function sets up the necessary pins and serial communication for the
 * HY-WDC6E anemometer module. It configures the RS485 DE and RE pins as outputs
 * and sets them to LOW to start in receive mode. It also initializes the
 * Serial1 interface with a baud rate of 9600, 8 data bits, no parity, and 1 stop bit.
 *
 * @note We use the Serial to print in DEBUG mode. Ensure that the RS485_DE_PIN, RS485_RE_PIN, RS485_RX_PIN, and RS485_TX_PIN
 *       are defined and connected correctly before calling this function.
 *
 */
void init_hywdc6e();

/**
 * @brief Checks if the device is connected.
 * 
 * This function returns a boolean value indicating whether the device
 * is currently connected.
 * 
 * @return true if the device is connected, false otherwise.
 */
bool is_connected();

/**
 * @brief Sends a Modbus request over RS485.
 *
 * This function sets the RS485 transceiver to transmit mode, sends the Modbus request,
 * waits for a short delay to ensure the data is sent, and then switches the transceiver
 * back to receive mode.
 *
 * @note The function assumes that the request data is stored in a global or previously defined
 *       variable named `request` and that the RS485_DE_PIN and RS485_RE_PIN are defined and
 *       configured correctly.
 */
void sendModbusRequest();

/**
 * @brief Prints the given data in hexadecimal format to the serial output if debugging is enabled.
 *
 * This function iterates over the provided data array and prints each byte in hexadecimal format
 * followed by a space. After printing all bytes, it prints a newline character.
 *
 * @param data Pointer to the array of bytes to be printed.
 * @param length The number of bytes in the data array.
 */
void debugPrint(const uint8_t *data, size_t length);

/**
 * @brief Reads raw data from Serial1 into a buffer until a timeout or buffer is full.
 *
 * This function attempts to read data from the Serial1 interface into the provided buffer.
 * It continues reading until either the specified timeout period has elapsed or the buffer
 * is full. The number of bytes read is stored in the variable pointed to by bytesRead.
 *
 * @param buffer Pointer to the buffer where the read data will be stored.
 * @param bufferSize Size of the buffer.
 * @param bytesRead Pointer to a variable where the number of bytes read will be stored.
 */
void get_raw_response(uint8_t *buffer, size_t bufferSize, size_t *bytesRead);

/**
 * @brief Converts a 4-byte array in little-endian format to a float.
 *
 * This function takes a pointer to an array of 4 bytes, interprets it as a
 * little-endian representation of a 32-bit floating-point number, and converts
 * it to a float. The function also prints the intermediate byte values in
 * hexadecimal format for debugging purposes.
 *
 * @param data Pointer to an array of 4 bytes representing the little-endian
 *             encoded floating-point number.
 * @return The floating-point number represented by the input byte array.
 */
float convertToFloatLittleEndian(unsigned char *data);

/**
 * @brief Converts a 2-byte array in little-endian format to an unsigned short.
 *
 * This function takes a pointer to a 2-byte array where the bytes are in little-endian order
 * and combines them into a single unsigned short value.
 *
 * @param data Pointer to a 2-byte array containing the data in little-endian format.
 * @return The combined unsigned short value.
 */
unsigned short convertToShortLittleEndian(const uint8_t *data);

// GET PARAMETER

/**
 * @brief Retrieves the wind direction from the given response.
 *
 * This function extracts the wind direction data from the specified response array,
 * converts it from little-endian format, and prints the raw and converted wind direction
 * values to the serial output.
 *
 * @param response A pointer to an array of uint8_t containing the response data.
 *                 The wind direction is expected to be located at indices 5 and 6.
 * @return The wind direction as a short integer.
 */
short get_wind_direction(const uint8_t *response);

/**
 * @brief Extracts and converts the wind speed from the given response.
 *
 * This function takes a pointer to a response array, extracts the wind speed
 * data from specific positions, converts it from a little-endian format to a 
 * float, and prints the raw wind speed data and the converted wind speed to 
 * the Serial monitor.
 *
 * @param response Pointer to the response array containing wind speed data.
 * @return The wind speed in meters per second as a float.
 */
float get_wind_speed(const uint8_t *response);

/**
 * @brief Extracts and converts the temperature from the response data.
 *
 * This function takes a pointer to a response array, extracts the temperature
 * data from specific indices, converts it from little-endian format to a float,
 * and prints the raw temperature data and the converted temperature to the serial monitor.
 *
 * @param response Pointer to the array containing the response data.
 * @return The temperature as a float in degrees Celsius.
 */
float get_temperature(const uint8_t *response);

/**
 * @brief Extracts and converts humidity data from the given response.
 *
 * This function takes a pointer to a response array, extracts the raw humidity
 * data from specific positions, converts it from little-endian format to a 
 * floating-point value, and prints the raw and converted humidity values to 
 * the serial output.
 *
 * @param response Pointer to the response array containing the humidity data.
 * @return The converted humidity value as a float.
 */
float get_humidity(const uint8_t *response);

/**
 * @brief Extracts and converts pressure data from a response array.
 *
 * This function takes a pointer to a response array, extracts the raw pressure
 * data from specific indices, converts it from little-endian format to a float,
 * and prints the raw pressure data and the converted pressure value to the serial
 * monitor.
 *
 * @param response Pointer to the response array containing the pressure data.
 * @return The converted pressure value in hPa as a float.
 */
float get_pressure(const uint8_t *response);

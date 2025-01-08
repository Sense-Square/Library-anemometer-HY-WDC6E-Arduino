
#include "hywdc6e-lib.h"

void init_hywdc6e()
{
    pinMode(RS485_DE_PIN, OUTPUT);   // Set the DE pin to OUTPUT
    pinMode(RS485_RE_PIN, OUTPUT);   // Set the RE pin to OUTPUT
    digitalWrite(RS485_DE_PIN, LOW); // Start in receive mode (LOW to receive)
    digitalWrite(RS485_RE_PIN, LOW); // Start in receive mode (LOW to receive)

    Serial1.begin(9600, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
}

void sendModbusRequest()
{
    digitalWrite(RS485_DE_PIN, HIGH); // Set to HIGH for transmit mode (TX)
    digitalWrite(RS485_RE_PIN, HIGH); // Set to HIGH for transmit mode (TX)
    Serial1.write(request_ane, sizeof(request_ane) + 2);
    delay(10);                       // Ensure data is sent before switching back
    digitalWrite(RS485_DE_PIN, LOW); // Set to LOW for receive mode (RX)
    digitalWrite(RS485_RE_PIN, LOW); // Set to LOW for receive mode (RX)
}

void debugPrint(const uint8_t *data, size_t length)
{
    if (DEBUG)
    {
        for (size_t i = 0; i < length; i++)
        {
            Serial.printf("%02X ", data[i]);
        }
        Serial.println();
    }
}

short get_wind_direction(const uint8_t *response) {
    unsigned char rawWindDirection[2] = {response[5], response[6]};

    Serial.print("rawWindDirection: ");
    for (int i = 0; i < 2; i++) {
        Serial.print(rawWindDirection[i], HEX);
        if (i < 1) Serial.print(", ");
    }
    Serial.println();

    unsigned short windDirection = convertToShortLittleEndian(rawWindDirection);
    Serial.print("Wind direction: ");
    Serial.println(windDirection);

    return windDirection;
}

float get_wind_speed(const uint8_t *response) {
    unsigned char rawWindSpeed[4] = {response[7], response[8], response[9], response[10]};

    Serial.print("rawWindSpeed: ");
    for (int i = 0; i < 4; i++) {
        Serial.print(rawWindSpeed[i], HEX);
        Serial.print(",");
    }
    Serial.println();

    float windSpeed = convertToFloatLittleEndian(rawWindSpeed);
    Serial.print("Wind speed (m/s): ");
    Serial.println(windSpeed, 3);

    return windSpeed;
}

float get_temperature(const uint8_t *response) {
    unsigned char rawTemperature[4] = {response[11], response[12], response[13], response[14]};

    Serial.print("rawTemperature: ");
    for (int i = 0; i < 4; i++) {
        Serial.print(rawTemperature[i], HEX);
        Serial.print(",");
    }
    Serial.println();

    float temperature = convertToFloatLittleEndian(rawTemperature);
    Serial.print("Temperature (C): ");
    Serial.println(temperature, 3);

    return temperature;
}

float get_humidity(const uint8_t *response) {
    unsigned char rawHumidity[4] = {response[15], response[16], response[17], response[18]};

    Serial.print("rawHumidity: ");
    for (int i = 0; i < 4; i++) {
        Serial.print(rawHumidity[i], HEX);
        Serial.print(",");
    }
    Serial.println();

    float humidity = convertToFloatLittleEndian(rawHumidity);
    Serial.print("Humidity (%): ");
    Serial.println(humidity, 3);

    return humidity;
}

float get_pressure(const uint8_t *response) {
    unsigned char rawPressure[4] = {response[19], response[20], response[21], response[22]};

    Serial.print("rawPressure: ");
    for (int i = 0; i < 4; i++) {
        Serial.print(rawPressure[i], HEX);
        Serial.print(",");
    }
    Serial.println();

    float pressure = convertToFloatLittleEndian(rawPressure);
    Serial.print("Pressure (hPa): ");
    Serial.println(pressure, 3);

    return pressure;
}

float convertToFloatLittleEndian(unsigned char *data)
{
    union HexToFloat
    {
        uint32_t hexValue;
        float floatValue;
    };

    unsigned char data_bE[4];

    data_bE[0] = data[2];
    data_bE[1] = data[3];
    data_bE[2] = data[0];
    data_bE[3] = data[1];

    for (int i = 0; i < 4; i++)
    {
        Serial.print(data_bE[i], HEX); // Stampa in formato esadecimale
        Serial.print(",");
    }

    uint32_t combinedValue = 0;

    for (int i = 0; i < 4; i++)
    {
        combinedValue = (combinedValue << 8) | (data_bE[i] & 0xFF);
    }

    // Conversione del valore esadecimale in float
    HexToFloat converter;
    converter.hexValue = combinedValue;
    float result = converter.floatValue;

    return result;
}

unsigned short convertToShortLittleEndian(const uint8_t *data)
{
    return (data[0] << 8) | data[1]; // Combine two bytes (Little Endian)
}

void get_raw_response(uint8_t *buffer, size_t bufferSize, size_t *bytesRead) {
  // Inizializza il contatore dei byte letti
  *bytesRead = 0;
  unsigned long startTime = millis();

  while (millis() - startTime < RESPONSE_TIMEOUT) {
    if (Serial1.available() > 0) {
      buffer[*bytesRead] = Serial1.read();
      (*bytesRead)++;
      if (*bytesRead >= bufferSize)
        break;
    }
  }

  // Print debug information if no response
  if (*bytesRead == 0) {
    Serial.println("No response received (timeout). Checking available bytes:");
    Serial.println(Serial1.available());  // Print available bytes
  }
}

bool is_connected(){
    sendModbusRequest();

    uint8_t response[256];
    size_t bytesRead = 0;
    get_raw_response(response, sizeof(response), &bytesRead);

    if (response[0] == 0x01)
    {
        return true;
    }
    else
    {
        return false;
    }
}
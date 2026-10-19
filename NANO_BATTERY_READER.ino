/*
  Arduino Nano battery reader for the SENTER RACING Wemos firmware.

  Battery divider:
    Battery + -- 100K -- A0 -- 15K -- GND
    Battery - ---------------------- common GND

  Two-way serial connection, 9600 baud:
    Wemos TX -- Nano RX (D0)
    Nano TX (D1) -- 1K --+-- Wemos RX
                          |
                         1.8K
                          |
                         GND
    Connect Nano GND and Wemos GND.
    Disconnect Nano TX from Wemos RX when uploading firmware to the Wemos.
    Disconnect Wemos TX from Nano RX when uploading firmware to the Nano.

  The Wemos sends Q,<sequence>,<checksum>. The Nano replies with
  R,<sequence>,<battery millivolts>,<checksum>. Both lines end with newline.
  The checksum is the XOR of all bytes before the final comma.
*/
#include <stdio.h>

const uint8_t BATTERY_PIN = A0;
const float BATTERY_R1 = 100000.0;
const float BATTERY_R2 = 10000.0;
const float ADC_REFERENCE_MILLIVOLTS = 5000.0;  // Adjust to the measured Nano AVcc.
const float ADC_MAX_VALUE = 1023.0;
const uint8_t BATTERY_SAMPLES = 20;
const uint32_t BATTERY_MAX_MILLIVOLTS = 16000UL;

char requestBuffer[20];
uint8_t requestLength = 0;
bool requestOverflow = false;

uint8_t serialChecksum(const char *data, uint8_t length) {
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < length; i++) checksum ^= (uint8_t)data[i];
  return checksum;
}

bool parseSerialNumber(
  const char *data,
  uint8_t length,
  uint8_t &position,
  char delimiter,
  bool hasDelimiter,
  uint32_t maximum,
  uint32_t &result
) {
  uint32_t value = 0;
  uint8_t digits = 0;
  while (position < length && (!hasDelimiter || data[position] != delimiter)) {
    char digit = data[position++];
    if (digit < '0' || digit > '9') return false;
    uint8_t number = (uint8_t)(digit - '0');
    if (value > (maximum - number) / 10UL) return false;
    value = value * 10UL + number;
    digits++;
  }
  if (digits == 0) return false;
  if (hasDelimiter) {
    if (position >= length || data[position] != delimiter) return false;
    position++;
  } else if (position != length) {
    return false;
  }
  result = value;
  return true;
}

uint32_t readBatteryMillivolts() {
  uint32_t adcTotal = 0;
  for (uint8_t i = 0; i < BATTERY_SAMPLES; i++) {
    adcTotal += analogRead(BATTERY_PIN);
    delayMicroseconds(200);
  }

  float adcAverage = adcTotal / (float)BATTERY_SAMPLES;
  if (adcAverage <= 5.0) return 0;

  float dividerRatio = (BATTERY_R1 + BATTERY_R2) / BATTERY_R2;
  uint32_t millivolts = (uint32_t)(
    adcAverage * ADC_REFERENCE_MILLIVOLTS * dividerRatio / ADC_MAX_VALUE + 0.5
  );
  if (millivolts > BATTERY_MAX_MILLIVOLTS) return 0;
  return millivolts;
}

void sendBatteryResponse(uint32_t sequence) {
  uint32_t millivolts = readBatteryMillivolts();
  char responsePayload[24];
  int payloadLength = snprintf(
    responsePayload,
    sizeof(responsePayload),
    "R,%u,%lu",
    (unsigned int)sequence,
    (unsigned long)millivolts
  );
  if (payloadLength <= 0 || payloadLength >= (int)sizeof(responsePayload)) return;

  Serial.print(responsePayload);
  Serial.print(',');
  Serial.println(serialChecksum(responsePayload, (uint8_t)payloadLength));
}

void processRequest() {
  if (requestOverflow || requestLength < 5) return;
  if (requestBuffer[0] != 'Q' || requestBuffer[1] != ',') return;

  uint8_t checksumSeparator = requestLength;
  for (uint8_t i = 2; i < requestLength; i++) {
    if (requestBuffer[i] == ',') checksumSeparator = i;
  }
  if (checksumSeparator >= requestLength) return;
  uint8_t checksumPosition = checksumSeparator + 1;
  uint32_t receivedChecksum = 0;
  if (!parseSerialNumber(
        requestBuffer,
        requestLength,
        checksumPosition,
        '\0',
        false,
        255,
        receivedChecksum
      )) return;
  if (receivedChecksum != serialChecksum(requestBuffer, checksumSeparator)) return;

  uint8_t position = 2;
  uint32_t sequence = 0;
  if (!parseSerialNumber(
        requestBuffer,
        checksumSeparator,
        position,
        ',',
        false,
        65535UL,
        sequence
      )) return;

  sendBatteryResponse(sequence);
}

void setup() {
  Serial.begin(9600);
  delay(1000);
}

void loop() {
  while (Serial.available() > 0) {
    char incoming = (char)Serial.read();
    if (incoming == '\r') continue;

    if (incoming == '\n') {
      if (requestLength > 0) processRequest();
      requestLength = 0;
      requestOverflow = false;
      continue;
    }

    if (requestLength >= sizeof(requestBuffer) - 1) {
      requestOverflow = true;
      continue;
    }
    if (!requestOverflow) {
      requestBuffer[requestLength++] = incoming;
    }
  }
}

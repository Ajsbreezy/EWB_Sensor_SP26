// PINOUT
/*
Red - 5V
Black - GND
Green - 4
Blue - 5
*/

HardwareSerial sensorSerial(1);

const uint8_t  HEADER_BYTE     = 0xFF;
const uint16_t READ_TIMEOUT_MS = 500;
const int      ERROR_DISTANCE  = -1;

void setup() {
  Serial.begin(115200);
  sensorSerial.begin(9600, SERIAL_8N1, 4, 5);  // RX=GPIO4, TX=GPIO5
}

void loop() {
  int dist = getDistance();
  if (dist != ERROR_DISTANCE) {
    Serial.print("distance=");
    Serial.print(dist);
    Serial.println("mm");
  } else {
    Serial.println("Error reading sensor");
  }
  // delay(1000) // or delay whatever
}

uint8_t readN(uint8_t *buf, size_t len) {
  size_t offset = 0, left = len;
  unsigned long curr = millis();
  while (left) {
    if (sensorSerial.available()) {
      buf[offset++] = sensorSerial.read();
      left--;
    }
    if (millis() - curr > READ_TIMEOUT_MS) break;
  }
  return offset;
}

int getDistance(void) {
  uint8_t data[4] = {0};
  uint8_t receivedByte = 0;
  unsigned long startTime = millis();

  while (millis() - startTime < READ_TIMEOUT_MS) {
    if (readN(&receivedByte, 1) == 1 && receivedByte == HEADER_BYTE) {
      data[0] = receivedByte;
      if (readN(&data[1], 3) == 3) {
        uint8_t checksum = data[0] + data[1] + data[2];
        if (checksum == data[3]) {
          return (data[1] << 8) | data[2];
        }
      }
    }
  }
  return ERROR_DISTANCE;
}
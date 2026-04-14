// EWB Malawi Water Tank Monitor
// Michael Chou, Andrew Schretzmayer, Eduardo Hernandez, Nathan Lee, Paul Wang
// Hardware:  Custom PCB with ESP32-C3-WROOM-02, Blues Notecard (I2C),
//            UART ultrasonic distance sensor, powered by 12.8V LiFePO4 battery
//            via dual AP64500 buck converters (5V + 3.3V rails)
//
// Behavior:  Wakes from deep sleep every hour, takes a distance reading from
//            the ultrasonic sensor, stores it in RTC memory. At the start of
//            each new day (Malawi time, UTC+2), uploads all 24 readings to
//            Notehub via the Blues Notecard over cellular, then clears the
//            buffer and starts collecting for the new day.
//
// Library:   "Blues Wireless Notecard" (note-arduino)
//
// =============================================================================
//
// WIRING / CONNECTOR PINOUT
//
//  Notecard (via Notecarrier header J1/J4, I2C):
//    N_SDA   -> ESP32-C3 GPIO2
//    N_SCL   -> ESP32-C3 GPIO3
//    N_ATTN  -> ESP32-C3 GPIO8  (optional, not used yet but there just in case)
//    4.7kΩ pull-ups on SDA and SCL to +3.3V (R9, R2 on PCB)
//
//  Ultrasonic Sensor (via J3, 4-pin JST PH, UART):
//    J3 Pin 1 -> 3.3V          (Red wire)
//    J3 Pin 2 -> GND           (Black wire)
//    J3 Pin 3 -> ESP32 GPIO7   (Blue wire  - sensor RX, ESP32 TX)
//    J3 Pin 4 -> ESP32 GPIO6   (Green wire - sensor TX, ESP32 RX)
//
//    NOTE: If sensor requires 5V power, you may need to jumper J3 Pin 1
//          to the +5V rail instead of +3.3V. Check sensor datasheet.
// =============================================================================

#include <Notecard.h>
#include <Wire.h>
#include "soc/rtc.h"
// PIN DEFINITIONS (should be matched to PCB schematic)
// I2C to Notecard
#define NOTE_SDA_PIN   2    // ESP32-C3 GPIO2 -> Notecard SDA
#define NOTE_SCL_PIN   3    // ESP32-C3 GPIO3 -> Notecard SCL
#define NOTE_ATTN_PIN  8    // ESP32-C3 GPIO8 -> Notecard ATTN (for future use)

// UART to ultrasonic distance sensor
#define SNS_RX_PIN     6    // ESP32-C3 GPIO6 -> Sensor TX
#define SNS_TX_PIN     7    // ESP32-C3 GPIO7 -> Sensor RX

// defines
#define SLEEP_DURATION_US    3600000000ULL   // 1 hour in microseconds
#define MAX_READINGS_PER_DAY 24              // one reading per hour
#define MALAWI_UTC_OFFSET    7200            // UTC+2 in seconds
#define SENSOR_BAUD          9600            // sensor UART baud rate
#define SENSOR_TIMEOUT_MS    500             // timeout waiting for sensor data

// SENSOR PROTOCOL
const uint8_t  HEADER_BYTE = 0xFF;       // sensor frame start byte
const int ERROR_DISTANCE = -1;         // returned on read failure

// NOTECARD API
Notecard notecard;
const char *PROJECT_UID = "com.gmail.mzlchou:ewbmalawiwatertank";

// RTC MEMORY (survives deep sleep)
// these variables persist across deep sleep cycles because they are stored
// in the ESP32's RTC slow memory, not regular RAM
RTC_DATA_ATTR int readings[MAX_READINGS_PER_DAY]; // stored distance values (mm)
RTC_DATA_ATTR int readingsCount = 0; // how many readings taken today
RTC_DATA_ATTR int lastDay = -1; // tracks current day for new day detection

// UART sensor serial port
// uses hardware serial port 1 (Serial1) so it doesn't conflict with
// the USB debug serial (Serial)
HardwareSerial sensorSerial(1);

// readN - Read exactly len bytes from the sensor UART with timeout
// =============================================================================
// Parameters:
//   buf  - pointer to buffer to fill with received bytes
//   len  - number of bytes to read
//
// Returns:
//   number of bytes actually read (may be less than len if timeout occurs)
//
// Notes:
//   Uses SENSOR_TIMEOUT_MS as the maximum wait time. If the sensor does not
//   send enough bytes within that window, returns however many were received.
// =============================================================================
uint8_t readN(uint8_t *buf, size_t len) {
    // offset tracks where in the buffer we are writing next
    size_t offset = 0;
    // left tracks how many bytes we still need to read
    size_t left = len;
    // record the current time so we can check for timeout
    unsigned long startTime = millis();

    // keep reading until we have all requested bytes or we time out
    while (left > 0) {
        // check if any bytes have arrived on the sensor serial line
        if (sensorSerial.available()) {
            // read one byte from the serial buffer and store it
            buf[offset] = sensorSerial.read();
            offset++;
            left--;
        }
        // check if we've exceeded the timeout window
        if (millis() - startTime > SENSOR_TIMEOUT_MS) {
            break;
        }
    }

    // return how many bytes we actually managed to read
    return offset;
}

// getDistance - Read one distance measurement from the ultrasonic sensor
// Parameters:  none
//
// Returns:
//   distance in millimeters (positive int), or ERROR_DISTANCE (-1) on failure
//
// Protocol:
//   The sensor continuously streams 4-byte frames:
//     Byte 0: 0xFF (header)
//     Byte 1: distance high byte
//     Byte 2: distance low byte
//     Byte 3: checksum (sum of bytes 0-2, lower 8 bits)
//
//   This function scans the incoming stream for a valid header byte, reads the
//   remaining 3 bytes, verifies the checksum, and returns the 16-bit distance.
//   If no valid frame is found within SENSOR_TIMEOUT_MS, returns -1.
int getDistance(void) {
    // buffer to hold one complete 4-byte frame from the sensor
    uint8_t data[4] = {0};
    // single byte buffer for scanning the stream one byte at a time
    uint8_t receivedByte = 0;
    // record the start time so we can enforce the overall timeout
    unsigned long startTime = millis();

    // keep scanning the stream until we find a valid frame or timeout
    while (millis() - startTime < SENSOR_TIMEOUT_MS) {
        // try to read one byte and check if it's the header (0xFF)
        if (readN(&receivedByte, 1) == 1 && receivedByte == HEADER_BYTE) {
            // found a header byte - store it as the first byte of the frame
            data[0] = receivedByte;

            // try to read the remaining 3 bytes (high byte, low byte, checksum)
            if (readN(&data[1], 3) == 3) {
                // calculate expected checksum: sum of first 3 bytes (truncated to 8 bits)
                uint8_t checksum = data[0] + data[1] + data[2];

                // verify the checksum byte matches our calculated value
                if (checksum == data[3]) {
                    // combine high and low bytes into a 16-bit distance in mm
                    return (data[1] << 8) | data[2];
                }
                // checksum mismatch means corrupted data - keep scanning
                // for another header byte
            }
        }
    }

    // no valid frame found within the timeout window
    return ERROR_DISTANCE;
}

// readSensor - Initialize sensor UART and take a distance reading
// Parameters:  none
//
// Returns:
//   distance in millimeters, or -1 if the sensor did not respond
//
// Notes:
//   Initializes Serial1 on the sensor pins each wake cycle. The sensor
//   continuously streams data once powered, so we just need to listen
//   for a valid frame after the UART is ready.
int readSensor() {
    // initialize UART on the sensor pins (RX=GPIO6 receives from sensor,
    // TX=GPIO7 sends to sensor). The sensor starts streaming distance
    // frames automatically as soon as it has power.
    sensorSerial.begin(SENSOR_BAUD, SERIAL_8N1, SNS_RX_PIN, SNS_TX_PIN);
    // short delay to let the sensor start streaming and fill the UART buffer
    delay(100);

    // read one valid distance frame from the sensor's continuous stream
    int distance_mm = getDistance();

    // release the UART pins until the next reading to save power
    sensorSerial.end();
    return distance_mm;
}

// getCurrentDay - Get the current day number from the Notecard's clock
// Parameters:  none
//
// Returns:
//   absolute day number (unix_time / 86400, adjusted for Malawi UTC+2),
//   or -1 if the Notecard hasn't synced time yet
//
// Notes:
//   Uses absolute day number instead of day-of-year to avoid bugs at
//   year boundaries. Two consecutive days will always differ by 1
//   regardless of December 31 -> January 1 transitions.
int getCurrentDay() {
    // request the current time from the Notecard
    J *req = notecard.newRequest("card.time");
    // send the request and wait for the JSON response
    J *rsp = notecard.requestAndResponse(req);

    // check if we got a response at all (NULL means I2C comm failure)
    if (rsp == NULL) {
        return -1;
    }

    // extract the unix timestamp from the JSON response
    // card.time returns {"time": <unix_timestamp>, "zone": "UTC", ...}
    long unixTime = (long)JGetNumber(rsp, "time");
    // free the response memory to prevent memory leaks
    notecard.deleteResponse(rsp);

    // time of 0 or negative means the Notecard hasn't completed its
    // first cellular sync yet and doesn't know the current time
    if (unixTime <= 0) {
        return -1;
    }

    // adjust for Malawi timezone (UTC+2 = +7200 seconds) so that
    // the day boundary aligns with local midnight, not UTC midnight
    unixTime += MALAWI_UTC_OFFSET;
    // divide by seconds-per-day to get an absolute day number
    return (int)(unixTime / 86400);
}

// configureNotecard - Set up Notehub connection (called on first boot only)
// Parameters:  none
// Returns:     void
//
// Configures:
//   - Associates device with the Notehub project (PROJECT_UID)
//   - Sets periodic sync mode with daily outbound sync (1440 minutes)
//   - Enables periodic GPS/cell location capture (once per day)
void configureNotecard() {
    Serial.println("Configuring Notecard...");

    // hub.set tells the Notecard which Notehub project to sync with
    // "periodic" mode means it connects to cellular on a schedule, not
    // continuously, which saves power for battery-operated deployments
    // "outbound" of 1440 minutes means it syncs queued notes once per day
    J *req = notecard.newRequest("hub.set");
    JAddStringToObject(req, "product", PROJECT_UID);
    JAddStringToObject(req, "mode", "periodic");
    JAddNumberToObject(req, "outbound", 1440);
    notecard.sendRequest(req);

    // enable location tracking so we can see where the device is on Notehub
    // captures GPS/cell tower location once per day (86400 seconds)
    req = notecard.newRequest("card.location.mode");
    JAddStringToObject(req, "mode", "periodic");
    JAddNumberToObject(req, "seconds", 86400);
    notecard.sendRequest(req);

    Serial.println("Notecard configured");
}

// uploadData - Send all stored readings to Notehub and clear the buffer
// Parameters:  none
// Returns:     void
//
// Builds a note.add request with a JSON body containing:
//   {
//     "readings": <count>,
//     "day_data": [<dist1>, <dist2>, ..., <distN>]
//   }
//
// On success: clears the readings array and resets readingsCount to 0.
// On failure: preserves readings so they can be retried on the next cycle.
// Forces an immediate hub.sync after queuing the note.

void uploadData() {
    Serial.println("\n=== UPLOADING DAILY DATA ===");

    // nothing to do if no readings were collected
    if (readingsCount == 0) {
        Serial.println("No data to upload");
        return;
    }

    Serial.print("Uploading ");
    Serial.print(readingsCount);
    Serial.println(" readings...");

    // create a note.add request — this adds a "note" (data record) to the
    // Notecard's outbound queue in the file "daily.qo"
    // the ".qo" suffix means "queue outbound" — the Notecard will send it
    // to Notehub on the next cellular sync
    // "sync":true tells the Notecard to prioritize syncing this note
    J *req = notecard.newRequest("note.add");
    JAddStringToObject(req, "file", "daily.qo");
    JAddBoolToObject(req, "sync", true);

    // build the JSON body object with the readings count
    J *body = JAddObjectToObject(req, "body");
    JAddNumberToObject(body, "readings", readingsCount);

    // create a JSON array and add each distance reading to it
    // this produces: "day_data": [123, 456, 789, ...]
    J *arr = JAddArrayToObject(body, "day_data");
    for (int i = 0; i < readingsCount; i++) {
        JAddItemToArray(arr, JCreateNumber(readings[i]));
    }

    // send the request to the Notecard and wait for the response
    J *rsp = notecard.requestAndResponse(req);
    if (rsp != NULL) {
        // check if the Notecard returned an error in the response JSON
        // this can happen if the Notecard's storage is full or other issues
        if (notecard.responseError(rsp)) {
            Serial.print("ERROR: Upload failed - ");
            Serial.println(JGetString(rsp, "err"));
            notecard.deleteResponse(rsp);
            // don't clear readings so we can retry on the next wake cycle
            return;
        }
        // free the response memory
        notecard.deleteResponse(rsp);

        Serial.println("Note queued successfully");

        // force the Notecard to connect to cellular and sync immediately
        // instead of waiting for the next periodic sync window
        req = notecard.newRequest("hub.sync");
        notecard.sendRequest(req);
        // give the Notecard time to start the cellular modem and begin sync
        delay(3000);

        // clear the readings array now that data has been sent
        readingsCount = 0;
        // zero out the array so stale data doesn't persist
        memset(readings, 0, sizeof(readings));
        Serial.println("Upload complete, readings cleared");
    } else {
        // NULL response means the I2C communication itself failed
        // this could indicate a wiring issue or the Notecard being busy
        Serial.println("ERROR: No response from Notecard, will retry next cycle");
        // don't clear readings so data is preserved for retry
    }
}

// logReading - Take a sensor reading and store it in RTC memory
// Parameters:  none
// Returns:     void
//
// Calls readSensor() to get the current distance, prints it to Serial
// for debugging, and appends it to the readings[] array in RTC memory.
// If the array is full (24 readings), prints a warning and does not store.

void logReading() {
    // read the current distance from the ultrasonic sensor
    int dist = readSensor();

    // print the reading to serial for debugging during development
    Serial.print("Reading #");
    // readingsCount contains the number of readings done so far 
    Serial.print(readingsCount + 1);
    Serial.print(": ");
    if (dist == ERROR_DISTANCE) {
        Serial.println("ERROR (no sensor response)");
    } else {
        Serial.print(dist);
        Serial.println(" mm");
    }

    // store the reading in the RTC memory array if there's room
    // even error readings (-1) are stored so we can see gaps in the data
    // Store in RTC memory array for one day
    if (readingsCount < MAX_READINGS_PER_DAY) {
        readings[readingsCount] = dist;
        readingsCount++;
        Serial.println("Stored in RTC memory");
    } else {
        // this should never happen with hourly readings and daily uploads,
        // but protects against edge cases like multiple failed uploads
        Serial.println("WARNING: readings array full (24 max)");
    }
}

// SETUP - Runs on every wake from deep sleep
// The ESP32-C3 fully resets after deep sleep, so setup() runs every time.
// RTC_DATA_ATTR variables persist across these resets, which is how we
// accumulate readings across 24 hourly wake cycles.
//
//   1. Initialize serial debug output
//   2. Initialize Notecard over I2C, verify connection
//   3. On first boot (readingsCount == 0), configure Notecard for project
//   4. Get current time from Notecard, determine current day
//   5. If new day detected, upload previous day's readings
//   6. Take a distance reading and store in RTC memory
//   7. Enter deep sleep for 1 hour

void setup() {
    // initialize USB serial for debug output (visible via USB-C on devkit,
    // or via UART header J5 on the custom PCB)
    Serial.begin(115200);
    // short delay to let the serial port stabilize after reset
    delay(2000);
    
    // Enable external 32.768kHz crystal for accurate deep sleep timing.
    //OSCILLATOR
    rtc_clk_32k_enable(true);
    delay(300);  // let crystal oscillation stabilize
    rtc_clk_slow_freq_set(RTC_SLOW_FREQ_32K_XTAL);

    Serial.println("\n=== Water Tank Monitor ===");
    Serial.print("Wakeup #");
    // readingsCount tells us how many readings have been taken so far today
    Serial.println(readingsCount + 1);

    //  NOTECARD INIT (I2C) 
    Serial.println("Initializing Notecard over I2C...");
    // tell the ESP32 which GPIO pins to use for I2C
    // this must be called BEFORE notecard.begin() which internally
    // calls Wire.begin(). The ESP32-C3 defaults to GPIO8/GPIO9 for I2C,
    // which are NOT the pins we wired on this PCB.
    Wire.setPins(NOTE_SDA_PIN, NOTE_SCL_PIN);
    // initialize the note-arduino library in I2C mode
    // (default Notecard I2C address is 0x17)
    notecard.begin();
    // enable debug output so all Notecard JSON traffic is printed to
    // Serial — useful during development, can be removed for production
    notecard.setDebugOutputStream(Serial);

    // verify the Notecard is alive by requesting its firmware version
    J *req = notecard.newRequest("card.version");
    J *rsp = notecard.requestAndResponse(req);
    if (rsp != NULL && !notecard.responseError(rsp)) {
        Serial.println("Notecard connected");
        // free the response JSON memory
        notecard.deleteResponse(rsp);
    } else {
        // if the Notecard doesn't respond, go back to sleep and try again
        // next hour. This can happen if the Notecard hasn't finished
        // booting, or if there's an I2C wiring issue.
        Serial.println("ERROR: Notecard not responding, sleeping to retry");
        if (rsp) notecard.deleteResponse(rsp);
        delay(1000);
        esp_sleep_enable_timer_wakeup(SLEEP_DURATION_US);
        esp_deep_sleep_start();
    }

    // FIRST RUN CONFIG 
    // on the very first boot (or after an upload resets readingsCount to 0),
    // send the hub.set and location config to the Notecard
    if (readingsCount == 0) {
        configureNotecard();
    }

    // TIME & DAY CHECK 
    // get the current day number from the Notecard's synced clock
    int currentDay = getCurrentDay();
    if (currentDay < 0) {
        // time not available usually means the Notecard hasn't completed
        // its first cellular sync yet and doesn't know what time it is.
        // sleep and try again next hour — it should sync within a few minutes
        // of first power-on.
        Serial.println("ERROR: Time not available yet, sleeping to retry");
        delay(1000);
        esp_sleep_enable_timer_wakeup(SLEEP_DURATION_US);
        esp_deep_sleep_start();
    }

    Serial.print("Current day: ");
    Serial.println(currentDay);

    // on the very first boot, initialize lastDay to the current day
    // so we don't immediately trigger an upload with an empty array
    if (lastDay == -1) {
        lastDay = currentDay;
    }

    // check if the calendar day has changed since our last reading.
    // if so, upload all of yesterday's readings before starting the new day.
    if (currentDay != lastDay) {
        Serial.println("\n*** NEW DAY DETECTED ***");
        uploadData();
        // update lastDay so we don't re-trigger until tomorrow
        lastDay = currentDay;
    }

    // TAKE READING 
    Serial.println("\nTaking reading...");
    logReading();

    // DEEP SLEEP 
    Serial.print("\nSleeping for ");
    Serial.print((unsigned long)(SLEEP_DURATION_US / 60000000ULL));
    Serial.println(" minutes...\n");
    // short delay to let the serial output finish printing before sleep
    delay(100);
    // configure the RTC wakeup timer and enter deep sleep
    // when the timer expires, the ESP32-C3 will fully reset and run
    // setup() again from the top. Only RTC_DATA_ATTR variables survive.
    esp_sleep_enable_timer_wakeup(SLEEP_DURATION_US);
    esp_deep_sleep_start();
}

void loop() {
    // never reached the ESP32-C3 resets after waking from deep sleep,
    // so execution always starts fresh at setup()
}

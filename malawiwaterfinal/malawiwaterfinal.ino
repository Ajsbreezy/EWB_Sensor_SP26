#include <NewPing.h>
#include <SPI.h>
#include <SD.h>
// ==================== CONFIG ====================
#define TRIGGER_PIN 3
#define ECHO_PIN    2
#define MAX_DISTANCE 400
// UART pins for ESP32-C3 to Notecard
#define NOTE_RX 19  // ESP32 GPIO 19 ← Notecard N TX
#define NOTE_TX 18  // ESP32 GPIO 18 → Notecard N RX
// SD card SPI pins
#define SD_CS   10
#define SD_SCK  4
#define SD_MOSI 5
#define SD_MISO 6
// Deep sleep duration (1 hour = 3600 seconds)
#define SLEEP_DURATION_SECONDS 3600
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
File myFile;
const char *SD_FILENAME = "/DAY.TXT";
const char *PROJECT_UID = "com.gmail.mzlchou:ewbmalawiwatertank";
// ==================== RTC MEMORY (survives deep sleep) ====================
RTC_DATA_ATTR int readingsCount = 0;
RTC_DATA_ATTR int lastDay = -1;
// ==================== NOTECARD UART FUNCTION ====================
String notecardCommand(String cmd) {
    // discards any leftover input buffer so that data from a previous 
    // command doesn't contaminate this response
    while (Serial1.available()) {
        Serial1.read();
    }
    // sends the command to the device over serial
    Serial1.println(cmd);
    // records the current time in milliseconds
    unsigned long start = millis();
    String response = "";
    // loops for up to 5 seconds waiting for a response
    while (millis() - start < 5000) {
        // checks whether any bytes have arrived on the serial line before trying to read
        if (Serial1.available()) {
            // gets one byte from the serial buffer
            char c = Serial1.read();
            response += c;
            // checks whether a complete JSON object has been received 
            if (c == '}' && response.indexOf('{') >= 0) {
                // waits 10ms after detecting the closing "}" to give the serial buffer a 
                // moment to fill with any remaining bytes
                delay(10);
                // adds any remaining bytes that arrived during the 10ms delay into 
                // the response string, ensuring nothing is cut off.
                while (Serial1.available()) {
                    response += (char)Serial1.read();
                }
                break;
            }
        }
    }
    // returns the full response string to the caller
    return response;
}
// ==================== PARSE JSON LONG ====================
long parseJsonLong(String json, String key) {
    // keyIdx contains the index of key in json
    int keyIdx = json.indexOf("\"" + key + "\":");
    // function returns -1 if key wasn't found in json
    if (keyIdx < 0) return -1;
    // startIdx contains the index of where the value begins
    int startIdx = keyIdx + key.length() + 3;
    int endIdx = startIdx;
    // loops through json starting at the beginning of the value and finds 
    // the end index of the value and assigns it to endIdx
    while (endIdx < json.length()) {
        // gets character at current position
        char c = json.charAt(endIdx);
        // breaks if the current character is a delimiter
        if (c == ',' || c == '}' || c == ' ' || c == '\n' || c == '\r') break;
        endIdx++;
    }
    // converts the string to a number and returns it
    String numStr = json.substring(startIdx, endIdx);
    numStr.trim();
    return numStr.toInt();
}
// ==================== CONVERT UNIX TIME TO DAY ====================
int getDayOfYear(long unixTime) {
    unixTime -= 18000; // Adjust for EST timezone
    long secondsPerDay = 86400;
    return (unixTime / secondsPerDay) % 365;
}
// ==================== LOG READING ====================
void logReading() {
    // sonar.ping_cm() returns the distance to an object and returns it in cm
    // it returns 0 if the object is out of range
    int dist = sonar.ping_cm();
    if (dist == 0) dist = -1;
    Serial.print("Reading #");
    // readingsCount contains the number of readings done so far 
    Serial.print(readingsCount + 1);
    Serial.print(": ");
    Serial.print(dist);
    Serial.println(" cm");
    myFile = SD.open(SD_FILENAME, FILE_APPEND);
    if (myFile) {
        myFile.print("Distance_cm: ");
        myFile.println(dist);
        myFile.close();
        readingsCount++;
        Serial.println("Logged to SD");
    } else {
        Serial.println("ERROR: SD write failed");
    }
}
// ==================== UPLOAD DATA ====================
void uploadData() {
    Serial.println("\n=== UPLOADING DAILY DATA ===");
    myFile = SD.open(SD_FILENAME);
    if (!myFile) {
        Serial.println("No data to upload");
        return;
    }
    // Build a JSON array string
    String dataArray = "[";
    int count = 0;
    while (myFile.available()) {
        String line = myFile.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) continue;
        // Remove "Distance_cm: " prefix if present
        int colonIdx = line.indexOf(':');
        String valueStr = line;
        if (colonIdx >= 0) {
            valueStr = line.substring(colonIdx + 1);
            valueStr.trim();
        }
        dataArray += valueStr + ",";
        count++;
    }
    myFile.close();
    if (dataArray.endsWith(",")) {
        dataArray.remove(dataArray.length() - 1); // remove trailing comma
    }
    dataArray += "]";
    Serial.print("Total readings: ");
    Serial.println(count);
    Serial.println("JSON array prepared:");
    Serial.println(dataArray);
    // Send to Notecard
    String noteCmd = "{\"req\":\"note.add\",\"file\":\"daily.qo\",\"sync\":true,\"body\":{\"readings\":" +
                     String(count) + ",\"day_data\":" + dataArray + "}}";
    Serial.println("Sending to Notecard...");
    String rsp = notecardCommand(noteCmd);
    if (rsp.indexOf("\"total\"") > 0) {
        Serial.println("✓ Note queued");
        // Force sync
        Serial.println("Forcing sync...");
        rsp = notecardCommand("{\"req\":\"hub.sync\"}");
        delay(3000);
        // Clear file and counter
        SD.remove(SD_FILENAME);
        readingsCount = 0;
        Serial.println("✓ Upload complete - Check Notehub Events for daily.qo!");
        Serial.println("✓ File cleared, ready for new day");
    } else {
        Serial.println("ERROR: Upload failed");
        Serial.println("Response: " + rsp);
    }
}
// ==================== SETUP ====================
void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n=== Water Tank Monitor ===");
    Serial.print("Wakeup #");
    Serial.println(readingsCount + 1);
    // ---------- SD CARD ----------
    Serial.println("Initializing SD card...");
    pinMode(SD_CS, OUTPUT);
    digitalWrite(SD_CS, HIGH);
    delay(500);
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    delay(200);
    if (!SD.begin(SD_CS, SPI, 400000)) {
        Serial.println("ERROR: SD init failed");
        delay(1000);
        esp_deep_sleep_start(); // Sleep and retry
    }
    Serial.println("✓ SD initialized");
    // ---------- NOTECARD ----------
    Serial.println("Initializing Notecard...");
    Serial1.begin(9600, SERIAL_8N1, NOTE_RX, NOTE_TX);
    delay(1000);
    while (Serial1.available()) Serial1.read();
    // Test connection
    String rsp = notecardCommand("{\"req\":\"card.version\"}");
    if (rsp.indexOf("version") > 0) {
        Serial.println("✓ Notecard connected");
    } else {
        Serial.println("ERROR: Notecard not responding");
        delay(1000);
        esp_deep_sleep_start();
    }
    // Get time
    Serial.println("Getting time...");
    String timeRsp = notecardCommand("{\"req\":\"card.time\"}");
    long unixTime = parseJsonLong(timeRsp, "time");
    if (unixTime <= 0) {
        Serial.println("ERROR: Time not available");
        delay(1000);
        esp_deep_sleep_start(); // Sleep and retry
    }
    int currentDay = getDayOfYear(unixTime);
    Serial.print("Current day of year: ");
    Serial.println(currentDay);
    // First run - configure Notecard
    if (readingsCount == 0) {
        Serial.println("First run - configuring Notecard...");
        String hubCmd = "{\"req\":\"hub.set\",\"product\":\"" + String(PROJECT_UID) +
                        "\",\"mode\":\"periodic\",\"outbound\":1440}"; // Sync daily
        rsp = notecardCommand(hubCmd);
        rsp = notecardCommand("{\"req\":\"card.location.mode\",\"mode\":\"periodic\",\"seconds\":86400}");
        Serial.println("✓ Notecard configured");
        lastDay = currentDay;
    }
    // Check if new day
    if (lastDay != -1 && currentDay != lastDay) {
        Serial.println("\n*** NEW DAY DETECTED ***");
        uploadData();
        lastDay = currentDay;
    }
    // ---------- TAKE READING ----------
    Serial.println("\nTaking reading...");
    logReading();
    // ---------- DEEP SLEEP ----------
    Serial.print("\nSleeping for ");
    Serial.print(SLEEP_DURATION_SECONDS / 60);
    Serial.println(" minutes...\n");
    delay(100); // Let serial finish
    esp_sleep_enable_timer_wakeup(SLEEP_DURATION_SECONDS * 1000000ULL);
    esp_deep_sleep_start();
}
void loop() {
    // Never reached - ESP32 resets after deep sleep
}

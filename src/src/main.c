// EWB Malawi Water Tank Monitor
// Michael Chou, Andrew Schretzmayer, Eduardo Hernandez, Nathan Lee, Paul Wang
// Hardware:  Custom PCB with ESP32-C3-WROOM-02, Blues Notecard (I2C),
//            UART ultrasonic distance sensor (UART), powered by 12.8V LiFePO4 battery
//            via dual AP64500 buck converters (5V + 3.3V rails)
//
// Behavior:  Wakes from deep sleep every hour, takes a distance reading from
//            the ultrasonic sensor, stores it in RTC memory. At the start of
//            each new day (Malawi time, UTC+2), uploads all 24 readings to
//            Notehub via the Blues Notecard over cellular, then clears the
//            buffer and starts collecting for the new day.
//
// Library:   "Blues Wireless Notecard"
//
// Framework:  ESP-IDF
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
// =============================================================================

#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "notecard.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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
#define PROJECTUID           "com.gmail.mzlchou:ewbmalawiwatertank"
// flag these variables so they are stored in ESP32's RTC memory
// to persist across deep sleeps
RTC_DATA_ATTR int readings[MAX_READINGS_PER_DAY];
RTC_DATA_ATTR int readingsCount = 0;
RTC_DATA_ATTR int lastDay = -1;
// SENSOR PROTOCOL
const uint8_t  HEADER_BYTE = 0xFF;       // sensor frame start byte
const int ERROR_DISTANCE = -1;         // returned on read failure

static const char *TAG = "notecard_basic";

static esp_err_t notecard_hardware_init(void)
{
    ESP_LOGI(TAG, "Initializing Notecard hardware...");

    // Configure Notecard for I2C communication using Kconfig defaults
    notecard_config_t config = NOTECARD_I2C_CONFIG_DEFAULT();

    config.i2c.sda_pin = NOTE_SDA_PIN; // GPIO2
    config.i2c.scl_pin = NOTE_SCL_PIN; // GPIO3

    // Initialize Notecard
    esp_err_t ret = notecard_init(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Notecard: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "Check wiring and power supply");
        return ret;
    }

    ESP_LOGI(TAG, "Notecard hardware initialized successfully");
    return ESP_OK;
}


static esp_err_t notecard_configure_hub(void)
{
    ESP_LOGI(TAG, "Configuring Notecard for Notehub connection...");

    // Create hub.set request
    J *req = NoteNewRequest("hub.set");
    if (req == NULL) {
        ESP_LOGE(TAG, "Failed to create hub.set request");
        return ESP_ERR_NO_MEM;
    }

    // Set Notehub product ID
    JAddStringToObject(req, "product", PROJECTUID);

    // set connection mode to minimum, meaning we have to manually sync
    // better for power
    JAddStringToObject(req, "mode", "minimum");

    // Send the request with retry
    bool success = NoteRequestWithRetry(req, 5);
    if (success) {
        ESP_LOGI(TAG, "Notecard configured for Notehub connection");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to configure Notecard");
        return ESP_FAIL;
    }
}

int getCurrentDay() {
    // Requesting time from the notecard
    J *req = NoteNewRequest("card.time");
    if (req != NULL) {
        J *rsp = NoteRequestResponse(req);
        if (rsp == NULL) {
            // response invalid
            ESP_LOGE(TAG, "Failed To Get Valid Current Time");
            return -1;
        } 
        // get UNIX timestamp
        long unixTime = (long)JGetNumber(rsp, "time");

        // free the response memory to prevent memory leaks
        NoteDeleteResponse(rsp);

        // time of 0 or negative means the Notecard hasn't completed first sync
        if (unixTime <= 0) {
            ESP_LOGE(TAG, "Notecard Hasn't Synced Yet");
            return -1;
        }
        
        // adjust for Malawi timezone (UTC+2 = +7200 seconds) so that
        // the day boundary aligns with local midnight, not UTC midnight
        unixTime += MALAWI_UTC_OFFSET;

        // divide by seconds-per-day to get an absolute day number
        return (int)(unixTime / 86400);

    } else {
        ESP_LOGE(TAG, "Failed To Request Time From Notecard");
        return -1;
    }
}



int get_distance() {
    uint8_t buf[4] = {0};

    // time our read started
    int64_t startTime = esp_timer_get_time(); // micro seconds

    while (esp_timer_get_time() - startTime < SENSOR_TIMEOUT_MS * 1000) {
        if (uart_read_bytes(UART_NUM_1, buf, 1, 
            pdMS_TO_TICKS(SENSOR_TIMEOUT_MS)) == 1 && buf[0] == HEADER_BYTE) {
                // try to read the remaining 3 bytes (high byte, low byte, checksum)
                if (uart_read_bytes(UART_NUM_1, &buf[1], 3, 
                    pdMS_TO_TICKS(SENSOR_TIMEOUT_MS)) == 3) {
            
                    // calculate expected sum of first 3 bytes (truncated to 8 bits)
                    if ((buf[0] + buf[1] + buf[2]) == buf[3]) {
                        return (buf[1] << 8) | buf[2];
                    } 

                    // keep scanning for another header byte
                }
            }
    }
    return ERROR_DISTANCE;
}

static void configure_sensor() {
    // initialize driver
    uart_driver_install(UART_NUM_1, 256, 0, 0, NULL, 0);

    // configs
    uart_config_t cfg = {
        .baud_rate = SENSOR_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    // set our configs
    uart_param_config(UART_NUM_1, &cfg);

    // set our pins (default pins)
    uart_set_pin(UART_NUM_1, SNS_TX_PIN, SNS_RX_PIN, UART_PIN_NO_CHANGE, 
        UART_PIN_NO_CHANGE);
}

void logReading() {
    // read the current distance from the ultrasonic sensor
    int dist = get_distance();

    if (dist == ERROR_DISTANCE) {
        ESP_LOGI(TAG, "Reading #%d: ERROR (no sensor response)", readingsCount + 1);
    } else {
        ESP_LOGI(TAG, "Reading #%d: %d mm", readingsCount + 1, dist);
    }

    // store the reading in the RTC memory array if there's room
    // even error readings (-1) are stored so we can see gaps in the data
    if (readingsCount < MAX_READINGS_PER_DAY) {
        readings[readingsCount] = dist;
        readingsCount++;
    } else {
        ESP_LOGW(TAG, "Readings array full (24 max)");
    }
}

void upload_data() {
    ESP_LOGI(TAG, "Uploading data...");

    if (readingsCount == 0) {
        ESP_LOGW(TAG, "No readings to upload");
        return;
    }

    J *req = NoteNewRequest("note.add");
    JAddStringToObject(req, "file", "daily.qo");

    J *body = JAddObjectToObject(req, "body");
    JAddNumberToObject(body, "readings", readingsCount);

    // add all our readings
    J *readings_arr = JAddArrayToObject(body, "day_data");
    for (int i = 0; i < readingsCount; i++) {
        JAddItemToArray(readings_arr, JCreateNumber(readings[i]));
    }

    // attempt to upload data
    J *rsp = NoteRequestResponse(req);

    if (rsp != NULL) {
        // check if the Notecard returned an error in the response JSON
        // this can happen if the Notecard's storage is full or other issues
        if (NoteResponseError(rsp)) {
            ESP_LOGE(TAG, "Upload failed: %s", JGetString(rsp, "err"));
            NoteDeleteResponse(rsp);
            // don't clear readings so we can retry on the next wake cycle
            return;
        }
        // free the response memory
        NoteDeleteResponse(rsp);

        // sync the notecard
        J *req = NoteNewRequest("hub.sync");
        J *syncRsp = NoteRequestResponse(req);
        if (syncRsp == NULL) {
            ESP_LOGE(TAG, "Could not sync");
        }
        if(syncrsp) NoteDeleteResponse(syncrsp);

        ESP_LOGI(TAG, "Note queued successfully");

        // clear the readings array now that data has been sent
        readingsCount = 0;
        // zero out the array so stale data doesn't persist
        memset(readings, 0, sizeof(readings));
        ESP_LOGI(TAG, "Upload complete");
    } else {
        // NULL response means the I2C communication itself failed
        // this could indicate a wiring issue or the Notecard being busy
        // don't clear readings so data is preserved for retry
        ESP_LOGE(TAG, "No response from Notecard, will retry next cycle");
    }

}



void app_main(void) {
    ESP_LOGI(TAG, "Wakeup #%d", readingsCount + 1);


    if (notecard_hardware_init() != ESP_OK)
        goto deep_sleep;

    // verify Notecard is alive
    J *rsp = NoteRequestResponse(NoteNewRequest("card.version"));
    if (rsp == NULL || NoteResponseError(rsp)) {
        ESP_LOGE(TAG, "Notecard Not Responding, Sleeping To Retry");
        if (rsp) NoteDeleteResponse(rsp);
        goto deep_sleep;
    }
    // free memory
    NoteDeleteResponse(rsp);

    // first boot, configure hub
    if (readingsCount == 0)
        notecard_configure_hub();

    // get current day
    int currentDay = getCurrentDay();
    if (currentDay < 0) {
        ESP_LOGE(TAG, "Time not available, sleeping to retry");
        goto deep_sleep;
    }

    // first day
    if (lastDay == -1)
        lastDay = currentDay;

    // upload new data for a new day
    if (currentDay != lastDay) {
        ESP_LOGI(TAG, "New day detected, uploading data");
        upload_data();
        lastDay = currentDay;
    }

    // get our sensor reading
    configure_sensor();
    logReading();
    uart_driver_delete(UART_NUM_1);

deep_sleep:
    esp_sleep_enable_timer_wakeup(SLEEP_DURATION_US);
    esp_deep_sleep_start();
}


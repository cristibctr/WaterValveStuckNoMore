#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_err.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <esp_netif.h>
#include <esp_log.h>
#include <esp_event.h>
#include <esp_wifi_default.h>
#include <esp_wifi.h>
#include <event_groups.h>
#include <esp32-hal-gpio.h>
#include "esp_sntp.h"
#include <time.h>

#define TAG "ValveActuator"
#define WIFI_SSID CONFIG_ESP_WIFI_SSID
#define WIFI_PASS CONFIG_ESP_WIFI_PASSWORD
#define WIFI_MAX_RETRY CONFIG_ESP_MAXIMUM_RETRY
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#ifndef TRIGGER_TIME
#define TRIGGER_TIME 2 // Hour of the day (0-23) to trigger the water valve
#endif

static esp_netif_t *netif = NULL;
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
static bool s_time_synchronized = false;

int32_t initWifi();
void connectWifi(const char* ssid, const char* password, wifi_auth_mode_t auth_mode);
void scanAndConnectToOpenWifi();
void toggleWater();
void ip_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
void time_sync_notification_cb(struct timeval *tv);
void initialize_sntp(void);

// Task to toggle the water valve at a specific time each day
void xTaskToggleWaterScheduler(void * pvParameter) {

  struct tm timeinfo;
  time_t now;
  // Wait briefly for time sync potentially happening very quickly after boot
  vTaskDelay(pdMS_TO_TICKS(10000));

  setenv("TZ", "EET-2EEST,M3.5.0/3,M10.5.0/4", 1); // Example: Eastern European Time
  tzset();

  for(;;) {
    time(&now);
    localtime_r(&now, &timeinfo);

    // Check if time is synchronized
    if (!s_time_synchronized || timeinfo.tm_year < (2025 - 1900)) { // Use a reasonable base year like 2025
        ESP_LOGW(TAG, "Time not synchronized. Toggling water now and waiting 24 hours.");
        toggleWater();
        // Delay exactly 24 hours
        vTaskDelay(pdMS_TO_TICKS(1000 * 60 * 60 * 24));
        continue; // Go back to the start of the loop to re-check time sync status
    }

    // --- Time is synchronized, proceed with scheduled toggle ---
    ESP_LOGI(TAG, "Time synchronized. Current time: %s", asctime(&timeinfo));

    // Calculate seconds until the next TRIGGER_TIME
    time_t trigger_time_today_t;
    struct tm trigger_timeinfo = timeinfo; // Copy current time structure
    trigger_timeinfo.tm_hour = TRIGGER_TIME;
    trigger_timeinfo.tm_min = 0;
    trigger_timeinfo.tm_sec = 0;
    trigger_time_today_t = mktime(&trigger_timeinfo); // Get epoch seconds for TRIGGER_TIME today

    int64_t delay_ms;
    if (now < trigger_time_today_t) {
        // Trigger time is later today
        delay_ms = (int64_t)(trigger_time_today_t - now) * 1000;
        ESP_LOGI(TAG, "Scheduling water toggle for today at %d:00. Waiting for %lld ms.", TRIGGER_TIME, delay_ms);
    } else {
        // Trigger time has passed for today, schedule for tomorrow
        trigger_timeinfo.tm_mday += 1; // Move to tomorrow
        mktime(&trigger_timeinfo); // Normalize the date (handles month/year rollovers)
        time_t trigger_time_tomorrow_t = mktime(&trigger_timeinfo);
        delay_ms = (int64_t)(trigger_time_tomorrow_t - now) * 1000;
        ESP_LOGI(TAG, "Scheduling water toggle for tomorrow at %d:00. Waiting for %lld ms.", TRIGGER_TIME, delay_ms);
    }

    if (delay_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }

    // Check time again right before triggering, in case of large clock adjustments
    time(&now);
    localtime_r(&now, &timeinfo);
    if (timeinfo.tm_hour == TRIGGER_TIME) {
        ESP_LOGI(TAG, "Triggering water valve now at %s", asctime(&timeinfo));
        toggleWater();
        // Delay slightly more than 24 hours to ensure we don't trigger twice
        // if the task wakes up slightly early next time.
        vTaskDelay(pdMS_TO_TICKS(1000 * 60 * 60 * 24 + 5000));
    } else {
        ESP_LOGW(TAG, "Woke up but time is not %d:00, recalculating delay.", TRIGGER_TIME);
        // Time might have jumped, loop will recalculate delay
    }
  }
}

// Toggles GPIO pin 12 HIGH for 2 seconds, then LOW.
void toggleWater() {
  gpio_set_level(GPIO_NUM_12, HIGH);
  vTaskDelay(pdMS_TO_TICKS(2000));
  gpio_set_level(GPIO_NUM_12, LOW);
}

// Task to manage Wi-Fi connection and initiate time synchronization.
void xTaskWifiConnect(void * pvParameters) {
  s_wifi_event_group = xEventGroupCreate();

  if (initWifi() == -1) {
    ESP_LOGE(TAG, "Failed to initialize Wi-Fi, aborting task.");
    vEventGroupDelete(s_wifi_event_group);
    vTaskDelete(NULL);
    return;
  }

  ESP_LOGI(TAG, "Wi-Fi initialization finished.");

  while(1) {
    ESP_LOGI(TAG, "Connecting to configured Wi-Fi: %s", WIFI_SSID);
    connectWifi(WIFI_SSID, WIFI_PASS, WIFI_AUTH_WPA2_PSK); // Try connecting to the known network

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE, // Clear bits on exit? No, handle manually.
            pdFALSE, // Wait for EITHER bit? Yes.
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Successfully connected to Wi-Fi.");
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT); // Clear the bit
        initialize_sntp(); // Initialize SNTP after connection
        // Connection successful, this task could suspend or periodically check connection status.
        // For now, just wait indefinitely or until disconnected.
        // Wait for disconnect event (handled by wifi_event_handler setting WIFI_FAIL_BIT)
         xEventGroupWaitBits(s_wifi_event_group, WIFI_FAIL_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
         ESP_LOGW(TAG, "Wi-Fi disconnected. Retrying connection process...");
         s_time_synchronized = false; // Reset time sync status on disconnect
         esp_sntp_stop(); // Stop SNTP
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGW(TAG, "Failed to connect to configured Wi-Fi: %s", WIFI_SSID);
        xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT); // Clear the bit

        ESP_LOGI(TAG, "Scanning for open Wi-Fi networks...");
        scanAndConnectToOpenWifi(); // Try scanning and connecting to an open network

        // Wait again for connection result after scan attempt
        bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

        if (bits & WIFI_CONNECTED_BIT) {
             ESP_LOGI(TAG, "Successfully connected to an open Wi-Fi network.");
             xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
             initialize_sntp();
             // Wait for disconnect
             xEventGroupWaitBits(s_wifi_event_group, WIFI_FAIL_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
             ESP_LOGW(TAG, "Wi-Fi disconnected. Retrying connection process...");
             s_time_synchronized = false;
             esp_sntp_stop();
        } else {
            ESP_LOGE(TAG, "Failed to connect to any network after scan.");
            xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT);
            // Wait before retrying the whole process
            vTaskDelay(pdMS_TO_TICKS(60000)); // Wait 60 seconds before retrying configured SSID
        }
    }
  }
}

// Connects to a specific Wi-Fi network
void connectWifi(const char* ssid, const char* password, wifi_auth_mode_t auth_mode) {
    s_retry_num = 0; // Reset retry counter for new connection attempt
    wifi_config_t wifi_config = {0}; // Initialize to zero
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);

    if (auth_mode != WIFI_AUTH_OPEN) {
        strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
        wifi_config.sta.threshold.authmode = auth_mode;
    } else {
         wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    }
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;


    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_LOGI(TAG, "Starting Wi-Fi connection attempt to SSID: %s", ssid);
    ESP_ERROR_CHECK(esp_wifi_start()); // Ensure Wi-Fi is started
    esp_wifi_connect(); // Start connection attempt (event handler manages retries)
}

// Scans for Wi-Fi networks and attempts to connect to the first open one found.
void scanAndConnectToOpenWifi() {
    ESP_ERROR_CHECK(esp_wifi_stop()); // Stop previous attempts/STA mode before scanning

    wifi_scan_config_t scan_config = {
        .ssid = 0,
        .bssid = 0,
        .channel = 0,
        .show_hidden = false
    };

    ESP_LOGI(TAG, "Starting Wi-Fi scan...");
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA)); // Ensure STA mode for scanning
    ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_config, true)); // Blocking scan

    uint16_t ap_count = 0;
    esp_wifi_scan_get_ap_num(&ap_count);
    ESP_LOGI(TAG, "Scan finished, found %d access points.", ap_count);

    if (ap_count == 0) {
        ESP_LOGW(TAG, "No networks found.");
        xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT); // Signal failure
        return;
    }

    wifi_ap_record_t *ap_list = (wifi_ap_record_t *)malloc(ap_count * sizeof(wifi_ap_record_t));
    if (ap_list == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for AP list");
        xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT); // Signal failure
        return;
    }

    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_count, ap_list));

    bool connected = false;
    for (int i = 0; i < ap_count; i++) {
        ESP_LOGD(TAG, "Found AP: %s, RSSI: %d, Auth: %d", (char*)ap_list[i].ssid, ap_list[i].rssi, ap_list[i].authmode);
        if (ap_list[i].authmode == WIFI_AUTH_OPEN) {
            ESP_LOGI(TAG, "Found open network: %s. Attempting connection.", (char*)ap_list[i].ssid);
            connectWifi((char*)ap_list[i].ssid, "", WIFI_AUTH_OPEN);
            connected = true; // Set flag indicating an attempt was made
            break; // Attempt connection to the first open network found
        }
    }

    free(ap_list);

    if (!connected) {
        ESP_LOGW(TAG, "No open networks found in scan results.");
        xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT); // Signal failure if no open AP was found to attempt connection
    }
    // If connected==true, the connection attempt was started.
    // The event handlers (wifi_event_handler, ip_event_handler) will signal success (WIFI_CONNECTED_BIT) or failure (WIFI_FAIL_BIT).
}


int32_t initWifi() {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ret = esp_netif_init();
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "TCP/IP initialization error");
    return -1;
  }

  ret = esp_event_loop_create_default();
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Event loop initialization error");
    return -1;
  }

  ret = esp_wifi_set_default_wifi_sta_handlers();
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to set default handlers");
    return -1;
  }

  netif = esp_netif_create_default_wifi_sta();
  if (netif == NULL)
  {
    ESP_LOGE(TAG, "Failed to create default WiFi STA interface");
    return -1;
  }

  // Register event handlers
  ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &ip_event_handler,
                                                        NULL,
                                                        NULL));

  return 1;
}

// Wi-Fi event handler
void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WIFI_EVENT_STA_START: Initiating connection...");
        // esp_wifi_connect() is called in connectWifi or scanAndConnectToOpenWifi
    } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WIFI_EVENT_STA_DISCONNECTED");
        if (s_retry_num < WIFI_MAX_RETRY) {
            esp_wifi_connect(); // Retry connection
            s_retry_num++;
            ESP_LOGI(TAG, "Retrying Wi-Fi connection (%d/%d)...", s_retry_num, WIFI_MAX_RETRY);
        } else {
            ESP_LOGE(TAG, "Connection failed after %d retries.", WIFI_MAX_RETRY);
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT); // Signal connection failure
        }
        s_time_synchronized = false; // Time is no longer valid
        esp_sntp_stop(); // Stop SNTP service if running
    }
}

// IP event handler
void ip_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        // Manually format the IP address to avoid potential macro issues
        esp_ip4_addr_t ip_addr = event->ip_info.ip;
        ESP_LOGI(TAG, "IP_EVENT_STA_GOT_IP: Got IP: %d.%d.%d.%d",
                 esp_ip4_addr1_16(&ip_addr),
                 esp_ip4_addr2_16(&ip_addr),
                 esp_ip4_addr3_16(&ip_addr),
                 esp_ip4_addr4_16(&ip_addr));
        s_retry_num = 0; // Reset retry counter on successful connection
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT); // Signal connection success
    }
}


// Callback function for SNTP synchronization events
void time_sync_notification_cb(struct timeval *tv) {
    ESP_LOGI(TAG, "SNTP: Time synchronized, system time set.");
    s_time_synchronized = true;

    // Log the synchronized time
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    ESP_LOGI(TAG, "Synchronized time: %s", asctime(&timeinfo));
}

// Initialize and start SNTP service
void initialize_sntp(void) {
    ESP_LOGI(TAG, "Initializing SNTP");
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org"); // Use NTP pool server
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    // esp_sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH); // Optional: use smooth time adjustment
    esp_sntp_init();
    ESP_LOGI(TAG, "SNTP initialized, waiting for time sync...");
}


// Main application entry point
void app_main() {
  // Initialize GPIO Pin 12 for output
  gpio_pad_select_gpio(GPIO_NUM_12);
  gpio_set_direction(GPIO_NUM_12, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_NUM_12, LOW);

  // Initial toggle for testing
  ESP_LOGI(TAG, "Performing initial valve toggle for testing.");
  toggleWater();

  // Start tasks
  xTaskCreate(xTaskWifiConnect, "wifi_connect", 4096, NULL, 5, NULL); // Increased stack size for Wi-Fi scan
  xTaskCreate(xTaskToggleWaterScheduler, "water_sched", 4096, NULL, 4, NULL); // Increased stack size for time calculations
}

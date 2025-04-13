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

#define TAG "ValveActuator"
#define WIFI_SSID "ValveActuator"
#define WIFI_PASS "ValveActuator"
#define WIFI_MAX_RETRY 3
#define WIFI_FAIL_BIT      BIT1

#ifndef TRIGGER_TIME
#define TRIGGER_TIME 2
#endif

static TaskHandle_t xTaskNotify = NULL;
static esp_netif_t *netif = NULL;
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

int32_t initWifi();
void connectWifi();
void toggleWater(void * pvParameter);
void ip_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

// Create the task that toggles the pin12 -- ie. transistor. At first it retrieves the time from flash memory and only toggles that pin for 2 seconds at a specific time of day.
void xTaskToggleWaterScheduler(void * pvParameter) {

  struct tm timeinfo;
  time_t now;
  setenv("TZ", "EET-2EEST,M3.5.0/3,M10.5.0/4", 1);
  tzset();
  for(;;) {
    toggleWater();
    time(&now);
    localtime_r(&now, &timeinfo);
    if (timeinfo.tm_year < (2023 - 1900)) {
      ESP_LOGW(TAG, "Time not yet set correctly?");
      TickType_t xLastWakeTime = xTaskGetTickCount();
      // trigger every 24 hours minus the two seconds it takes to be triggered
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000*60*60*24) - pdMS_TO_TICKS(2000));
    } else {
      // TODO: vTaskDelayUntil current time is equal to TRIGGER_TIME military time
    }
  }
}

void toggleWater() {
  gpio_set_level(GPIO_NUM_12, HIGH);
  vTaskDelay(pdMS_TO_TICKS(2000));
  gpio_set_level(GPIO_NUM_12, LOW);
}

// Create a task that tries to connect to any free wifi nearby or a specific list of ssids with passwords. Once connected notify the time task (semaphore or rtos task notification).
void xTaskWifiConnect(void * pvParameters) {
  configASSERT( xTaskNotify == NULL );
  s_wifi_event_group = xEventGroupCreate();
  xTaskNotify = xTaskGetCurrentTaskHandle();
  xTaskCreate(xTaskTimeRetrieve, "time", 2048, NULL, 3, NULL);

  if (initWifi() == -1)
    return;

  ESP_LOGI(TAG, "Wi-Fi initialization finished.");

  for(;;) {
    // connect to wifi
    // TODO: Try to connect to the known wifi the first time and then if WIFI_FAIL_BIT event is triggered scan all other networks and connect to any available one without a password
    connectWifi();
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
      WIFI_FAIL_BIT,
      pdFALSE,
      pdFALSE,
      portMAX_DELAY);
      
  }
}

void connectWifi() {
  wifi_config_t wifi_config = {
      .sta = {
          .ssid = WIFI_SSID,
          .password = WIFI_PASS,
          .threshold.authmode = WIFI_AUTH_WPA2_PSK,
      },
  };
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());
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

  ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL, NULL));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

  return 1;
}

void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Wi-Fi started, trying to connect...");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retry Wi-Fi connection (%d/%d)", s_retry_num, WIFI_MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "Connection failed after maximum retries.");
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xTaskNotifyGive(xTaskNotify);
    }
}

// Create a task that tries to receive the utc time. Once retrieved it stores it into flash and releases the semaphore.
void xTaskTimeRetrieve(void * pvParameters) {
  configASSERT( xTaskNotify != NULL);
  uint32_t ulNotificationValue = ulTaskNotifyTake(1, portMAX_DELAY);
  if(ulNotificationValue == 1) {
    // TODO: retrieve time
  }
}

// Create the main function that starts the gpio task and the wifi task
void app_main() {
  xTaskCreate(xTaskWifiConnect, "wifi", 2048, NULL, 4, NULL);
  xTaskCreate(xTaskToggleWaterScheduler, "water", 2048, NULL, 5, NULL);
}
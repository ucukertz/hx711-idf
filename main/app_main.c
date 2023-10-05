#include <stdio.h>
#include <sdkconfig.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_spi_flash.h>
#include <esp_log.h>

#include "hx711.h"

static const char* TAG = "HX711";

static hx711_t hx711;

const gpio_num_t GPIO_HX711_MISO    = GPIO_NUM_19;
const gpio_num_t GPIO_HX711_SCK     = GPIO_NUM_18;
static const uint32_t HX711_CALIB_DEVIATION_MAX = 20000;
static const uint8_t HX711_CALIB_AVG_READ = 10;

void app_main(void)
{
  hx711_init(&hx711, GPIO_HX711_MISO, GPIO_HX711_SCK, HX711_GAIN_128);
  hx711_set_max_deviation(&hx711, HX711_CALIB_DEVIATION_MAX);
  hx711_set_wait_timeout(&hx711, 1000);

  while (1) {
    ESP_LOGI(TAG, "Reading...");
    int32_t read_avg = hx711_read_avg(&hx711, HX711_CALIB_AVG_READ);
    if (read_avg == -1) {
      ESP_LOGE(TAG, "ADC value read failed");
      vTaskDelay(pdMS_TO_TICKS(5000));
      continue;
    }
    ESP_LOGI(TAG, "ADC avg: %d", read_avg);
  }
}

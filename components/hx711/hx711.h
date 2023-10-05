#ifndef __HX711_H
#define __HX711_H

#include <driver/gpio.h>
#include <stdbool.h>
#include <esp_err.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>

#include <esp32/rom/ets_sys.h>

#ifdef __cplusplus
extern "C" {
#endif

// Gain/channel
typedef enum {
  HX711_GAIN_128 = 1, // Channel A
  HX711_GAIN_32, // Channel B
  HX711_GAIN_64 // Channel A
} hx711_gain_t;

// HX711 device descriptor
typedef struct {
  gpio_num_t miso;
  gpio_num_t sck;
  hx711_gain_t gain;
  uint32_t timeout_ms;
  uint32_t max_deviation;
  uint8_t max_fail;
} hx711_t;

bool hx711_init(hx711_t* hx711, gpio_num_t miso, gpio_num_t sck, hx711_gain_t gain);
bool hx711_set_gain(hx711_t* hx711, hx711_gain_t gain);
bool hx711_is_ready(hx711_t* hx711);

void hx711_set_wait_timeout(hx711_t* hx711, uint32_t timeout_ms);
void hx711_set_max_deviation(hx711_t* hx711, uint32_t max_deviation);
void hx711_set_max_fail(hx711_t* hx711, uint32_t max_fail);
bool hx711_wait(hx711_t* hx711, size_t timeout_ms);

uint32_t hx711_read_raw(hx711_t* hx711);
int32_t hx711_read_data(hx711_t* hx711);
int32_t hx711_read_avg(hx711_t* hx711, uint16_t read_n);

#ifdef __cplusplus
}
#endif

#endif // __HX711_H
#include "hx711.h"

static const uint32_t TIMEOUT_MIN_MS = portTICK_RATE_MS;
static const uint32_t DEFAULT_TIMEOUT_MS = 200;
static const uint8_t DEFAULT_MAX_FAIL = 10;
static const uint32_t DEFAULT_MAX_DEVIATION = 300000;

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

/**
 * @brief Initialize HX711 device descriptor for sensor reading
 * @param hx711 HX711 device descriptor
 * @param miso GPIO number of HX711 MISO
 * @param sck GPIO number of HX711 SCK
 * @param gain Gain used for reading
 * @return true when initialization is successful
 */
bool hx711_init(hx711_t* hx711, gpio_num_t miso, gpio_num_t sck, hx711_gain_t gain)
{
  hx711->miso = miso;
  hx711->sck = sck;
  gpio_set_direction(hx711->miso, GPIO_MODE_INPUT);
  gpio_set_direction(hx711->sck, GPIO_MODE_OUTPUT);

  gpio_set_level(hx711->sck, 0);

  hx711->gain = gain;
  hx711->timeout_ms = DEFAULT_TIMEOUT_MS;
  hx711->max_deviation = DEFAULT_MAX_DEVIATION;
  hx711->max_fail = DEFAULT_MAX_FAIL;

  // Protect from invalid configuration
  if (hx711->gain < 1 || hx711->gain > HX711_GAIN_64) hx711->gain = HX711_GAIN_64;

  return hx711_set_gain(hx711, hx711->gain);
}

/**
 * @brief Set gain used for reading HX711
 * @param hx711 HX711 device descriptor
 * @param gain Gain used for reading
 * @return true when changing gain is successful
 */
bool hx711_set_gain(hx711_t* hx711, hx711_gain_t gain)
{
  if (!hx711_wait(hx711, hx711->timeout_ms)) return false;

  hx711_read_raw(hx711);
  hx711->gain = gain;

  return true;
}

/**
 * @brief Check whether HX711 is ready for reading
 * @param hx711 HX711 device descriptor
 * @return true if it is ready
 */
bool hx711_is_ready(hx711_t* hx711)
{
  if (!gpio_get_level(hx711->miso)) return true;

  return false;
}

/**
 * @brief Set wait timeout for HX711 reading
 * @param hx711 HX711 device descriptor
 * @param timeout_ms timeout in milliseconds
 */
void hx711_set_wait_timeout(hx711_t* hx711, uint32_t timeout_ms)
{
  if (timeout_ms < TIMEOUT_MIN_MS) timeout_ms = TIMEOUT_MIN_MS;
  hx711->timeout_ms = timeout_ms;
}

/**
 * @brief Set maximum acceptable deviation when reading average HX711 values. Reading HX711 average value will fail if this value is exceeded.
 * @param hx711 HX711 device descriptor
 * @param max_deviation Maximum allowed deviation
 */
void hx711_set_max_deviation(hx711_t* hx711, uint32_t max_deviation)
{
  hx711->max_deviation = max_deviation;
}

/**
 * @brief Set maximum acceptable failures when reading average HX711 values. Reading HX711 average value will fail if this value is exceeded.
 * @param hx711 HX711 device descriptor
 * @param max_fail Maximum allowed failures
 */
void hx711_set_max_fail(hx711_t* hx711, uint32_t max_fail)
{
  hx711->max_fail = max_fail;
}

/**
 * @brief Wait for HX711 readiness
 * @param hx711 HX711 device descriptor
 * @param timeout_ms Maximum time to wait in milliseconds
 * @return true when HX711 is ready within set timeout
 */
bool hx711_wait(hx711_t* hx711, size_t timeout_ms)
{
  uint64_t start_ms = esp_timer_get_time()/1000;
  while (esp_timer_get_time()/1000 - start_ms < timeout_ms)
  {
    if (hx711_is_ready(hx711)) return true;
    vTaskDelay(portTICK_RATE_MS);
  }

  return false;
}

/**
 * @brief Read one raw HX711 value
 * @param hx711 HX711 device descriptor
 * @return Raw HX711 value
 */
uint32_t hx711_read_raw(hx711_t* hx711)
{
  portENTER_CRITICAL(&mux);

  // Data shift-in
  uint32_t data = 0;
  for (size_t i = 0; i < 24; i++)
  {
    gpio_set_level(hx711->sck, 1);
    ets_delay_us(1);
    data |= gpio_get_level(hx711->miso) << (23 - i);
    gpio_set_level(hx711->sck, 0);
    ets_delay_us(1);
  }

  // Gain and channel configuration for next reading
  for (size_t i = 0; i < hx711->gain; i++)
  {
    gpio_set_level(hx711->sck, 1);
    ets_delay_us(1);
    gpio_set_level(hx711->sck, 0);
    ets_delay_us(1);
  }

  portEXIT_CRITICAL(&mux);

  return data;
}

/**
 * @brief Read one parsed HX711 value
 * @param hx711 HX711 device descriptor
 * @return Parsed HX711 value
 */
int32_t hx711_read_data(hx711_t* hx711)
{
  uint32_t raw = hx711_read_raw(hx711);

  if (raw & 0x800000) raw |= 0xff000000;
  int32_t data = *((int32_t *)&raw);

  return data;
}

/**
 * @brief Read average parsed HX711 values
 * @param hx711 HX711 device descriptor
 * @param read_n Number of readings to be averaged
 * @return Averaged parsed HX711 values or -1 on failure
 */
int32_t hx711_read_avg(hx711_t* hx711, uint16_t read_n)
{
  int32_t data_total = 0;
  int32_t data_last_valid;
  int32_t data_last;
  uint8_t cnt_fail = 0;
  bool first_read = 1;
  bool last_valid_set = 0;

  for (uint16_t i = 0; i < read_n+1; i++) {
    bool ready = hx711_wait(hx711, hx711->timeout_ms);
    if (!ready) return -1;
    data_last = hx711_read_data(hx711);

     // Discard obvious invalid data
    if (data_last == -1 || data_last == 0) {
      cnt_fail++;
      if (cnt_fail >= hx711->max_fail) return -1;
      i--;
      continue;
    }

    // Discard data if it deviates too much from last valid reading
    if (last_valid_set) {
      if (data_last > data_last_valid+hx711->max_deviation ||
        data_last < data_last_valid-hx711->max_deviation) {

        cnt_fail++;
        if (cnt_fail >= hx711->max_fail) return -1;
        i--;
        continue;
      }
    }

    // Discard data of first reading because HX711 may just woke up and gives faulty data
    if (!first_read) {
      data_total += data_last;
      data_last_valid = data_last;
      last_valid_set = 1;
    }
    first_read = 0;
  }

  return data_total/read_n;
}
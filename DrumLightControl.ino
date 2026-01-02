#include <Arduino.h>
#include "driver/i2s.h"

#define GPIO4  ((gpio_num_t) D2)
#define GPIO5  ((gpio_num_t) D3)
#define GPIO6  ((gpio_num_t) D4)
#define GPIO7  ((gpio_num_t) D5)
#define GPIO3  ((gpio_num_t) D1)
#define GPIO10 ((gpio_num_t) D10)
#define GPIO8  ((gpio_num_t) D8)

// ================== I2S PIN CONFIG ==================
#define I2S_BCLK_PIN   GPIO3
#define I2S_WS_PIN     GPIO4
#define I2S_DATA_PIN   GPIO10

// ================== DETECTOR CONFIG =================
#define SAMPLE_RATE        4000
#define BUFFER_SAMPLES     64       // Reduced from 128 for snappier response
#define PEAK_THRESHOLD     1000
#define HOLD_TIME_MS       200
#define TRIGGER_PLOT_LEVEL PEAK_THRESHOLD

#define I2S_PORT I2S_NUM_0

int16_t sampleBuffer[BUFFER_SAMPLES];

// Detector state
bool triggerActive = false;
unsigned long lastTriggerTime = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = BUFFER_SAMPLES,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCLK_PIN,
    .ws_io_num = I2S_WS_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_DATA_PIN
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
  i2s_zero_dma_buffer(I2S_PORT);
}

void loop() {
  size_t bytesRead = 0;

  i2s_read(
    I2S_PORT,
    sampleBuffer,
    sizeof(sampleBuffer),
    &bytesRead,
    portMAX_DELAY
  );

  int samplesRead = bytesRead / sizeof(int16_t);
  int16_t peak = 0;

  // Peak detection
  for (int i = 0; i < samplesRead; i++) {
    int16_t v = sampleBuffer[i];
    if (v < 0) v = -v;
    if (v > peak) peak = v;
  }

  // Envelope
  static int16_t lastPeak = 0;
  peak = (peak * 3 + lastPeak) / 4;
  lastPeak = peak;

  unsigned long now = millis();

  // Threshold logic with holdoff
  if (peak > PEAK_THRESHOLD) {
    if (!triggerActive && (now - lastTriggerTime > HOLD_TIME_MS)) {
      triggerActive = true;
      lastTriggerTime = now;
    }
  } else {
    triggerActive = false;
  }

  // ================== SERIAL PLOTTER OUTPUT ==================
  Serial.print(peak);
  Serial.print(",");
  Serial.print(triggerActive ? TRIGGER_PLOT_LEVEL : 0);
  Serial.print(",");
  Serial.println(PEAK_THRESHOLD);
}


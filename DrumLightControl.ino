#include <Arduino.h>
#include "driver/i2s_std.h"

// ================== I2S PIN CONFIG ==================
#define I2S_BCLK_PIN   GPIO_NUM_3
#define I2S_WS_PIN     GPIO_NUM_4
#define I2S_DATA_PIN   GPIO_NUM_10

// ================== DETECTOR CONFIG =================
#define SAMPLE_RATE        4000
#define BUFFER_SAMPLES     64       // Reduced from 128 for snappier response
#define PEAK_THRESHOLD     1000
#define HOLD_TIME_MS       200
#define TRIGGER_PLOT_LEVEL PEAK_THRESHOLD

#define I2S_PORT I2S_NUM_0

// ================== I2S Channel Handler and Buffer =================
int32_t sampleBuffer[BUFFER_SAMPLES];
i2s_chan_handle_t rx_chan;

// Detector state
bool triggerActive = false;
unsigned long lastTriggerTime = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Channel
  i2s_chan_config_t chan_cfg =
    I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  i2s_new_channel(&chan_cfg, NULL, &rx_chan);

  // Clock: same rate you used successfully (e.g. 4 kHz)
  i2s_std_clk_config_t clk_cfg =
    I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE);

  // SLOT CONFIG — THIS IS THE KEY LINE
  i2s_std_slot_config_t slot_cfg =
    I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
        I2S_DATA_BIT_WIDTH_32BIT,   // 24-bit data in 32-bit slot
        I2S_SLOT_MODE_MONO
        );

  // Select LEFT channel (INMP441 L/R pin = GND)
  slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

  i2s_std_gpio_config_t gpio_cfg = {
    .mclk = I2S_GPIO_UNUSED,
    .bclk = I2S_BCLK_PIN,
    .ws   = I2S_WS_PIN,
    .dout = I2S_GPIO_UNUSED,
    .din  = I2S_DATA_PIN,
  };

   i2s_std_config_t std_cfg = {
        .clk_cfg  = clk_cfg,
        .slot_cfg = slot_cfg,
        .gpio_cfg = gpio_cfg,
    };

    i2s_channel_init_std_mode(rx_chan, &std_cfg);
    i2s_channel_enable(rx_chan);

}

void loop() {
  size_t bytesRead = 0;

  i2s_channel_read(
    rx_chan,
    sampleBuffer,
    sizeof(sampleBuffer),
    &bytesRead,
    portMAX_DELAY
  );

  int samplesRead = bytesRead / sizeof(sampleBuffer[0]);
  int32_t peak = 0;

  // Peak detection
  for (int i = 0; i < samplesRead; i++) {

    int32_t v = sampleBuffer[i];

    // 24-bits resolution is alot. Toss the 8 least significant bits
    v >>= 16;

    if (v < 0) v = -v;
    if (v > peak) peak = v;
  }

  // Envelope
  static int32_t lastPeak = 0;
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


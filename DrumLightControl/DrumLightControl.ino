#include <Arduino.h>
#include "driver/i2s_std.h"
#include <FastLED.h>

// ================== SENSITIVITY KNOBS ==================
#define PEAK_THRESHOLD     1000
#define HOLD_TIME_MS       50

// ================== PIN CONFIG ==================
#define I2S_BCLK_PIN   GPIO_NUM_3
#define I2S_WS_PIN     GPIO_NUM_4
#define I2S_DATA_PIN   GPIO_NUM_10
#define LED_PIN        GPIO_NUM_8

// ================== I2S DETECTOR CONFIG =================
#define SAMPLE_RATE        4000
#define BUFFER_SAMPLES     64
#define TRIGGER_PLOT_LEVEL PEAK_THRESHOLD

// ================= LED CONFIG =================

#define NUM_LEDS      54
#define LED_TYPE      WS2812B
#define COLOR_ORDER   GRB

#define MAX_BRIGHTNESS 128     // Max is 255. BEWARE, too high is dangerous
#define DECAY_FACTOR   0.50f   // Smaller is faster
#define LED_UPDATE_MS  5       // Smaller is smoother

// ================= LED STATE =================

float currentBrightness = 0.0f;
CRGB leds[NUM_LEDS];

// ================== I2S STATE  =================

int32_t sampleBuffer[BUFFER_SAMPLES];
i2s_chan_handle_t rx_chan;
bool triggerActive = false;
unsigned long lastTriggerTime = 0;

// ================================================================================
// Code Begin
// ================================================================================

void setup()
{
  Serial.begin(115200);
  delay(1000);

  // ======================================================================
  // I2S Init

  i2s_chan_config_t chan_cfg =
    I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  i2s_new_channel(&chan_cfg, NULL, &rx_chan);

  i2s_std_clk_config_t clk_cfg =
    I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE);

  i2s_std_slot_config_t slot_cfg =
    I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
        I2S_DATA_BIT_WIDTH_32BIT,
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

  // ======================================================================
  // LED Init
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(MAX_BRIGHTNESS);
}

void loop()
{
  unsigned long now = millis();
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

  // Threshold logic with holdoff
  if (peak > PEAK_THRESHOLD) {
    if (!triggerActive && (now - lastTriggerTime > HOLD_TIME_MS)) {
      triggerActive = true;
      lastTriggerTime = now;

      // Threshold detected --> Do the LED stimulus action
      LED_Update_Stimulus();
    }
  } else {
    triggerActive = false;
  }

  // ================== BRIGHTNESS DECAY ==================
  LED_Update_Loop( now );

  // ================== SERIAL PLOTTER OUTPUT ==================
  Serial.print(peak);
  Serial.print(",");
  Serial.print(triggerActive ? TRIGGER_PLOT_LEVEL : 0);
  Serial.print(",");
  Serial.println(PEAK_THRESHOLD);
}

// ================================================================================
// LED Update Routines
// ================================================================================

/*
 *  This is called once when the drum hit is detected
 */
void LED_Update_Stimulus()
{
  currentBrightness = MAX_BRIGHTNESS;

  // Max for each is 255
  uint8_t red   = 240;
  uint8_t green = 30;
  uint8_t blue  = 20;

  // Try other colors from here:
  // https://fastled.io/docs/d7/d82/struct_c_r_g_b_aeb40a08b7cb90c1e21bd408261558b99.html#aeb40a08b7cb90c1e21bd408261558b99
  fill_solid(leds, NUM_LEDS, CRGB( red, green, blue ));
}

/*
 *  This is called once per loop
 */
void LED_Update_Loop( unsigned long now )
{
  static unsigned long lastUpdateMs = 0;

  // Exponential brightness decay
  if (now - lastUpdateMs >= LED_UPDATE_MS) {
    lastUpdateMs = now;

    if (currentBrightness > 1.0f) {
      currentBrightness *= DECAY_FACTOR;
    } else {
      currentBrightness = 0.0f;
    }

    FastLED.setBrightness((uint8_t)currentBrightness);
    FastLED.show();
  }
}

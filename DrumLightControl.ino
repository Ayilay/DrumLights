#include <Arduino.h>
#include "driver/i2s_std.h"

#include <FastLED.h>


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
#define LED_PIN        GPIO8

// ================= LED CONFIG =================
#define NUM_LEDS       50
#define LED_TYPE       WS2812B
#define COLOR_ORDER    GRB
#define MAX_BRIGHTNESS 80
#define DECAY_FACTOR   0.90f   // 0.90 = fast decay, 0.98 = slow decay
#define UPDATE_MS      20      // decay update rate

// ================== DETECTOR CONFIG =================
#define SAMPLE_RATE        4000
#define PEAK_THRESHOLD     1000
#define HOLD_TIME_MS       100
#define TRIGGER_PLOT_LEVEL 12000
#define BUFFER_SAMPLES     64       // 64 for faster refresh
//#define SMOOTH_MIC_TRACE            // Enable envelope for peak detection

// Global state
unsigned long now = millis();

// Detector state
int16_t sampleBuffer[BUFFER_SAMPLES];
i2s_chan_handle_t rx_chan;
bool triggerActive = false;
unsigned long lastTriggerTime = 0;

#define PROCESS_DIV 10     // effectively ~800 Hz processing

int blockCounter = 0;     // counts blocks for decimation

/* ================= LED STATE ================= */
CRGB leds[NUM_LEDS];
float currentBrightness = 0.0f;
uint32_t lastUpdateMs = 0;


void setup() {
  Serial.begin(115200);
  delay(1000);

  // -------- I2S CHANNEL --------
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  i2s_new_channel(&chan_cfg, NULL, &rx_chan);

  // -------- STANDARD MODE --------
  i2s_std_config_t std_cfg = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
    std_cfg.slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(
                     I2S_DATA_BIT_WIDTH_16BIT,
                     I2S_SLOT_MODE_MONO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = I2S_BCLK_PIN,
      .ws   = I2S_WS_PIN,
      .dout = I2S_GPIO_UNUSED,
      .din  = I2S_DATA_PIN,
      .invert_flags = {
        .mclk_inv = false,
        .bclk_inv = false,
        .ws_inv   = false,
      },
    },
  };

  i2s_channel_init_std_mode(rx_chan, &std_cfg);
  i2s_channel_enable(rx_chan);
  Serial.println( "I2S init done" );


  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(MAX_BRIGHTNESS);
  Serial.println( "LED Init done" );
}

/* ================= STIMULUS ================= */

void stimulusEvent() {
  currentBrightness = MAX_BRIGHTNESS;

  // Set desired stimulus color here
  fill_solid(leds, NUM_LEDS, CRGB::HotPink);
}

void process_every_10th_block()
{
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


  int samplesRead = bytesRead / sizeof(int16_t);
  int16_t peak = 0;
  now = millis();

  // CUSTOM Decimator... Increment block counter
  //blockCounter++;
  //if (blockCounter >= PROCESS_DIV) {
  //  blockCounter = 0;           // reset counter
  //  process_every_10th_block(); // run detector at ~1/10th rate
  //}

  // Peak detection
  for (int i = 0; i < samplesRead; i++) {
    int16_t v = ((uint32_t)sampleBuffer[i]) >> 16;
    if (v < 0) v = -v;
    if (v > peak) peak = v;
  }

// Per ChatGPT: Add simple envelope to smooth out the mic trace
#ifdef SMOOTH_MIC_TRACE
  static int16_t lastPeak = 0;
  peak = (peak * 3 + lastPeak) / 4;
  lastPeak = peak;
#endif

  // Threshold logic with holdoff
  if (peak > PEAK_THRESHOLD) {
    if (!triggerActive && (now - lastTriggerTime > HOLD_TIME_MS)) {
      triggerActive = true;
      lastTriggerTime = now;
      stimulusEvent();
    }
  } else {
    triggerActive = false;
  }


  /* ----- Brightness decay ----- */
  #if 1
  if (now - lastUpdateMs >= UPDATE_MS) {
    lastUpdateMs = now;

    if (currentBrightness > 1.0f) {
      currentBrightness *= DECAY_FACTOR;
    } else {
      currentBrightness = 0.0f;
    }

    FastLED.setBrightness((uint8_t)currentBrightness);
    FastLED.show();
  }
  #endif

  // ================== SERIAL PLOTTER OUTPUT ==================
  Serial.print(peak);
  Serial.print(",");
  Serial.print(triggerActive ? TRIGGER_PLOT_LEVEL : 0);
  Serial.print(",");
  Serial.println(PEAK_THRESHOLD);
}

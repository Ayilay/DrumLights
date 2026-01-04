#include <Arduino.h>
#include "driver/i2s_std.h"
#include <FastLED.h>

// ================== SENSITIVITY KNOBS ==================
#define HOLD_TIME_MS       50

// ================== PIN CONFIG ==================
#define I2S_BCLK_PIN   GPIO_NUM_3
#define I2S_WS_PIN     GPIO_NUM_4
#define I2S_DATA_PIN   GPIO_NUM_10
#define LED_PIN        GPIO_NUM_8

// ================== I2S DETECTOR CONFIG =================
#define SAMPLE_RATE        4000
#define BUFFER_SAMPLES     64
#define TRIGGER_PLOT_LEVEL settings.thresh

// ================= LED CONFIG =================

#define NUM_LEDS      54
#define LED_TYPE      WS2812B
#define COLOR_ORDER   GRB

#define MAX_BRIGHTNESS 128     // Max is 255. BEWARE, too high is dangerous
#define DECAY_FACTOR   0.50f   // Smaller is faster
#define LED_UPDATE_MS  5       // Smaller is smoother

//------------------------------------------------------------
//  CLI Types and Structs
//------------------------------------------------------------

typedef void (*command_fn_t)(const char *arg, uintptr_t cookie);

struct cli_command_t {
  const char*  name;
  const char*  help;
  command_fn_t handler;
  uintptr_t    cookie;
};

struct settable_vars {
  bool     stream;
  uint32_t thresh;

  uint32_t red;
  uint32_t green;
  uint32_t blue;
} settings;


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

void init_default_settings()
{
  memset( &settings, 0, sizeof(settings) );

  // Hot Pink:
  //    0xff = 255
  //    0x69 = 105
  //    0xb4 = 180

  settings.red   = 0xff;
  settings.green = 0x69;
  settings.blue  = 0xb4;

  settings.thresh = 1000;
  settings.stream = false;
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  init_default_settings();

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
  if (peak > settings.thresh) {
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
  if( settings.stream )
  {
    Serial.print(peak);
    Serial.print(",");
    Serial.print(triggerActive ? TRIGGER_PLOT_LEVEL : 0);
    Serial.print(",");
    Serial.println(settings.thresh);
  }
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
  uint8_t red   = settings.red;
  uint8_t green = settings.green;
  uint8_t blue  = settings.blue;

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

//------------------------------------------------------------
//  Command Table and Prototypes
//------------------------------------------------------------

// Command Handler Prototypes
void cmd_help(const char *arg, uintptr_t cookie);
void cmd_stim(const char *arg, uintptr_t cookie);
void cmd_color(const char *arg, uintptr_t cookie);
void cmd_thresh(const char *arg, uintptr_t cookie);
void cmd_stream(const char *arg, uintptr_t cookie);

enum color {
  COLOR_RED = 0,
  COLOR_GRN = 1,
  COLOR_BLU = 2,
};

#define CLI_BLANK \
  { "", "", NULL, NULL }

cli_command_t commands[] = {
  { "help","          Show this help text",    cmd_help, NULL },


  CLI_BLANK,
  { "stim","   Stimulate the LEDs", cmd_stim,  NULL },
  { "stream"," <on|off> Enable/Disable data stream", cmd_stream,  NULL },

  { "thresh"," [val] Get/Set the microphone threshold", cmd_thresh, NULL },
  { "red","    [val] Get/Set the red value",   cmd_color, COLOR_RED },
  { "grn","    [val] Get/Set the green value", cmd_color, COLOR_GRN },
  { "blu","    [val] Get/Set the blue value",  cmd_color, COLOR_BLU },
};

//------------------------------------------------------------
//  CLI Internal State
//------------------------------------------------------------

constexpr size_t NUM_COMMANDS = sizeof(commands) / sizeof(commands[0]);

#define CLI_BUF_SIZE 64
char cli_buf[CLI_BUF_SIZE];
size_t cli_cursor = 0;

//------------------------------------------------------------
//  CLI Processor Functions
//------------------------------------------------------------

void cli_process_line(char *line) {
  // Split command and arguments
  char *cmd = strtok(line, " ");
  char *arg = strtok(NULL, "");

  if (!cmd) return;

  for (size_t i = 0; i < NUM_COMMANDS; i++) {
    if (strlen(cmd) > 0 && strcmp(cmd, commands[i].name) == 0) {
      uintptr_t cookie = commands[i].cookie;
      commands[i].handler(arg, cookie);
      return;
    }
  }

  Serial.println("Unknown command. Type 'help'.");
}

void cli_poll() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r')
      continue;

    if (c == '\n') {
      cli_buf[cli_cursor] = '\0';
      cli_process_line(cli_buf);
      cli_cursor = 0;
    } else if (cli_cursor < CLI_BUF_SIZE - 1) {
      cli_buf[cli_cursor++] = c;
    }
  }
}

//------------------------------------------------------------
//  Command Handlers
//------------------------------------------------------------

void cmd_help(const char *arg, uintptr_t cookie) {
  Serial.println("Available commands:");
  for (size_t i = 0; i < NUM_COMMANDS; i++) {
    Serial.print("  ");
    Serial.print(commands[i].name);
    Serial.println(commands[i].help);
  }

  Serial.println();
}

void cmd_stim(const char *arg, uintptr_t cookie) {
  LED_Update_Stimulus();
}

void cmd_stream(const char *arg, uintptr_t cookie) {

  if( ! arg ){
    Serial.println( "cmd_stream requires 'on' or 'off'" );
    return;
  }

  if( 0 == strcmp( arg, "on" ) )
  {
    Serial.println( "Enable data stream" );
    settings.stream = true;
  }
  else if( 0 == strcmp( arg, "on" ) )
  {
    Serial.println( "Disable data stream" );
    settings.stream = false;
  }
  else
  {
    Serial.print( "Unrecognized option " );
    Serial.println( arg );
  }
}

void cmd_color(const char *arg, uintptr_t cookie) {

  // No arg: print the value and exit
  if( ! arg ){
    uint32_t val;
    switch((enum color) cookie) {
      case COLOR_RED:
        val = settings.red;
        Serial.print( "red: " );
        Serial.println( val );
        break;

      case COLOR_GRN:
        val = settings.green;
        Serial.print( "green: " );
        Serial.println( val );
        break;

      case COLOR_BLU:
        val = settings.blue;
        Serial.print( "blue: " );
        Serial.println( val );
        break;

      default:
        break;
    }
    return;
  }

  // We accept 0x hes format AND decimal format
  uint32_t val = strtol(arg, NULL, 0);
  if( val > 255 ){
    Serial.print( "invalid num " );
    Serial.print( val );
    Serial.println( ". Must be wthin 0-255" );
    return;
  }

  Serial.print( "Setting " );
  switch((enum color) cookie) {
    case COLOR_RED:
      settings.red = val;
      Serial.print( "red " );
      break;

    case COLOR_GRN:
      settings.green = val;
      Serial.print( "green " );
      break;

    case COLOR_BLU:
      settings.blue = val;
      Serial.print( "blue " );
      break;

    default:
      break;
  }
  Serial.print( "to " );
  Serial.println( val);
}

void cmd_thresh(const char *arg, uintptr_t cookie) {
  Serial.println("Not implemented!");
}

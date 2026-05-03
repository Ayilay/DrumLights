#include <Arduino.h>
#include "driver/i2s_std.h"
#include <FastLED.h>
#include <OneButton.h>

/**
 * XIAO ESP32-C3 Pinout:
 *
 *            /--|   |--\
 *  GPIO2  ---+  |USB|  +--- 5V
 *  GPIO3  ---+         +--- GND
 *  GPIO4  ---+         +--- 3V3
 *  GPIO5  ---+         +--- GPIO10
 *  GPIO6  ---+         +--- GPIO9
 *  GPIO7  ---+         +--- GPIO8
 *  GPIO21 ---+         +--- GPIO20
 *            \---------/
 */

// ================== SENSITIVITY KNOBS ==================
#define HOLD_TIME_MS       50

// ================== PIN CONFIG ==================
#define I2S_BCLK_PIN   GPIO_NUM_5
#define I2S_WS_PIN     GPIO_NUM_6
#define I2S_DATA_PIN   GPIO_NUM_10
#define LED_PIN        GPIO_NUM_8
#define BTN_PIN        GPIO_NUM_7

// ================== I2S DETECTOR CONFIG =================
#define SAMPLE_RATE        4000
#define BUFFER_SAMPLES     64
#define TRIGGER_PLOT_LEVEL settings.thresh

// ================= LED CONFIG =================

/**
 *
 * LEDs per Meter = 60
 *
 *  Drum    Circumference   NUM_LEDS
 *  Tom0    35"   0.889  m   54
 *  Tom1    39"   0.9906 m   60
 *  Tom2    45"   1.143  m   69
 *  Snare   45"   1.143  m   69
 *  Base    64"   1.6256 m   98
 *
 */

#define NUM_LEDS      100
#define LED_TYPE      WS2812B
#define COLOR_ORDER   GRB

#define LED_UPDATE_MS  2       // Smaller is smoother

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
  uint32_t brightness;
  float    decay;

  uint32_t red;
  uint32_t green;
  uint32_t blue;
} settings;


// ================= LED STATE =================

CRGB leds[NUM_LEDS];

// ================= MENU BUTTON =================

OneButton menuBtn;

// ================== I2S STATE  =================

int32_t sampleBuffer[BUFFER_SAMPLES];
i2s_chan_handle_t rx_chan;
bool triggerActive = false;
unsigned long lastTriggerTime = 0;

// ================== BLINKER TASK =================
typedef enum {
  BLINK_RED   = 1,
  BLINK_GREEN = 2,
  BLINK_BLUE  = 3,
  HIT_STIM    = 4,
} BlinkMessage;

TaskHandle_t BlinkTaskHandle = NULL;
QueueHandle_t blinkerQueue   = NULL;

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

  settings.decay = 0.95;      // Smaller is faster
  settings.brightness = 128;
  settings.thresh = 1000;
  settings.stream = false;
}

void setup()
{
  Serial.begin(115200);

  init_default_settings();

  // Menu Button Init
  menuBtn.setup(
      BTN_PIN,      // Input pin for the button
      INPUT_PULLUP, // INPUT and enable the internal pull-up resistor
      true          // Button is active LOW
      );

  menuBtn.attachClick(handleClick);
  menuBtn.attachDoubleClick(handleDoubleClick);
  menuBtn.attachLongPressStart(handleLongStart);
  menuBtn.attachLongPressStop(handleLongStop);
  menuBtn.setClickMs( 150 );

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
  FastLED.setBrightness(settings.brightness);

  // ======================================================================
  // Blinker Task + Queue Init
  blinkerQueue = xQueueCreate( 1, sizeof(BlinkMessage) );

  xTaskCreate(
    BlinkTask,         // Task function
    "Blinker",         // Task name
    10000,             // Stack size (bytes)
    NULL,              // Parameters
    10,                // Priority
    &BlinkTaskHandle   // Task handle
  );
}

void BlinkTask_HandleStim() {

  float currentBrightness = settings.brightness;

  // Max for each is 255
  uint8_t red   = settings.red;
  uint8_t green = settings.green;
  uint8_t blue  = settings.blue;

  fill_solid(leds, NUM_LEDS, CRGB( red, green, blue ));
  FastLED.setBrightness((uint8_t)currentBrightness);
  FastLED.show();

  // Yes yes embedded float bad
  // But the exponential decay works better...
  while (currentBrightness > 1.0f) {

    currentBrightness *= settings.decay;
    FastLED.setBrightness((uint8_t)currentBrightness);
    FastLED.show();
    delay( LED_UPDATE_MS );
  }

  currentBrightness = 0.0f;
  FastLED.setBrightness((uint8_t)currentBrightness);
  FastLED.show();
}

void Strip_BlinkAll( CRGB color, int duration )
{
  fill_solid(leds, NUM_LEDS, color);
  FastLED.setBrightness(settings.brightness);
  FastLED.show();
  delay(duration);
  FastLED.setBrightness(0);
  FastLED.show();
}

void BlinkTask(void *parameter) {
  (void) parameter;

  BlinkMessage command;
  CRGB color;

  for (;;) {
    // Wait forever until command is placed into queue
    xQueueReceive( blinkerQueue, (void*) &command, portMAX_DELAY );

    switch( command ){
      case BLINK_RED:
        Strip_BlinkAll( CRGB( settings.brightness, 0, 0 ), 200 );
        break;
      case BLINK_GREEN:
        Strip_BlinkAll( CRGB( 0, settings.brightness, 0 ), 200 );
        break;
      case BLINK_BLUE:
        Strip_BlinkAll( CRGB( 0, 0, settings.brightness ), 200 );
        break;
      case HIT_STIM:
        BlinkTask_HandleStim();

      default:
        // Unrecognized; do nothing
        break;
    }
  }
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

  // ================== SERIAL PLOTTER OUTPUT ==================
  if( settings.stream )
  {
    Serial.print(peak);
    Serial.print(",");
    Serial.print(triggerActive ? TRIGGER_PLOT_LEVEL : 0);
    Serial.print(",");
    Serial.println(settings.thresh);
  }

  cli_poll();

  menuBtn.tick();
}

// Handler function for a single click:
static void handleClick() {
  Serial.println("Clicked!");
  BlinkMessage command = BLINK_BLUE;
  xQueueSend( blinkerQueue, (void*) &command, 0 );
}

static void handleDoubleClick() {
  BlinkMessage command = BLINK_RED;
  Serial.println("Double Pressed!");
  xQueueSend( blinkerQueue, (void*) &command, 0 );
};

static void handleLongStart() {
  BlinkMessage command = BLINK_GREEN;
  Serial.println("HOLDING ...");
  xQueueSend( blinkerQueue, (void*) &command, 0 );
};

static void handleLongStop() {
  BlinkMessage command = BLINK_GREEN;
  Serial.println("Released!");
  xQueueSend( blinkerQueue, (void*) &command, 0 );
};

// ================================================================================
// LED Update Routines
// ================================================================================

/*
 *  This is called once when the drum hit is detected
 */
void LED_Update_Stimulus()
{
  BlinkMessage command = HIT_STIM;
  xQueueSend( blinkerQueue, (void*) &command, 0 );
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
void cmd_bright(const char *arg, uintptr_t cookie);

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
  { "bright"," [val] Get/Set the LED brightness", cmd_bright, NULL },
  { "decay","  [val] Get/Set the LED decay speed", cmd_decay, NULL },
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
  else if( 0 == strcmp( arg, "off" ) )
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

  // We accept 0x hex format AND decimal format
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

void cmd_decay(const char *arg, uintptr_t cookie) {
  // No arg: print the value and exit
  if( ! arg ){
    Serial.print( "Decay (0.0-1.0): " );
    Serial.println( settings.decay );

    return;
  }

  float val = strtof(arg, NULL);
  if( val > 1.0 ){
    Serial.print( "invalid num " );
    Serial.print( val );
    Serial.println( ". Must be wthin 0.0-1.0" );
    return;
  } else {
    Serial.print( "parsed " );
    Serial.print( arg );
    Serial.print( " as " );
    Serial.println( val );
  }

  settings.decay = val;
  Serial.print( "Set new decay to " );
  Serial.println( val);
}

void cmd_bright(const char *arg, uintptr_t cookie) {
  // No arg: print the value and exit
  if( ! arg ){
    Serial.print( "Brightness (0-255): " );
    Serial.println( settings.brightness );

    return;
  }

  // We accept 0x hex format AND decimal format
  uint32_t val = strtol(arg, NULL, 0);
  if( val > 255 ){
    Serial.print( "invalid num " );
    Serial.print( val );
    Serial.println( ". Must be wthin 0-255" );
    return;
  }

  settings.brightness = val;
  Serial.print( "Set new brightness to " );
  Serial.println( val);
}

void cmd_thresh(const char *arg, uintptr_t cookie) {
  // No arg: print the value and exit
  if( ! arg ){
    Serial.print( "Threshold (0-32767): " );
    Serial.println( settings.thresh );

    return;
  }

  // We accept 0x hex format AND decimal format
  uint32_t val = strtol(arg, NULL, 0);
  if( val > 0x7fff ){
    Serial.print( "invalid num " );
    Serial.print( val );
    Serial.println( ". Must be wthin 0-32767" );
    return;
  }

  settings.thresh = val;
  Serial.print( "Set new thresh to " );
  Serial.println( val);
}

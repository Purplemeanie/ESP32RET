/*
 ESP32RET.ino

 Created: June 1, 2020
 Author: Collin Kidder

Copyright (c) 2014-2020 Collin Kidder, Michael Neuweiler

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "config.h"
#include <esp32_can.h>
#include <SPI.h>
#include <esp32_mcp2517fd.h>
#include <Preferences.h>
#include <FastLED.h>
#include "ELM327_Emulator.h"
#include "SerialConsole.h"
#include "wifi_manager.h"
#include "gvret_comm.h"
#include "can_manager.h"
#include "lawicel.h"
#include "esp_system.h"


#include "driver/twai.h"
#include "esp_err.h"

#include <Arduino.h>
#include "esp_timer.h"
#include "esp_freertos_hooks.h"

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ---------- ST7789 (Feather ESP32-S3 TFT uses these macros from the variant) ----------
#ifndef TFT_CS
  #error "This board variant does not define TFT_CS/TFT_DC/TFT_RST. Check board selection."
#endif

Adafruit_ST7789 tft(TFT_CS, TFT_DC, TFT_RST);

//on the S3 we want the default pins to be different
#ifdef CONFIG_IDF_TARGET_ESP32S3
MCP2517FD CAN1(10, 3);
#endif

byte i = 0;

uint32_t lastFlushMicros = 0;

bool markToggle[6];
uint32_t lastMarkTrigger = 0;

EEPROMSettings settings;
SystemSettings SysSettings;
Preferences nvPrefs;
char deviceName[20];
char otaHost[40];
char otaFilename[100];

uint8_t espChipRevision;

ELM327Emu elmEmulator;

WiFiManager wifiManager;

GVRET_Comm_Handler serialGVRET; //gvret protocol over the serial to USB connection
GVRET_Comm_Handler wifiGVRET; //GVRET over the wifi telnet port
CANManager canManager; //keeps track of bus load and abstracts away some details of how things are done
LAWICELHandler lawicel;

SerialConsole console;

CRGB leds[A5_NUM_LEDS]; //A5 has the largest # of LEDs so use that one even for A0 or EVTV

CAN_COMMON *canBuses[NUM_BUSES];

static volatile uint64_t idle_us[2] = {0, 0};

// Called from the Idle task on each CPU
static bool idle_hook_cpu0()
{
  static uint64_t last = 0;
  uint64_t now = esp_timer_get_time();
  if (last) idle_us[0] += (now - last);
  last = now;
  return true; // keep calling
}

static bool idle_hook_cpu1()
{
  static uint64_t last = 0;
  uint64_t now = esp_timer_get_time();
  if (last) idle_us[1] += (now - last);
  last = now;
  return true;
}

// Set up the CPU load callbacks
static void cpu_load_init()
{
  // Register idle hooks for both cores
  esp_register_freertos_idle_hook_for_cpu(idle_hook_cpu0, 0);
  esp_register_freertos_idle_hook_for_cpu(idle_hook_cpu1, 1);
}

// Grab the CPU load info every period_ms milliseconds
static bool cpu_load_sample(float &load0, float &load1, uint32_t period_ms = 1000) {
  static uint64_t last_t_us = 0;
  static uint64_t last_idle0 = 0, last_idle1 = 0;

  uint64_t now_us = esp_timer_get_time();
  if (!last_t_us) { last_t_us = now_us; return false; }

  if ((now_us - last_t_us) < (uint64_t)period_ms * 1000ULL) return false;

  uint64_t dt = now_us - last_t_us;
  uint64_t i0 = idle_us[0] - last_idle0;
  uint64_t i1 = idle_us[1] - last_idle1;

  last_t_us = now_us;
  last_idle0 = idle_us[0];
  last_idle1 = idle_us[1];

  float idle0_pct = dt ? (100.0f * (float)i0 / (float)dt) : 0;
  float idle1_pct = dt ? (100.0f * (float)i1 / (float)dt) : 0;

  load0 = 100.0f - idle0_pct;
  load1 = 100.0f - idle1_pct;
  if (load0 < 0) load0 = 0; if (load0 > 100) load0 = 100;
  if (load1 < 0) load1 = 0; if (load1 > 100) load1 = 100;
  return true;
}

// ---------- Simple dashboard drawing ----------
static void draw_bar(int x, int y, int w, int h, float pct, uint16_t outline, uint16_t fill, uint16_t bg) {
  tft.drawRect(x, y, w, h, outline);

  int innerW = w - 2;
  int innerH = h - 2;
  int filled = (int)((pct / 100.0f) * innerW + 0.5f);

  // clear + fill (min flicker, tiny region)
  tft.fillRect(x + 1, y + 1, innerW, innerH, bg);
  if (filled > 0) tft.fillRect(x + 1, y + 1, filled, innerH, fill);
}

static void draw_bar_delta(int x, int y, int w, int h, float pct,
                           uint16_t outline, uint16_t fill, uint16_t bg,
                           int &prevFilled)
{
  tft.drawRect(x, y, w, h, outline);

  int innerW = w - 2;
  int filled = (int)((pct / 100.0f) * innerW + 0.5f);
  if (filled < 0) filled = 0;
  if (filled > innerW) filled = innerW;

  if (prevFilled < 0) {
    // first draw
    tft.fillRect(x + 1, y + 1, innerW, h - 2, bg);
    if (filled) tft.fillRect(x + 1, y + 1, filled, h - 2, fill);
  } else if (filled > prevFilled) {
    // grew: fill just the added part
    tft.fillRect(x + 1 + prevFilled, y + 1, filled - prevFilled, h - 2, fill);
  } else if (filled < prevFilled) {
    // shrank: clear just the removed part
    tft.fillRect(x + 1 + filled, y + 1, prevFilled - filled, h - 2, bg);
  }

  prevFilled = filled;
}

#ifndef TFT_BACKLIGHT
#define TFT_BACKLIGHT 45
#endif

static void ui_init() {
  pinMode(TFT_BACKLIGHT, OUTPUT);
  digitalWrite(TFT_BACKLIGHT, HIGH);

  // Feather ESP32-S3 TFT is typically 240x135 (landscape), but init wants native panel dims.
  // Most Adafruit examples use (240, 135) for this board.
  tft.init(135, 240);
  tft.setRotation(1); // try 1/3 if orientation is wrong
  tft.fillScreen(ST77XX_BLACK);

  tft.setTextWrap(false);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

  //tft.setCursor(6, 6);
  //tft.print("PurpleMeanie ESP32RET");

  //tft.setCursor(6, 24);
  //tft.print("CPU load");
}

static void ui_update(float load0, float load1, uint32_t can0_fps)
{
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

  // ---- CPU header ----
  tft.setCursor(6, 10);
  tft.print("CPU LOAD");

  // ---- CPU numbers ----
  tft.setCursor(6, 30);
  tft.printf("C0:%5.1f%% C1:%5.1f%%", load0, load1);

  // ---- Bars ----
  static int prev0 = -1, prev1 = -1;

  draw_bar_delta(6, 55, 228, 12, load0,
                 ST77XX_WHITE, ST77XX_GREEN, ST77XX_BLACK, prev0);

  draw_bar_delta(6, 75, 228, 12, load1,
                 ST77XX_WHITE, ST77XX_CYAN, ST77XX_BLACK, prev1);

  // ---- CAN RX ----
  tft.setCursor(6, 100);
  tft.printf("CAN0:%5u fps", (unsigned)can0_fps);

  // ---- Heap ----
  tft.setCursor(6, 120);
  tft.printf("Heap:%6u", (unsigned)ESP.getFreeHeap());
}

static void cpu_load_print_every(uint32_t period_ms = 1000)
{
  static uint64_t last_t_us = 0;
  static uint64_t last_idle0 = 0, last_idle1 = 0;

  uint64_t now_us = esp_timer_get_time();
  if (!last_t_us) { last_t_us = now_us; return; }

  if ((now_us - last_t_us) < (uint64_t)period_ms * 1000ULL) return;

  uint64_t dt = now_us - last_t_us;
  uint64_t i0 = idle_us[0] - last_idle0;
  uint64_t i1 = idle_us[1] - last_idle1;

  last_t_us = now_us;
  last_idle0 = idle_us[0];
  last_idle1 = idle_us[1];

  float idle0_pct = (dt ? (100.0f * (float)i0 / (float)dt) : 0);
  float idle1_pct = (dt ? (100.0f * (float)i1 / (float)dt) : 0);

  float load0 = 100.0f - idle0_pct;
  float load1 = 100.0f - idle1_pct;

  Serial.printf("CPU load: core0=%.1f%% core1=%.1f%% (idle0=%.1f%% idle1=%.1f%%)\n",
                load0, load1, idle0_pct, idle1_pct);
}

static void dump_twai_state(const char* where) {
  twai_status_info_t info;
  esp_err_t e = twai_get_status_info(&info);
  Serial.printf("[%s] twai_get_status_info: %s\n", where, esp_err_to_name(e));
  if (e == ESP_OK) {
    Serial.printf("[%s] TWAI state=%d msgs_to_rx=%d rx_err=%d tx_err=%d bus_err=%d\n",
                  where, (int)info.state, (int)info.msgs_to_rx,
                  (int)info.rx_error_counter, (int)info.tx_error_counter, (int)info.bus_error_count);
  }
}


static const char* reset_reason_str(esp_reset_reason_t r)
{
  switch (r) {
    case ESP_RST_UNKNOWN:   return "UNKNOWN";
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT (external pin)";
    case ESP_RST_SW:        return "SW (software reset)";
    case ESP_RST_PANIC:     return "PANIC (crash)";
    case ESP_RST_INT_WDT:   return "INT_WDT (interrupt watchdog)";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT (other watchdog)";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "???";
  }
}

void print_reset_reason()
{
  esp_reset_reason_t reason = esp_reset_reason();
  Serial.printf("Reset reason: %d (%s)\n", (int)reason, reset_reason_str(reason));
}

//initializes all the system EEPROM values. Chances are this should be broken out a bit but
//there is only one checksum check for all of them so it's simple to do it all here.
void loadSettings()
{
    Logger::console("Loading settings....");

    //Logger::console("%i\n", espChipRevision);

    for (int i = 0; i < NUM_BUSES; i++) canBuses[i] = nullptr;

    nvPrefs.begin(PREF_NAME, false);

    settings.useBinarySerialComm = nvPrefs.getBool("binarycomm", false);
    settings.logLevel = nvPrefs.getUChar("loglevel", 1); //info
    settings.wifiMode = nvPrefs.getUChar("wifiMode", 2); //Wifi defaults to creating an AP
    settings.enableBT = nvPrefs.getBool("enable-bt", false);
    settings.enableLawicel = nvPrefs.getBool("enableLawicel", true);
    settings.sendingBus = nvPrefs.getInt("sendingBus", 0);
    settings.sendToConsole = nvPrefs.getBool("sendToConsole", true);
    canManager.setSendToConsole(settings.sendToConsole);
    
    uint8_t defaultVal = (espChipRevision > 2) ? 0 : 1; //0 = A0, 1 = EVTV ESP32
#ifdef CONFIG_IDF_TARGET_ESP32S3
    defaultVal = 3;
#endif
    defaultVal = 4; //4 = ESP32-S3 TFT
    settings.systemType = nvPrefs.getUChar("systype", defaultVal);

    if (settings.systemType == 0)
    {
        Logger::console("Running on Macchina A0");
        canBuses[0] = &CAN0;
        SysSettings.LED_CANTX = 255;
        SysSettings.LED_CANRX = 255;
        SysSettings.LED_LOGGING = 255;
        SysSettings.LED_CONNECTION_STATUS = 0;
        SysSettings.fancyLED = true;
        SysSettings.logToggle = false;
        SysSettings.txToggle = true;
        SysSettings.rxToggle = true;
        SysSettings.lawicelAutoPoll = false;
        SysSettings.lawicelMode = false;
        SysSettings.lawicellExtendedMode = false;
        SysSettings.lawicelTimestamping = false;
        SysSettings.numBuses = 1; //Currently we support CAN0
        SysSettings.isWifiActive = false;
        SysSettings.isWifiConnected = false;
        strcpy(deviceName, MACC_NAME);
        strcpy(otaHost, "macchina.cc");
        strcpy(otaFilename, "/a0/files/a0ret.bin");
        pinMode(13, OUTPUT);
        digitalWrite(13, LOW);
        delay(100);
        FastLED.addLeds<LED_TYPE, A0_LED_PIN, COLOR_ORDER>(leds, A0_NUM_LEDS).setCorrection( TypicalLEDStrip );
        FastLED.setBrightness(  BRIGHTNESS );
        leds[0] = CRGB::Red;
        FastLED.show();
        pinMode(21, OUTPUT);
        digitalWrite(21, LOW);
        CAN0.setCANPins(GPIO_NUM_4, GPIO_NUM_5);
    }

    if (settings.systemType == 1)
    {
        Logger::console("Running on EVTV ESP32 Board");
        canBuses[0] = &CAN0;
        canBuses[1] = &CAN1;
        SysSettings.LED_CANTX = 255;
        SysSettings.LED_CANRX = 255;
        SysSettings.LED_LOGGING = 255;
        SysSettings.LED_CONNECTION_STATUS = 255;
        SysSettings.fancyLED = false;
        SysSettings.logToggle = false;
        SysSettings.txToggle = true;
        SysSettings.rxToggle = true;
        SysSettings.lawicelAutoPoll = false;
        SysSettings.lawicelMode = false;
        SysSettings.lawicellExtendedMode = false;
        SysSettings.lawicelTimestamping = false;
        SysSettings.numBuses = 2;
        SysSettings.isWifiActive = false;
        SysSettings.isWifiConnected = false;
        strcpy(deviceName, EVTV_NAME);
        strcpy(otaHost, "media3.evtv.me");
        strcpy(otaFilename, "/esp32ret.bin");
    }

    if (settings.systemType == 2)
    {
        Logger::console("Running on Macchina 5-CAN");
        canBuses[0] = &CAN0; //SWCAN on this hardware - DLC pin 1
        canBuses[1] = &CAN1; //DLC pins 1 and 9. Overlaps with SWCAN
        canBuses[2] = new MCP2517FD(33, 39); //DLC pins 3/11
        canBuses[3] = new MCP2517FD(25, 34); //DLC pins 6/14
        canBuses[4] = new MCP2517FD(14, 13); //DLC pins 12/13

        //reconfigure the two already defined CAN buses to use the actual pins for this board.
        CAN0.setCANPins(GPIO_NUM_4, GPIO_NUM_5); //rx, tx - This is the SWCAN interface
        CAN1.setINTPin(36);
        CAN1.setCSPin(32);
        SysSettings.LED_CANTX = 0;
        SysSettings.LED_CANRX = 1;
        SysSettings.LED_LOGGING = 2;
        SysSettings.LED_CONNECTION_STATUS = 3;
        SysSettings.fancyLED = true;
        SysSettings.logToggle = false;
        SysSettings.txToggle = true;
        SysSettings.rxToggle = true;
        SysSettings.lawicelAutoPoll = false;
        SysSettings.lawicelMode = false;
        SysSettings.lawicellExtendedMode = false;
        SysSettings.lawicelTimestamping = false;
        SysSettings.numBuses = 5;
        SysSettings.isWifiActive = false;
        SysSettings.isWifiConnected = false;


        FastLED.addLeds<LED_TYPE, A5_LED_PIN, COLOR_ORDER>(leds, A5_NUM_LEDS).setCorrection( TypicalLEDStrip );
        FastLED.setBrightness(  BRIGHTNESS );
        //With the board facing up and looking at the USB end the LEDs are 0 1 2 (USB) 3
        //can test LEDs here for debugging but normally leave first three off and set connection to RED.
        //leds[0] = CRGB::White;
        //leds[1] = CRGB::Blue;
        //leds[2] = CRGB::Green;
        leds[3] = CRGB::Red;
        FastLED.show();

        strcpy(deviceName, MACC_NAME);
        strcpy(otaHost, "macchina.cc");
        strcpy(otaFilename, "/a0/files/a0ret.bin");
        //Single wire interface
        pinMode(SW_EN, OUTPUT);
        pinMode(SW_MODE0, OUTPUT);
        pinMode(SW_MODE1, OUTPUT);
        digitalWrite(SW_EN, LOW);      //MUST be LOW to use CAN1 channel 
        //HH = Normal Mode
        digitalWrite(SW_MODE0, HIGH);
        digitalWrite(SW_MODE1, HIGH);
    }

    if (settings.systemType == 3)
    {
        Logger::console("Running on EVTV ESP32-S3 Board");
        canBuses[0] = &CAN0;
        canBuses[1] = &CAN1;
        //CAN1.setINTPin(3);
        //CAN1.setCSPin(10);
        SysSettings.LED_CANTX = 255;//18;
        SysSettings.LED_CANRX = 255;//18;
        SysSettings.LED_LOGGING = 255;
        SysSettings.LED_CONNECTION_STATUS = 255;
        SysSettings.fancyLED = false;
        SysSettings.logToggle = false;
        SysSettings.txToggle = true;
        SysSettings.rxToggle = true;
        SysSettings.lawicelAutoPoll = false;
        SysSettings.lawicelMode = false;
        SysSettings.lawicellExtendedMode = false;
        SysSettings.lawicelTimestamping = false;
        SysSettings.numBuses = 2;
        SysSettings.isWifiActive = false;
        SysSettings.isWifiConnected = false;
        strcpy(deviceName, EVTV_NAME);
        strcpy(otaHost, "media3.evtv.me");
        strcpy(otaFilename, "/esp32s3ret.bin");
    }

    if (settings.systemType == 4)
    {
        Logger::console("Running on ESP32-S3 TFT Board");
        canBuses[0] = &CAN0;
        canBuses[1] = nullptr;
        SysSettings.LED_CANTX = 255;
        SysSettings.LED_CANRX = 255;
        SysSettings.LED_LOGGING = 255;
        SysSettings.LED_CONNECTION_STATUS = 0;
        SysSettings.fancyLED = true;
        SysSettings.logToggle = false;
        SysSettings.txToggle = true;
        SysSettings.rxToggle = true;
        SysSettings.lawicelAutoPoll = false;
        SysSettings.lawicelMode = false;
        SysSettings.lawicellExtendedMode = false;
        SysSettings.lawicelTimestamping = false;
        SysSettings.numBuses = 1; //Currently we support CAN0
        SysSettings.isWifiActive = false;
        SysSettings.isWifiConnected = false;
        strcpy(deviceName, S3TFT_NAME);
        strcpy(otaHost, "purplemeanie.co.uk");
        strcpy(otaFilename, "none.bin");
        pinMode(34, OUTPUT);
        digitalWrite(34, HIGH);  // Turn on NeoPixel
        ui_init();
        //pinMode(45, OUTPUT);
        //digitalWrite(45, HIGH);  // Turn on TFT Display backlight (could PWM this if we wanted to control brightness)
        delay(100);
        FastLED.addLeds<LED_TYPE, S3TFT_LED_PIN, COLOR_ORDER>(leds, S3TFT_NUM_LEDS).setCorrection( TypicalLEDStrip );
        FastLED.setBrightness(  BRIGHTNESS );
        leds[0] = CRGB::Red;
        FastLED.show();
        //pinMode(21, OUTPUT);
        //digitalWrite(21, LOW);
        CAN0.setCANPins(GPIO_NUM_5, GPIO_NUM_6);
    }

    if (nvPrefs.getString("SSID", settings.SSID, 32) == 0)
    {
        strcpy(settings.SSID, deviceName);
        strcat(settings.SSID, "SSID");
    }

    if (nvPrefs.getString("wpa2Key", settings.WPA2Key, 64) == 0)
    {
        strcpy(settings.WPA2Key, "aBigSecret");
    }
    if (nvPrefs.getString("btname", settings.btName, 32) == 0)
    {
        strcpy(settings.btName, "ELM327-");
        strcat(settings.btName, deviceName);
    }

    char buff[80];
    for (int i = 0; i < SysSettings.numBuses; i++)
    {
        sprintf(buff, "can%ispeed", i);
        settings.canSettings[i].nomSpeed = nvPrefs.getUInt(buff, 500000);
        sprintf(buff, "can%i_en", i);
        settings.canSettings[i].enabled = nvPrefs.getBool(buff, (i < 2)?true:false);
        sprintf(buff, "can%i-listenonly", i);
        settings.canSettings[i].listenOnly = nvPrefs.getBool(buff, false);
        sprintf(buff, "can%i-fdspeed", i);
        settings.canSettings[i].fdSpeed = nvPrefs.getUInt(buff, 5000000);
        sprintf(buff, "can%i-fdmode", i);
        settings.canSettings[i].fdMode = nvPrefs.getBool(buff, false);
    }

    nvPrefs.end();

    Logger::setLoglevel((Logger::LogLevel)settings.logLevel);

    for (int rx = 0; rx < NUM_BUSES; rx++) SysSettings.lawicelBusReception[rx] = true; //default to showing messages on RX 
}

void setup()
{
#ifdef CONFIG_IDF_TARGET_ESP32S3
    //for the ESP32S3 it will block if nothing is connected to USB and that can slow down the program
    //if nothing is connected. But, you can't set 0 or writing rapidly to USB will lose data. It needs
    //some sort of timeout but I'm not sure exactly how much is needed or if there is a better way
    //to deal with this issue.
    Serial.setTxTimeoutMs(2);
#endif
    Serial.begin(1000000); //for production
    //Serial.begin(115200); //for testing
    //delay(2000); //just for testing. Don't use in production

    cpu_load_init();
    ui_init();

    // Do a bit of hanging around so we capture any reboot issues (PANICs)
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0) < 1500) { delay(10); } // helps on USB CDC
    delay(50);

    print_reset_reason();
    //dump_twai_state("boot");

    espChipRevision = ESP.getChipRevision();
    
    Serial.printf("ESP-IDF version: %s\n", esp_get_idf_version());

    Serial.printf("Arduino Running on core: %d\n", xPortGetCoreID());

    Serial.print("Build number: ");
    Serial.println(CFG_BUILD_NUM);

    SysSettings.isWifiConnected = false;

    loadSettings();

    // Do some early logging to see if there are any issues before we get to the main loop. 
    // If there is a PANIC reset we want to be sure to capture it in the logs and not miss it because we're not logging yet.
    const int maxBuses = (int)(sizeof(canBuses)/sizeof(canBuses[0]));  // if canBuses is a real array here
    Serial.printf("numBuses=%d (MAX=%d)\n", SysSettings.numBuses, maxBuses);
    if (SysSettings.numBuses > maxBuses) abort();

    //CAN0.setDebuggingMode(true);
    //CAN1.setDebuggingMode(true);
    
    canManager.setup();

    if (settings.enableBT) 
    {
        Serial.println("Starting bluetooth");
        elmEmulator.setup();
        if (SysSettings.fancyLED && (settings.wifiMode == 0) )
        {
            leds[0] = CRGB::Green;
            FastLED.show();
        }
    }
    
    /*else*/ wifiManager.setup();

    SysSettings.lawicelMode = false;
    SysSettings.lawicelAutoPoll = false;
    SysSettings.lawicelTimestamping = false;
    SysSettings.lawicelPollCounter = 0;
    
    //elmEmulator.setup();

    Serial.print("Free heap after setup: ");
    Serial.println(esp_get_free_heap_size());

    Serial.print("Done with init\n");
}

/*
Send a fake frame out USB and maybe to file to show where the mark was triggered at. The fake frame has bits 31 through 3
set which can never happen in reality since frames are either 11 or 29 bit IDs. So, this is a sign that it is a mark frame
and not a real frame. The bottom three bits specify which mark triggered.
*/
void sendMarkTriggered(int which)
{
    CAN_FRAME frame;
    frame.id = 0xFFFFFFF8ull + which;
    frame.extended = true;
    frame.length = 0;
    frame.rtr = 0;
    canManager.displayFrame(frame, 0);
}

extern "C" TaskHandle_t xTaskGetHandle(const char *pcName);

static void dump_can_task_presence(uint8_t bus) {
  char n1[16], n2[16];
  snprintf(n1, sizeof(n1), "CAN_LORX_%u", (unsigned)bus);
  snprintf(n2, sizeof(n2), "CAN_RX_%u",   (unsigned)bus);

  TaskHandle_t h1 = xTaskGetHandle(n1);
  TaskHandle_t h2 = xTaskGetHandle(n2);

  Serial.printf("[tasks] %s=%p %s=%p\n", n1, (void*)h1, n2, (void*)h2);
}

/*
Loop executes as often as possible all the while interrupts fire in the background.
The serial comm protocol is as follows:
All commands start with 0xF1 this helps to synchronize if there were comm issues
Then the next byte specifies which command this is.
Then the command data bytes which are specific to the command
Lastly, there is a checksum byte just to be sure there are no missed or duped bytes
Any bytes between checksum and 0xF1 are thrown away

Yes, this should probably have been done more neatly but this way is likely to be the
fastest and safest with limited function calls
*/
void loop()
{
    //uint32_t temp32;    
    bool isConnected = false;
    int serialCnt;
    uint8_t in_byte;

    /*if (Serial)*/ isConnected = true;
  
    static bool once=false;
    if (!once) { once=true; dump_can_task_presence(0); }
  
    float l0, l1;
    if (cpu_load_sample(l0, l1, 1000)) {
        //Serial.printf("CPU load: core0=%.1f%% core1=%.1f%% (loop core=%d)\n", l0, l1, xPortGetCoreID());
        auto *esp = (ESP32CAN*)canBuses[0];
        esp->sampleRates1Hz();
        uint32_t fps = esp->rx_rate_fps;
        ui_update(l0, l1, esp->rx_rate_fps);
    } 

    if (SysSettings.lawicelPollCounter > 0) SysSettings.lawicelPollCounter--;
    //}

    canManager.loop();
    /*if (!settings.enableBT)*/ wifiManager.loop();

    size_t wifiLength = wifiGVRET.numAvailableBytes();
    size_t serialLength = serialGVRET.numAvailableBytes();
    size_t maxLength = (wifiLength>serialLength) ? wifiLength : serialLength;

    //If the max time has passed or the buffer is almost filled then send buffered data out
    if ((micros() - lastFlushMicros > SER_BUFF_FLUSH_INTERVAL) || (maxLength > (WIFI_BUFF_SIZE - 40)) ) 
    {
        lastFlushMicros = micros();
        if (serialLength > 0) 
        {
            Serial.write(serialGVRET.getBufferedBytes(), serialLength);
            serialGVRET.clearBufferedBytes();
        }
        if (wifiLength > 0)
        {
            wifiManager.sendBufferedData();
        }
    }

    serialCnt = 0;
    while ( (Serial.available() > 0) && serialCnt < 128 ) 
    {
        serialCnt++;
        in_byte = Serial.read();
        serialGVRET.processIncomingByte(in_byte);
    }

    elmEmulator.loop();
}

#include "version.h"
#include <algorithm>
#include <Wire.h>
#include <Arduino.h>
#include "ConfigNVS.h"
#include "pin_config.h"
#include "Arduino_GFX_Library.h"
#include "Arduino_DriveBus_Library.h"
#include <ESP_IOExpander_Library.h>
#include "XPowersLib.h"
//#include <Arduino_FT3x68.h>

#include <BleConnectionStatus.h>
#include <BleCompositeHID.h>
#include <XboxGamepadDevice.h>
#include <MouseDevice.h>

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_bt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

#include "esp_coexist.h"
#include "cybernow_proto.h"
#include "gamepad_merged.h"
#include "TouchMouse.h"

#include "splash_images.h"
#include "audio.h"
#include "HWCDC.h"
#include "GenericGamepadOut.h"
#include "XBoxOut.h"
#include "IGamepadOut.h"

HWCDC USBSerial;

// for reading battery state
XPowersPMU power;

//#include <driver/gpio.h>
//#include <esp_rom_gpio.h>
//#include "soc/io_mux_reg.h"    // gives you IO_MUX_MTMS_REG
//#include "soc/gpio_sig_map.h"  // gives you FUNC_GPIO42

//int ledPin = 5;  // LED connected to digital pin 13

#define _EXAMPLE_CHIP_CLASS(name, ...) ESP_IOExpander_##name(__VA_ARGS__)
#define EXAMPLE_CHIP_CLASS(name, ...) _EXAMPLE_CHIP_CLASS(name, ##__VA_ARGS__)

#define MAX_HISTORY     6
#define FADE_TIME_MS  2500    // fade-out duration (2.5 s)
#define RADIUS         5     // radius≈12 for ~25px diameter
#define BLACK_FRAME_COUNT 200
#define MAX_BRIGHTNESS 128 

// ESP Now mac
uint8_t mac[6] = {0};
// ESP Now msg counter
static uint16_t g_seq = 0;
static uint32_t loop_count = 0;
volatile uint32_t txCount = 0;

static uint32_t lastSendUs = 0;
//constexpr uint32_t MIN_INTERVAL_US = 4000;
constexpr uint32_t MAX_INTERVAL_US = 100000;

// io expander present
bool expander_present = false;

// message queue 
static QueueHandle_t g_pkt_queue = nullptr;

void initQueue() {
  if (!g_pkt_queue) {
    g_pkt_queue = xQueueCreate(8, sizeof(CyberPkt16));
    if (!g_pkt_queue) {
      USBSerial.println("Failed to create ESP-NOW packet queue");
    }
  }
}

void initQueue_old() {
  if (!g_pkt_queue) {
    g_pkt_queue = xQueueCreate(1, sizeof(HalfPacket));
    if (!g_pkt_queue) {
      USBSerial.println("Failed to create ESP-NOW packet queue");
    }
  }
}


// Joystick change threshold
//const uint16_t JOYSTICK_DEADZONE = 250;

GamepadMerged gamepad_handler(NULL);

struct Touch {
    int16_t x, y;
    uint32_t t;   // timestamp of the touch
};

Touch history[MAX_HISTORY];
uint8_t histIdx = 0;
int16_t W, H;

TouchMouse tmouse;

// mouse callbacks
static void MouseMove(int dx, int dy, void* user) {
  USBSerial.printf("MouseMove (%d, %d)\n", dx, dy);
  ((MouseDevice*)user)->mouseMove(dx, dy);
  ((MouseDevice*)user)->sendMouseReport();
}
static void MouseButtonPress(uint8_t buttons, void* user) {
  if (buttons & TM_MOUSE_BTN_LEFT) {
    USBSerial.printf("MouseButtonPress");
    ((MouseDevice*)user)->mousePress(MOUSE_LOGICAL_LEFT_BUTTON);
    ((MouseDevice*)user)->sendMouseReport();
  }
}
static void MouseButtonRelease(uint8_t buttons, void* user) {
  if (buttons & TM_MOUSE_BTN_LEFT) {
    USBSerial.printf("MouseButtonRelease");
    ((MouseDevice*)user)->mouseRelease(MOUSE_LOGICAL_LEFT_BUTTON);
    ((MouseDevice*)user)->sendMouseReport();
  }
}

static void MouseScroll(int v, int h, void* user) {
  USBSerial.printf("MouseScroll (v=%d, h=%d) - nop\n", v, h);
}

// mouse callbacks for sending events to the left controller
static void SendMouseScroll(int v, int h, void* user) {
  USBSerial.printf("SendMouseScroll (v=%d, h=%d) - nop\n", v, h);
}

static void SendMouseMove(int dx, int dy, void* user) {
  return;
  USBSerial.printf("SendMouseMove (%d, %d)\n", dx, dy);
  //((MouseDevice*)user)->mouseMove(dx, dy);
  //((MouseDevice*)user)->sendMouseReport();

  CyberPkt16 frame{};
  frame.mouse.ver     = CYBER_PROTO_MOUSE;
  frame.mouse.srcRole = CYBER_ROLE_RIGHT;
  frame.mouse.seq     = ++g_seq;
  frame.mouse.ms      = millis();
  frame.mouse.mx      = dx;
  frame.mouse.my      = dy;
  frame.mouse.buttons = 0;

  esp_err_t err = esp_now_send(cfg.peer_mac, frame.raw, sizeof(frame.raw)); // always 16
  if (err != ESP_OK) {
    USBSerial.printf("[ESP-NOW] send err=%d\n", err);
  }
  
}

static void SendMouseButtonPress(uint8_t buttons, void* user) {
  if (buttons & TM_MOUSE_BTN_LEFT) {
    USBSerial.printf("SendMouseButtonPress");
    //((MouseDevice*)user)->mousePress(MOUSE_LOGICAL_LEFT_BUTTON);
    //((MouseDevice*)user)->sendMouseReport();

    CyberPkt16 frame{};
    frame.mouse.ver     = CYBER_PROTO_MOUSE;
    frame.mouse.srcRole = CYBER_ROLE_RIGHT;
    frame.mouse.seq     = ++g_seq;
    frame.mouse.ms      = millis();
    frame.mouse.mx      = 0;
    frame.mouse.my      = 0;
    frame.mouse.buttons = BTN_MOUSE1_PRESS;

    esp_err_t err = esp_now_send(cfg.peer_mac, frame.raw, sizeof(frame.raw)); // always 16
    if (err != ESP_OK) {
      USBSerial.printf("[ESP-NOW] send err=%d\n", err);
    }
  }
}
static void SendMouseButtonRelease(uint8_t buttons, void* user) {
  if (buttons & TM_MOUSE_BTN_LEFT) {
    USBSerial.printf("SendMouseButtonRelease");
    //((MouseDevice*)user)->mouseRelease(MOUSE_LOGICAL_LEFT_BUTTON);
    //((MouseDevice*)user)->sendMouseReport();


    CyberPkt16 frame{};
    frame.mouse.ver     = CYBER_PROTO_MOUSE;
    frame.mouse.srcRole = CYBER_ROLE_RIGHT;
    frame.mouse.seq     = ++g_seq;
    frame.mouse.ms      = millis();
    frame.mouse.mx      = 0;
    frame.mouse.my      = 0;
    frame.mouse.buttons = BTN_MOUSE1_RELEASE;

    esp_err_t err = esp_now_send(cfg.peer_mac, frame.raw, sizeof(frame.raw)); // always 16
    if (err != ESP_OK) {
      USBSerial.printf("[ESP-NOW] send err=%d\n", err);
    }
  }
}

void OnVibrateEvent(XboxGamepadOutputReportData data) {
  if (data.weakMotorMagnitude > 0 || data.strongMotorMagnitude > 0) {
    //digitalWrite(ledPin, LOW);
  } else {
    //digitalWrite(ledPin, HIGH);
  }
  // TODO haptics event send from right to left if meant for left... 

  //USBSerial.println("Vibration event. Weak motor: " + String(data.weakMotorMagnitude) + " Strong motor: " + String(data.strongMotorMagnitude));
}

int frame; // count frames, black screen reset after BLACK_FRAME_COUNT 

uint16_t joyCenterRawX;
uint16_t joyCenterRawY;

XboxGamepadDevice *xbox_gamepad;
GamepadDevice *generic_gamepad;
XboxOut xbox_out;
GenericOut generic_out;
IGamepadOut* gamepad_out = nullptr;

MouseDevice *mouse;
BleCompositeHID *compositeHID;

//previous power button state
int prevPower = LOW;
uint32_t time_powerpress = millis()+1000000;
bool poweroff_notice = false;

ESP_IOExpander *expander = nullptr;

Arduino_DataBus    *bus;
Arduino_GFX *gfx;
std::shared_ptr<Arduino_IIC_DriveBus> IIC_Bus;
Arduino_FT3x68 *FT3168;
uint32_t age_ms;

void Arduino_IIC_Touch_Interrupt(void);

void Arduino_IIC_Touch_Interrupt(void) {
  FT3168->IIC_Interrupt_Flag = true;
}

static void onNowRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len)
{

  //const uint8_t* mac = info->src_addr; // sender MAC (what you used to receive as 1st param)

  // Just drop the packet if it can have no local implications
  if (!compositeHID->isConnected()) return; 

  CyberPkt16 u;
  if (!parseCyberPkt(u, data, len)) {
      //USBSerial.println("ESP-NOW: ERROR Expected len=16, got len=" + String(len) + ". Dropping.");
      return;
  }

  //age_ms = (int32_t)(millis() - u.halfgamepad.ms);

  // quick reject based on version/role (ver is u.raw[0], srcRole is u.raw[1])
  const uint8_t ver = u.raw[0];
  const uint8_t role = u.raw[1];

  if (cfg.right_not_left) {
    if (role != CYBER_ROLE_LEFT) {
      //USBSerial.println("ESP-NOW: ERROR Expected CYBER_ROLE_LEFT. Dropping.");
      return;
    }
    if (ver != CYBER_PROTO_HALFGAMEPAD) {
      //USBSerial.println("ESP-NOW: ERROR Expected CYBER_PROTO_HALFGAMEPAD. Dropping.");
      return;
    }
  }
  else {
    return;
    if (role != CYBER_ROLE_RIGHT) {
      //USBSerial.println("ESP-NOW: ERROR Expected CYBER_ROLE_RIGHT. Dropping.");
      return;
    }
    if (ver != CYBER_PROTO_MOUSE)  {
      //USBSerial.println("ESP-NOW: ERROR Expected CYBER_PROTO_MOUSE. Dropping.");
      return;
    }
  }

  // put the message in the queue for processing in loop()
  BaseType_t higherWoken = pdFALSE;
  xQueueSendFromISR(g_pkt_queue, &u, &higherWoken);
  //xQueueOverwriteFromISR(g_pkt_queue, &(u.halfgamepad), &higherWoken);
  if (higherWoken) portYIELD_FROM_ISR();

}

static void dump_wifi_state(const char* tag) {
  wifi_mode_t mode; esp_wifi_get_mode(&mode);
  wifi_second_chan_t sc; uint8_t ch=0; esp_wifi_get_channel(&ch, &sc);
  uint8_t mac[6]; esp_wifi_get_mac(WIFI_IF_STA, mac);
  USBSerial.printf("[%s] mode=%d ch=%u mac=%02X:%02X:%02X:%02X:%02X:%02X\n",
    tag,(int)mode,(unsigned)ch, mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
}

static void initEspNow(uint8_t (&mac)[6])
{
/*
  // 0) Ensure STA isn't trying to connect / scan for remembered networks
  WiFi.persistent(false);              // don't write flash
  WiFi.disconnect(true, true);         // drop connection + erase STA config (NVS)
  esp_wifi_set_storage(WIFI_STORAGE_RAM);
*/
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
  //ESP_ERROR_CHECK( esp_wifi_start() );
  // 1) Start Wi-Fi (Arduino often starts it already; calling twice is risky)
  // guard it:
  /*
  esp_err_t st = esp_wifi_start();
  if (st != ESP_OK && st != ESP_ERR_WIFI_CONN && st != ESP_ERR_WIFI_NOT_STOPPED && st != ESP_ERR_WIFI_INIT_STATE) {
    ESP_ERROR_CHECK(st);
  }*/

    // 2) No power save (do it after start as well; some stacks reset it)
  //ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));


  // 3) Coexistence: prefer balanced or Wi-Fi (helps ESPNOW RX under BLE advertising)
  esp_coex_preference_set(ESP_COEX_PREFER_BALANCE);
  // or: esp_coex_preference_set(ESP_COEX_PREFER_WIFI);

  // Either of these works:
  // esp_wifi_get_mac requires WiFi driver inited (mode set is enough)
  ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, mac));
  // or: esp_read_mac(mac, ESP_MAC_WIFI_STA);

  if (cfg.right_not_left) USBSerial.printf("Right ");
  else USBSerial.printf("Left ");

  // Print this so you can paste it into LEFT/RIGHT sketch as the peer MAC
  USBSerial.printf("hub ESPNOW MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);

  // Fix the Wi-Fi channel for both sides
  esp_wifi_set_promiscuous(true);
  ESP_ERROR_CHECK(esp_wifi_set_channel(cfg.wifi_channel, WIFI_SECOND_CHAN_NONE));
  esp_wifi_set_promiscuous(false);

  // Optional coexistence/power tweaks (helps BLE+ESP-NOW stability on S3)
  //esp_wifi_set_ps(WIFI_PS_MIN_MODEM); // modest power save without big latency hit

  //if (esp_now_init() != ESP_OK) {
  //  USBSerial.println("ESP-NOW init failed"); while(true) delay(1000);
  //}

  // 5) (Re)init ESP-NOW cleanly after BLE has touched the controller
  esp_now_deinit(); // ignore error if not inited yet
  ESP_ERROR_CHECK( esp_now_init() );

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, cfg.peer_mac, 6);
  //memcpy(peer.peer_addr, BCAST, 6);
  peer.channel = cfg.wifi_channel;
  peer.encrypt = false;         // Optional: set up PMK/LTK for encryption if you want
  //if (esp_now_add_peer(&peer) != ESP_OK) {
  //  USBSerial.println("Failed to add peer");
  //}
  ESP_ERROR_CHECK( esp_now_add_peer(&peer) );

  // setup the callback queue
  initQueue();
  
  ESP_ERROR_CHECK( esp_now_register_recv_cb(onNowRecv) );

  // 6) “prime” to avoid any first-packet weirdness
  //uint8_t dummy[1] = {0};
  //for (int i=0;i<10;i++) { esp_now_send(cfg.peer_mac, dummy, sizeof(dummy)); delay(5); }
}

void show_splash(bool right_not_left)
{

  //gfx->fillScreen(BLACK);

  const uint8_t *src_bytes = right_not_left ? splash_right_map : splash_left_map;
  const uint16_t *logo_pixels = (const uint16_t *)src_bytes;   // RGB565 = 2 bytes/pixel

#ifdef HAS_GFX
  gfx->draw16bitRGBBitmap(
    0, 0,                    // x, y
    logo_pixels,
    LOGO_W,
    LOGO_H
  );

#endif  
  /*
  uint32_t n = (uint32_t)LOGO_W * (uint32_t)LOGO_H;

  gfx->startWrite();
  gfx->writeAddrWindow(0, 0, LOGO_W, LOGO_H);   // set region once
  gfx->writePixels((uint16_t *)logo_pixels, n, true);  // push all pixels
  gfx->endWrite();*/

}

void adcOn() {
  //power.enableTemperatureMeasure();
  // Enable internal ADC detection
  power.enableBattDetection();
  power.enableVbusVoltageMeasure();
  power.enableBattVoltageMeasure();
  //power.enableSystemVoltageMeasure();
}

void adcOff() {
  //power.disableTemperatureMeasure();
  // Enable internal ADC detection
  power.disableBattDetection();
  power.disableVbusVoltageMeasure();
  power.disableBattVoltageMeasure();
  //power.disableSystemVoltageMeasure();
}

int16_t mapCenteredToXbox(int raw, uint16_t centerRaw)
{
    // raw is assumed already in [0,4095]
    int d = raw - (int)centerRaw;

    if (d == 0) return 0;

    if (d > 0) {
        int posRange = 4095 - (int)centerRaw;      // available headroom to the right
        if (posRange <= 0) return 0;
        int32_t v = (int32_t)d * XBOX_STICK_MAX / posRange;
        if (v > XBOX_STICK_MAX) v = XBOX_STICK_MAX;
        return (int16_t)v;
    } else {
        int negRange = (int)centerRaw;             // available headroom to the left
        if (negRange <= 0) return 0;
        int32_t v = (int32_t)d * (-XBOX_STICK_MIN) / negRange; // d is negative
        if (v < XBOX_STICK_MIN) v = XBOX_STICK_MIN;
        return (int16_t)v;
    }
}

static inline uint16_t readAxisRaw(uint8_t pin, bool invert)
{
    int raw = analogRead(pin);
    raw = constrain(raw, 0, 4095);
    if (invert) raw = 4095 - raw;
    return (uint16_t)raw;
}

int16_t readJoyAxis(uint8_t pin, bool invert, uint16_t centerRaw)
{
    uint16_t raw = readAxisRaw(pin, invert);
    return mapCenteredToXbox((int)raw, centerRaw);
}

// measure center of joystick (assuming untouched!)
void calibrateJoyCenter()
{
    const int N = 64;
    uint32_t sx = 0, sy = 0;

    for (int i = 0; i < N; ++i) {
        sx += readAxisRaw(cfg.joyX, cfg.invertX);
        sy += readAxisRaw(cfg.joyY, cfg.invertY);
        delay(2);
    }

    joyCenterRawX = sx / N;
    joyCenterRawY = sy / N;
}


inline void applyRadialDeadzone(int16_t& x, int16_t& y, int16_t dz)
{
    int32_t mag2 = (int32_t)x * x + (int32_t)y * y;
    int32_t dz2  = (int32_t)dz * dz;

    if (mag2 <= dz2) {
        x = 0;
        y = 0;
    }
}

void setup() {
  USBSerial.begin(115200);
  USBSerial.setRxBufferSize(2048);
  delay(100);  // give the host time to open the port

  //USBSerial.println();
  //USBSerial.println("=== CyberFinger FW starting (PlatformIO) ===");
  //USBSerial.flush();

  //USBSerial.println("Starting provisionOrLoad()");
  //USBSerial.flush();
  provisionOrLoad(USBSerial, 500);
  USBSerial.println("provisionOrLoad() completed.");
  USBSerial.flush();

  Wire.begin(IIC_SDA, IIC_SCL);
  Wire.setClock(400000);

  expander = new EXAMPLE_CHIP_CLASS(TCA95xx_8bit,
                                    (i2c_port_t)0, ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_000,
                                    IIC_SCL, IIC_SDA);
  expander->init();
  expander->begin();
  expander->pinMode(0, OUTPUT);
  expander->pinMode(1, OUTPUT);
  expander->pinMode(2, OUTPUT);
  expander->digitalWrite(0, LOW);
  expander->digitalWrite(1, LOW);
  expander->digitalWrite(2, LOW);
  delay(20);
  expander->digitalWrite(0, HIGH);
  expander->digitalWrite(1, HIGH);
  expander->digitalWrite(2, HIGH);

  IIC_Bus = std::make_shared<Arduino_HWIIC>(IIC_SDA, IIC_SCL, &Wire);

  expander_present = true;
  // to read from the power button
  expander->pinMode(4, INPUT);

  // Battery sensor
  bool result = power.begin(Wire, AXP2101_SLAVE_ADDRESS, IIC_SDA, IIC_SCL);
  if (result == false) {
    USBSerial.println("PMU is not online...");
  }
  else {
    power.disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
    //power.setChargeTargetVoltage(2);
    // Clear all interrupt flags
    power.clearIrqStatus();
    // Enable the required interrupt function

    adcOn();
  }



  #ifdef HAS_SOUND
  if (cfg.play_sound) {
    // turn on audio
    pa_on();


    if (i2s_driver_init() != ESP_OK) {
      USBSerial.println("i2s driver init failed");
      abort();
    }
    else {
        USBSerial.println("i2s driver init suceeded");
    }

    if (es8311_codec_init() != ESP_OK) {
        USBSerial.println("es8311 codec init failed");
        abort();
    }
        else {
        USBSerial.println("es8311 codec init suceeded");

    }

    play_sound(SOUND_BOOT,0);

  }
  #endif
  

  #ifdef HAS_GFX

  bus = new Arduino_ESP32QSPI(LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);
  gfx = new Arduino_SH8601(bus, -1, 0, false, LCD_WIDTH, LCD_HEIGHT);

  gfx->begin();

  W = gfx->width();   // native width
  H = gfx->height();  // native height

  // Rotation not supported - See https://github.com/moononournation/Arduino_GFX/blob/master/src/display/Arduino_SH8601.cpp - "SH8601 does not support rotation"

  gfx->fillScreen(BLACK);

  gfx->setTextColor(BLUE);
  gfx->setTextSize(4);

  gfx->Display_Brightness(MAX_BRIGHTNESS-1);

  show_splash(cfg.right_not_left);
  
  /*
  for (int i = 0; i <= MAX_BRIGHTNESS; i++)  //0-255
  {
    gfx->Display_Brightness(i);
    delay(2);
  }*/

  #endif

  #ifdef HAS_TOUCH

  FT3168 = new Arduino_FT3x68(IIC_Bus, FT3168_DEVICE_ADDRESS, DRIVEBUS_DEFAULT_VALUE, TP_INT, Arduino_IIC_Touch_Interrupt);

  while (FT3168->begin() == false) {
    USBSerial.println("FT3168 initialization fail");
    delay(2000);
  }
  USBSerial.println("FT3168 initialization successfully");

  FT3168->IIC_Write_Device_State(FT3168->Arduino_IIC_Touch::Device::TOUCH_POWER_MODE,
                                FT3168->Arduino_IIC_Touch::Device_Mode::TOUCH_POWER_MONITOR);

    // … your expander & FT3168 init as before …

  String proxMode = FT3168->IIC_Read_Device_State(
      Arduino_IIC_Touch::Device::TOUCH_PROXIMITY_SENSING_MODE);
  USBSerial.printf("Proximity sensing mode = %s\n", proxMode.c_str());

  String gestureMode = FT3168->IIC_Read_Device_State(
      Arduino_IIC_Touch::Device::TOUCH_GESTUREID_MODE);
  USBSerial.printf("Gesture mode = %s\n", gestureMode.c_str());

  String pwr = FT3168->IIC_Read_Device_State(
      Arduino_IIC_Touch::Device::TOUCH_POWER_MODE);
  USBSerial.printf("Power mode = %s\n", pwr.c_str());

  #endif


    // zero‑out history
  for (auto &p : history) p = {0,0,0};

  const char *devName = cfg.right_not_left
        ? "CyberFinger-right-v1b"
        : "CyberFinger-left-v1b";
  
  compositeHID = new BleCompositeHID(devName, "SciCortex Technologies Corp.", 100);

  BLEHostConfiguration hostConfig;

  if (cfg.right_not_left) {

    if (!cfg.linux_mode) {
      // Uncomment one of the following two config types depending on which controller version you want to use
      // The XBox series X controller only works on linux kernels >= 6.5
      //XboxOneSControllerDeviceConfiguration* config = new XboxOneSControllerDeviceConfiguration();
      XboxSeriesXControllerDeviceConfiguration *config = new XboxSeriesXControllerDeviceConfiguration();

      // The composite HID device pretends to be a valid Xbox controller via vendor and product IDs (VID/PID).
      // Platforms like windows/linux need this in order to pick an XInput driver over the generic BLE GATT HID driver.
      hostConfig = config->getIdealHostConfiguration();
      // report the correct firmware version    

      // Set up gamepad
      xbox_gamepad = new XboxGamepadDevice(config);

      // Set up vibration event handler
      FunctionSlot<XboxGamepadOutputReportData> vibrationSlot(OnVibrateEvent);
      xbox_gamepad->onVibrate.attach(vibrationSlot);
      //TODO -> how to get L/R vibrate?

      xbox_out.set(xbox_gamepad);
      gamepad_out =  &xbox_out;
      gamepad_handler.setGamepad(&xbox_out);
    }
    else {

      generic_gamepad = new GamepadDevice();

      generic_out.set(generic_gamepad);
      gamepad_out = &generic_out;
      gamepad_handler.setGamepad(&generic_out);
    }

  }

  GamepadMerged::Config gpcfg;
  gpcfg.stickDeadzone = cfg.stick_deadzone;
  gpcfg.triggerDeadzone = cfg.trigger_deadzone;
  gamepad_handler.setConfig(gpcfg);
  // measure center of potentiometer
  calibrateJoyCenter();  
  
  // Set up mouse on both sides
  if (!cfg.linux_mode) mouse = new MouseDevice();

  USBSerial.println("Using VID source: " + String(hostConfig.getVidSource(), HEX));
  USBSerial.println("Using VID: " + String(hostConfig.getVid(), HEX));
  USBSerial.println("Using PID: " + String(hostConfig.getPid(), HEX));
  USBSerial.println("Using GUID version: " + String(hostConfig.getGuidVersion(), HEX));
  USBSerial.println("Using serial number: " + String(hostConfig.getSerialNumber()));

  analogReadResolution(12);

  //assign pins in the cfg to gpio behaviours
  pinMode(cfg.buttonAX, INPUT_PULLUP);
  pinMode(cfg.buttonBY, INPUT_PULLUP);
  pinMode(cfg.buttonST, INPUT_PULLUP);
  pinMode(cfg.buttonBP, INPUT_PULLUP);
  pinMode(cfg.buttonStartSelect, INPUT_PULLUP);

  analogSetPinAttenuation(cfg.joyX, ADC_11db);  // Enable full 0–3.3V range
  analogSetPinAttenuation(cfg.joyY, ADC_11db);

  // Add the mouse
  if (!cfg.linux_mode) compositeHID->addDevice(mouse);

  hostConfig.setFirmwareRevision(FW_VERSION_FULL_STR);
  if (cfg.right_not_left) {
    if (!cfg.linux_mode) {
    // right is gamepad

    //gamepad->resetButtons();
    xbox_gamepad->resetInputs();

    // Add all child devices to the top-level composite HID device to manage them
    compositeHID->addDevice(xbox_gamepad);
    }
    else {
      generic_gamepad->resetButtons();
      compositeHID->addDevice(generic_gamepad);
    }
  }

  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
  compositeHID->begin(hostConfig);

  if (cfg.right_not_left) {
    gamepad_out->setLeftThumb(0, 0);
    gamepad_out->setRightThumb(0, 0);
    gamepad_out->sendGamepadReport();
  }

  // Start comm with left/right
  initEspNow(mac);

  if (cfg.right_not_left) {
    // invert axes if needed
    tmouse.invertX = false;
    tmouse.invertY = true;
    tmouse.invertScrollV = true;   // "natural" scroll feel
    tmouse.invertScrollH = false;


  }
  else {
    // invert axes if needed
    tmouse.invertX = true;
    tmouse.invertY = false;
    tmouse.invertScrollV = false;   // "natural" scroll feel
    tmouse.invertScrollH = true;
  }
  if (!cfg.linux_mode) {
    TM_Callbacks cb{};
    cb.move = MouseMove;
    cb.press = MouseButtonPress;
    cb.release = MouseButtonRelease;
    cb.scroll = MouseScroll;
    cb.user = mouse;
    tmouse.begin(cb);
  }

  #ifdef HAS_GFX

  if (cfg.boot_debug) {

    gfx->fillScreen(WHITE);
    gfx->setTextColor(GREEN);
    gfx->setTextSize(2);
    gfx->setCursor(0, 0);
    gfx->println("CFG: "+configToJsonString()+"\n");

    gfx->setTextSize(2);
    gfx->setTextColor(RED);
    gfx->printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", 
      mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
    gfx->printf("PEER: %02X:%02X:%02X:%02X:%02X:%02X\n", 
      cfg.peer_mac[0],cfg.peer_mac[1],cfg.peer_mac[2],cfg.peer_mac[3],cfg.peer_mac[4],cfg.peer_mac[5]);
    delay(2000);
  }  
  delay(3000);
  gfx->fillScreen(BLACK);      // start with black
  gfx->Display_Brightness((int)(MAX_BRIGHTNESS*0.5));

  #endif

  if (cfg.right_not_left) {
    USBSerial.println("Start cyberfinger right device SUCCESS.");
    USBSerial.print("Right ESP-NOW sender MAC: "); 
  }
  else {
    USBSerial.println("Start cyberfinger left device SUCCESS.");
    USBSerial.print("Left ESP-NOW sender MAC: ");     
  }
  USBSerial.println(WiFi.macAddress());
}

void loop() {
  bool changed = false;
  int32_t x, y, x2, y2;

  // check battery status
  static uint32_t last = -100000;
  const uint32_t periodMs = 60 * 1000; // e.g. every 60s

  if (millis() - last >= periodMs) {
    last = millis();

    uint8_t pct = 100;
    if (power.isBatteryConnect()) {
      pct = (uint8_t)power.getBatteryPercent();
      if (pct > 100) pct = 100; // just in case
    }

    compositeHID->setBatteryLevel(pct);  // battery % to the host
  }

  // --- 1) handle any new touch and store it ---
  #ifdef HAS_TOUCH
  if (FT3168->IIC_Interrupt_Flag && !cfg.linux_mode) {
      FT3168->IIC_Interrupt_Flag = false;

      uint32_t num_fingers = (uint32_t)FT3168->IIC_Read_Device_Value(
          FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_FINGER_NUMBER);

      if (num_fingers>=2) 
      {
        USBSerial.printf("Multi-touch mouse event: %d", num_fingers);
        num_fingers = 2;
      }

      x = (int32_t)FT3168->IIC_Read_Device_Value(
          FT3168->Arduino_IIC_Touch::Value_Information::TOUCH1_COORDINATE_X);
      y = (int32_t)FT3168->IIC_Read_Device_Value(
          FT3168->Arduino_IIC_Touch::Value_Information::TOUCH1_COORDINATE_Y);

      x2 = 0; y2 = 0;
      if (num_fingers == 2) {
      x2 = (int32_t)FT3168->IIC_Read_Device_Value(
          FT3168->Arduino_IIC_Touch::Value_Information::TOUCH2_COORDINATE_X);
      y2 = (int32_t)FT3168->IIC_Read_Device_Value(
          FT3168->Arduino_IIC_Touch::Value_Information::TOUCH2_COORDINATE_Y);
      }

      tmouse.handleSampleMulti(num_fingers, y, x, y2, x2);

      if (x > 20 && y > 20) {
          history[histIdx] = { int16_t(x), int16_t(y), millis() };
          histIdx = (histIdx + 1) % MAX_HISTORY;
      }
  }
  #endif

  // --- 2) redraw the entire frame from history ---



  #ifdef HAS_GFX
  if (!poweroff_notice && frame % BLACK_FRAME_COUNT == 0) {
    gfx->fillScreen(BLACK);
  } 
  frame++;


  uint32_t now = millis();

  #ifdef HAS_TOUCH

  const uint8_t STEPS     = 8;           // number of color updates
  const uint8_t MAX_STEP  = STEPS - 1;   // 0…7
  // No more fillScreen()!

  // redraw each entry at its new brightness (or erase at step 0)
  for (uint8_t i = 0; i < MAX_HISTORY; i++) {
    uint8_t idx = (histIdx + i) % MAX_HISTORY;
    auto &pt = history[idx];
    if (pt.t == 0) continue;            // empty slot

    uint32_t age = now - pt.t;
    if (age >= FADE_TIME_MS) {
      // final erase: draw black circle, then forget this entry
      gfx->fillCircle(pt.x, pt.y, RADIUS, BLACK);
      pt.t = 0;
    } else {
      // map age→step in [MAX_STEP…0]
      uint8_t step = map(age, 0, FADE_TIME_MS, MAX_STEP, 0);
      // brightness in [0…255], 0→black, 255→full blue
      uint8_t b = (step * MAX_BRIGHTNESS) / MAX_STEP;
      uint16_t color = gfx->color565(b, 0, b);
      gfx->fillCircle(pt.x, pt.y, RADIUS, color);
    }
  }
  #endif //HAS_TOUCH
  #endif //HAS_GFX

  //USBSerial.println("OnNowRecv pkt age="+String(age_ms));

  // read the power button and give feedback of immenant shutdown
  int currPower = HIGH;
  if (expander_present) {
    currPower = expander->digitalRead(4);
  }

  if (currPower != prevPower) {
    if (currPower == HIGH) {
      USBSerial.println("Button Power pressed");
      time_powerpress = millis();
    } else {
      USBSerial.println("Button Power released");
      if (poweroff_notice) {
        #ifdef HAS_GFX
        gfx->fillScreen(BLACK);
        #endif //HAS_GFX
        poweroff_notice = false;
      }        
    }
    prevPower = currPower;
  }

  // give user feedback already at 2s ... to anticipate a power-down at ~5s (hardware implements that)
  if (currPower==HIGH && (millis() - time_powerpress)>2000) {

    if (!poweroff_notice) {
      #ifdef HAS_SOUND
      if (cfg.play_sound) play_sound(SOUND_SHUTDOWN, 1000);
      #endif
      #ifdef HAS_GFX
      show_splash(cfg.right_not_left);
      #endif
      poweroff_notice = true;
    }
  }

 
  // process gamepad
  HalfPacket local{};
  local.btnMask = 0;
  local.btnMask |= (digitalRead(cfg.buttonAX) == LOW) ? PKT_AX : 0;
  local.btnMask |= (digitalRead(cfg.buttonBY) == LOW) ? PKT_BY : 0;
  local.btnMask |= (digitalRead(cfg.buttonST)  == LOW) ? PKT_ST  : 0;
  local.btnMask |= (digitalRead(cfg.buttonBP)  == LOW) ? PKT_BP  : 0;
  local.btnMask |= (digitalRead(cfg.buttonStartSelect) == LOW) ? PKT_STARTSELECT : 0;

  /*
  // Read and scale joystick values
  //uint16_t joyX = map(analogRead(pinJoyX), 0, 4095, 0, XBOX_STICK_MAX);
  //uint16_t joyY = map(analogRead(pinJoyY), 0, 4095, 0, XBOX_STICK_MAX);
  uint16_t rawX = analogRead(cfg.joyX);
  uint16_t rawY = analogRead(cfg.joyY);
  rawX = constrain(rawX, 0, 4095);
  rawY = constrain(rawY, 0, 4095);
  if (cfg.invertX) rawX = 4095 - rawX;  // the X-axis is flipped, due to how it's mounted
  if (cfg.invertY) rawY = 4095 - rawY; 
  int16_t joyX = map(rawX, 0, 4095, XBOX_STICK_MIN, XBOX_STICK_MAX);
  int16_t joyY = map(rawY, 0, 4095, XBOX_STICK_MIN, XBOX_STICK_MAX);
  */

  //local.jx = readJoyX();
  //local.jy = readJoyY();
  int16_t joyX = readJoyAxis(cfg.joyX, cfg.invertX, joyCenterRawX);
  int16_t joyY = readJoyAxis(cfg.joyY, cfg.invertY, joyCenterRawY);

  applyRadialDeadzone(joyX, joyY, cfg.stick_deadzone);
  local.jx = joyX;
  local.jy = joyY;
  
  if (cfg.right_not_left) {
    gamepad_handler.setRightLocal(local);
    HalfPacket u;
    while(xQueueReceive(g_pkt_queue, &u, 0) == pdTRUE) {
      gamepad_handler.setLeftRemote(u);
    }
    if (compositeHID->isConnected()) gamepad_handler.updateAndSendIfChanged();
  }
  else {
    CyberPkt16 u;
    //bool mouse_changed=false;
    //int16_t  mx=0, my=0; 

    gamepad_handler.setLeftLocal(local);

    // wait at least cfg.esp_interval_us before sending again
    // wait no more than MAX_INTERVAL_US until sending again
    uint32_t now = micros();
    if ((now - lastSendUs) >= std::min(cfg.esp_interval_us, (uint16_t)1000)) {
      if (gamepad_handler.sendLeftPacketToRightIfChanged(cfg.peer_mac)) {
        lastSendUs = now;
      } else if ((now - lastSendUs) >= MAX_INTERVAL_US) {
        gamepad_handler.sendLeftPacketToRight(cfg.peer_mac);
        lastSendUs = now;
      }
    }

    /*
    while (xQueueReceive(g_pkt_queue, &u, 0) == pdTRUE) {
      // process all queued packets
      // onNowRecv already checked version and source
      if (u.mouse.buttons | BTN_MOUSE1_PRESS ) {mouse->mousePress(MOUSE_LOGICAL_LEFT_BUTTON); mouse_changed=true;}
      if (u.mouse.buttons | BTN_MOUSE1_RELEASE ) {mouse->mouseRelease(MOUSE_LOGICAL_LEFT_BUTTON); mouse_changed=true;}
      if (u.mouse.mx!=0 || u.mouse.my!=0) {mx+=u.mouse.mx; my+=u.mouse.my; mouse_changed=true;}
    }
    if (mx!=0 || my!=0) {
      mouse->mouseMove(mx, my);
      mouse_changed=true;
    }
    else mouse->mouseMove(0,0);

    if (compositeHID->isConnected() && mouse_changed) mouse->sendMouseReport();
    */
    
  }
  vTaskDelay(2);
}




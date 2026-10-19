/*
  ============================================================
            SENTER RACING - MAIN FIRMWARE
            ESP8266 / Wemos D1 Mini
            VERSION - LiPo 3S
  ============================================================
  BATTERY
  ------------------------------------------------------------
  LiPo 3S
  FULL      = 12.60V
  NOMINAL   = 11.10V
  CRITICAL  = 9.00V
  RECOVERY  = 10.80V

  VOLTAGE DIVIDER (DI ARDUINO NANO)
  ------------------------------------------------------------
  R1 = 100K
  R2 = 15K

  12.60V battery -> sekitar 1.64V ke A0 Nano
  Wemos meminta data baterai dari Nano setiap 250 ms.
  Nano membalas request dengan sequence dan nilai millivolt.

  PIN
  ------------------------------------------------------------
  D1  -> OLED SCL
  D2  -> OLED SDA
  D3  -> BUTTON MINUS
  D4  -> DS18B20
  D5  -> LED PWM
  D6  -> MAIN BUTTON
  D7  -> BUTTON PLUS
  D0  -> FAN PWM
  D8  -> UNUSED
  RX  <- Nano D1/TX (9600 baud; pembagi: 1K seri, 1.8K ke GND)
  TX  -> Nano D0/RX (9600 baud; logika 3.3V Wemos aman untuk Nano)

  Wemos sends Q,<sequence>,<checksum>; Nano replies with
  R,<sequence>,<millivolts>,<checksum>. Checksum is XOR before final comma.

  NANO DISCONNECTED
  ------------------------------------------------------------
  OLED yang sama menampilkan peringatan jika Nano tidak terdeteksi.

  MODE
  ------------------------------------------------------------
  1x MAIN = LOW
  2x MAIN = MEDIUM
  3x MAIN = HIGH
  4x MAIN = STROBO
  5x MAIN = TURBO

  LED ON
  ------------------------------------------------------------
  1x short press = OFF
  long press     = LOW -> MED -> HIGH -> LOW

  LOW BATTERY
  ------------------------------------------------------------
  MEDIUM -> diarahkan LOW
  HIGH   -> diarahkan LOW
  TURBO  -> diarahkan LOW
  STROBO -> tetap STROBO, max 15%
  LOW    -> tetap LOW, max 15%
*/
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <stdio.h>
bool ledOn = false;
float userLEDTarget = 0.0;
float ledPWM = 0.0;
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SDA_PIN D2
#define SCL_PIN D1
Adafruit_SSD1306 display(
  SCREEN_WIDTH,
  SCREEN_HEIGHT,
  &Wire,
  OLED_RESET
);
bool oledOK = false;
bool warningDisplayInitialized = false;
bool lastNanoWarningState = false;
#define LED_PWM_PIN D5
#define FAN_PWM_PIN D0
#define BTN_MINUS D3
#define BTN_MAIN  D6
#define BTN_PLUS  D7
#define DS18B20_PIN D4
OneWire oneWire(DS18B20_PIN);
DallasTemperature sensors(&oneWire);
float temperatureC = 25.0;
bool temperatureValid = false;
bool temperatureRequestPending = false;
unsigned long temperatureRequestStart = 0;
#define TEMP_CONVERSION_TIME 800
#define PWM_FREQUENCY_DEFAULT_KHZ 2
#define PWM_FREQUENCY_MIN_KHZ 1
#define PWM_FREQUENCY_MAX_KHZ 20
#define PWM_MAX 1023
unsigned long modeMemoryChangeAt = 0;
const unsigned long MODE_MEMORY_SAVE_DELAY = 2000UL;
enum LightMode {
  MODE_LOW,
  MODE_MEDIUM,
  MODE_HIGH,
  MODE_STROBE,
  MODE_TURBO
};
LightMode currentMode = MODE_LOW;
float lowOutputLimit = 15.0;
float mediumOutputLimit = 25.0;
float highOutputLimit = 40.0;
float turboOutputLimit = 100.0;
bool turboTimeoutApplied = false;
#define BATTERY_SERIAL_BAUD 9600
#define BATTERY_SERIAL_TIMEOUT 1000UL
#define NANO_CONNECTION_TIMEOUT 6000UL
#define BATTERY_REQUEST_INTERVAL 250UL
#define BATTERY_MAX_MILLIVOLTS 16000UL
#define BATTERY_FILTER_ALPHA 0.30
#define BATTERY_FULL_VOLTAGE      12.60
#define BATTERY_NOMINAL_VOLTAGE   11.10
#define BATTERY_CRITICAL_DEFAULT  9.00
float criticalBatteryVoltage = BATTERY_CRITICAL_DEFAULT;
#define BATTERY_RECOVERY_DEFAULT  10.80
float recoveryBatteryVoltage = BATTERY_RECOVERY_DEFAULT;
#define BATTERY_READ_INTERVAL 250
#define BATTERY_LOW_PERCENT_DEFAULT 20
float lowBatteryWarningVoltage = 11.00;
uint8_t lowBatteryOutputLimit = 15;
#define BATTERY_BLINK_INTERVAL 500
float batteryVoltage = 0.0;
float filteredBatteryVoltage = -1.0;
float batteryDisplayVoltage = -1.0;  // Filter khusus tampilan OLED agar angka desimal stabil.
int batteryPercent = 0;
bool batteryCritical = false;
bool batteryPresent = false;
bool nanoConnected = false;
unsigned long lastBatteryRead = 0;
unsigned long lastBatteryPacketAt = 0;
uint16_t receivedBatteryMillivolts = 0;
bool batteryPacketReceived = false;
uint16_t lastBatteryRequestSequence = 0;
unsigned long lastBatteryRequestAt = 0;
bool batteryRequestSent = false;
bool batteryRequestPending = false;
char batterySerialBuffer[24];
uint8_t batterySerialLength = 0;
bool batterySerialOverflow = false;
uint32_t nanoRxByteCount = 0;
uint16_t nanoResponseCount = 0;
uint16_t nanoRejectedFrameCount = 0;
uint16_t nanoChecksumErrorCount = 0;
uint16_t nanoSequenceErrorCount = 0;
unsigned long lastNanoDiagnosticDisplayAt = 0;
unsigned long lastBatteryBlink = 0;
bool batteryBlinkState = true;
const char* getModeName() {
  switch (currentMode) {
    case MODE_LOW:
      return "LOW";
    case MODE_MEDIUM:
      return "MED";
    case MODE_HIGH:
      return "HIGH";
    case MODE_STROBE:
      return "STROBO";
    case MODE_TURBO:
      return "TURBO";
  }
  return "LOW";
}
float getModeMin() {
  switch (currentMode) {
    case MODE_LOW:
      return 0.1;
    case MODE_MEDIUM:
      return 0.1;
    case MODE_HIGH:
      return 0.1;
    case MODE_STROBE:
      return 0.1;
    case MODE_TURBO:
      return 1.0;
  }
  return 0.1;
}

float getModeMax() {
  switch (currentMode) {
    case MODE_LOW:
      return lowOutputLimit;
    case MODE_MEDIUM:
      return mediumOutputLimit;
    case MODE_HIGH:
      return highOutputLimit;
    case MODE_STROBE:
      return 100.0;
    case MODE_TURBO:
      return turboTimeoutApplied ? min(turboOutputLimit, highOutputLimit) : turboOutputLimit;
  }
  return 15.0;
}

float getEffectiveModeMax() {
  if (batteryCritical) return min(getModeMax(), (float)lowBatteryOutputLimit);
  if (batteryPresent && batteryVoltage > criticalBatteryVoltage && batteryVoltage <= lowBatteryWarningVoltage) {
    return min(getModeMax(), (float)lowBatteryOutputLimit);
  }
  return getModeMax();
}

#define EEPROM_SIZE 256
#define EEPROM_MAGIC 0x53454E54UL
#define EEPROM_VERSION 2
#define EEPROM_ADDR 0
#define EEPROM_STROBE_SPEED_ADDR 32
#define STROBE_SPEED_MIN 1
#define STROBE_SPEED_MAX 30
#define STROBE_SPEED_DEFAULT 17
struct ModeMemoryData {
  uint32_t magic;
  uint8_t version;
  float low;
  float medium;
  float high;
  float strobo;
  float turbo;
};
ModeMemoryData storedMemory;
float lowMemory = 10.0;
float mediumMemory = 18.0;
float highMemory = 30.0;
float strobeMemory = 60.0;
float turboMemory = 100.0;
bool modeMemoryDirty = false;
bool buttonsLocked = false;
bool strobeState = false;
unsigned long lastStrobe = 0;
uint8_t strobeSpeedHz = STROBE_SPEED_DEFAULT;
bool strobeSpeedDirty = false;
float getModeMemory() {
  switch (currentMode) {
    case MODE_LOW:
      return lowMemory;
    case MODE_MEDIUM:
      return mediumMemory;
    case MODE_HIGH:
      return highMemory;
    case MODE_STROBE:
      return strobeMemory;
    case MODE_TURBO:
      return turboMemory;
  }
  return 10.0;
}

unsigned long getStrobeInterval() {
  return 1000UL /
         (2UL * (unsigned long)strobeSpeedHz);
}
#define LED_DOWN_SPEED_THERMAL 0.10
#define LED_SMOOTH_INTERVAL 10
unsigned long lastLEDSmooth = 0;
int fanPWM = 0;
int lastFanPWMOutput = -1;
#define LED_POWER_DEFAULT 60.0
bool settingMode = false;
bool settingEditing = false;
bool settingResetConfirm = false;
uint8_t settingResetChoice = 0; // 0=cancel, 1=confirm reset
// Settings UI levels: 0=category list, 1=category items, editing=true=value editor.
uint8_t settingPage = 0;
uint8_t settingCategoryIndex = 0;
uint8_t settingSubmenuIndex = 0;
uint8_t settingMenuIndex = 0; // current flat setting ID, used by the editor
uint8_t settingEditField = 0;
#define SETTING_HOLD_TIME 5000UL
#define EEPROM_FAN_LIMIT_ADDR 40
#define EEPROM_BATTERY_CAL_ADDR 44
#define EEPROM_FADE_ENABLED_ADDR 52
#define EEPROM_FADE_DURATION_ADDR 53
#define EEPROM_LED_WATT_ADDR 56
#define EEPROM_OLED_BRIGHTNESS_ADDR 60
#define EEPROM_DISPLAY_TIMEOUT_ADDR 61
#define EEPROM_DISPLAY_TIMEOUT_UNIT_ADDR 67
#define EEPROM_SHOW_VOLTAGE_ADDR 62
#define EEPROM_SHOW_BATTERY_PERCENT_ADDR 63
#define EEPROM_SHOW_POWER_ADDR 64
#define EEPROM_SHOW_TEMPERATURE_ADDR 65
#define EEPROM_SHOW_FAN_ADDR 66
#define EEPROM_FADE_OUT_ENABLED_ADDR 68
#define EEPROM_FADE_OUT_DURATION_ADDR 69
#define EEPROM_LOW_OUTPUT_ADDR 72
#define EEPROM_MED_OUTPUT_ADDR 76
#define EEPROM_HIGH_OUTPUT_ADDR 80
#define EEPROM_TURBO_OUTPUT_ADDR 84
#define EEPROM_TURBO_TIMEOUT_ADDR 88
#define EEPROM_FAN_MIN_SPEED_ADDR 89
#define EEPROM_SHOW_PWM_ADDR 92
#define EEPROM_DISPLAY_MODE_ADDR 93
#define EEPROM_FAN_START_TEMP_ADDR 94
#define EEPROM_FAN_STOP_TEMP_ADDR 98
#define EEPROM_FAN_MAX_SPEED_ADDR 102
#define EEPROM_LED_TEMP_LIMIT_ADDR 104
#define EEPROM_LOW_BATT_WARN_ADDR 108
#define EEPROM_LOW_BATT_LIMIT_ADDR 112
#define EEPROM_CRITICAL_VOLT_ADDR 116
#define EEPROM_RECOVERY_VOLT_ADDR 120
#define EEPROM_PWM_FREQUENCY_ADDR 124
#define EEPROM_TURBO_FAN_MODE_ADDR 125
#define EEPROM_TURBO_FAN_START_ADDR 126
uint8_t pwmFrequencyKHz = PWM_FREQUENCY_DEFAULT_KHZ;
bool pwmFrequencyDirty = false;
float fanTempLimit = 70.0; // retained for compatibility with existing config
float fanStartTemp = 45.0;
float fanStopTemp = 40.0;
uint8_t fanMaxSpeed = 100;
bool fanTemperatureActive = false;
bool turboFanBoostEnabled = false;
uint8_t turboFanStartSpeed = 30;
float batteryCalOffset = 0.0;
bool fadeEnabled = true;          // Fade IN
uint8_t fadeDurationSec = 3;      // Fade IN duration
bool fadeOutEnabled = true;
uint8_t fadeOutDurationSec = 3;
uint8_t turboTimeoutSec = 30;     // 0 = disabled; otherwise reduce Turbo to HIGH ceiling
unsigned long modeStartedAt = 0;
uint8_t fanMinSpeed = 30;         // minimum automatic fan speed when active
float ledWattCapacity = LED_POWER_DEFAULT;
uint8_t oledBrightness = 70; // 1-100%
uint8_t displayTimeoutValue = 1;
bool displayTimeoutInMinutes = false;
uint8_t displayMode = 2;
bool showPWM = true;
bool showVoltage = true;
bool showBatteryPercent = true;
bool showPowerWatt = true;
bool showTemperature = true;
bool showFanStatus = true;
unsigned long oledIdleSince = 0;
bool settingMainRaw = HIGH;
bool settingMainStable = HIGH;
unsigned long settingMainDebounceAt = 0;
unsigned long settingMainPressAt = 0;
bool settingMainLongDone = false;
bool settingMainBackDone = false;
#define SETTING_BACK_HOLD_TIME 1000UL
#define SETTING_VALUE_REPEAT_DELAY 400UL
#define SETTING_VALUE_REPEAT_INTERVAL 150UL
bool settingPlusRaw = HIGH;
bool settingPlusStable = HIGH;
unsigned long settingPlusDebounceAt = 0;
unsigned long settingPlusRepeatAt = 0;
bool settingMinusRaw = HIGH;
bool settingMinusStable = HIGH;
unsigned long settingMinusDebounceAt = 0;
unsigned long settingMinusRepeatAt = 0;
#define SETTING_DEBOUNCE_MS 25
#define SETTING_MENU_COUNT 15
#define SETTING_VISIBLE_ROWS 5
#define THERMAL_START 75.0
#define THERMAL_CUTOFF 85.0 // independent hard safety cutoff
float ledTempLimit = 75.0;
#define THERMAL_RECOVERY 68.0
bool thermalProtect = false;
bool thermalEmergency = false;
bool safeMode = false;
bool tempFault = false;
bool coolingFault = false;
bool oledDisplayActive = true;
#define OLED_OFF_DELAY 3000
unsigned long lastWarningBlink = 0;
bool warningBlinkState = true;
#define WARNING_BLINK_INTERVAL 500
#define SAFE_PWM_LIMIT 10.0
int coolingRiseCounter = 0;
float previousTemperature = 25.0;
#define COOLING_FAULT_COUNT 3
float ledPower = 0.00;
float getEstimatedLEDPower() {
  float power =
    (ledPWM / 100.0) *
    ledWattCapacity;
  return constrain(
           power,
           0.0,
           ledWattCapacity
         );
}
bool mainRawState = HIGH;
bool mainStableState = HIGH;
unsigned long mainDebounceStart = 0;
unsigned long mainPressStart = 0;
unsigned long mainLastRelease = 0;
bool mainLongHandled = false;
uint8_t mainClickCount = 0;
#define MAIN_LONG_PRESS 700
#define MAIN_CLICK_TIMEOUT 250
#define MAIN_DEBOUNCE 25
bool plusLastState = HIGH;
unsigned long plusPressStart = 0;
unsigned long plusLastRepeat = 0;
bool plusFirstRepeatDone = false;
#define BUTTON_REPEAT_START 450
#define BUTTON_REPEAT_INTERVAL 80
bool minusLastState = HIGH;
unsigned long minusPressStart = 0;
unsigned long minusLastRepeat = 0;
bool minusFirstRepeatDone = false;
unsigned long bothHoldStart = 0;
bool bothLockHandled = false;
#define BOTH_HOLD_TIME 1500
unsigned long lastTempRead = 0;
unsigned long lastThermalControl = 0;
unsigned long lastDisplay = 0;
const unsigned long TEMP_INTERVAL = 1000;
const unsigned long THERMAL_INTERVAL = 1000;
const unsigned long DISPLAY_INTERVAL = 250;
float getSafetyTarget() {
  float target =
    userLEDTarget;
  if (currentMode == MODE_TURBO && turboTimeoutApplied && target > highOutputLimit) {
    target = highOutputLimit;
  }
  if (!ledOn) {
    return 0.0;
  }
  // Soft low-battery output ceiling; critical protection below still forces OFF.
  if (batteryPresent && batteryVoltage > criticalBatteryVoltage && batteryVoltage <= lowBatteryWarningVoltage) {
    target = min(target, (float)lowBatteryOutputLimit);
  }
  if (batteryCritical) {
    return 0.0;
  }
  if (safeMode) {
    if (      target >
              SAFE_PWM_LIMIT
       ) {
      target =
        SAFE_PWM_LIMIT;
    }
  }
  if (thermalEmergency) {
    target = 0.0;
  }
  return target;
}

void executeMainClicks() {
  uint8_t clicks = mainClickCount;
  mainClickCount = 0;
  mainLastRelease = 0;
  if (clicks == 0) {
    return;
  }
  if (clicks == 1) {
    if (ledOn) {
      ledOff();
    } else {
      setMode(MODE_LOW);
    }
    return;
  }
  if (clicks == 2) {
    setMode(MODE_MEDIUM);
    return;
  }
  if (clicks == 3) {
    setMode(MODE_HIGH);
    return;
  }
  if (clicks == 4) {
    setMode(MODE_STROBE);
    return;
  }
  setMode(MODE_TURBO);
}

void handlePlusButton() {
  bool state = digitalRead(BTN_PLUS);
  unsigned long now = millis();
  if (buttonsLocked) {
    plusLastState = state;
    return;
  }
  if (currentMode == MODE_STROBE && ledOn && digitalRead(BTN_MAIN) == LOW) {
    if (state == LOW && plusLastState == HIGH) {
      changeStrobeSpeed(+1);
    }
    plusLastState = state;
    return;
  }
  if (state == LOW && plusLastState == HIGH) {
    plusPressStart = now;
    plusLastRepeat = now;
    plusFirstRepeatDone = false;
    changeTarget(1.0);
  }
  if (state == LOW) {
    if (now - plusPressStart >= BUTTON_REPEAT_START) {
      if (!plusFirstRepeatDone) {
        plusFirstRepeatDone = true;
        plusLastRepeat = now;
      }
      if (now - plusLastRepeat >= BUTTON_REPEAT_INTERVAL) {
        plusLastRepeat = now;
        changeTarget(1.0);
      }
    }
  }
  plusLastState = state;
}

void handleMinusButton() {
  bool state = digitalRead(BTN_MINUS);
  unsigned long now = millis();
  if (buttonsLocked) {
    minusLastState = state;
    return;
  }
  if (currentMode == MODE_STROBE && ledOn && digitalRead(BTN_MAIN) == LOW) {
    if (state == LOW && minusLastState == HIGH) {
      changeStrobeSpeed(-1);
    }
    minusLastState = state;
    return;
  }
  if (state == LOW && minusLastState == HIGH) {
    minusPressStart = now;
    minusLastRepeat = now;
    minusFirstRepeatDone = false;
    changeTarget(-1.0);
  }
  if (state == LOW) {
    if (now - minusPressStart >= BUTTON_REPEAT_START) {
      if (!minusFirstRepeatDone) {
        minusFirstRepeatDone = true;
        minusLastRepeat = now;
      }
      if (now - minusLastRepeat >= BUTTON_REPEAT_INTERVAL) {
        minusLastRepeat = now;
        changeTarget(-1.0);
      }
    }
  }
  minusLastState = state;
}

void handleLockButtons() {
  bool plus = digitalRead(BTN_PLUS) == LOW;
  bool minus = digitalRead(BTN_MINUS) == LOW;
  unsigned long now = millis();
  if (plus && minus) {
    if (bothHoldStart == 0) {
      bothHoldStart = now;
    }
    if (!bothLockHandled && now - bothHoldStart >= BOTH_HOLD_TIME) {
      buttonsLocked = !buttonsLocked;
      bothLockHandled = true;
    }
  } else {
    bothHoldStart = 0;
    bothLockHandled = false;
  }
}

void forceSettingOutputsOff() {
  ledOn = false;
  userLEDTarget = 0.0;
  ledPWM = 0.0;
  outputLEDPWM(0);
  fanPWM = 0;
  lastFanPWMOutput = 0;
  analogWrite(FAN_PWM_PIN, 0);
  oledDisplayActive = true;
  if (oledOK) display.ssd1306_command(SSD1306_DISPLAYON);
}
enum SettingId : uint8_t {
  SET_STROBE_SPEED = 0, SET_PWM_FREQUENCY, SET_LED_WATT, SET_LOW_OUTPUT, SET_MED_OUTPUT, SET_HIGH_OUTPUT,
  SET_TURBO_OUTPUT, SET_TURBO_TIMEOUT, SET_FADE_IN_ENABLE, SET_FADE_IN_TIME,
  SET_FADE_OUT_ENABLE, SET_FADE_OUT_TIME, SET_FAN_TEMP_LIMIT, SET_FAN_START_TEMP,
  SET_FAN_STOP_TEMP, SET_FAN_MIN_SPEED, SET_FAN_MAX_SPEED, SET_LED_TEMP_LIMIT,
  SET_BATTERY_CAL, SET_LOW_BATT_WARNING, SET_LOW_BATT_LIMIT, SET_CRITICAL_VOLTAGE,
  SET_RECOVERY_VOLTAGE, SET_SHOW_PWM, SET_SHOW_VOLTAGE, SET_SHOW_BATTERY_PERCENT,
  SET_SHOW_POWER, SET_SHOW_TEMPERATURE, SET_SHOW_FAN, SET_DISPLAY_MODE,
  SET_DISPLAY_TIMEOUT, SET_OLED_BRIGHTNESS, SET_TURBO_FAN_MODE,
  SET_TURBO_FAN_START, SET_RESET_DEFAULTS,
  SETTING_TOTAL_COUNT
};
const char *settingCategoryLabels[] = {
  "DISPLAY SETTINGS", "LED & POWER", "FAN & TEMPT", "BATTERY SETTINGS", "SYSTEM"
};
#define SETTING_CATEGORY_COUNT 5

const uint8_t settingCategoryItems[SETTING_CATEGORY_COUNT][12] = {
  {SET_SHOW_PWM, SET_SHOW_VOLTAGE, SET_SHOW_BATTERY_PERCENT, SET_SHOW_POWER, SET_SHOW_TEMPERATURE, SET_SHOW_FAN, SET_DISPLAY_MODE, SET_DISPLAY_TIMEOUT, SET_OLED_BRIGHTNESS, 255, 255},
  {SET_PWM_FREQUENCY, SET_LED_WATT, SET_LOW_OUTPUT, SET_MED_OUTPUT, SET_HIGH_OUTPUT, SET_TURBO_OUTPUT, SET_TURBO_TIMEOUT, SET_FADE_IN_ENABLE, SET_FADE_IN_TIME, SET_FADE_OUT_ENABLE, SET_FADE_OUT_TIME, SET_STROBE_SPEED},
  {SET_FAN_TEMP_LIMIT, SET_FAN_START_TEMP, SET_FAN_STOP_TEMP, SET_FAN_MIN_SPEED, SET_FAN_MAX_SPEED, SET_LED_TEMP_LIMIT, SET_TURBO_FAN_MODE, SET_TURBO_FAN_START, 255, 255, 255, 255},
  {SET_BATTERY_CAL, SET_LOW_BATT_WARNING, SET_LOW_BATT_LIMIT, SET_CRITICAL_VOLTAGE, SET_RECOVERY_VOLTAGE, 255, 255, 255, 255, 255, 255},
  {SET_RESET_DEFAULTS, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255}
};

const uint8_t settingCategoryItemCounts[SETTING_CATEGORY_COUNT] = {9, 12, 8, 5, 1};
uint8_t getCurrentSettingId() {
  if (settingCategoryIndex >= SETTING_CATEGORY_COUNT) return SET_STROBE_SPEED;
  if (settingSubmenuIndex >= settingCategoryItemCounts[settingCategoryIndex]) return SET_STROBE_SPEED;
  return settingCategoryItems[settingCategoryIndex][settingSubmenuIndex];
}
const char *getSettingLabel(uint8_t id) {
  switch (id) {
    case SET_SHOW_PWM: return "PWM";
    case SET_SHOW_VOLTAGE: return "VOLT";
    case SET_SHOW_BATTERY_PERCENT: return "BATT %";
    case SET_SHOW_POWER: return "WATT";
    case SET_SHOW_TEMPERATURE: return "TEMPT";
    case SET_SHOW_FAN: return "FAN";
    case SET_DISPLAY_MODE: return "DISPLAY MODE";
    case SET_DISPLAY_TIMEOUT: return "TIMEOUT";
    case SET_OLED_BRIGHTNESS: return "BRIGHTNESS";
    case SET_PWM_FREQUENCY: return "PWM FREQUENCY";
    case SET_LED_WATT: return "WATT CAPACITY";
    case SET_LOW_OUTPUT: return "LOW LIMIT";
    case SET_MED_OUTPUT: return "MED LIMIT";
    case SET_HIGH_OUTPUT: return "HIGH LIMIT";
    case SET_TURBO_OUTPUT: return "POWER LIMIT";
    case SET_TURBO_TIMEOUT: return "TURBO TIMEOUT";
    case SET_FADE_IN_ENABLE: return "FADE IN";
    case SET_FADE_IN_TIME: return "FADE IN SPEED";
    case SET_FADE_OUT_ENABLE: return "FADE OUT";
    case SET_FADE_OUT_TIME: return "FADE OUT SPEED";
    case SET_STROBE_SPEED: return "STROBE SPEED";
    case SET_FAN_START_TEMP: return "FAN START C";
    case SET_FAN_STOP_TEMP: return "FAN STOP C";
    case SET_FAN_MIN_SPEED: return "FAN MIN %";
    case SET_FAN_MAX_SPEED: return "FAN MAX %";
    case SET_FAN_TEMP_LIMIT: return "FAN MAX TEMP";
    case SET_LED_TEMP_LIMIT: return "LED TEMP LIMIT";
    case SET_TURBO_FAN_MODE: return "TURBO FAN MODE";
    case SET_TURBO_FAN_START: return "TURBO START %";
    case SET_BATTERY_CAL: return "VOLT CAL";
    case SET_LOW_BATT_WARNING: return "LOW BATT WARN";
    case SET_LOW_BATT_LIMIT: return "LOW BATT LIMIT";
    case SET_CRITICAL_VOLTAGE: return "CRITICAL VOLT";
    case SET_RECOVERY_VOLTAGE: return "RECOVERY VOLT";
    case SET_RESET_DEFAULTS: return "RESET ALL DEFAULT";
    default: return "SETTING";
  }
}

void applyPWMFrequency() {
  analogWriteFreq((uint32_t)pwmFrequencyKHz * 1000UL);
}

void adjustSettingValue(int direction) {
  uint8_t id = getCurrentSettingId();
  switch (id) {
    case SET_SHOW_PWM: showPWM = !showPWM; break;
    case SET_SHOW_VOLTAGE: showVoltage = !showVoltage; break;
    case SET_SHOW_BATTERY_PERCENT: showBatteryPercent = !showBatteryPercent; break;
    case SET_SHOW_POWER: showPowerWatt = !showPowerWatt; break;
    case SET_SHOW_TEMPERATURE: showTemperature = !showTemperature; break;
    case SET_SHOW_FAN: showFanStatus = !showFanStatus; break;
    case SET_FADE_IN_ENABLE: fadeEnabled = !fadeEnabled; break;
    case SET_FADE_OUT_ENABLE: fadeOutEnabled = !fadeOutEnabled; break;
    case SET_DISPLAY_MODE: displayMode = (displayMode + (direction > 0 ? 1 : 2)) % 3; oledIdleSince = millis(); if (displayMode == 0 || (displayMode == 1 && ledOn) || displayMode == 2) {
        oledDisplayActive = true;
        if (oledOK) display.ssd1306_command(SSD1306_DISPLAYON);
      } break;
    case SET_DISPLAY_TIMEOUT:
      if (settingEditField == 0) {
        displayTimeoutValue = (uint8_t)constrain((int)displayTimeoutValue + direction, 1, 60);
      } else {
        displayTimeoutInMinutes = !displayTimeoutInMinutes;
      }
      oledIdleSince = millis();
      break;
    case SET_OLED_BRIGHTNESS: oledBrightness = (uint8_t)constrain((int)oledBrightness + direction * 5, 1, 100); applyOLEDSettings(); break;
    case SET_PWM_FREQUENCY: {
        uint8_t previousFrequency = pwmFrequencyKHz;
        pwmFrequencyKHz = (uint8_t)constrain((int)pwmFrequencyKHz + direction, PWM_FREQUENCY_MIN_KHZ, PWM_FREQUENCY_MAX_KHZ);
        if (pwmFrequencyKHz != previousFrequency) {
          applyPWMFrequency();
          pwmFrequencyDirty = true;
        }
        break;
      }
    case SET_LED_WATT: ledWattCapacity = constrain(ledWattCapacity + direction, 1.0, 100.0); break;
    case SET_LOW_OUTPUT: lowOutputLimit = constrain(lowOutputLimit + direction, 1.0, 30.0); break;
    case SET_MED_OUTPUT: mediumOutputLimit = constrain(mediumOutputLimit + direction, 1.0, 50.0); break;
    case SET_HIGH_OUTPUT: highOutputLimit = constrain(highOutputLimit + direction, 1.0, 75.0); break;
    case SET_TURBO_OUTPUT: turboOutputLimit = constrain(turboOutputLimit + direction, 1.0, 100.0); break;
    case SET_TURBO_TIMEOUT: turboTimeoutSec = (uint8_t)constrain((int)turboTimeoutSec + direction * 5, 0, 180); turboTimeoutApplied = false; modeStartedAt = millis(); break;
    case SET_FADE_IN_TIME: fadeDurationSec = (uint8_t)constrain((int)fadeDurationSec + direction, 1, 10); break;
    case SET_FADE_OUT_TIME: fadeOutDurationSec = (uint8_t)constrain((int)fadeOutDurationSec + direction, 1, 10); break;
    case SET_STROBE_SPEED: strobeSpeedHz = (uint8_t)constrain((int)strobeSpeedHz + direction, STROBE_SPEED_MIN, STROBE_SPEED_MAX); strobeSpeedDirty = true; saveStrobeSpeed(); break;
    case SET_FAN_TEMP_LIMIT: fanTempLimit = constrain(fanTempLimit + direction, 45.0, 80.0); break;
    case SET_FAN_START_TEMP: fanStartTemp = constrain(fanStartTemp + direction, 35.0, 75.0); if (fanStartTemp <= fanStopTemp) fanStartTemp = fanStopTemp + 1; break;
    case SET_FAN_STOP_TEMP: fanStopTemp = constrain(fanStopTemp + direction, 25.0, 70.0); if (fanStopTemp >= fanStartTemp) fanStopTemp = fanStartTemp - 1; break;
    case SET_FAN_MIN_SPEED: fanMinSpeed = (uint8_t)constrain((int)fanMinSpeed + direction * 5, 0, (int)fanMaxSpeed); break;
    case SET_FAN_MAX_SPEED: fanMaxSpeed = (uint8_t)constrain((int)fanMaxSpeed + direction * 5, (int)max((uint8_t)fanMinSpeed, (uint8_t)20), 100); break;
    case SET_LED_TEMP_LIMIT: ledTempLimit = constrain(ledTempLimit + direction, 65.0, 82.0); break;
    case SET_TURBO_FAN_MODE: turboFanBoostEnabled = !turboFanBoostEnabled; break;
    case SET_TURBO_FAN_START: turboFanStartSpeed = (uint8_t)constrain((int)turboFanStartSpeed + direction * 5, 0, 100); break;
    case SET_BATTERY_CAL: batteryCalOffset = round(constrain(batteryCalOffset + 0.10 * direction, -8.00, 2.00) * 100.0) / 100.0; lastBatteryRead = 0; break;
    case SET_LOW_BATT_WARNING: lowBatteryWarningVoltage = constrain(lowBatteryWarningVoltage + 0.1 * direction, 10.0, 12.0); break;
    case SET_LOW_BATT_LIMIT: lowBatteryOutputLimit = (uint8_t)constrain((int)lowBatteryOutputLimit + direction, 1, 50); break;
    case SET_CRITICAL_VOLTAGE: criticalBatteryVoltage = constrain(criticalBatteryVoltage + 0.1 * direction, 8.5, 9.6); if (recoveryBatteryVoltage <= criticalBatteryVoltage) recoveryBatteryVoltage = criticalBatteryVoltage + 0.5; break;
    case SET_RECOVERY_VOLTAGE: recoveryBatteryVoltage = constrain(recoveryBatteryVoltage + 0.1 * direction, criticalBatteryVoltage + 0.5, 12.0); break;
    default: return;
  }
  if (id != SET_PWM_FREQUENCY) saveSettingConfig();
}

void handleSettingMenu() {
  bool mainPressed = false, mainReleased = false;
  bool plusPressed = false, plusReleased = false;
  bool minusPressed = false, minusReleased = false;
  unsigned long now = millis();
  forceSettingOutputsOff();
  updateSettingButton(BTN_MAIN, settingMainRaw, settingMainStable, settingMainDebounceAt, mainPressed, mainReleased);
  updateSettingButton(BTN_PLUS, settingPlusRaw, settingPlusStable, settingPlusDebounceAt, plusPressed, plusReleased);
  updateSettingButton(BTN_MINUS, settingMinusRaw, settingMinusStable, settingMinusDebounceAt, minusPressed, minusReleased);
  if (mainPressed) {
    settingMainPressAt = now;
    settingMainLongDone = false;
    settingMainBackDone = false;
  }
  if (settingMainStable == LOW && !settingMainLongDone && now - settingMainPressAt >= SETTING_HOLD_TIME) {
    settingMainLongDone = true;
    settingMode = false;
    settingEditing = false;
    settingResetConfirm = false;
    settingResetChoice = 0;
    settingPage = 0;
    mainClickCount = 0;
    mainLastRelease = 0;
    mainLongHandled = true;
    mainRawState = LOW;
    mainStableState = LOW;
    mainPressStart = now;
    forceSettingOutputsOff();
    oledDisplayActive = true;
    oledIdleSince = millis();
    if (oledOK) display.ssd1306_command(SSD1306_DISPLAYON);
    return;
  }
  if (settingMainStable == LOW && !settingMainBackDone &&
      now - settingMainPressAt >= SETTING_BACK_HOLD_TIME &&
      now - settingMainPressAt < SETTING_HOLD_TIME) {
    settingMainBackDone = true;
    if (settingResetConfirm) {
      settingResetConfirm = false;  // cancel confirmation
      settingResetChoice = 0;
    } else if (settingEditing) {
      settingEditing = false;       // value editor -> item list
      settingEditField = 0;
    } else if (settingPage == 1) {
      settingPage = 0;              // item list -> category list
      settingSubmenuIndex = 0;
    }
    // At the category list, keep holding MAIN until 5 seconds to exit.
  }

  if (mainReleased && !settingMainLongDone && !settingMainBackDone) {
    if (settingResetConfirm) {
      if (settingResetChoice == 1) {
        resetSettingDefaults();
        settingCategoryIndex = 0;
        settingSubmenuIndex = 0;
        settingPage = 0;
      }
      settingResetConfirm = false;
      settingResetChoice = 0;
    } else if (settingEditing) {
      if (getCurrentSettingId() == SET_DISPLAY_TIMEOUT) {
        settingEditField = (settingEditField + 1) % 2;
      } else {
        settingEditing = false;
        settingEditField = 0;
      }
    } else if (settingPage == 0) {
      settingPage = 1;
      settingSubmenuIndex = 0;
    } else {
      uint8_t id = getCurrentSettingId();
      if (id == SET_RESET_DEFAULTS) {
        settingResetConfirm = true;
        settingResetChoice = 0;
      } else {
        settingMenuIndex = id;
        settingEditing = true;
        settingEditField = 0;
      }
    }
  }
  if (mainReleased) {
    settingMainLongDone = false;
    settingMainBackDone = false;
  }

  if (settingResetConfirm) {
    if (plusPressed && !minusPressed) settingResetChoice = 1;
    if (minusPressed && !plusPressed) settingResetChoice = 0;
  } else if (settingEditing) {
    if (getCurrentSettingId() == SET_PWM_FREQUENCY) {
      if (plusPressed && !minusPressed) {
        adjustSettingValue(+1);
        settingPlusRepeatAt = now + SETTING_VALUE_REPEAT_DELAY;
      } else if (minusPressed && !plusPressed) {
        adjustSettingValue(-1);
        settingMinusRepeatAt = now + SETTING_VALUE_REPEAT_DELAY;
      } else if (settingPlusStable == LOW && settingMinusStable != LOW && (int32_t)(now - settingPlusRepeatAt) >= 0) {
        adjustSettingValue(+1);
        settingPlusRepeatAt = now + SETTING_VALUE_REPEAT_INTERVAL;
      } else if (settingMinusStable == LOW && settingPlusStable != LOW && (int32_t)(now - settingMinusRepeatAt) >= 0) {
        adjustSettingValue(-1);
        settingMinusRepeatAt = now + SETTING_VALUE_REPEAT_INTERVAL;
      }
    } else {
      if (plusPressed) adjustSettingValue(+1);
      if (minusPressed) adjustSettingValue(-1);
    }
  } else if (settingPage == 0) {
    if (plusPressed && !minusPressed) settingCategoryIndex = (settingCategoryIndex + 1) % SETTING_CATEGORY_COUNT;
    if (minusPressed && !plusPressed) settingCategoryIndex = (settingCategoryIndex + SETTING_CATEGORY_COUNT - 1) % SETTING_CATEGORY_COUNT;
  } else {
    if (plusPressed && minusPressed) {
      settingPage = 0;
    } else {
      uint8_t count = settingCategoryItemCounts[settingCategoryIndex];
      if (plusPressed) settingSubmenuIndex = (settingSubmenuIndex + 1) % count;
      if (minusPressed) settingSubmenuIndex = (settingSubmenuIndex + count - 1) % count;
      settingMenuIndex = getCurrentSettingId();
    }
  }
  if (pwmFrequencyDirty && settingPlusStable != LOW && settingMinusStable != LOW) {
    saveSettingConfig();
    pwmFrequencyDirty = false;
  }
}
void saveSettingConfig() {
  uint8_t fadeValue = fadeEnabled ? 1 : 0;
  uint8_t voltageValue = showVoltage ? 1 : 0;
  uint8_t percentValue = showBatteryPercent ? 1 : 0;
  uint8_t powerValue = showPowerWatt ? 1 : 0;
  uint8_t temperatureValue = showTemperature ? 1 : 0;
  uint8_t fanValue = showFanStatus ? 1 : 0;
  uint8_t pwmValue = showPWM ? 1 : 0;
  uint8_t timeoutUnitValue = displayTimeoutInMinutes ? 1 : 0;
  EEPROM.put(EEPROM_FAN_LIMIT_ADDR, fanTempLimit);
  EEPROM.put(EEPROM_BATTERY_CAL_ADDR, batteryCalOffset);
  EEPROM.put(EEPROM_FADE_ENABLED_ADDR, fadeValue);
  EEPROM.put(EEPROM_FADE_DURATION_ADDR, fadeDurationSec);
  EEPROM.put(EEPROM_LED_WATT_ADDR, ledWattCapacity);
  EEPROM.put(EEPROM_OLED_BRIGHTNESS_ADDR, oledBrightness);
  EEPROM.put(EEPROM_DISPLAY_TIMEOUT_ADDR, displayTimeoutValue);
  EEPROM.put(EEPROM_DISPLAY_TIMEOUT_UNIT_ADDR, timeoutUnitValue);
  EEPROM.put(EEPROM_SHOW_VOLTAGE_ADDR, voltageValue);
  EEPROM.put(EEPROM_SHOW_BATTERY_PERCENT_ADDR, percentValue);
  EEPROM.put(EEPROM_SHOW_POWER_ADDR, powerValue);
  EEPROM.put(EEPROM_SHOW_TEMPERATURE_ADDR, temperatureValue);
  EEPROM.put(EEPROM_SHOW_FAN_ADDR, fanValue);
  uint8_t fadeOutValue = fadeOutEnabled ? 1 : 0;
  EEPROM.put(EEPROM_FADE_OUT_ENABLED_ADDR, fadeOutValue);
  EEPROM.put(EEPROM_FADE_OUT_DURATION_ADDR, fadeOutDurationSec);
  EEPROM.put(EEPROM_LOW_OUTPUT_ADDR, lowOutputLimit);
  EEPROM.put(EEPROM_MED_OUTPUT_ADDR, mediumOutputLimit);
  EEPROM.put(EEPROM_HIGH_OUTPUT_ADDR, highOutputLimit);
  EEPROM.put(EEPROM_TURBO_OUTPUT_ADDR, turboOutputLimit);
  EEPROM.put(EEPROM_TURBO_TIMEOUT_ADDR, turboTimeoutSec);
  EEPROM.put(EEPROM_FAN_MIN_SPEED_ADDR, fanMinSpeed);
  EEPROM.put(EEPROM_SHOW_PWM_ADDR, pwmValue);
  EEPROM.put(EEPROM_DISPLAY_MODE_ADDR, displayMode);
  EEPROM.put(EEPROM_FAN_START_TEMP_ADDR, fanStartTemp);
  EEPROM.put(EEPROM_FAN_STOP_TEMP_ADDR, fanStopTemp);
  EEPROM.put(EEPROM_FAN_MAX_SPEED_ADDR, fanMaxSpeed);
  EEPROM.put(EEPROM_LED_TEMP_LIMIT_ADDR, ledTempLimit);
  EEPROM.put(EEPROM_LOW_BATT_WARN_ADDR, lowBatteryWarningVoltage);
  EEPROM.put(EEPROM_LOW_BATT_LIMIT_ADDR, lowBatteryOutputLimit);
  EEPROM.put(EEPROM_CRITICAL_VOLT_ADDR, criticalBatteryVoltage);
  EEPROM.put(EEPROM_RECOVERY_VOLT_ADDR, recoveryBatteryVoltage);
  EEPROM.put(EEPROM_PWM_FREQUENCY_ADDR, pwmFrequencyKHz);
  EEPROM.put(EEPROM_TURBO_FAN_MODE_ADDR, (uint8_t)(turboFanBoostEnabled ? 1 : 0));
  EEPROM.put(EEPROM_TURBO_FAN_START_ADDR, turboFanStartSpeed);
  EEPROM.commit();
  applyOLEDSettings();
}

void resetSettingDefaults() {
  lowMemory = 10.0;
  mediumMemory = 18.0;
  highMemory = 30.0;
  strobeMemory = 60.0;
  turboMemory = 100.0;
  modeMemoryDirty = true;
  saveModeMemory();
  strobeSpeedHz = STROBE_SPEED_DEFAULT;
  strobeSpeedDirty = true;
  saveStrobeSpeed();
  pwmFrequencyKHz = PWM_FREQUENCY_DEFAULT_KHZ;
  applyPWMFrequency();
  pwmFrequencyDirty = false;
  fanTempLimit = 70.0;
  batteryCalOffset = 0.0;
  fadeEnabled = true;
  fadeDurationSec = 3;
  fadeOutEnabled = true;
  fadeOutDurationSec = 3;
  lowOutputLimit = 15.0;
  mediumOutputLimit = 25.0;
  highOutputLimit = 40.0;
  turboOutputLimit = 100.0;
  turboTimeoutSec = 30;
  turboTimeoutApplied = false;
  fanMinSpeed = 30;
  fanMaxSpeed = 100;
  turboFanBoostEnabled = false;
  turboFanStartSpeed = 30;
  fanStartTemp = 45.0;
  fanStopTemp = 40.0;
  fanTemperatureActive = false;
  ledTempLimit = 75.0;
  lowBatteryWarningVoltage = 11.0;
  lowBatteryOutputLimit = 15;
  criticalBatteryVoltage = 9.0;
  recoveryBatteryVoltage = 10.8;
  showPWM = true;
  displayMode = 2;
  ledWattCapacity = LED_POWER_DEFAULT;
  oledBrightness = 70;
  displayTimeoutValue = 1;
  displayTimeoutInMinutes = false;
  showVoltage = true;
  showBatteryPercent = true;
  showPowerWatt = true;
  showTemperature = true;
  showFanStatus = true;
  saveSettingConfig();
}

void updateOLEDPowerState() {
  unsigned long now = millis();
  if (settingMode) {
    oledDisplayActive = true;
    if (oledOK) display.ssd1306_command(SSD1306_DISPLAYON);
    return;
  }
  if (displayMode == 0) { // ALWAYS ON
    oledIdleSince = 0;
    if (!oledDisplayActive) {
      oledDisplayActive = true;
      display.ssd1306_command(SSD1306_DISPLAYON);
    }
    return;
  }
  if (ledOn) { // Wake display whenever LED turns on
    oledIdleSince = 0;
    if (!oledDisplayActive) {
      oledDisplayActive = true;
      display.ssd1306_command(SSD1306_DISPLAYON);
    }
    return;
  }
  if (!oledIdleSince) oledIdleSince = now;
  unsigned long timeoutUnitMs = displayTimeoutInMinutes ? 60000UL : 1000UL;
  unsigned long delayMs = displayMode == 1 ? 0UL : (unsigned long)displayTimeoutValue * timeoutUnitMs;
  if (oledDisplayActive && now - oledIdleSince >= delayMs) {
    oledDisplayActive = false;
    display.clearDisplay(); display.display(); display.ssd1306_command(SSD1306_DISPLAYOFF);
  }
}

void updateWarningBlink() {
  unsigned long now =
    millis();
  if (
    now -
    lastWarningBlink >=
    WARNING_BLINK_INTERVAL
  ) {
    lastWarningBlink =
      now;
    warningBlinkState =
      !warningBlinkState;
  }
}

void updateBatteryWarningBlink() {
  unsigned long now =
    millis();
  bool batteryWeak =
    batteryPresent &&
    batteryVoltage <= lowBatteryWarningVoltage;
  if (!batteryWeak) {
    batteryBlinkState = true;
    lastBatteryBlink = now;
    return;
  }
  if (
    now -
    lastBatteryBlink >=
    BATTERY_BLINK_INTERVAL
  ) {

    lastBatteryBlink =
      now;

    batteryBlinkState =
      !batteryBlinkState;
  }
}


// ============================================================
// BATTERY ICON
// ============================================================

void drawBatteryIcon(
  int x,
  int y,
  int percent
) {

  percent =
    constrain(
      percent,
      0,
      100
    );


  display.drawRect(
    x,
    y,
    18,
    8,
    SSD1306_WHITE
  );


  display.fillRect(
    x + 18,
    y + 2,
    2,
    4,
    SSD1306_WHITE
  );


  int fillWidth =
    map(
      percent,
      0,
      100,
      0,
      14
    );


  if (fillWidth > 0) {

    display.fillRect(
      x + 2,
      y + 2,
      fillWidth,
      4,
      SSD1306_WHITE
    );
  }
}


// ============================================================
// LOCK ICON
// ============================================================

void drawLockIcon(
  int x,
  int y
) {

  display.drawPixel(
    x + 2,
    y,
    SSD1306_WHITE
  );

  display.drawPixel(
    x + 3,
    y,
    SSD1306_WHITE
  );

  display.drawPixel(
    x + 4,
    y,
    SSD1306_WHITE
  );

  display.drawPixel(
    x + 1,
    y + 1,
    SSD1306_WHITE
  );

  display.drawPixel(
    x + 5,
    y + 1,
    SSD1306_WHITE
  );

  display.drawPixel(
    x + 1,
    y + 2,
    SSD1306_WHITE
  );

  display.drawPixel(
    x + 5,
    y + 2,
    SSD1306_WHITE
  );

  display.drawPixel(
    x + 1,
    y + 3,
    SSD1306_WHITE
  );

  display.drawPixel(
    x + 5,
    y + 3,
    SSD1306_WHITE
  );


  display.fillRect(
    x,
    y + 4,
    7,
    5,
    SSD1306_WHITE
  );


  display.drawPixel(
    x + 3,
    y + 5,
    SSD1306_BLACK
  );

  display.drawPixel(
    x + 2,
    y + 6,
    SSD1306_BLACK
  );

  display.drawPixel(
    x + 3,
    y + 6,
    SSD1306_BLACK
  );

  display.drawPixel(
    x + 4,
    y + 6,
    SSD1306_BLACK
  );
}


// ============================================================
// FAN ICON
// ============================================================

void drawFanIcon(
  int x,
  int y
) {

  display.fillCircle(
    x + 4,
    y + 4,
    1,
    SSD1306_WHITE
  );


  display.drawLine(
    x + 4,
    y + 3,
    x + 4,
    y,
    SSD1306_WHITE
  );


  display.drawLine(
    x + 5,
    y + 5,
    x + 7,
    y + 7,
    SSD1306_WHITE
  );


  display.drawLine(
    x + 3,
    y + 5,
    x,
    y + 6,
    SSD1306_WHITE
  );
}


void tampilkanOLED() {

  if (!oledOK) {
    return;
  }

  if (settingMode) {
    if (!oledDisplayActive) {
      oledDisplayActive = true;
      display.ssd1306_command(SSD1306_DISPLAYON);
    }
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    // Yellow OLED band (top 16 px): title only. Blue area: menu/content.
    const char *title = settingResetConfirm ? "CONFIRM RESET" : (settingEditing ? "EDIT SETTING" : (settingPage == 0 ? "SETTINGS" : settingCategoryLabels[settingCategoryIndex]));
    int16_t x1, y1; uint16_t tw, th;
    display.getTextBounds(title, 0, 0, &x1, &y1, &tw, &th);
    display.setCursor((128 - tw) / 2, 4);
    display.print(title);
    display.drawLine(0, 15, 127, 15, SSD1306_WHITE);

    if (settingResetConfirm) {
      display.setTextSize(1);
      display.setCursor(8, 23);
      display.print("Reset all settings?");
      display.setCursor(8, 35);
      display.print("Defaults will load");
      display.setTextSize(1);
      display.fillRect(2, 49, 60, 12, settingResetChoice == 0 ? SSD1306_WHITE : SSD1306_BLACK);
      display.setTextColor(settingResetChoice == 0 ? SSD1306_BLACK : SSD1306_WHITE);
      display.setCursor(15, 51);
      display.print("CANCEL");
      display.fillRect(66, 49, 60, 12, settingResetChoice == 1 ? SSD1306_WHITE : SSD1306_BLACK);
      display.setTextColor(settingResetChoice == 1 ? SSD1306_BLACK : SSD1306_WHITE);
      display.setCursor(78, 51);
      display.print("RESET");
      display.setTextColor(SSD1306_WHITE);
    } else if (settingEditing) {
      uint8_t id = getCurrentSettingId();
      display.setTextSize(1);
      display.setCursor(0, 19);
      display.print(getSettingLabel(id));
      if (id == SET_BATTERY_CAL) {
        // Tampilkan tegangan baterai realtime yang sudah termasuk offset kalibrasi.
        char voltageText[12];
        if (batteryPresent) snprintf(voltageText, sizeof(voltageText), "%.1fV", batteryDisplayVoltage >= 0.0 ? batteryDisplayVoltage : batteryVoltage);
        else snprintf(voltageText, sizeof(voltageText), "--.-V");
        int16_t vx, vy; uint16_t vw, vh;
        display.getTextBounds(voltageText, 0, 0, &vx, &vy, &vw, &vh);
        display.setCursor(126 - vw, 19);
        display.print(voltageText);
        display.setTextSize(2);
        display.setCursor(0, 34);
      } else {
        display.setTextSize(2);
        display.setCursor(0, 31);
      }
      switch (id) {
        case SET_SHOW_PWM: display.print(showPWM ? "ON" : "OFF"); break;
        case SET_SHOW_VOLTAGE: display.print(showVoltage ? "ON" : "OFF"); break;
        case SET_SHOW_BATTERY_PERCENT: display.print(showBatteryPercent ? "ON" : "OFF"); break;
        case SET_SHOW_POWER: display.print(showPowerWatt ? "ON" : "OFF"); break;
        case SET_SHOW_TEMPERATURE: display.print(showTemperature ? "ON" : "OFF"); break;
        case SET_SHOW_FAN: display.print(showFanStatus ? "ON" : "OFF"); break;
        case SET_DISPLAY_MODE: display.print(displayMode == 0 ? "ALWAYS" : displayMode == 1 ? "AUTO LED" : "TIMEOUT"); break;
        case SET_PWM_FREQUENCY: display.print(pwmFrequencyKHz); display.print(" kHz"); break;
        case SET_FAN_START_TEMP: display.print(fanStartTemp, 0); display.print(" C"); break;
        case SET_FAN_STOP_TEMP: display.print(fanStopTemp, 0); display.print(" C"); break;
        case SET_FAN_MAX_SPEED: display.print(fanMaxSpeed); display.print(" %"); break;
        case SET_LED_TEMP_LIMIT: display.print(ledTempLimit, 0); display.print(" C"); break;
        case SET_TURBO_FAN_MODE: display.print(turboFanBoostEnabled ? "BOOST" : "AUTO"); break;
        case SET_TURBO_FAN_START: display.print(turboFanStartSpeed); display.print(" %"); break;
        case SET_LOW_BATT_WARNING: display.print(lowBatteryWarningVoltage, 1); display.print(" V"); break;
        case SET_LOW_BATT_LIMIT: display.print(lowBatteryOutputLimit); display.print(" %"); break;
        case SET_CRITICAL_VOLTAGE: display.print(criticalBatteryVoltage, 1); display.print(" V"); break;
        case SET_RECOVERY_VOLTAGE: display.print(recoveryBatteryVoltage, 1); display.print(" V"); break;
        case SET_STROBE_SPEED: display.print(strobeSpeedHz); display.print(" Hz"); break;
        case SET_FADE_IN_ENABLE: display.print(fadeEnabled ? "ON" : "OFF"); break;
        case SET_FADE_OUT_ENABLE: display.print(fadeOutEnabled ? "ON" : "OFF"); break;
        case SET_FADE_IN_TIME: display.print(fadeDurationSec); display.print(" sec"); break;
        case SET_FADE_OUT_TIME: display.print(fadeOutDurationSec); display.print(" sec"); break;
        case SET_LED_WATT: display.print(ledWattCapacity, 0); display.print(" W"); break;
        case SET_LOW_OUTPUT: display.print(lowOutputLimit, 0); display.print(" %"); break;
        case SET_MED_OUTPUT: display.print(mediumOutputLimit, 0); display.print(" %"); break;
        case SET_HIGH_OUTPUT: display.print(highOutputLimit, 0); display.print(" %"); break;
        case SET_TURBO_OUTPUT: display.print(turboOutputLimit, 0); display.print(" %"); break;
        case SET_TURBO_TIMEOUT: if (turboTimeoutSec == 0) display.print("OFF"); else {
            display.print(turboTimeoutSec);
            display.print(" s");
          } break;
        case SET_FAN_TEMP_LIMIT: display.print(fanTempLimit, 0); display.print(" C"); break;
        case SET_FAN_MIN_SPEED: display.print(fanMinSpeed); display.print(" %"); break;
        case SET_BATTERY_CAL: if (batteryCalOffset >= 0) display.print("+"); display.print(batteryCalOffset, 1); display.print("V"); break;
        case SET_OLED_BRIGHTNESS: display.print(oledBrightness); display.print("%"); break;
        case SET_DISPLAY_TIMEOUT: {
            bool valueSelected = settingEditField == 0;
            display.fillRect(0, 30, 40, 21, valueSelected ? SSD1306_WHITE : SSD1306_BLACK);
            display.setTextColor(valueSelected ? SSD1306_BLACK : SSD1306_WHITE);
            display.setCursor(3, 32);
            display.print(displayTimeoutValue);
            bool unitSelected = settingEditField == 1;
            display.fillRect(45, 30, 83, 21, unitSelected ? SSD1306_WHITE : SSD1306_BLACK);
            display.setTextColor(unitSelected ? SSD1306_BLACK : SSD1306_WHITE);
            display.setCursor(49, 32);
            display.print(displayTimeoutInMinutes ? "MENIT" : "DETIK");
            display.setTextColor(SSD1306_WHITE);
            display.setTextSize(1);
            display.setCursor(0, 54);
            display.print("MAIN:NEXT HOLD:BACK");
            display.setTextSize(2);
            break;
          }
        case SET_RESET_DEFAULTS: display.print("RESET?"); break;
      }
    } else if (settingPage == 0) {
      const uint8_t visible = 5;
      uint8_t first = settingCategoryIndex >= visible ? settingCategoryIndex - visible + 1 : 0;
      uint8_t maxFirst = SETTING_CATEGORY_COUNT > visible ? SETTING_CATEGORY_COUNT - visible : 0;
      if (first > maxFirst) first = maxFirst;
      for (uint8_t row = 0; row < visible; row++) {
        uint8_t i = first + row;
        if (i >= SETTING_CATEGORY_COUNT) break;
        int16_t y = 18 + row * 9;
        bool selected = i == settingCategoryIndex;
        if (selected) {
          display.fillRect(0, y, 128, 9, SSD1306_WHITE);
          display.setTextColor(SSD1306_BLACK);
        }
        else display.setTextColor(SSD1306_WHITE);
        display.setCursor(2, y);
        display.print(selected ? "> " : "  ");
        display.print(settingCategoryLabels[i]);
      }
      display.setTextColor(SSD1306_WHITE);
    } else {
      uint8_t count = settingCategoryItemCounts[settingCategoryIndex];
      const uint8_t visible = 5;
      uint8_t first = settingSubmenuIndex >= visible ? settingSubmenuIndex - visible + 1 : 0;
      uint8_t maxFirst = count > visible ? count - visible : 0;
      if (first > maxFirst) first = maxFirst;
      for (uint8_t row = 0; row < visible; row++) {
        uint8_t idx = first + row;
        if (idx >= count) break;
        uint8_t id = settingCategoryItems[settingCategoryIndex][idx];
        int16_t y = 18 + row * 8;
        bool selected = idx == settingSubmenuIndex;
        if (selected) {
          display.fillRect(0, y, 128, 8, SSD1306_WHITE);
          display.setTextColor(SSD1306_BLACK);
        }
        else display.setTextColor(SSD1306_WHITE);
        display.setCursor(2, y);
        display.print(selected ? ">" : " ");
        display.print(getSettingLabel(id));
        display.setCursor(96, y);
        switch (id) {
          case SET_SHOW_PWM: display.print(showPWM ? "ON" : "OFF"); break;
          case SET_SHOW_VOLTAGE: display.print(showVoltage ? "ON" : "OFF"); break;
          case SET_SHOW_BATTERY_PERCENT: display.print(showBatteryPercent ? "ON" : "OFF"); break;
          case SET_SHOW_POWER: display.print(showPowerWatt ? "ON" : "OFF"); break;
          case SET_SHOW_TEMPERATURE: display.print(showTemperature ? "ON" : "OFF"); break;
          case SET_SHOW_FAN: display.print(showFanStatus ? "ON" : "OFF"); break;
          case SET_DISPLAY_MODE: display.print(displayMode == 0 ? "ALWAYS" : displayMode == 1 ? "AUTO" : "TIMEOUT"); break;
          case SET_PWM_FREQUENCY: display.print(pwmFrequencyKHz); display.print("kHz"); break;
          case SET_FAN_START_TEMP: display.print(fanStartTemp, 0); display.print("C"); break;
          case SET_FAN_STOP_TEMP: display.print(fanStopTemp, 0); display.print("C"); break;
          case SET_FAN_MAX_SPEED: display.print(fanMaxSpeed); display.print("%"); break;
          case SET_LED_TEMP_LIMIT: display.print(ledTempLimit, 0); display.print("C"); break;
          case SET_TURBO_FAN_MODE: display.print(turboFanBoostEnabled ? "BOOST" : "AUTO"); break;
          case SET_TURBO_FAN_START: display.print(turboFanStartSpeed); display.print("%"); break;
          case SET_LOW_BATT_WARNING: display.print(lowBatteryWarningVoltage, 1); break;
          case SET_LOW_BATT_LIMIT: display.print(lowBatteryOutputLimit); display.print("%"); break;
          case SET_CRITICAL_VOLTAGE: display.print(criticalBatteryVoltage, 1); break;
          case SET_RECOVERY_VOLTAGE: display.print(recoveryBatteryVoltage, 1); break;
          case SET_STROBE_SPEED: display.print(strobeSpeedHz); display.print("Hz"); break;
          case SET_FADE_IN_ENABLE: display.print(fadeEnabled ? "ON" : "OFF"); break;
          case SET_FADE_OUT_ENABLE: display.print(fadeOutEnabled ? "ON" : "OFF"); break;
          case SET_FADE_IN_TIME: display.print(fadeDurationSec); display.print("s"); break;
          case SET_FADE_OUT_TIME: display.print(fadeOutDurationSec); display.print("s"); break;
          case SET_LED_WATT: display.print(ledWattCapacity, 0); display.print("W"); break;
          case SET_LOW_OUTPUT: display.print(lowOutputLimit, 0); display.print("%"); break;
          case SET_MED_OUTPUT: display.print(mediumOutputLimit, 0); display.print("%"); break;
          case SET_HIGH_OUTPUT: display.print(highOutputLimit, 0); display.print("%"); break;
          case SET_TURBO_OUTPUT: display.print(turboOutputLimit, 0); display.print("%"); break;
          case SET_TURBO_TIMEOUT: if (turboTimeoutSec == 0) display.print("OFF"); else {
              display.print(turboTimeoutSec);
              display.print("s");
            } break;
          case SET_FAN_TEMP_LIMIT: display.print(fanTempLimit, 0); display.print("C"); break;
          case SET_FAN_MIN_SPEED: display.print(fanMinSpeed); display.print("%"); break;
          case SET_BATTERY_CAL: if (batteryCalOffset >= 0) display.print("+"); display.print(batteryCalOffset, 1); break;
          case SET_OLED_BRIGHTNESS: display.print(oledBrightness); display.print("%"); break;
          case SET_DISPLAY_TIMEOUT: display.print(displayTimeoutValue); display.print(displayTimeoutInMinutes ? "m" : "s"); break;
          default: break;
        }
      }
      display.setTextColor(SSD1306_WHITE);
    }
    display.display();
    return;
  }

  updateOLEDPowerState();


  if (!oledDisplayActive) {
    return;
  }


  display.clearDisplay();

  display.setTextColor(WHITE);


  // ==========================================================
  // TOP BAR
  //
  // KIRI  : MODE + TARGET
  //         STROBO Hz di bawahnya
  //
  // KANAN : PERCENT + BATTERY ICON
  //         VOLTAGE di bawah battery icon
  // ==========================================================

  display.setTextSize(1);


  // ----------------------------------------------------------
  // MODE + TARGET
  // ----------------------------------------------------------

  display.setCursor(
    0,
    0
  );

  if (ledOn) {
    display.print(getModeName());
    if (showPWM) {
      display.print(" ");
      display.print((int)ledPWM);
      display.print("%");
    }
  }


  // ----------------------------------------------------------
  // STROBO FREQUENCY
  // Hanya tampil saat mode STROBO
  // Posisi tepat di bawah MODE + TARGET
  // ----------------------------------------------------------

  if (ledOn && currentMode == MODE_STROBE) {

    display.setTextSize(1);

    display.setCursor(
      0,
      9
    );

    display.print(
      strobeSpeedHz
    );

    display.print(
      " Hz"
    );
  }


  // ----------------------------------------------------------
  // BATTERY VOLTAGE
  // ----------------------------------------------------------

  String voltageText;

  if (
    batteryPresent &&
    batteryVoltage > 0.0
  ) {

    voltageText =
      String(
        batteryDisplayVoltage >= 0.0 ? batteryDisplayVoltage : batteryVoltage,
        1
      ) +
      "V";

  } else {

    voltageText =
      "--.-V";
  }


  // ----------------------------------------------------------
  // BATTERY PERCENT
  // ----------------------------------------------------------

  String batteryText =
    String(
      batteryPercent
    ) +
    "%";


  // ----------------------------------------------------------
  // HITUNG LEBAR ELEMEN BATTERY
  // ----------------------------------------------------------

  int16_t vx1, vy1;
  uint16_t vw, vh;

  int16_t bx1, by1;
  uint16_t bw, bh;


  display.getTextBounds(
    voltageText,
    0,
    0,
    &vx1,
    &vy1,
    &vw,
    &vh
  );


  display.getTextBounds(
    batteryText,
    0,
    0,
    &bx1,
    &by1,
    &bw,
    &bh
  );


  // ----------------------------------------------------------
  // POSISI BATTERY
  //
  // ICON     : paling kanan
  // PERCENT  : sebelah kiri icon
  // VOLTAGE  : di bawah icon
  //            rata kanan dengan sisi layar
  // ----------------------------------------------------------

  const int batteryIconX = 108;

  const int batteryPercentX =
    batteryIconX - 3 - bw;


  // ----------------------------------------------------------
  // VOLTAGE X
  // Sisi kanan voltage selalu di x = 127
  // ----------------------------------------------------------

  const int batteryVoltageRightX = 127;

  const int batteryVoltageX =
    batteryVoltageRightX - vw;


  // ----------------------------------------------------------
  // VOLTAGE
  // ----------------------------------------------------------

  if (showVoltage) {
    display.setCursor(batteryVoltageX, 9);
    display.print(voltageText);
  }


  // ----------------------------------------------------------
  // PERCENT
  // ----------------------------------------------------------

  if (showBatteryPercent) {
    display.setCursor(batteryPercentX, 0);
    display.print(batteryText);
  }


  // ----------------------------------------------------------
  // BATTERY ICON
  // ----------------------------------------------------------

  bool batteryWeak =
    batteryPresent &&
    batteryVoltage <= lowBatteryWarningVoltage;


  if (
    !batteryWeak ||
    batteryBlinkState
  ) {

    drawBatteryIcon(
      batteryIconX,
      0,
      batteryPercent
    );
  }


  // ==========================================================
  // FAULT
  // ==========================================================

  if (
    (tempFault ||
     coolingFault) &&
    warningBlinkState
  ) {

    display.setTextSize(1);


    String faultText;


    if (tempFault) {

      faultText =
        "!TEMP";

    } else {

      faultText =
        "!FAN";
    }


    int16_t x1, y1;

    uint16_t w, h;


    display.getTextBounds(
      faultText,
      0,
      0,
      &x1,
      &y1,
      &w,
      &h
    );


    int faultX =
      (128 - w) / 2;


    display.setCursor(
      faultX,
      9
    );


    display.print(
      faultText
    );
  }


  // ==========================================================
  // LOCK
  // ==========================================================

  if (buttonsLocked) {

    drawLockIcon(
      118,
      9
    );
  }


  // ==========================================================
  // GARIS ATAS
  // ==========================================================

  display.drawLine(
    0,
    18,
    127,
    18,
    WHITE
  );


  // ==========================================================
  // LOW BATTERY WARNING
  // ==========================================================

  bool showBatteryWarning =
    batteryPresent &&
    batteryVoltage <= lowBatteryWarningVoltage;


  if (!ledOn) {
    display.setTextSize(3);
    int16_t offX1, offY1;
    uint16_t offWidth, offHeight;
    display.getTextBounds("OFF", 0, 0, &offX1, &offY1, &offWidth, &offHeight);
    display.setCursor((128 - offWidth) / 2, 22);
    display.print("OFF");
  } else if (showBatteryWarning) {

    if (warningBlinkState) {

      display.setTextSize(2);


      String warningText =
        "LOW BATT";


      int16_t wx1, wy1;

      uint16_t ww, wh;


      display.getTextBounds(
        warningText,
        0,
        0,
        &wx1,
        &wy1,
        &ww,
        &wh
      );


      int warningX =
        (128 - ww) / 2;


      display.setCursor(
        warningX,
        27
      );


      display.print(
        warningText
      );
    }

  } else if (showPowerWatt) {
    display.setTextSize(3);
    float displayedOutput = userLEDTarget;
    if (currentMode == MODE_TURBO && turboTimeoutApplied && displayedOutput > highOutputLimit) displayedOutput = highOutputLimit;
    float displayPower = (ledPWM / 100.0) * ledWattCapacity;
    String powerText = String(displayPower, 1) + "W";
    int16_t px1, py1;
    uint16_t pw, ph;
    display.getTextBounds(powerText, 0, 0, &px1, &py1, &pw, &ph);
    int powerX = (128 - pw) / 2;
    display.setCursor(powerX, 22);
    display.print(powerText);
  }


  // ==========================================================
  // GARIS BAWAH
  // ==========================================================

  display.drawLine(
    0,
    48,
    127,
    48,
    WHITE
  );


  // ==========================================================
  // TEMPERATURE
  // ==========================================================

  display.setTextSize(1);


  if (showTemperature) {
    display.drawLine(2, 52, 2, 59, WHITE);
    display.drawLine(3, 52, 3, 59, WHITE);
    display.drawCircle(2, 60, 3, WHITE);
    display.fillCircle(2, 60, 1, WHITE);
    display.setCursor(9, 52);
    if (temperatureValid && !tempFault) {
      display.print((int)temperatureC);
    } else {
      display.print("--");
    }
    display.print((char)247);
    display.print("C");
  }


  // ==========================================================
  // FAN
  // ==========================================================

  if (showFanStatus) {
    drawFanIcon(96, 51);
    display.setCursor(107, 52);
    display.print((int)fanPWM);
    display.print("%");
  }


  // ==========================================================
  // DISPLAY
  // ==========================================================

  display.display();
}


void loadStrobeSpeed() {

  uint8_t storedSpeed =
    STROBE_SPEED_DEFAULT;

  EEPROM.get(
    EEPROM_STROBE_SPEED_ADDR,
    storedSpeed
  );

  if (
    storedSpeed < STROBE_SPEED_MIN ||
    storedSpeed > STROBE_SPEED_MAX
  ) {
    strobeSpeedHz =
      STROBE_SPEED_DEFAULT;
  } else {
    strobeSpeedHz =
      storedSpeed;
  }

  strobeSpeedDirty = false;
}


void updateStrobe() {

  if (!ledOn) {

    strobeState = false;
    ledPWM = 0.0;
    outputLEDPWM(0);
    return;
  }


  if (
    currentMode !=
    MODE_STROBE
  ) {

    strobeState = false;

    return;
  }


  if (
    thermalProtect ||
    thermalEmergency ||
    safeMode
  ) {

    return;
  }


  unsigned long now =
    millis();


  if (
    now -
    lastStrobe >=
    getStrobeInterval()
  ) {

    lastStrobe =
      now;


    strobeState =
      !strobeState;


    if (strobeState) {

      outputLEDPWM(
        ledPWM
      );

    } else {

      outputLEDPWM(0);
    }
  }
}


void saveStrobeSpeed() {

  if (!strobeSpeedDirty) {
    return;
  }

  EEPROM.put(
    EEPROM_STROBE_SPEED_ADDR,
    strobeSpeedHz
  );

  EEPROM.commit();

  strobeSpeedDirty = false;
}


void applyOLEDSettings() {
  if (oledOK) {
    // SSD1306 SETCONTRAST command followed by 8-bit contrast value.
    display.ssd1306_command(0x81);
    display.ssd1306_command((uint8_t)map(oledBrightness, 1, 100, 1, 255));
  }
}

void bacaBattery() {

  unsigned long now =
    millis();


  if (
    now - lastBatteryRead <
    BATTERY_READ_INTERVAL
  ) {

    return;
  }

  lastBatteryRead =
    now;

  if (
    !batteryPacketReceived ||
    now - lastBatteryPacketAt > BATTERY_SERIAL_TIMEOUT ||
    receivedBatteryMillivolts == 0
  ) {
    batteryPresent = false;
    batteryVoltage = 0.0;
    filteredBatteryVoltage = -1.0;
    batteryDisplayVoltage = -1.0;
    batteryPercent = 0;
    return;
  }

  batteryPresent = true;
  float measuredBatteryVoltage = receivedBatteryMillivolts / 1000.0;

  // Rata-rata antar pembacaan agar angka voltase lebih stabil.
  if (filteredBatteryVoltage < 0.0) {
    filteredBatteryVoltage = measuredBatteryVoltage;
  } else {
    filteredBatteryVoltage += BATTERY_FILTER_ALPHA *
                              (measuredBatteryVoltage - filteredBatteryVoltage);
  }

  // Kalibrasi offset diterapkan setelah penyaringan.
  batteryVoltage = filteredBatteryVoltage + batteryCalOffset;
  if (batteryVoltage < 0.0) batteryVoltage = 0.0;

  // Filter tambahan khusus tampilan: meredam perubahan digit desimal di OLED.
  if (batteryDisplayVoltage < 0.0) {
    batteryDisplayVoltage = batteryVoltage;
  } else {
    batteryDisplayVoltage += 0.12 * (batteryVoltage - batteryDisplayVoltage);
  }

  // ==========================================================
  // BATTERY PERCENT - LiPo 3S
  //
  // 12.60V = 100%
  // 12.40V = 90%
  // 12.20V = 80%
  // 12.00V = 70%
  // 11.80V = 60%
  // 11.60V = 50%
  // 11.40V = 40%
  // 11.20V = 30%
  // 11.00V = 20%
  // 10.80V = 10%
  // 10.50V = 5%
  // 9.00V  = 0%
  // ==========================================================

  if (batteryVoltage >= 12.60) {

    batteryPercent = 100;

  }

  else if (batteryVoltage >= 12.40) {

    batteryPercent =
      map(
        (int)(batteryVoltage * 100),
        1240,
        1260,
        90,
        100
      );

  }

  else if (batteryVoltage >= 12.20) {

    batteryPercent =
      map(
        (int)(batteryVoltage * 100),
        1220,
        1240,
        80,
        90
      );

  }

  else if (batteryVoltage >= 12.00) {

    batteryPercent =
      map(
        (int)(batteryVoltage * 100),
        1200,
        1220,
        70,
        80
      );

  }

  else if (batteryVoltage >= 11.80) {

    batteryPercent =
      map(
        (int)(batteryVoltage * 100),
        1180,
        1200,
        60,
        70
      );

  }

  else if (batteryVoltage >= 11.60) {

    batteryPercent =
      map(
        (int)(batteryVoltage * 100),
        1160,
        1180,
        50,
        60
      );

  }

  else if (batteryVoltage >= 11.40) {

    batteryPercent =
      map(
        (int)(batteryVoltage * 100),
        1140,
        1160,
        40,
        50
      );

  }

  else if (batteryVoltage >= 11.20) {

    batteryPercent =
      map(
        (int)(batteryVoltage * 100),
        1120,
        1140,
        30,
        40
      );

  }

  else if (batteryVoltage >= 11.00) {

    batteryPercent =
      map(
        (int)(batteryVoltage * 100),
        1100,
        1120,
        20,
        30
      );

  }

  else if (batteryVoltage >= 10.80) {

    batteryPercent =
      map(
        (int)(batteryVoltage * 100),
        1080,
        1100,
        10,
        20
      );

  }

  else if (batteryVoltage >= 10.50) {

    batteryPercent =
      map(
        (int)(batteryVoltage * 100),
        1050,
        1080,
        5,
        10
      );

  }

  else if (batteryVoltage >= 9.00) {

    batteryPercent =
      map(
        (int)(batteryVoltage * 100),
        900,
        1050,
        0,
        5
      );

  }

  else {

    batteryPercent = 0;
  }


  batteryPercent =
    constrain(
      batteryPercent,
      0,
      100
    );
}

uint8_t serialChecksum(const char *data, uint8_t length) {
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < length; i++) checksum ^= (uint8_t)data[i];
  return checksum;
}

bool parseSerialNumber(
  const char *data,
  uint8_t length,
  uint8_t &position,
  char delimiter,
  bool hasDelimiter,
  uint32_t maximum,
  uint32_t &result
) {
  uint32_t value = 0;
  uint8_t digits = 0;
  while (position < length && (!hasDelimiter || data[position] != delimiter)) {
    char digit = data[position++];
    if (digit < '0' || digit > '9') return false;
    uint8_t number = (uint8_t)(digit - '0');
    if (value > (maximum - number) / 10UL) return false;
    value = value * 10UL + number;
    digits++;
  }
  if (digits == 0) return false;
  if (hasDelimiter) {
    if (position >= length || data[position] != delimiter) return false;
    position++;
  } else if (position != length) {
    return false;
  }
  result = value;
  return true;
}

void processNanoResponse() {
  if (batterySerialOverflow || batterySerialLength < 7) {
    nanoRejectedFrameCount++;
    return;
  }
  if (batterySerialBuffer[0] != 'R' || batterySerialBuffer[1] != ',') {
    nanoRejectedFrameCount++;
    return;
  }

  uint8_t checksumSeparator = batterySerialLength;
  for (uint8_t i = 2; i < batterySerialLength; i++) {
    if (batterySerialBuffer[i] == ',') checksumSeparator = i;
  }
  if (checksumSeparator >= batterySerialLength) {
    nanoRejectedFrameCount++;
    return;
  }
  uint8_t receivedChecksumPosition = checksumSeparator + 1;
  uint32_t receivedChecksum = 0;
  if (!parseSerialNumber(
        batterySerialBuffer,
        batterySerialLength,
        receivedChecksumPosition,
        '\0',
        false,
        255,
        receivedChecksum
      )) {
    nanoRejectedFrameCount++;
    return;
  }
  if (receivedChecksum != serialChecksum(batterySerialBuffer, checksumSeparator)) {
    nanoChecksumErrorCount++;
    return;
  }

  uint8_t position = 2;
  uint32_t sequence = 0;
  uint32_t millivolts = 0;
  if (!parseSerialNumber(
        batterySerialBuffer,
        checksumSeparator,
        position,
        ',',
        true,
        65535UL,
        sequence
      ) ||
      !parseSerialNumber(
        batterySerialBuffer,
        checksumSeparator,
        position,
        ',',
        false,
        BATTERY_MAX_MILLIVOLTS,
        millivolts
      )) {
    nanoRejectedFrameCount++;
    return;
  }
  if (!batteryRequestPending || sequence != lastBatteryRequestSequence) {
    nanoSequenceErrorCount++;
    return;
  }

  receivedBatteryMillivolts = (uint16_t)millivolts;
  lastBatteryPacketAt = millis();
  batteryPacketReceived = true;
  batteryRequestPending = false;
  nanoResponseCount++;
}

void bacaDataBatteryNano() {
  while (Serial.available() > 0) {
    char incoming = (char)Serial.read();
    nanoRxByteCount++;
    if (incoming == '\r') continue;

    if (incoming == '\n') {
      if (batterySerialLength > 0) processNanoResponse();
      batterySerialLength = 0;
      batterySerialOverflow = false;
      continue;
    }

    if (batterySerialLength >= sizeof(batterySerialBuffer) - 1) {
      batterySerialOverflow = true;
      continue;
    }
    if (!batterySerialOverflow) {
      batterySerialBuffer[batterySerialLength++] = incoming;
    }
  }
}

void requestNanoBattery() {
  unsigned long now = millis();

  // Jika request sebelumnya belum mendapat balasan,
  // tunggu terlebih dahulu agar nomor urut tidak berubah.
  if (batteryRequestPending) {
    if (now - lastBatteryRequestAt < BATTERY_SERIAL_TIMEOUT) {
      return;
    }

    // Request timeout: izinkan pengiriman ulang.
    batteryRequestPending = false;
  }

  // Batasi interval pengiriman request.
  if (batteryRequestSent &&
      now - lastBatteryRequestAt < BATTERY_REQUEST_INTERVAL) {
    return;
  }

  uint16_t nextSequence =
    (uint16_t)(lastBatteryRequestSequence + 1U);

  char requestPayload[8];

  int payloadLength = snprintf(
                        requestPayload,
                        sizeof(requestPayload),
                        "Q,%u",
                        (unsigned int)nextSequence
                      );

  if (payloadLength <= 0 ||
      payloadLength >= (int)sizeof(requestPayload)) {
    return;
  }

  uint8_t checksum = serialChecksum(
                       requestPayload,
                       (uint8_t)payloadLength
                     );

  // Simpan status request sebelum dikirim.
  lastBatteryRequestSequence = nextSequence;
  lastBatteryRequestAt = now;
  batteryRequestSent = true;
  batteryRequestPending = true;

  Serial.print(requestPayload);
  Serial.print(',');
  Serial.println(checksum);
}

void updateNanoConnection() {
  nanoConnected = batteryPacketReceived &&
                  millis() - lastBatteryPacketAt <= NANO_CONNECTION_TIMEOUT;
}

void updateNanoWarningDisplay() {
  if (!oledOK) return;
  unsigned long now = millis();
  bool warningStateChanged = !warningDisplayInitialized ||
                             lastNanoWarningState != nanoConnected;
  if (!warningStateChanged && (nanoConnected || now - lastNanoDiagnosticDisplayAt < 500UL)) return;

  oledDisplayActive = true;
  display.ssd1306_command(SSD1306_DISPLAYON);
  display.clearDisplay();
  if (!nanoConnected) {
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(22, 0);
    display.print("WARNING");
    display.drawLine(0, 20, 127, 20, SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(10, 23);
    display.print("NANO NOT FOUND");
    display.setCursor(0, 35);
    display.print("RX:");
    display.print(nanoRxByteCount);
    display.print(" OK:");
    display.print(nanoResponseCount);
    display.setCursor(0, 47);
    display.print("BAD:");
    display.print(nanoRejectedFrameCount);
    display.print(" CRC:");
    display.print(nanoChecksumErrorCount);
    display.setCursor(0, 55);
    display.print("SEQ:");
    display.print(nanoSequenceErrorCount);
  }
  display.display();
  lastNanoWarningState = nanoConnected;
  warningDisplayInitialized = true;
  lastNanoDiagnosticDisplayAt = now;
}

void forceNanoDisconnectedOutputsOff() {
  ledOn = false;
  userLEDTarget = 0.0;
  ledPWM = 0.0;
  fanPWM = 0;
  outputLEDPWM(0);
  setFanPWM(0);
}

void bacaSuhu() {

  // Maksimal kegagalan beruntun sebelum sensor dinyatakan bermasalah
  const uint8_t TEMP_ERROR_LIMIT = 3;
  static uint8_t tempErrorCount = 0;

  unsigned long now = millis();

  // Mulai konversi suhu secara asynchronous
  if (!temperatureRequestPending) {

    sensors.requestTemperatures();

    temperatureRequestPending = true;
    temperatureRequestStart = now;

    return;
  }

  // Tunggu proses konversi DS18B20 selesai
  if (now - temperatureRequestStart < TEMP_CONVERSION_TIME) {
    return;
  }

  // Baca hasil konversi
  float t = sensors.getTempCByIndex(0);

  temperatureRequestPending = false;

  // Periksa apakah hasil pembacaan valid
  bool invalidReading =
    (t == DEVICE_DISCONNECTED_C) ||
    (t < -50.0f) ||
    (t > 125.0f) ||
    !isfinite(t);

  if (invalidReading) {

    // Hitung kegagalan berturut-turut
    if (tempErrorCount < TEMP_ERROR_LIMIT) {
      tempErrorCount++;
    }

    // Aktifkan fault hanya setelah 3 kali gagal berturut-turut
    if (tempErrorCount >= TEMP_ERROR_LIMIT) {
      tempFault = true;
      temperatureValid = false;
    }

    // Pertahankan suhu terakhir yang valid selama
    // kegagalan belum mencapai batas
    return;
  }

  // Pembacaan berhasil: simpan suhu terbaru
  temperatureC = t;

  // Reset penghitung kegagalan
  tempErrorCount = 0;

  // Pulihkan status sensor
  temperatureValid = true;
  tempFault = false;
}

void captureCurrentModeMemory() {

  switch (currentMode) {

    case MODE_LOW:

      if (lowMemory != userLEDTarget) {

        lowMemory = userLEDTarget;

        modeMemoryDirty = true;
      }

      break;


    case MODE_MEDIUM:

      if (mediumMemory != userLEDTarget) {

        mediumMemory = userLEDTarget;

        modeMemoryDirty = true;
      }

      break;


    case MODE_HIGH:

      if (highMemory != userLEDTarget) {

        highMemory = userLEDTarget;

        modeMemoryDirty = true;
      }

      break;


    case MODE_STROBE:

      /*
        Saat low battery, STROBO hanya dibatasi sementara
        sampai 15%.

        Jangan simpan batas sementara tersebut ke memory.
      */

      if (!batteryCritical) {

        if (strobeMemory != userLEDTarget) {

          strobeMemory = userLEDTarget;

          modeMemoryDirty = true;
        }
      }

      break;


    case MODE_TURBO:

      // Do not overwrite the user's Turbo memory merely because the timer
      // temporarily reduced the output to the HIGH ceiling.
      if (!turboTimeoutApplied && turboMemory != userLEDTarget) {

        turboMemory = userLEDTarget;

        modeMemoryDirty = true;
      }

      break;
  }
}

void changeStrobeSpeed(int amount) {

  if (!ledOn) {
    return;
  }

  if (buttonsLocked) {
    return;
  }

  if (currentMode != MODE_STROBE) {
    return;
  }

  int newSpeed =
    (int)strobeSpeedHz + amount;

  newSpeed = constrain(
               newSpeed,
               STROBE_SPEED_MIN,
               STROBE_SPEED_MAX
             );

  if (newSpeed != strobeSpeedHz) {
    strobeSpeedHz =
      (uint8_t)newSpeed;

    strobeSpeedDirty = true;

    // Terapkan speed baru langsung, tanpa menyentuh
    // brightness / userLEDTarget.
    lastStrobe =
      millis() - getStrobeInterval();
  }
}


void changeTarget(float amount) {

  if (!ledOn || buttonsLocked) {
    return;
  }

  float minVal = getModeMin();
  float maxVal = getEffectiveModeMax();

  userLEDTarget += amount;

  userLEDTarget = constrain(
                    userLEDTarget,
                    minVal,
                    maxVal
                  );

  // Perbarui memori untuk mode yang sedang aktif
  captureCurrentModeMemory();

  // Mulai ulang waktu tunggu penyimpanan setiap kali
  // pengguna mengubah tingkat kecerahan
  if (modeMemoryDirty) {
    modeMemoryChangeAt = millis();
  }

  // STROBO: perbarui tingkat PWM tanpa mengubah
  // status fase ON/OFF strobo
  if (currentMode == MODE_STROBE) {

    ledPWM = userLEDTarget;

    if (strobeState) {
      outputLEDPWM(ledPWM);
    } else {
      outputLEDPWM(0);
    }
  }
}

void checkBatteryProtection() {

  // ==========================================================
  // BATTERY TIDAK TERPASANG
  // ==========================================================

  if (!batteryPresent) {
    batteryCritical = false;
    ledOn = false;
    userLEDTarget = 0.0;
    ledPWM = 0.0;
    outputLEDPWM(0);
    fanPWM = 0;
    lastFanPWMOutput = 0;
    analogWrite(FAN_PWM_PIN, 0);

    return;

  }

  // ==========================================================
  // BATTERY CRITICAL <= 9.00V
  // LED DAN FAN DIPAKSA OFF
  // ==========================================================

  if (
    batteryVoltage > 0.0 &&
    batteryVoltage <= criticalBatteryVoltage
  ) {

    batteryCritical = true;

    // Paksa LED OFF tanpa mengubah mode dan target pengguna.
    ledOn = false;
    userLEDTarget = 0.0;
    ledPWM = 0.0;

    outputLEDPWM(0);

    // Paksa kipas OFF, tidak boleh ada PWM aktif.
    fanPWM = 0;

    lastFanPWMOutput = 0;

    analogWrite(FAN_PWM_PIN, 0);

    return;

  }

  // ==========================================================
  // BATTERY RECOVERY >= 10.80V
  // ==========================================================

  if (
    batteryCritical &&
    batteryVoltage >= recoveryBatteryVoltage
  ) {

    batteryCritical = false;

  }

  // Jika masih dalam status critical, kipas tetap OFF.
  if (batteryCritical) {

    ledPWM = 0.0;

    outputLEDPWM(0);

    fanPWM = 0;

    lastFanPWMOutput = 0;

    analogWrite(FAN_PWM_PIN, 0);

  }
}


void checkCoolingFault() {

  if (!temperatureValid) {

    return;
  }


  if (
    fanPWM >= 100 &&
    temperatureC >= 75
  ) {

    if (
      temperatureC >
      previousTemperature + 0.2
    ) {

      coolingRiseCounter++;

    } else {

      coolingRiseCounter = 0;
    }


    if (
      coolingRiseCounter >=
      COOLING_FAULT_COUNT
    ) {

      coolingFault = true;


      enterSafeMode(
        "COOLING"
      );


      setFanPWM(100);
    }

  } else {

    coolingRiseCounter = 0;


    if (
      temperatureC < 68.0
    ) {

      coolingFault = false;
    }
  }


  previousTemperature =
    temperatureC;
}

void checkSafeRecovery() {

  if (!safeMode) {
    return;
  }


  if (tempFault) {
    return;
  }


  if (coolingFault) {
    return;
  }


  if (
    temperatureValid &&
    !thermalEmergency
  ) {

    safeMode = false;
  }
}

void enterSafeMode(
  const char *reason
) {

  safeMode = true;

  setFanPWM(
    ledOn ? 20 : 0
  );
}


void handleMainButton() {

  unsigned long now =
    millis();


  bool rawState =
    digitalRead(BTN_MAIN);


  if (
    rawState !=
    mainRawState
  ) {

    mainRawState =
      rawState;

    mainDebounceStart =
      now;
  }


  if (
    now -
    mainDebounceStart <
    MAIN_DEBOUNCE
  ) {

    return;
  }


  if (
    mainStableState ==
    mainRawState
  ) {

    // ========================================================
    // SETTING MODE: MAIN ditahan 5 detik saat mode LOW
    // ========================================================

    if (
      mainStableState == LOW &&
      currentMode == MODE_LOW &&
      !mainLongHandled &&
      now - mainPressStart >= SETTING_HOLD_TIME
    ) {
      mainLongHandled = true;
      mainClickCount = 0;
      mainLastRelease = 0;
      settingMode = true;
      settingEditing = false;
      settingResetConfirm = false;
      settingResetChoice = 0;
      settingPage = 0;
      settingCategoryIndex = 0;
      settingSubmenuIndex = 0;
      settingMenuIndex = 0;
      // MAIN masih ditahan untuk masuk Settings; anggap tombol sedang ditekan
      // dan abaikan event release ini agar tidak otomatis memilih DISPLAY SETTINGS.
      settingMainRaw = LOW;
      settingMainStable = LOW;
      settingMainDebounceAt = now;
      settingPlusRaw = digitalRead(BTN_PLUS);
      settingPlusStable = settingPlusRaw;
      settingPlusDebounceAt = now;
      settingMinusRaw = digitalRead(BTN_MINUS);
      settingMinusStable = settingMinusRaw;
      settingMinusDebounceAt = now;
      settingMainLongDone = true;
      settingMainBackDone = true;
      settingMainPressAt = now;
      forceSettingOutputsOff();
      return;
    }

    // ========================================================
    // LONG PRESS NORMAL
    // ========================================================

    if (
      mainStableState == LOW &&
      ledOn &&
      currentMode != MODE_LOW &&
      !mainLongHandled
    ) {

      if (
        now -
        mainPressStart >=
        MAIN_LONG_PRESS
      ) {

        mainLongHandled = true;

        mainClickCount = 0;

        mainLastRelease = 0;

        // STROBO: MAIN ditahan menjadi modifier untuk speed.
        // Jangan jalankan fungsi long-press pindah mode.
        if (currentMode == MODE_STROBE) {
          return;
        }


        if (
          currentMode ==
          MODE_LOW
        ) {

          setMode(
            MODE_MEDIUM
          );

        }

        else if (
          currentMode ==
          MODE_MEDIUM
        ) {

          setMode(
            MODE_HIGH
          );

        }

        else if (
          currentMode ==
          MODE_HIGH
        ) {

          setMode(
            MODE_LOW
          );

        }

        else {

          setMode(
            MODE_LOW
          );
        }
      }
    }


    // ========================================================
    // CLICK TIMEOUT
    // ========================================================

    if (
      mainClickCount > 0 &&
      mainLastRelease > 0
    ) {

      if (
        now -
        mainLastRelease >=
        MAIN_CLICK_TIMEOUT
      ) {

        executeMainClicks();
      }
    }


    return;
  }


  mainStableState =
    mainRawState;


  if (
    mainStableState ==
    LOW
  ) {

    mainPressStart =
      now;

    mainLongHandled =
      false;

    return;
  }


  if (
    mainStableState ==
    HIGH
  ) {

    // Pertahankan long-press LOW -> MEDIUM bila dilepas
    // sebelum 5 detik. Tahan 5 detik masuk ke menu setting.
    if (
      currentMode == MODE_LOW &&
      ledOn &&
      !mainLongHandled &&
      now - mainPressStart >= MAIN_LONG_PRESS &&
      now - mainPressStart < SETTING_HOLD_TIME
    ) {
      mainClickCount = 0;
      mainLastRelease = 0;
      mainLongHandled = true;
      setMode(MODE_MEDIUM);
      return;
    }

    if (mainLongHandled) {

      mainLongHandled =
        false;

      return;
    }


    if (
      mainClickCount <
      5
    ) {

      mainClickCount++;
    }


    mainLastRelease =
      now;
  }
}

void kontrolFan() {

  // ==========================================================
  // BATTERY CRITICAL: FAN WAJIB OFF
  // Prioritas tertinggi, tidak boleh ditimpa kondisi lain.
  // ==========================================================

  if (batteryCritical) {

    fanPWM = 0;

    lastFanPWMOutput = 0;

    analogWrite(FAN_PWM_PIN, 0);

    return;

  }

  // ==========================================================
  // SAFE MODE
  // ==========================================================

  if (safeMode) {

    setFanPWM(ledOn ? 20 : 0);

    return;

  }

  // ==========================================================
  // THERMAL EMERGENCY
  // ==========================================================

  if (thermalEmergency) {

    setFanPWM(100);

    return;

  }

  if (thermalProtect) {

    setFanPWM(100);

    return;

  }

  if (currentMode == MODE_TURBO && ledOn && turboFanBoostEnabled) {
    if (!fanTemperatureActive && temperatureC >= fanStartTemp) fanTemperatureActive = true;
    if (fanTemperatureActive && temperatureC <= fanStopTemp) fanTemperatureActive = false;

    if (!fanTemperatureActive) {
      setFanPWM(turboFanStartSpeed);
    } else {
      float range = fanTempLimit - fanStartTemp;
      if (range < 1.0) range = 1.0;
      uint8_t turboFanMaxSpeed = max(turboFanStartSpeed, fanMaxSpeed);
      int pwmPercent = turboFanStartSpeed +
                       (int)((temperatureC - fanStartTemp) *
                             (turboFanMaxSpeed - turboFanStartSpeed) / range);
      setFanPWM(constrain(pwmPercent, (int)turboFanStartSpeed, (int)turboFanMaxSpeed));
    }
    return;
  }

  // ==========================================================
  // AUTO TEMPERATURE CONTROL
  // ==========================================================

  if (!fanTemperatureActive && temperatureC >= fanStartTemp) fanTemperatureActive = true;
  if (fanTemperatureActive && temperatureC <= fanStopTemp) fanTemperatureActive = false;
  if (!fanTemperatureActive) {
    setFanPWM(0);
  } else {
    float range = fanTempLimit - fanStartTemp;
    if (range < 1.0) range = 1.0;
    int pwmPercent = fanMinSpeed + (int)((temperatureC - fanStartTemp) * (fanMaxSpeed - fanMinSpeed) / range);
    setFanPWM(constrain(pwmPercent, (int)fanMinSpeed, (int)fanMaxSpeed));
  }
}

void kontrolThermal() {
  if (batteryCritical) {
    ledPWM = 0.0;
    outputLEDPWM(0);
    fanPWM = 0;
    lastFanPWMOutput = 0;
    analogWrite(FAN_PWM_PIN, 0);
    return;
  }


  if (!temperatureValid) {
    enterSafeMode("TEMP INVALID");
    return;
  }

  // ==========================================================
  // THERMAL CUTOFF
  // ==========================================================

  if (
    temperatureC >= THERMAL_CUTOFF
  ) {

    thermalEmergency = true;

    thermalProtect = true;

    setFanPWM(100);

    ledOn = false;

    userLEDTarget = 0.0;

    outputLEDPWM(0);

    ledPWM = 0.0;

    return;

  }

  // ==========================================================
  // THERMAL RECOVERY
  // ==========================================================

  if (thermalEmergency) {

    setFanPWM(100);

    if (
      temperatureC > THERMAL_RECOVERY
    ) {

      return;
    }

    thermalEmergency = false;

    thermalProtect = false;

    return;

  }

  // ==========================================================
  // THERMAL PROTECT
  // ==========================================================

  if (
    temperatureC >= ledTempLimit
  ) {

    thermalProtect = true;

    setFanPWM(100);

    return;

  }

  if (thermalProtect) {

    if (
      temperatureC > THERMAL_RECOVERY
    ) {

      return;
    }

    thermalProtect = false;

  }
}

void ledOff() {
  captureCurrentModeMemory();
  saveModeMemory();
  saveStrobeSpeed();
  ledOn = false;
  turboTimeoutApplied = false;
  userLEDTarget = 0.0;
  if (!fadeOutEnabled) {
    ledPWM = 0.0;
    outputLEDPWM(0);
  }
  setFanPWM(0);
}

void loadModeMemory() {

  EEPROM.get(
    EEPROM_ADDR,
    storedMemory
  );


  bool valid =
    storedMemory.magic ==
    EEPROM_MAGIC &&

    storedMemory.version ==
    EEPROM_VERSION &&

    isfinite(
      storedMemory.low
    ) &&

    isfinite(
      storedMemory.medium
    ) &&

    isfinite(
      storedMemory.high
    ) &&

    isfinite(
      storedMemory.strobo
    ) &&

    isfinite(
      storedMemory.turbo
    );


  if (!valid) {

    lowMemory = 10.0;
    mediumMemory = 18.0;
    highMemory = 30.0;
    strobeMemory = 60.0;
    turboMemory = 100.0;

    storedMemory.magic =
      EEPROM_MAGIC;

    storedMemory.version =
      EEPROM_VERSION;

    storedMemory.low =
      lowMemory;

    storedMemory.medium =
      mediumMemory;

    storedMemory.high =
      highMemory;

    storedMemory.strobo =
      strobeMemory;

    storedMemory.turbo =
      turboMemory;


    EEPROM.put(
      EEPROM_ADDR,
      storedMemory
    );

    EEPROM.commit();

  } else {

    lowMemory =
      constrain(
        storedMemory.low,
        0.1,
        15.0
      );


    mediumMemory =
      constrain(
        storedMemory.medium,
        10.0,
        25.0
      );


    highMemory =
      constrain(
        storedMemory.high,
        15.0,
        40.0
      );


    strobeMemory =
      constrain(
        storedMemory.strobo,
        0.1,
        100.0
      );


    turboMemory =
      constrain(
        storedMemory.turbo,
        1.0,
        100.0
      );
  }


  modeMemoryDirty = false;
}

void loadSettingConfig() {
  float storedFanLimit = 70.0;
  float storedBatteryOffset = 0.0;
  float storedWattCapacity = LED_POWER_DEFAULT;
  uint8_t storedFade = 1;
  uint8_t storedFadeDuration = 3;
  uint8_t storedBrightness = 180;
  uint8_t storedTimeout = 5;
  uint8_t storedTimeoutUnit = 0xFF;
  uint8_t storedVoltage = 1;
  uint8_t storedPercent = 1;
  uint8_t storedPower = 1;
  uint8_t storedTemperature = 1;
  uint8_t storedFanDisplay = 1;
  uint8_t storedFadeOut = 1;
  uint8_t storedFadeOutDuration = 3;
  float storedLowOutput = 15.0;
  float storedMediumOutput = 25.0;
  float storedHighOutput = 40.0;
  float storedTurboOutput = 100.0;
  uint8_t storedTurboTimeout = 30;
  uint8_t storedFanMinSpeed = 30;
  uint8_t storedPWM = 1, storedDisplayMode = 2, storedFanMaxSpeed = 100, storedLowBattLimit = 15;
  uint8_t storedPWMFrequency = 0xFF;
  uint8_t storedTurboFanMode = 0xFF;
  uint8_t storedTurboFanStart = 30;
  float storedFanStartTemp = 45.0, storedFanStopTemp = 40.0, storedLedTempLimit = 75.0, storedLowBattWarning = 11.0, storedCriticalVoltage = 9.0, storedRecoveryVoltage = 10.8;

  EEPROM.get(EEPROM_FAN_LIMIT_ADDR, storedFanLimit);
  EEPROM.get(EEPROM_BATTERY_CAL_ADDR, storedBatteryOffset);
  EEPROM.get(EEPROM_FADE_ENABLED_ADDR, storedFade);
  EEPROM.get(EEPROM_FADE_DURATION_ADDR, storedFadeDuration);
  EEPROM.get(EEPROM_LED_WATT_ADDR, storedWattCapacity);
  EEPROM.get(EEPROM_OLED_BRIGHTNESS_ADDR, storedBrightness);
  EEPROM.get(EEPROM_DISPLAY_TIMEOUT_ADDR, storedTimeout);
  EEPROM.get(EEPROM_DISPLAY_TIMEOUT_UNIT_ADDR, storedTimeoutUnit);
  EEPROM.get(EEPROM_SHOW_VOLTAGE_ADDR, storedVoltage);
  EEPROM.get(EEPROM_SHOW_BATTERY_PERCENT_ADDR, storedPercent);
  EEPROM.get(EEPROM_SHOW_POWER_ADDR, storedPower);
  EEPROM.get(EEPROM_SHOW_TEMPERATURE_ADDR, storedTemperature);
  EEPROM.get(EEPROM_SHOW_FAN_ADDR, storedFanDisplay);
  EEPROM.get(EEPROM_FADE_OUT_ENABLED_ADDR, storedFadeOut);
  EEPROM.get(EEPROM_FADE_OUT_DURATION_ADDR, storedFadeOutDuration);
  EEPROM.get(EEPROM_LOW_OUTPUT_ADDR, storedLowOutput);
  EEPROM.get(EEPROM_MED_OUTPUT_ADDR, storedMediumOutput);
  EEPROM.get(EEPROM_HIGH_OUTPUT_ADDR, storedHighOutput);
  EEPROM.get(EEPROM_TURBO_OUTPUT_ADDR, storedTurboOutput);
  EEPROM.get(EEPROM_TURBO_TIMEOUT_ADDR, storedTurboTimeout);
  EEPROM.get(EEPROM_FAN_MIN_SPEED_ADDR, storedFanMinSpeed);
  EEPROM.get(EEPROM_SHOW_PWM_ADDR, storedPWM);
  EEPROM.get(EEPROM_DISPLAY_MODE_ADDR, storedDisplayMode);
  EEPROM.get(EEPROM_FAN_START_TEMP_ADDR, storedFanStartTemp);
  EEPROM.get(EEPROM_FAN_STOP_TEMP_ADDR, storedFanStopTemp);
  EEPROM.get(EEPROM_FAN_MAX_SPEED_ADDR, storedFanMaxSpeed);
  EEPROM.get(EEPROM_LED_TEMP_LIMIT_ADDR, storedLedTempLimit);
  EEPROM.get(EEPROM_LOW_BATT_WARN_ADDR, storedLowBattWarning);
  EEPROM.get(EEPROM_LOW_BATT_LIMIT_ADDR, storedLowBattLimit);
  EEPROM.get(EEPROM_CRITICAL_VOLT_ADDR, storedCriticalVoltage);
  EEPROM.get(EEPROM_RECOVERY_VOLT_ADDR, storedRecoveryVoltage);
  EEPROM.get(EEPROM_PWM_FREQUENCY_ADDR, storedPWMFrequency);
  EEPROM.get(EEPROM_TURBO_FAN_MODE_ADDR, storedTurboFanMode);
  EEPROM.get(EEPROM_TURBO_FAN_START_ADDR, storedTurboFanStart);

  fanTempLimit = (isfinite(storedFanLimit) && storedFanLimit >= 45.0 && storedFanLimit <= 70.0) ? storedFanLimit : 70.0;
  batteryCalOffset = (isfinite(storedBatteryOffset) && storedBatteryOffset >= -8.00 && storedBatteryOffset <= 2.00) ? storedBatteryOffset : 0.0;
  fadeEnabled = storedFade <= 1 ? storedFade == 1 : true;
  fadeDurationSec = (storedFadeDuration >= 1 && storedFadeDuration <= 10) ? storedFadeDuration : 3;
  ledWattCapacity = (isfinite(storedWattCapacity) && storedWattCapacity >= 10.0 && storedWattCapacity <= 200.0) ? storedWattCapacity : LED_POWER_DEFAULT;
  oledBrightness = (storedBrightness >= 1 && storedBrightness <= 100) ? storedBrightness : 70;
  bool validStoredTimeout = storedTimeout >= 1 && storedTimeout <= 60;
  if (storedTimeoutUnit <= 1) {
    displayTimeoutValue = validStoredTimeout ? storedTimeout : 1;
    displayTimeoutInMinutes = storedTimeoutUnit == 1;
  } else if (validStoredTimeout) {
    displayTimeoutValue = storedTimeout;
    displayTimeoutInMinutes = true;
  } else {
    displayTimeoutValue = 1;
    displayTimeoutInMinutes = false;
  }
  showVoltage = storedVoltage <= 1 ? storedVoltage == 1 : true;
  showBatteryPercent = storedPercent <= 1 ? storedPercent == 1 : true;
  showPowerWatt = storedPower <= 1 ? storedPower == 1 : true;
  showTemperature = storedTemperature <= 1 ? storedTemperature == 1 : true;
  showFanStatus = storedFanDisplay <= 1 ? storedFanDisplay == 1 : true;
  fadeOutEnabled = storedFadeOut <= 1 ? storedFadeOut == 1 : true;
  fadeOutDurationSec = (storedFadeOutDuration >= 1 && storedFadeOutDuration <= 10) ? storedFadeOutDuration : 3;
  lowOutputLimit = (isfinite(storedLowOutput) && storedLowOutput >= 1.0 && storedLowOutput <= 30.0) ? storedLowOutput : 15.0;
  mediumOutputLimit = (isfinite(storedMediumOutput) && storedMediumOutput >= 1.0 && storedMediumOutput <= 50.0) ? storedMediumOutput : 25.0;
  highOutputLimit = (isfinite(storedHighOutput) && storedHighOutput >= 1.0 && storedHighOutput <= 75.0) ? storedHighOutput : 40.0;
  turboOutputLimit = (isfinite(storedTurboOutput) && storedTurboOutput >= 1.0 && storedTurboOutput <= 100.0) ? storedTurboOutput : 100.0;
  turboTimeoutSec = storedTurboTimeout <= 180 ? storedTurboTimeout : 30;
  fanMinSpeed = (storedFanMinSpeed <= 70) ? storedFanMinSpeed : 30;
  showPWM = storedPWM <= 1 ? storedPWM == 1 : true;
  displayMode = storedDisplayMode <= 2 ? storedDisplayMode : 2;
  fanStartTemp = (isfinite(storedFanStartTemp) && storedFanStartTemp >= 35 && storedFanStartTemp <= 75) ? storedFanStartTemp : 45.0;
  fanStopTemp = (isfinite(storedFanStopTemp) && storedFanStopTemp >= 25 && storedFanStopTemp < fanStartTemp) ? storedFanStopTemp : 40.0;
  fanMaxSpeed = (storedFanMaxSpeed >= max((uint8_t)20, fanMinSpeed) && storedFanMaxSpeed <= 100) ? storedFanMaxSpeed : 100;
  ledTempLimit = (isfinite(storedLedTempLimit) && storedLedTempLimit >= 65 && storedLedTempLimit <= 82) ? storedLedTempLimit : 75.0;
  lowBatteryWarningVoltage = (isfinite(storedLowBattWarning) && storedLowBattWarning >= 10 && storedLowBattWarning <= 12) ? storedLowBattWarning : 11.0;
  lowBatteryOutputLimit = (storedLowBattLimit >= 1 && storedLowBattLimit <= 50) ? storedLowBattLimit : 15;
  criticalBatteryVoltage = (isfinite(storedCriticalVoltage) && storedCriticalVoltage >= 8.5 && storedCriticalVoltage <= 9.6) ? storedCriticalVoltage : 9.0;
  recoveryBatteryVoltage = (isfinite(storedRecoveryVoltage) && storedRecoveryVoltage > criticalBatteryVoltage && storedRecoveryVoltage <= 12.0) ? storedRecoveryVoltage : max(10.8f, criticalBatteryVoltage + 0.5f);
  pwmFrequencyKHz = (storedPWMFrequency >= PWM_FREQUENCY_MIN_KHZ && storedPWMFrequency <= PWM_FREQUENCY_MAX_KHZ) ? storedPWMFrequency : PWM_FREQUENCY_DEFAULT_KHZ;
  turboFanBoostEnabled = storedTurboFanMode <= 1 ? storedTurboFanMode == 1 : false;
  turboFanStartSpeed = storedTurboFanStart <= 100 ? storedTurboFanStart : 30;
  applyOLEDSettings();
}


void outputLEDPWM(float percent) {
  if (!nanoConnected) percent = 0.0f;

  percent = constrain(
              percent,
              0.0f,
              100.0f
            );

  // Konversi persentase ke PWM 10-bit.
  // Tidak dibalik: semakin besar percent,
  // semakin besar duty cycle HIGH pada pin D5.
  int pwm = (int)round(
              (percent / 100.0f) * PWM_MAX
            );

  pwm = constrain(
          pwm,
          0,
          PWM_MAX
        );

  static int lastLEDPWMOutput = -1;

  if (pwm != lastLEDPWMOutput) {

    analogWrite(
      LED_PWM_PIN,
      pwm
    );

    lastLEDPWMOutput = pwm;
  }
}

void saveModeMemory() {

  if (!modeMemoryDirty) {
    return;
  }

  storedMemory.magic =
    EEPROM_MAGIC;

  storedMemory.version =
    EEPROM_VERSION;

  storedMemory.low =
    lowMemory;

  storedMemory.medium =
    mediumMemory;

  storedMemory.high =
    highMemory;

  storedMemory.strobo =
    strobeMemory;

  storedMemory.turbo =
    turboMemory;


  EEPROM.put(
    EEPROM_ADDR,
    storedMemory
  );

  EEPROM.commit();

  modeMemoryDirty = false;
}

void setFanPWM(int percent) {
  if (!nanoConnected) percent = 0;
  if (!temperatureValid || tempFault) percent = 0;

  percent =
    constrain(
      percent,
      0,
      100
    );


  fanPWM =
    percent;


  int pwm =
    map(
      percent,
      0,
      100,
      0,
      PWM_MAX
    );


  if (
    percent !=
    lastFanPWMOutput
  ) {

    analogWrite(
      FAN_PWM_PIN,
      pwm
    );

    lastFanPWMOutput =
      percent;
  }
}

void setMode(
  LightMode newMode
) {
  if (
    batteryCritical &&
    (
      newMode == MODE_MEDIUM ||
      newMode == MODE_HIGH ||
      newMode == MODE_TURBO
    )
  ) {
    newMode =
      MODE_LOW;
  }
  // While off, userLEDTarget is intentionally zero; never save it over mode memory.
  if (currentMode != newMode && ledOn) {
    captureCurrentModeMemory();
    saveModeMemory();
    saveStrobeSpeed();
  }

  oledDisplayActive = true;

  if (oledOK) {

    display.ssd1306_command(
      SSD1306_DISPLAYON
    );
  }
  currentMode =
    newMode;
  modeStartedAt = millis();
  turboTimeoutApplied = false;
  userLEDTarget =
    getModeMemory();
  float maxAllowed =
    getEffectiveModeMax();
  userLEDTarget =
    constrain(
      userLEDTarget,
      getModeMin(),
      maxAllowed
    );
  if (currentMode == MODE_TURBO) {
    setFanPWM(100);
  }
  ledOn = true;
  if (
    !thermalEmergency &&
    !thermalProtect
  ) {
    if (!fadeEnabled && getSafetyTarget() >= ledPWM) {
      ledPWM = getSafetyTarget();
      outputLEDPWM(ledPWM);
    }
  }
  if (
    currentMode ==
    MODE_STROBE
  ) {
    strobeState = false;
    lastStrobe =
      millis() -
      getStrobeInterval();
  }
}

void updateModeMemorySave() {
  if (!modeMemoryDirty) {
    return;
  }  // Tunggu sampai perubahan kecerahan berhenti selama 2 detik
  if (millis() - modeMemoryChangeAt >= MODE_MEMORY_SAVE_DELAY) {
    saveModeMemory();
  }
}



void updateSettingButton(uint8_t pin, bool &rawState, bool &stableState, unsigned long &changedAt, bool &pressed, bool &released) {
  bool reading = digitalRead(pin);
  unsigned long now = millis();
  pressed = false;
  released = false;

  if (reading != rawState) {
    rawState = reading;
    changedAt = now;
  }

  if (now - changedAt < SETTING_DEBOUNCE_MS) return;

  if (stableState != rawState) {
    stableState = rawState;
    if (stableState == LOW) pressed = true;
    else released = true;
  }
}

void updateSmoothLED() {

  unsigned long now =
    millis();


  if (
    now -
    lastLEDSmooth <
    LED_SMOOTH_INTERVAL
  ) {

    return;
  }

  lastLEDSmooth =
    now;

  if (batteryCritical) {
    ledPWM = 0.0;
    outputLEDPWM(0);
    return;
  }

  float safetyTarget =
    getSafetyTarget();
  if (thermalEmergency) {
    ledPWM -=
      LED_DOWN_SPEED_THERMAL;
    if (ledPWM < 0.0) {
      ledPWM = 0.0;
    }
    outputLEDPWM(
      ledPWM
    );
    return;
  }

  if (thermalProtect) {

    ledPWM -=
      LED_DOWN_SPEED_THERMAL;
    if (ledPWM < 0.0) {
      ledPWM = 0.0;
    }
    outputLEDPWM(
      ledPWM
    );

    return;
  }
  if (safetyTarget > ledPWM && fadeEnabled) {
    float stepIn = 100.0f * (float)LED_SMOOTH_INTERVAL / ((float)fadeDurationSec * 1000.0f);
    ledPWM = min(ledPWM + stepIn, safetyTarget);
  } else if (safetyTarget < ledPWM && fadeOutEnabled) {
    float stepOut = 100.0f * (float)LED_SMOOTH_INTERVAL / ((float)fadeOutDurationSec * 1000.0f);
    ledPWM = max(ledPWM - stepOut, safetyTarget);
  } else {
    ledPWM = safetyTarget;
  }
  outputLEDPWM(ledPWM);
}

void updateTurboTimeout() {
  if (!ledOn || currentMode != MODE_TURBO) {
    turboTimeoutApplied = false;
    return;
  }
  if (turboTimeoutSec == 0 || turboTimeoutApplied) return;
  if (millis() - modeStartedAt >= (unsigned long)turboTimeoutSec * 1000UL) {
    turboTimeoutApplied = true;
  }
}

void setup() {
  Serial.begin(BATTERY_SERIAL_BAUD);
  WiFi.mode(
    WIFI_OFF
  );
  WiFi.forceSleepBegin();
  delay(1);
  delay(1000);

  EEPROM.begin(
    EEPROM_SIZE
  );
  loadModeMemory();
  loadStrobeSpeed();
  loadSettingConfig();
  pinMode(
    LED_PWM_PIN,
    OUTPUT
  );
  pinMode(
    FAN_PWM_PIN,
    OUTPUT
  );
  pinMode(
    BTN_MAIN,
    INPUT_PULLUP
  );
  pinMode(
    BTN_PLUS,
    INPUT_PULLUP
  );

  pinMode(
    BTN_MINUS,
    INPUT_PULLUP
  );
  applyPWMFrequency();
  analogWriteRange(
    PWM_MAX
  );
  outputLEDPWM(0);
  setFanPWM(0);
  ledOn = false;
  userLEDTarget = 0.0;
  ledPWM = 0.0;
  Wire.begin(
    SDA_PIN,
    SCL_PIN
  );
  if (
    display.begin(
      SSD1306_SWITCHCAPVCC,
      0x3C
    )
  ) {
    oledOK = true;
    applyOLEDSettings();
  } else {
    oledOK = false;
  }
  sensors.begin();
  sensors.setWaitForConversion(false);
  temperatureRequestPending =
    false;
  temperatureValid =
    false;
  temperatureC =
    25.0;
  oledDisplayActive =
    true;
}

void loop() {
  unsigned long now =
    millis();
  requestNanoBattery();
  bacaDataBatteryNano();
  updateNanoConnection();
  bacaBattery();
  updateNanoWarningDisplay();
  if (!nanoConnected) {
    forceNanoDisconnectedOutputsOff();
    return;
  }
  checkBatteryProtection();
  updateBatteryWarningBlink();
  updateWarningBlink();
  // Jika menu sedang aktif, jangan jalankan handler tombol normal.
  // Handler normal dapat menafsirkan MAIN sebagai klik mode.
  if (settingMode) {
    handleSettingMenu();
    if (now - lastDisplay >= DISPLAY_INTERVAL) {
      lastDisplay = now;
      tampilkanOLED();
    }
    return;
  }
  handleLockButtons();
  handleMainButton();
  updateTurboTimeout();
  if (settingMode) {
    handleSettingMenu();
    if (now - lastDisplay >= DISPLAY_INTERVAL) {
      lastDisplay = now;
      tampilkanOLED();
    }
    return;
  }
  handlePlusButton();
  handleMinusButton();
  if (
    now -
    lastTempRead >=
    TEMP_INTERVAL
  ) {
    lastTempRead =
      now;
    bacaSuhu();
    kontrolFan();
  }
  if (
    now -
    lastThermalControl >=
    THERMAL_INTERVAL
  ) {

    lastThermalControl =
      now;
    kontrolThermal();
    checkCoolingFault();
    checkSafeRecovery();
    kontrolFan();
  }

  if (currentMode == MODE_STROBE) {
    updateStrobe();
  } else {
    updateSmoothLED();
  }
  updateModeMemorySave();
  if (
    now -
    lastDisplay >=
    DISPLAY_INTERVAL
  ) {
    lastDisplay =
      now;
    tampilkanOLED();
  }
}

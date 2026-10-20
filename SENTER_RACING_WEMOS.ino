/*
  ============================================================
            SENTER RACING - MAIN FIRMWARE
            ESP8266 / Wemos D1 Mini
            BATTERY PROFILE CONFIGURABLE
  ============================================================
  BATTERY
  ------------------------------------------------------------
  Battery chemistry and series count are selected in BATTERY SETTINGS.
  LiFePO4: 3.2V nominal, 3.65V full, 3.2V warning, 2.8V cutoff per cell.
  LiPo/Li-ion: 4.2V full, 3.5V warning, 3.0V cutoff per cell.
  Voltage thresholds and battery percentage scale with the selected series count.

  VOLTAGE DIVIDER (DI ARDUINO NANO)
  ------------------------------------------------------------
  R1 = 100K
  R2 = 15K

  16.80V battery -> sekitar 2.20V ke A0 Nano (4S LiPo maximum)
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
  R,<sequence>,<millivolts>,<checksum>. P and W control Nano sampling.
  Checksum is XOR before final comma.

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
#include <string.h>
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
#define BATTERY_MAX_MILLIVOLTS 20000UL
#define BATTERY_FILTER_ALPHA 0.30
#define BATTERY_READ_INTERVAL 250
uint8_t lowBatteryOutputLimit = 15;
#define BATTERY_BLINK_INTERVAL 500
enum BatteryChemistry : uint8_t {
  BATTERY_LIFEPO4 = 0,
  BATTERY_POLYMER,
  BATTERY_LITHIUM
};
BatteryChemistry batteryChemistry = BATTERY_POLYMER;
uint8_t batterySeriesCount = 3;

float getBatteryCellFullVoltage() {
  return batteryChemistry == BATTERY_LIFEPO4 ? 3.65f : 4.20f;
}

float getBatteryWarningVoltage() {
  float cellVoltage = batteryChemistry == BATTERY_LIFEPO4 ? 3.20f : 3.50f;
  return cellVoltage * batterySeriesCount;
}

float getBatteryCriticalVoltage() {
  float cellVoltage = batteryChemistry == BATTERY_LIFEPO4 ? 2.80f : 3.00f;
  return cellVoltage * batterySeriesCount;
}

float getBatteryRecoveryVoltage() {
  float cellVoltage = batteryChemistry == BATTERY_LIFEPO4 ? 3.10f : 3.30f;
  return cellVoltage * batterySeriesCount;
}

float batteryVoltage = 0.0;
float filteredBatteryVoltage = -1.0;
float batteryDisplayVoltage = -1.0;  // Filter khusus tampilan OLED agar angka desimal stabil.
bool batteryCalibrationDirty = false;
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
bool nanoPaused = false;
bool nanoWakePending = false;
bool freshBatteryRequired = false;
bool mainScreenRequested = false;
uint8_t pendingMainClicks = 0;
unsigned long nanoWakeStartedAt = 0;
#define NANO_WAKE_TIMEOUT 1500UL
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
  if (batteryPresent && batteryVoltage > getBatteryCriticalVoltage() && batteryVoltage <= getBatteryWarningVoltage()) {
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
uint8_t settingEditField = 0;
float settingScrollPosition = 0.0f;
unsigned long settingScrollUpdatedAt = 0;
uint8_t settingScrollPage = 255;
uint8_t settingScrollCategory = 255;
float settingValueScrollPosition = 0.0f;
unsigned long settingValueScrollUpdatedAt = 0;
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
#define EEPROM_LOW_BATT_LIMIT_ADDR 112
#define EEPROM_PWM_FREQUENCY_ADDR 124
#define EEPROM_TURBO_FAN_MODE_ADDR 125
#define EEPROM_TURBO_FAN_START_ADDR 126
#define EEPROM_SCREEN_SAVER_ADDR 127
#define EEPROM_INVERT_LED_PWM_ADDR 128
#define EEPROM_INVERT_FAN_PWM_ADDR 129
#define EEPROM_BATTERY_CHEMISTRY_ADDR 132
#define EEPROM_BATTERY_SERIES_ADDR 133
uint8_t pwmFrequencyKHz = PWM_FREQUENCY_DEFAULT_KHZ;
bool pwmFrequencyDirty = false;
bool invertLedPWM = false;
bool invertFanPWM = false;
void writeFanPWMOutput(int percent) {
  percent = constrain(percent, 0, 100);
  int pwm = (int)round((percent / 100.0f) * PWM_MAX);
  if (invertFanPWM) pwm = PWM_MAX - pwm;
  analogWrite(FAN_PWM_PIN, pwm);
}
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
uint8_t screenSaverStyle = 1;
uint8_t rollingScreenSaverStyle = 0;
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
bool lowBatteryAlertActive = false;
unsigned long lowBatteryAlertStartedAt = 0;
#define LOW_BATTERY_ALERT_DURATION 3000UL
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
const unsigned long SETTING_DISPLAY_INTERVAL = 80;
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
  if (batteryPresent && batteryVoltage > getBatteryCriticalVoltage() && batteryVoltage <= getBatteryWarningVoltage()) {
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

bool hasFreshZeroBatteryReading() {
  return batteryPacketReceived &&
         receivedBatteryMillivolts == 0 &&
         millis() - lastBatteryPacketAt <= BATTERY_SERIAL_TIMEOUT;
}

bool isBatteryLow() {
  return batteryCritical ||
         hasFreshZeroBatteryReading() ||
         (batteryPresent && batteryVoltage <= getBatteryWarningVoltage());
}

void startLowBatteryAlert() {
  lowBatteryAlertActive = true;
  lowBatteryAlertStartedAt = millis();
  oledIdleSince = lowBatteryAlertStartedAt;
  oledDisplayActive = true;
  if (oledOK) {
    display.ssd1306_command(SSD1306_DISPLAYON);
  }
}

void executeMainClicks() {
  uint8_t clicks = mainClickCount;
  mainClickCount = 0;
  mainLastRelease = 0;
  if (clicks == 0) {
    return;
  }
  bool requestsLight = clicks > 1 || !ledOn;
  if (requestsLight && (!nanoConnected || freshBatteryRequired)) {
    if (nanoWakePending) pendingMainClicks = clicks;
    else mainScreenRequested = false;
    return;
  }
  mainScreenRequested = false;
  if (requestsLight && isBatteryLow()) {
    startLowBatteryAlert();
  }
  if (requestsLight && (batteryCritical || hasFreshZeroBatteryReading())) {
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
  writeFanPWMOutput(0);
  oledDisplayActive = true;
  if (oledOK) display.ssd1306_command(SSD1306_DISPLAYON);
}
enum SettingId : uint8_t {
  SET_STROBE_SPEED = 0, SET_PWM_FREQUENCY, SET_LED_WATT, SET_LOW_OUTPUT, SET_MED_OUTPUT, SET_HIGH_OUTPUT,
  SET_TURBO_OUTPUT, SET_TURBO_TIMEOUT, SET_FADE_IN_ENABLE, SET_FADE_IN_TIME,
  SET_FADE_OUT_ENABLE, SET_FADE_OUT_TIME, SET_FAN_TEMP_LIMIT, SET_FAN_START_TEMP,
  SET_FAN_STOP_TEMP, SET_FAN_MIN_SPEED, SET_FAN_MAX_SPEED, SET_LED_TEMP_LIMIT,
  SET_BATTERY_CAL, SET_BATTERY_CHEMISTRY, SET_BATTERY_SERIES, SET_LOW_BATT_LIMIT,
  SET_SHOW_PWM, SET_SHOW_VOLTAGE, SET_SHOW_BATTERY_PERCENT,
  SET_SHOW_POWER, SET_SHOW_TEMPERATURE, SET_SHOW_FAN, SET_DISPLAY_MODE,
  SET_DISPLAY_TIMEOUT, SET_OLED_BRIGHTNESS, SET_TURBO_FAN_MODE,
  SET_TURBO_FAN_START, SET_RESET_DEFAULTS,
  SET_SCREEN_SAVER,
  SET_INVERT_LED_PWM, SET_INVERT_FAN_PWM,
  SETTING_TOTAL_COUNT
};
#define SETTING_CATEGORY_COUNT 5

const char *settingCategoryLabels[SETTING_CATEGORY_COUNT] = {
  "DISPLAY SETTINGS", "LED AND POWER", "FAN AND TEMPERATURE", "BATTERY SETTINGS", "SYSTEM"
};

const uint8_t settingCategoryItems[SETTING_CATEGORY_COUNT][13] = {
  {SET_SHOW_PWM, SET_SHOW_VOLTAGE, SET_SHOW_BATTERY_PERCENT, SET_SHOW_POWER, SET_SHOW_TEMPERATURE, SET_SHOW_FAN, SET_DISPLAY_MODE, SET_DISPLAY_TIMEOUT, SET_OLED_BRIGHTNESS, SET_SCREEN_SAVER, 255, 255},
  {SET_PWM_FREQUENCY, SET_INVERT_LED_PWM, SET_LED_WATT, SET_LOW_OUTPUT, SET_MED_OUTPUT, SET_HIGH_OUTPUT, SET_TURBO_OUTPUT, SET_TURBO_TIMEOUT, SET_FADE_IN_ENABLE, SET_FADE_IN_TIME, SET_FADE_OUT_ENABLE, SET_FADE_OUT_TIME, SET_STROBE_SPEED},
  {SET_INVERT_FAN_PWM, SET_FAN_TEMP_LIMIT, SET_FAN_START_TEMP, SET_FAN_STOP_TEMP, SET_FAN_MIN_SPEED, SET_FAN_MAX_SPEED, SET_LED_TEMP_LIMIT, SET_TURBO_FAN_MODE, SET_TURBO_FAN_START, 255, 255, 255, 255},
  {SET_BATTERY_CAL, SET_BATTERY_CHEMISTRY, SET_BATTERY_SERIES, SET_LOW_BATT_LIMIT, 255, 255, 255, 255, 255, 255, 255},
  {SET_RESET_DEFAULTS, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255}
};

const uint8_t settingCategoryItemCounts[SETTING_CATEGORY_COUNT] = {10, 13, 9, 4, 1};
uint8_t getCurrentSettingId() {
  if (settingCategoryIndex >= SETTING_CATEGORY_COUNT) return SET_STROBE_SPEED;
  if (settingSubmenuIndex >= settingCategoryItemCounts[settingCategoryIndex]) return SET_STROBE_SPEED;
  return settingCategoryItems[settingCategoryIndex][settingSubmenuIndex];
}
const char *getSettingLabel(uint8_t id) {
  switch (id) {
    case SET_SHOW_PWM: return "PWM OUTPUT";
    case SET_SHOW_VOLTAGE: return "BATTERY VOLTAGE";
    case SET_SHOW_BATTERY_PERCENT: return "BATTERY PERCENT";
    case SET_SHOW_POWER: return "POWER WATTS";
    case SET_SHOW_TEMPERATURE: return "TEMPERATURE";
    case SET_SHOW_FAN: return "FAN STATUS";
    case SET_DISPLAY_MODE: return "DISPLAY MODE";
    case SET_DISPLAY_TIMEOUT: return "DISPLAY TIMEOUT";
    case SET_SCREEN_SAVER: return "SCREEN SAVER STYLE";
    case SET_INVERT_LED_PWM: return "INVERT LED PWM";
    case SET_INVERT_FAN_PWM: return "INVERT FAN PWM";
    case SET_OLED_BRIGHTNESS: return "BRIGHTNESS";
    case SET_PWM_FREQUENCY: return "PWM FREQUENCY";
    case SET_LED_WATT: return "LED POWER CAPACITY";
    case SET_LOW_OUTPUT: return "LOW MODE LIMIT";
    case SET_MED_OUTPUT: return "MEDIUM MODE LIMIT";
    case SET_HIGH_OUTPUT: return "HIGH MODE LIMIT";
    case SET_TURBO_OUTPUT: return "TURBO POWER LIMIT";
    case SET_TURBO_TIMEOUT: return "TURBO TIME LIMIT";
    case SET_FADE_IN_ENABLE: return "FADE IN";
    case SET_FADE_IN_TIME: return "FADE IN DURATION";
    case SET_FADE_OUT_ENABLE: return "FADE OUT";
    case SET_FADE_OUT_TIME: return "FADE OUT DURATION";
    case SET_STROBE_SPEED: return "STROBE FREQUENCY";
    case SET_FAN_START_TEMP: return "FAN START TEMP";
    case SET_FAN_STOP_TEMP: return "FAN STOP TEMP";
    case SET_FAN_MIN_SPEED: return "FAN MIN SPEED";
    case SET_FAN_MAX_SPEED: return "FAN MAX SPEED";
    case SET_FAN_TEMP_LIMIT: return "FAN TEMP LIMIT";
    case SET_LED_TEMP_LIMIT: return "LED TEMP LIMIT";
    case SET_TURBO_FAN_MODE: return "TURBO FAN MODE";
    case SET_TURBO_FAN_START: return "TURBO FAN START";
    case SET_BATTERY_CAL: return "BATTERY CALIBRATION";
    case SET_BATTERY_CHEMISTRY: return "BATTERY TYPE";
    case SET_BATTERY_SERIES: return "CELLS IN SERIES";
    case SET_LOW_BATT_LIMIT: return "BATTERY POWER LIMIT";
    case SET_RESET_DEFAULTS: return "RESET ALL SETTINGS";
    default: return "SETTING";
  }
}

bool isCheckboxSetting(uint8_t id) {
  switch (id) {
    case SET_SHOW_PWM:
    case SET_SHOW_VOLTAGE:
    case SET_SHOW_BATTERY_PERCENT:
    case SET_SHOW_POWER:
    case SET_SHOW_TEMPERATURE:
    case SET_SHOW_FAN:
    case SET_INVERT_LED_PWM:
    case SET_INVERT_FAN_PWM:
    case SET_FADE_IN_ENABLE:
    case SET_FADE_OUT_ENABLE:
      return true;
    default:
      return false;
  }
}

bool isCheckboxSettingEnabled(uint8_t id) {
  switch (id) {
    case SET_SHOW_PWM: return showPWM;
    case SET_SHOW_VOLTAGE: return showVoltage;
    case SET_SHOW_BATTERY_PERCENT: return showBatteryPercent;
    case SET_SHOW_POWER: return showPowerWatt;
    case SET_SHOW_TEMPERATURE: return showTemperature;
    case SET_SHOW_FAN: return showFanStatus;
    case SET_INVERT_LED_PWM: return invertLedPWM;
    case SET_INVERT_FAN_PWM: return invertFanPWM;
    case SET_FADE_IN_ENABLE: return fadeEnabled;
    case SET_FADE_OUT_ENABLE: return fadeOutEnabled;
    default: return false;
  }
}

void formatSettingOption(uint8_t id, int8_t offset, char *value, size_t valueSize) {
  switch (id) {
    case SET_DISPLAY_MODE: {
        uint8_t option = (displayMode + 3 + offset) % 3;
        snprintf(value, valueSize, "%s", option == 0 ? "ALWAYS" : option == 1 ? "AUTO LED" : "TIMEOUT");
        break;
      }
    case SET_DISPLAY_TIMEOUT:
      if (settingEditField == 0) snprintf(value, valueSize, "%u %s", (unsigned)constrain((int)displayTimeoutValue + offset, 1, 60), displayTimeoutInMinutes ? "MIN" : "SEC");
      else snprintf(value, valueSize, "%s", ((offset + (displayTimeoutInMinutes ? 1 : 0) + 2) % 2) ? "MINUTES" : "SECONDS");
      break;
    case SET_SCREEN_SAVER: {
        uint8_t option = (screenSaverStyle + 6 + offset) % 6;
        const char *labels[] = {"OFF", "ROBOT", "SCAN", "SLEEP", "EYES", "ROLLING"};
        snprintf(value, valueSize, "%s", labels[option]);
        break;
      }
    case SET_INVERT_LED_PWM: snprintf(value, valueSize, "%s", (invertLedPWM != ((offset % 2) != 0)) ? "ON" : "OFF"); break;
    case SET_INVERT_FAN_PWM: snprintf(value, valueSize, "%s", (invertFanPWM != ((offset % 2) != 0)) ? "ON" : "OFF"); break;
    case SET_OLED_BRIGHTNESS: snprintf(value, valueSize, "%u%%", (unsigned)constrain((int)oledBrightness + offset * 5, 1, 100)); break;
    case SET_PWM_FREQUENCY: snprintf(value, valueSize, "%u KHZ", (unsigned)constrain((int)pwmFrequencyKHz + offset, PWM_FREQUENCY_MIN_KHZ, PWM_FREQUENCY_MAX_KHZ)); break;
    case SET_LED_WATT: snprintf(value, valueSize, "%.0f W", constrain(ledWattCapacity + offset, 1.0, 100.0)); break;
    case SET_LOW_OUTPUT: snprintf(value, valueSize, "%.0f%%", constrain(lowOutputLimit + offset, 1.0, 30.0)); break;
    case SET_MED_OUTPUT: snprintf(value, valueSize, "%.0f%%", constrain(mediumOutputLimit + offset, 1.0, 50.0)); break;
    case SET_HIGH_OUTPUT: snprintf(value, valueSize, "%.0f%%", constrain(highOutputLimit + offset, 1.0, 75.0)); break;
    case SET_TURBO_OUTPUT: snprintf(value, valueSize, "%.0f%%", constrain(turboOutputLimit + offset, 1.0, 100.0)); break;
    case SET_TURBO_TIMEOUT: {
        int option = constrain((int)turboTimeoutSec + offset * 5, 0, 180);
        if (option == 0) snprintf(value, valueSize, "OFF");
        else snprintf(value, valueSize, "%d SEC", option);
        break;
      }
    case SET_FADE_IN_ENABLE: snprintf(value, valueSize, "%s", (fadeEnabled != ((offset % 2) != 0)) ? "ON" : "OFF"); break;
    case SET_FADE_IN_TIME: snprintf(value, valueSize, "%u SEC", (unsigned)constrain((int)fadeDurationSec + offset, 1, 10)); break;
    case SET_FADE_OUT_ENABLE: snprintf(value, valueSize, "%s", (fadeOutEnabled != ((offset % 2) != 0)) ? "ON" : "OFF"); break;
    case SET_FADE_OUT_TIME: snprintf(value, valueSize, "%u SEC", (unsigned)constrain((int)fadeOutDurationSec + offset, 1, 10)); break;
    case SET_STROBE_SPEED: snprintf(value, valueSize, "%u HZ", (unsigned)constrain((int)strobeSpeedHz + offset, STROBE_SPEED_MIN, STROBE_SPEED_MAX)); break;
    case SET_FAN_TEMP_LIMIT: snprintf(value, valueSize, "%.0f C", constrain(fanTempLimit + offset, 45.0, 80.0)); break;
    case SET_FAN_START_TEMP: snprintf(value, valueSize, "%.0f C", constrain(fanStartTemp + offset, max(35.0, fanStopTemp + 1.0), 75.0)); break;
    case SET_FAN_STOP_TEMP: snprintf(value, valueSize, "%.0f C", constrain(fanStopTemp + offset, 25.0, min(70.0, fanStartTemp - 1.0))); break;
    case SET_FAN_MIN_SPEED: snprintf(value, valueSize, "%u%%", (unsigned)constrain((int)fanMinSpeed + offset * 5, 0, (int)fanMaxSpeed)); break;
    case SET_FAN_MAX_SPEED: snprintf(value, valueSize, "%u%%", (unsigned)constrain((int)fanMaxSpeed + offset * 5, (int)max((uint8_t)fanMinSpeed, (uint8_t)20), 100)); break;
    case SET_LED_TEMP_LIMIT: snprintf(value, valueSize, "%.0f C", constrain(ledTempLimit + offset, 65.0, 82.0)); break;
    case SET_TURBO_FAN_MODE: snprintf(value, valueSize, "%s", (turboFanBoostEnabled != ((offset % 2) != 0)) ? "BOOST" : "AUTO"); break;
    case SET_TURBO_FAN_START: snprintf(value, valueSize, "%u%%", (unsigned)constrain((int)turboFanStartSpeed + offset * 5, 0, 100)); break;
    case SET_BATTERY_CAL: snprintf(value, valueSize, "%+.2f V", round(constrain(batteryCalOffset + 0.01 * offset, -8.0, 2.0) * 100.0) / 100.0); break;
    case SET_BATTERY_CHEMISTRY: {
        uint8_t option = (batteryChemistry + 3 + offset) % 3;
        const char *labels[] = {"LIFEPO4", "POLYMER", "LITHIUM"};
        snprintf(value, valueSize, "%s", labels[option]);
        break;
      }
    case SET_BATTERY_SERIES:
      snprintf(value, valueSize, "%uS", (unsigned)constrain((int)batterySeriesCount + offset, 1, 4));
      break;
    case SET_LOW_BATT_LIMIT: snprintf(value, valueSize, "%u%%", (unsigned)constrain((int)lowBatteryOutputLimit + offset, 1, 50)); break;
    default: snprintf(value, valueSize, "SELECT"); break;
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
    case SET_INVERT_LED_PWM: invertLedPWM = !invertLedPWM; break;
    case SET_INVERT_FAN_PWM: invertFanPWM = !invertFanPWM; break;
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
    case SET_SCREEN_SAVER:
      screenSaverStyle = (screenSaverStyle + (direction > 0 ? 1 : 5)) % 6;
      if (screenSaverStyle == 5) rollingScreenSaverStyle = 0;
      break;
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
    case SET_BATTERY_CAL: {
        float previousOffset = batteryCalOffset;
        batteryCalOffset = round(constrain(batteryCalOffset + 0.01 * direction, -8.00, 2.00) * 100.0) / 100.0;
        if (batteryCalOffset != previousOffset) {
          batteryCalibrationDirty = true;
          if (filteredBatteryVoltage >= 0.0f) {
            batteryVoltage = max(0.0f, filteredBatteryVoltage + batteryCalOffset);
            batteryDisplayVoltage = batteryVoltage;
          }
        }
        break;
      }
    case SET_BATTERY_CHEMISTRY:
      batteryChemistry = (BatteryChemistry)((batteryChemistry + (direction > 0 ? 1 : 2)) % 3);
      break;
    case SET_BATTERY_SERIES:
      batterySeriesCount = (uint8_t)constrain((int)batterySeriesCount + direction, 1, 4);
      break;
    case SET_LOW_BATT_LIMIT: lowBatteryOutputLimit = (uint8_t)constrain((int)lowBatteryOutputLimit + direction, 1, 50); break;
    default: return;
  }
  if (id != SET_PWM_FREQUENCY && id != SET_BATTERY_CAL) saveSettingConfig();
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
    if (batteryCalibrationDirty) saveSettingConfig();
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
      if (getCurrentSettingId() == SET_BATTERY_CAL && batteryCalibrationDirty) {
        saveSettingConfig();
      }
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
        if (getCurrentSettingId() == SET_BATTERY_CAL && batteryCalibrationDirty) {
          saveSettingConfig();
        }
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
      } else if (isCheckboxSetting(id)) {
        adjustSettingValue(+1);
      } else {
        settingEditing = true;
        settingEditField = 0;
        settingValueScrollPosition = 0.0f;
        settingValueScrollUpdatedAt = now;
      }
    }
  }
  if (mainReleased) {
    settingMainLongDone = false;
    settingMainBackDone = false;
  }

  bool plusStep = plusPressed;
  bool minusStep = minusPressed;
  if (plusPressed) {
    settingPlusRepeatAt = now + SETTING_VALUE_REPEAT_DELAY;
  } else if (settingPlusStable == LOW && (int32_t)(now - settingPlusRepeatAt) >= 0) {
    plusStep = true;
    settingPlusRepeatAt = now + SETTING_VALUE_REPEAT_INTERVAL;
  }
  if (minusPressed) {
    settingMinusRepeatAt = now + SETTING_VALUE_REPEAT_DELAY;
  } else if (settingMinusStable == LOW && (int32_t)(now - settingMinusRepeatAt) >= 0) {
    minusStep = true;
    settingMinusRepeatAt = now + SETTING_VALUE_REPEAT_INTERVAL;
  }

  if (settingResetConfirm) {
    if (plusStep && !minusStep) settingResetChoice = 1;
    if (minusStep && !plusStep) settingResetChoice = 0;
  } else if (settingEditing) {
    if (plusStep && !minusStep) {
      if (getCurrentSettingId() == SET_BATTERY_CAL) {
        adjustSettingValue(+1);
      } else {
        char previousValue[20];
        formatSettingOption(getCurrentSettingId(), 0, previousValue, sizeof(previousValue));
        adjustSettingValue(+1);
        char currentValue[20];
        char nextValue[20];
        formatSettingOption(getCurrentSettingId(), 0, currentValue, sizeof(currentValue));
        formatSettingOption(getCurrentSettingId(), 1, nextValue, sizeof(nextValue));
        settingValueScrollPosition = strcmp(previousValue, currentValue) == 0 ||
                                     strcmp(nextValue, currentValue) == 0
                                       ? 0.0f
                                       : -1.0f;
        settingValueScrollUpdatedAt = now;
      }
    }
    if (minusStep && !plusStep) {
      if (getCurrentSettingId() == SET_BATTERY_CAL) {
        adjustSettingValue(-1);
      } else {
        char previousValue[20];
        formatSettingOption(getCurrentSettingId(), 0, previousValue, sizeof(previousValue));
        adjustSettingValue(-1);
        char currentValue[20];
        char previousOption[20];
        formatSettingOption(getCurrentSettingId(), 0, currentValue, sizeof(currentValue));
        formatSettingOption(getCurrentSettingId(), -1, previousOption, sizeof(previousOption));
        settingValueScrollPosition = strcmp(previousValue, currentValue) == 0 ||
                                     strcmp(previousOption, currentValue) == 0
                                       ? 0.0f
                                       : 1.0f;
        settingValueScrollUpdatedAt = now;
      }
    }
  } else if (settingPage == 0) {
    if (plusStep && !minusStep) settingCategoryIndex = (settingCategoryIndex + 1) % SETTING_CATEGORY_COUNT;
    if (minusStep && !plusStep) settingCategoryIndex = (settingCategoryIndex + SETTING_CATEGORY_COUNT - 1) % SETTING_CATEGORY_COUNT;
  } else {
    if (plusStep && minusStep) {
      settingPage = 0;
    } else {
      uint8_t count = settingCategoryItemCounts[settingCategoryIndex];
      if (plusStep) settingSubmenuIndex = (settingSubmenuIndex + 1) % count;
      if (minusStep) settingSubmenuIndex = (settingSubmenuIndex + count - 1) % count;
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
  uint8_t invertLedPWMValue = invertLedPWM ? 1 : 0;
  uint8_t invertFanPWMValue = invertFanPWM ? 1 : 0;
  uint8_t timeoutUnitValue = displayTimeoutInMinutes ? 1 : 0;
  uint8_t batteryChemistryValue = (uint8_t)batteryChemistry;
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
  EEPROM.put(EEPROM_SCREEN_SAVER_ADDR, screenSaverStyle);
  EEPROM.put(EEPROM_FAN_START_TEMP_ADDR, fanStartTemp);
  EEPROM.put(EEPROM_FAN_STOP_TEMP_ADDR, fanStopTemp);
  EEPROM.put(EEPROM_FAN_MAX_SPEED_ADDR, fanMaxSpeed);
  EEPROM.put(EEPROM_LED_TEMP_LIMIT_ADDR, ledTempLimit);
  EEPROM.put(EEPROM_LOW_BATT_LIMIT_ADDR, lowBatteryOutputLimit);
  EEPROM.put(EEPROM_BATTERY_CHEMISTRY_ADDR, batteryChemistryValue);
  EEPROM.put(EEPROM_BATTERY_SERIES_ADDR, batterySeriesCount);
  EEPROM.put(EEPROM_PWM_FREQUENCY_ADDR, pwmFrequencyKHz);
  EEPROM.put(EEPROM_TURBO_FAN_MODE_ADDR, (uint8_t)(turboFanBoostEnabled ? 1 : 0));
  EEPROM.put(EEPROM_TURBO_FAN_START_ADDR, turboFanStartSpeed);
  EEPROM.put(EEPROM_INVERT_LED_PWM_ADDR, invertLedPWMValue);
  EEPROM.put(EEPROM_INVERT_FAN_PWM_ADDR, invertFanPWMValue);
  EEPROM.commit();
  batteryCalibrationDirty = false;
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
  invertLedPWM = false;
  invertFanPWM = false;
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
  lowBatteryOutputLimit = 15;
  batteryChemistry = BATTERY_POLYMER;
  batterySeriesCount = 3;
  showPWM = true;
  displayMode = 2;
  screenSaverStyle = 1;
  rollingScreenSaverStyle = 0;
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
  if (mainScreenRequested) {
    oledIdleSince = now;
    if (!oledDisplayActive) {
      oledDisplayActive = true;
      display.ssd1306_command(SSD1306_DISPLAYON);
    }
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
  bool batteryWeak = isBatteryLow();
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


void drawSettingMenuItem(const char *label, int16_t y, bool showCheckbox, bool checked) {
  const uint8_t maxCharacters = showCheckbox ? 8 : 10;
  const uint8_t maxLines = 3;
  char lines[3][11] = {};
  uint8_t lineCount = 1;
  uint8_t lineLength = 0;
  uint8_t labelIndex = 0;
  while (label[labelIndex] != '\0') {
    while (label[labelIndex] == ' ') labelIndex++;
    uint8_t wordStart = labelIndex;
    while (label[labelIndex] != '\0' && label[labelIndex] != ' ') labelIndex++;
    uint8_t wordLength = labelIndex - wordStart;
    if (lineLength > 0 && wordLength + 1 > maxCharacters - lineLength) {
      lineCount++;
      lineLength = 0;
    }
    if (lineLength > 0) lines[lineCount - 1][lineLength++] = ' ';
    while (wordLength > 0 && lineCount <= maxLines) {
      uint8_t remaining = maxCharacters - lineLength;
      uint8_t copyLength = min(wordLength, remaining);
      memcpy(lines[lineCount - 1] + lineLength, label + wordStart, copyLength);
      lineLength += copyLength;
      wordStart += copyLength;
      wordLength -= copyLength;
      if (wordLength > 0) {
        lineCount++;
        lineLength = 0;
      }
    }
    if (lineCount > maxLines) break;
  }

  int16_t textWidth = showCheckbox ? 104 : SCREEN_WIDTH;
  int16_t textY = y + (SCREEN_HEIGHT - 16 - lineCount * 16) / 2;
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  for (uint8_t lineIndex = 0; lineIndex < lineCount; lineIndex++) {
    int16_t x1, y1;
    uint16_t textPixelWidth, textPixelHeight;
    display.getTextBounds(lines[lineIndex], 0, 0, &x1, &y1, &textPixelWidth, &textPixelHeight);
    display.setCursor((textWidth - textPixelWidth) / 2, textY + lineIndex * 16);
    display.print(lines[lineIndex]);
  }
  if (showCheckbox) {
    const int16_t checkboxX = 112;
    const int16_t checkboxY = y + (SCREEN_HEIGHT - 16 - 12) / 2;
    display.drawRect(checkboxX, checkboxY, 12, 12, SSD1306_WHITE);
    if (checked) {
      display.drawLine(checkboxX + 2, checkboxY + 5, checkboxX + 5, checkboxY + 9, SSD1306_WHITE);
      display.drawLine(checkboxX + 5, checkboxY + 9, checkboxX + 10, checkboxY + 2, SSD1306_WHITE);
    }
  }
}

void drawSettingsHeader() {
  uint8_t *buffer = display.getBuffer();
  if (buffer) {
    memset(buffer, 0, SCREEN_WIDTH * 2);
  }

  const char *title = "SETTINGS";
  int16_t x1, y1;
  uint16_t titleWidth, titleHeight;
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.getTextBounds(title, 0, 0, &x1, &y1, &titleWidth, &titleHeight);
  display.setCursor((SCREEN_WIDTH - titleWidth) / 2, (16 - titleHeight) / 2);
  display.print(title);
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

    if (settingResetConfirm) {
      display.setTextSize(2);
      display.setCursor(8, 24);
      display.print(settingResetChoice == 0 ? ">CANCEL" : "CANCEL");
      display.setCursor(8, 44);
      display.print(settingResetChoice == 1 ? ">RESET" : "RESET");
    } else if (settingEditing) {
      uint8_t id = getCurrentSettingId();
      if (id == SET_BATTERY_CAL) {
        char rawVoltage[16];
        char adjustedVoltage[16];
        if (batteryPresent && filteredBatteryVoltage >= 0.0f) {
          snprintf(rawVoltage, sizeof(rawVoltage), "%.2fV", filteredBatteryVoltage);
          snprintf(adjustedVoltage, sizeof(adjustedVoltage), "%.2fV", batteryVoltage);
        } else {
          snprintf(rawVoltage, sizeof(rawVoltage), "--.--V");
          snprintf(adjustedVoltage, sizeof(adjustedVoltage), "--.--V");
        }

        display.setTextSize(1);
        display.setCursor(17, 20);
        display.print("RAW");
        display.setCursor(78, 20);
        display.print("ADJUSTED");

        int16_t x1, y1;
        uint16_t textWidth, textHeight;
        display.getTextBounds(rawVoltage, 0, 0, &x1, &y1, &textWidth, &textHeight);
        display.setCursor((64 - textWidth) / 2, 32);
        display.print(rawVoltage);
        display.getTextBounds(adjustedVoltage, 0, 0, &x1, &y1, &textWidth, &textHeight);
        display.setCursor(64 + (64 - textWidth) / 2, 32);
        display.print(adjustedVoltage);

        display.setCursor(31, 53);
        display.print("+/- ADJUST");
      } else {
        unsigned long now = millis();
        float distance = -settingValueScrollPosition;
        float maxStep = (now - settingValueScrollUpdatedAt) * 0.01f;
        if (maxStep > distance && distance > 0.0f) maxStep = distance;
        if (maxStep > -distance && distance < 0.0f) maxStep = -distance;
        settingValueScrollPosition += distance > 0.0f ? maxStep : distance < 0.0f ? -maxStep : 0.0f;
        settingValueScrollUpdatedAt = now;
        for (int8_t optionOffset = -1; optionOffset <= 1; optionOffset++) {
          int16_t y = (int16_t)(32 + (optionOffset - settingValueScrollPosition) * 16);
          if (y < 16 || y > SCREEN_HEIGHT - 16) continue;
          char value[20];
          char currentValue[20];
          formatSettingOption(id, optionOffset, value, sizeof(value));
          formatSettingOption(id, 0, currentValue, sizeof(currentValue));
          if (optionOffset != 0 && strcmp(value, currentValue) == 0) continue;
          bool duplicate = false;
          if (optionOffset != 0) {
            for (int8_t previousOffset = -1; previousOffset < optionOffset; previousOffset++) {
              int16_t previousY = (int16_t)(32 + (previousOffset - settingValueScrollPosition) * 16);
              if (previousY < 16 || previousY > SCREEN_HEIGHT - 16) continue;
              char previousValue[20];
              formatSettingOption(id, previousOffset, previousValue, sizeof(previousValue));
              if (strcmp(value, previousValue) == 0) {
                duplicate = true;
                break;
              }
            }
          }
          if (duplicate) continue;
          char displayValue[22];
          snprintf(displayValue, sizeof(displayValue), optionOffset == 0 ? ">%s" : "%s", value);
          int16_t x1, y1;
          uint16_t valueWidth, valueHeight;
          display.setTextSize(2);
          display.getTextBounds(displayValue, 0, 0, &x1, &y1, &valueWidth, &valueHeight);
          display.setCursor((SCREEN_WIDTH - valueWidth) / 2, y);
          display.print(displayValue);
        }
      }
    } else {
      uint8_t count = settingPage == 0
                        ? SETTING_CATEGORY_COUNT
                        : settingCategoryItemCounts[settingCategoryIndex];
      uint8_t selectedIndex = settingPage == 0
                                ? settingCategoryIndex
                                : settingSubmenuIndex;
      unsigned long now = millis();

      if (settingScrollPage != settingPage ||
          (settingPage == 1 && settingScrollCategory != settingCategoryIndex)) {
        settingScrollPosition = selectedIndex;
        settingScrollPage = settingPage;
        settingScrollCategory = settingCategoryIndex;
      } else {
        unsigned long elapsed = now - settingScrollUpdatedAt;
        float maxStep = elapsed * 0.006f;
        float distance = (float)selectedIndex - settingScrollPosition;
        if (maxStep > distance && distance > 0.0f) maxStep = distance;
        if (maxStep > -distance && distance < 0.0f) maxStep = -distance;
        settingScrollPosition += distance > 0.0f ? maxStep : distance < 0.0f ? -maxStep : 0.0f;
      }
      settingScrollUpdatedAt = now;

      int16_t firstIndex = (int16_t)settingScrollPosition;
      for (int8_t offset = -1; offset <= 1; offset++) {
        int16_t index = firstIndex + offset;
        if (index < 0 || index >= count) continue;
        int16_t y = (int16_t)(16 + (index - settingScrollPosition) * (SCREEN_HEIGHT - 16));
        if (y <= 16 - (SCREEN_HEIGHT - 16) || y >= SCREEN_HEIGHT) continue;
        if (settingPage == 0) {
          drawSettingMenuItem(settingCategoryLabels[index], y, false, false);
        } else {
          uint8_t id = settingCategoryItems[settingCategoryIndex][index];
          drawSettingMenuItem(
            getSettingLabel(id),
            y,
            isCheckboxSetting(id),
            isCheckboxSettingEnabled(id)
          );
        }
      }
    }

    drawSettingsHeader();
    display.display();
    return;
  }

  if (lowBatteryAlertActive) {
    if (millis() - lowBatteryAlertStartedAt >= LOW_BATTERY_ALERT_DURATION) {
      lowBatteryAlertActive = false;
    } else {
      if (!oledDisplayActive) {
        oledDisplayActive = true;
        display.ssd1306_command(SSD1306_DISPLAYON);
      }
      drawLowBatteryAlert();
      display.display();
      return;
    }
  }

  updateOLEDPowerState();


  if (!oledDisplayActive) {
    return;
  }

  if (!ledOn && screenSaverStyle != 0) {
    if (!mainScreenRequested) {
      drawScreenSaver();
      display.display();
      return;
    }
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
    batteryVoltage <= getBatteryWarningVoltage();


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
    batteryVoltage <= getBatteryWarningVoltage();


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

  static const uint8_t percentageSteps[] = {100, 90, 80, 70, 60, 50, 40, 30, 20, 10, 5, 0};
  static const float lithiumCellSteps[] = {4.20f, 4.10f, 4.00f, 3.90f, 3.80f, 3.70f, 3.60f, 3.50f, 3.40f, 3.30f, 3.20f, 3.00f};
  static const float lifepo4CellSteps[] = {3.65f, 3.45f, 3.40f, 3.35f, 3.32f, 3.30f, 3.28f, 3.25f, 3.20f, 3.10f, 3.00f, 2.80f};
  const float *cellSteps = batteryChemistry == BATTERY_LIFEPO4
                             ? lifepo4CellSteps
                             : lithiumCellSteps;
  float cellVoltage = batteryVoltage / batterySeriesCount;

  if (cellVoltage >= getBatteryCellFullVoltage()) {
    batteryPercent = 100;
  } else if (cellVoltage <= cellSteps[11]) {
    batteryPercent = 0;
  } else {
    for (uint8_t i = 0; i < 11; i++) {
      if (cellVoltage >= cellSteps[i + 1]) {
        float fraction = (cellVoltage - cellSteps[i + 1]) /
                         (cellSteps[i] - cellSteps[i + 1]);
        batteryPercent = (int)round(
                           percentageSteps[i + 1] +
                           fraction * (percentageSteps[i] - percentageSteps[i + 1])
                         );
        break;
      }
    }
  }

  batteryPercent = constrain(batteryPercent, 0, 100);
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
  if (freshBatteryRequired) {
    freshBatteryRequired = false;
    nanoWakePending = false;
  }
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
  if (nanoPaused) return;

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

void sendNanoControl(char command) {
  uint8_t checksum = serialChecksum(&command, 1);
  Serial.print(command);
  Serial.print(',');
  Serial.println(checksum);
}

void resetBatterySample() {
  batteryPacketReceived = false;
  batteryPresent = false;
  batteryVoltage = 0.0;
  filteredBatteryVoltage = -1.0;
  batteryDisplayVoltage = -1.0;
  batteryPercent = 0;
  receivedBatteryMillivolts = 0;
  lastBatteryRead = millis() - BATTERY_READ_INTERVAL;
}

void resumeNanoForUser() {
  if (!nanoPaused) return;

  while (Serial.available() > 0) {
    Serial.read();
    nanoRxByteCount++;
  }
  batterySerialLength = 0;
  batterySerialOverflow = false;
  batteryRequestPending = false;
  batteryRequestSent = false;
  resetBatterySample();
  nanoConnected = false;
  nanoPaused = false;
  nanoWakePending = true;
  freshBatteryRequired = true;
  nanoWakeStartedAt = millis();
  oledIdleSince = nanoWakeStartedAt;
  oledDisplayActive = true;
  if (oledOK) display.ssd1306_command(SSD1306_DISPLAYON);
  sendNanoControl('W');
}

void updateNanoPauseState() {
  if (nanoPaused ||
      nanoWakePending ||
      ledOn ||
      ledPWM > 0.0f ||
      fanPWM != 0 ||
      settingMode ||
      lowBatteryAlertActive ||
      oledDisplayActive ||
      mainStableState != HIGH ||
      mainClickCount != 0 ||
      modeMemoryDirty ||
      strobeSpeedDirty) {
    return;
  }

  sendNanoControl('P');
  nanoPaused = true;
  nanoConnected = false;
  batteryRequestPending = false;
  batteryRequestSent = false;
}

void updateNanoConnection() {
  nanoConnected = batteryPacketReceived &&
                  millis() - lastBatteryPacketAt <= NANO_CONNECTION_TIMEOUT;
}

void updateNanoWarningDisplay() {
  if (!oledOK) return;
  if (nanoWakePending) return;
  unsigned long now = millis();
  bool warningStateChanged = !warningDisplayInitialized ||
                             lastNanoWarningState != nanoConnected;
  if (!warningStateChanged &&
      (nanoConnected || !oledDisplayActive || now - lastNanoDiagnosticDisplayAt < 500UL)) return;

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
    writeFanPWMOutput(0);

    return;

  }

  // ==========================================================
  // BATTERY CRITICAL: chemistry and series-dependent cutoff
  // LED DAN FAN DIPAKSA OFF
  // ==========================================================

  if (
    batteryVoltage > 0.0 &&
    batteryVoltage <= getBatteryCriticalVoltage()
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

    writeFanPWMOutput(0);

    return;

  }

  // ==========================================================
  // BATTERY RECOVERY: chemistry and series-dependent threshold
  // ==========================================================

  if (
    batteryCritical &&
    batteryVoltage >= getBatteryRecoveryVoltage()
  ) {

    batteryCritical = false;

  }

  // Jika masih dalam status critical, kipas tetap OFF.
  if (batteryCritical) {

    ledPWM = 0.0;

    outputLEDPWM(0);

    fanPWM = 0;

    lastFanPWMOutput = 0;

    writeFanPWMOutput(0);

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
      settingScrollPage = 255;
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

    if (!ledOn) {
      mainScreenRequested = true;
      oledIdleSince = now;
      oledDisplayActive = true;
      if (oledOK) display.ssd1306_command(SSD1306_DISPLAYON);
    }

    if (nanoPaused) {
      resumeNanoForUser();
    }
    if (!ledOn) {
      tampilkanOLED();
    }

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

    writeFanPWMOutput(0);

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
    writeFanPWMOutput(0);
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
  if (ledOn && screenSaverStyle == 5) {
    rollingScreenSaverStyle = rollingScreenSaverStyle >= 4
                                ? 1
                                : rollingScreenSaverStyle + 1;
  }
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
  uint8_t storedScreenSaver = 0xFF;
  uint8_t storedPWMFrequency = 0xFF;
  uint8_t storedTurboFanMode = 0xFF;
  uint8_t storedTurboFanStart = 30;
  uint8_t storedInvertLedPWM = 0xFF;
  uint8_t storedInvertFanPWM = 0xFF;
  float storedFanStartTemp = 45.0, storedFanStopTemp = 40.0, storedLedTempLimit = 75.0;
  uint8_t storedBatteryChemistry = 0xFF, storedBatterySeries = 0xFF;

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
  EEPROM.get(EEPROM_SCREEN_SAVER_ADDR, storedScreenSaver);
  EEPROM.get(EEPROM_FAN_START_TEMP_ADDR, storedFanStartTemp);
  EEPROM.get(EEPROM_FAN_STOP_TEMP_ADDR, storedFanStopTemp);
  EEPROM.get(EEPROM_FAN_MAX_SPEED_ADDR, storedFanMaxSpeed);
  EEPROM.get(EEPROM_LED_TEMP_LIMIT_ADDR, storedLedTempLimit);
  EEPROM.get(EEPROM_LOW_BATT_LIMIT_ADDR, storedLowBattLimit);
  EEPROM.get(EEPROM_BATTERY_CHEMISTRY_ADDR, storedBatteryChemistry);
  EEPROM.get(EEPROM_BATTERY_SERIES_ADDR, storedBatterySeries);
  EEPROM.get(EEPROM_PWM_FREQUENCY_ADDR, storedPWMFrequency);
  EEPROM.get(EEPROM_TURBO_FAN_MODE_ADDR, storedTurboFanMode);
  EEPROM.get(EEPROM_TURBO_FAN_START_ADDR, storedTurboFanStart);
  EEPROM.get(EEPROM_INVERT_LED_PWM_ADDR, storedInvertLedPWM);
  EEPROM.get(EEPROM_INVERT_FAN_PWM_ADDR, storedInvertFanPWM);

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
  screenSaverStyle = storedScreenSaver <= 5 ? storedScreenSaver : 1;
  rollingScreenSaverStyle = screenSaverStyle == 5 ? 1 : 0;
  fanStartTemp = (isfinite(storedFanStartTemp) && storedFanStartTemp >= 35 && storedFanStartTemp <= 75) ? storedFanStartTemp : 45.0;
  fanStopTemp = (isfinite(storedFanStopTemp) && storedFanStopTemp >= 25 && storedFanStopTemp < fanStartTemp) ? storedFanStopTemp : 40.0;
  fanMaxSpeed = (storedFanMaxSpeed >= max((uint8_t)20, fanMinSpeed) && storedFanMaxSpeed <= 100) ? storedFanMaxSpeed : 100;
  ledTempLimit = (isfinite(storedLedTempLimit) && storedLedTempLimit >= 65 && storedLedTempLimit <= 82) ? storedLedTempLimit : 75.0;
  lowBatteryOutputLimit = (storedLowBattLimit >= 1 && storedLowBattLimit <= 50) ? storedLowBattLimit : 15;
  batteryChemistry = storedBatteryChemistry <= BATTERY_LITHIUM
                       ? (BatteryChemistry)storedBatteryChemistry
                       : BATTERY_POLYMER;
  batterySeriesCount = storedBatterySeries >= 1 && storedBatterySeries <= 4
                         ? storedBatterySeries
                         : 3;
  pwmFrequencyKHz = (storedPWMFrequency >= PWM_FREQUENCY_MIN_KHZ && storedPWMFrequency <= PWM_FREQUENCY_MAX_KHZ) ? storedPWMFrequency : PWM_FREQUENCY_DEFAULT_KHZ;
  turboFanBoostEnabled = storedTurboFanMode <= 1 ? storedTurboFanMode == 1 : false;
  turboFanStartSpeed = storedTurboFanStart <= 100 ? storedTurboFanStart : 30;
  invertLedPWM = storedInvertLedPWM <= 1 ? storedInvertLedPWM == 1 : false;
  invertFanPWM = storedInvertFanPWM <= 1 ? storedInvertFanPWM == 1 : false;
  applyOLEDSettings();
}


void outputLEDPWM(float percent) {
  if (!nanoConnected) percent = 0.0f;

  percent = constrain(
              percent,
              0.0f,
              100.0f
            );

  int pwm = (int)round(
              (percent / 100.0f) * PWM_MAX
            );

  if (invertLedPWM) pwm = PWM_MAX - pwm;

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


  if (
    percent !=
    lastFanPWMOutput
  ) {

    writeFanPWMOutput(percent);

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

void drawScreenSaver() {
  unsigned long now = millis();
  uint8_t gazePhase = (now / 900UL) % 4;
  int8_t gazeOffset = gazePhase == 1 ? -5 : gazePhase == 3 ? 5 : 0;
  bool blink = now % 4200UL < 180UL;
  uint8_t animationStyle = screenSaverStyle == 5
                             ? rollingScreenSaverStyle
                             : screenSaverStyle;

  display.clearDisplay();

  if (animationStyle == 1) {
    const int16_t eyeY = 20;
    if (blink) {
      display.drawLine(22, eyeY + 12, 55, eyeY + 12, SSD1306_WHITE);
      display.drawLine(73, eyeY + 12, 106, eyeY + 12, SSD1306_WHITE);
    } else {
      display.fillRoundRect(22, eyeY, 34, 25, 10, SSD1306_WHITE);
      display.fillRoundRect(73, eyeY, 34, 25, 10, SSD1306_WHITE);
      display.fillCircle(39 + gazeOffset, eyeY + 12, 5, SSD1306_BLACK);
      display.fillCircle(90 + gazeOffset, eyeY + 12, 5, SSD1306_BLACK);
      display.drawPixel(37 + gazeOffset, eyeY + 10, SSD1306_WHITE);
      display.drawPixel(88 + gazeOffset, eyeY + 10, SSD1306_WHITE);
    }
  } else if (animationStyle == 2) {
    uint16_t scanPosition = (now / 12UL) % 80;
    int8_t scanOffset = scanPosition < 40
                          ? (int8_t)(scanPosition / 4) - 5
                          : 5 - (int8_t)((scanPosition - 40) / 4);
    display.fillCircle(39, 32, 15, SSD1306_WHITE);
    display.fillCircle(89, 32, 15, SSD1306_WHITE);
    display.drawLine(24, 32, 104, 32, SSD1306_BLACK);
    display.fillCircle(39 + scanOffset, 32, 6, SSD1306_BLACK);
    display.fillCircle(89 + scanOffset, 32, 6, SSD1306_BLACK);
    display.drawPixel(37 + scanOffset, 30, SSD1306_WHITE);
    display.drawPixel(87 + scanOffset, 30, SSD1306_WHITE);
  } else if (animationStyle == 3) {
    bool asleep = now % 6000UL < 900UL;
    if (asleep) {
      display.drawLine(25, 34, 54, 34, SSD1306_WHITE);
      display.drawLine(76, 34, 105, 34, SSD1306_WHITE);
      display.drawLine(27, 35, 52, 35, SSD1306_WHITE);
      display.drawLine(78, 35, 103, 35, SSD1306_WHITE);
    } else {
      display.fillRoundRect(24, 27, 32, 15, 7, SSD1306_WHITE);
      display.fillRoundRect(74, 27, 32, 15, 7, SSD1306_WHITE);
      display.fillRect(24, 27, 32, 7, SSD1306_BLACK);
      display.fillRect(74, 27, 32, 7, SSD1306_BLACK);
      display.fillCircle(39 + gazeOffset, 37, 4, SSD1306_BLACK);
      display.fillCircle(89 + gazeOffset, 37, 4, SSD1306_BLACK);
    }
  } else if (animationStyle == 4) {
    uint8_t lookPhase = (now / 950UL) % 8;
    int8_t lookX = 0;
    int8_t lookY = 0;
    switch (lookPhase) {
      case 1: lookX = -6; break;
      case 3: lookX = 6; break;
      case 5: lookY = -3; break;
      case 7: lookY = 3; break;
      default: break;
    }

    uint16_t blinkPhase = now % 5100UL;
    bool eyesClosed = blinkPhase < 150UL ||
                      (blinkPhase >= 260UL && blinkPhase < 360UL);
    const int16_t leftX = 16;
    const int16_t rightX = 72;
    const int16_t eyeY = 19;

    display.drawLine(leftX + 3, 14, leftX + 13, 11, SSD1306_WHITE);
    display.drawLine(leftX + 13, 11, leftX + 27, 12, SSD1306_WHITE);
    display.drawLine(rightX + 3, 12, rightX + 17, 11, SSD1306_WHITE);
    display.drawLine(rightX + 17, 11, rightX + 29, 14, SSD1306_WHITE);

    if (eyesClosed) {
      display.drawLine(leftX + 2, eyeY + 14, leftX + 10, eyeY + 11, SSD1306_WHITE);
      display.drawLine(leftX + 10, eyeY + 11, leftX + 20, eyeY + 11, SSD1306_WHITE);
      display.drawLine(leftX + 20, eyeY + 11, leftX + 28, eyeY + 14, SSD1306_WHITE);
      display.drawLine(rightX + 2, eyeY + 14, rightX + 10, eyeY + 11, SSD1306_WHITE);
      display.drawLine(rightX + 10, eyeY + 11, rightX + 20, eyeY + 11, SSD1306_WHITE);
      display.drawLine(rightX + 20, eyeY + 11, rightX + 28, eyeY + 14, SSD1306_WHITE);
    } else {
      display.fillRoundRect(leftX, eyeY, 32, 29, 14, SSD1306_WHITE);
      display.fillRoundRect(rightX, eyeY, 32, 29, 14, SSD1306_WHITE);
      display.drawRoundRect(leftX, eyeY, 32, 29, 14, SSD1306_WHITE);
      display.drawRoundRect(rightX, eyeY, 32, 29, 14, SSD1306_WHITE);

      int16_t leftPupilX = leftX + 16 + lookX;
      int16_t rightPupilX = rightX + 16 + lookX;
      int16_t pupilY = eyeY + 14 + lookY;
      display.fillCircle(leftPupilX, pupilY, 7, SSD1306_BLACK);
      display.fillCircle(rightPupilX, pupilY, 7, SSD1306_BLACK);
      display.drawCircle(leftPupilX, pupilY, 8, SSD1306_BLACK);
      display.drawCircle(rightPupilX, pupilY, 8, SSD1306_BLACK);
      display.fillCircle(leftPupilX, pupilY, 3, SSD1306_WHITE);
      display.fillCircle(rightPupilX, pupilY, 3, SSD1306_WHITE);
      display.drawPixel(leftPupilX - 2, pupilY - 3, SSD1306_WHITE);
      display.drawPixel(rightPupilX - 2, pupilY - 3, SSD1306_WHITE);
    }
  }
}

void drawLowBatteryAlert() {
  display.clearDisplay();

  if (batteryBlinkState) {
    drawBatteryIcon(54, 8, batteryPresent ? batteryPercent : 0);
  }

  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  int16_t x1, y1;
  uint16_t textWidth, textHeight;
  display.getTextBounds("LOW BATT", 0, 0, &x1, &y1, &textWidth, &textHeight);
  display.setCursor((SCREEN_WIDTH - textWidth) / 2, 29);
  display.print("LOW BATT");

  display.setTextSize(1);
  display.setCursor(52, 54);
  if (batteryPresent) {
    display.print(batteryVoltage, 1);
  } else {
    display.print("0.0");
  }
  display.print("V");
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
  if (!nanoPaused) {
    bacaDataBatteryNano();
    updateNanoConnection();
    requestNanoBattery();
    bacaBattery();
    checkBatteryProtection();
    updateNanoWarningDisplay();
    updateBatteryWarningBlink();
    updateWarningBlink();

    if (nanoWakePending && now - nanoWakeStartedAt >= NANO_WAKE_TIMEOUT) {
      nanoWakePending = false;
      freshBatteryRequired = false;
      pendingMainClicks = 0;
      mainScreenRequested = false;
    }

    if (!freshBatteryRequired && pendingMainClicks > 0) {
      mainClickCount = pendingMainClicks;
      pendingMainClicks = 0;
      executeMainClicks();
    }
  }
  // Jika menu sedang aktif, jangan jalankan handler tombol normal.
  // Handler normal dapat menafsirkan MAIN sebagai klik mode.
  if (settingMode) {
    handleSettingMenu();
    if (now - lastDisplay >= SETTING_DISPLAY_INTERVAL) {
      lastDisplay = now;
      tampilkanOLED();
    }
    return;
  }
  if (!nanoConnected && !nanoPaused && !nanoWakePending) {
    forceNanoDisconnectedOutputsOff();
    if (now - lastDisplay >= DISPLAY_INTERVAL) {
      lastDisplay = now;
      tampilkanOLED();
    }
    updateNanoPauseState();
    return;
  }
  handleLockButtons();
  handleMainButton();
  updateTurboTimeout();
  if (settingMode) {
    handleSettingMenu();
    if (now - lastDisplay >= SETTING_DISPLAY_INTERVAL) {
      lastDisplay = now;
      tampilkanOLED();
    }
    return;
  }
  if (!nanoConnected) {
    forceNanoDisconnectedOutputsOff();
    if (now - lastDisplay >= DISPLAY_INTERVAL) {
      lastDisplay = now;
      tampilkanOLED();
    }
    updateNanoPauseState();
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
  updateNanoPauseState();
}

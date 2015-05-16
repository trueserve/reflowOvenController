// ----------------------------------------------------------------------------
// Reflow Oven Controller
// (c) 2014 Karl Pitrich <karl@pitrich.com>
// (c) 2014 true <trueamx@gmail.com>
// (c) 2012-2013 Ed Simmons
// ----------------------------------------------------------------------------

// --CONFIGURATION-------------------------------------------------------------
#define BOARD_TYPE          1   // 0 = kp's design, 1 = true's design
#define LINE_FREQ           60  // 50 or 60 supported

#define HEATER_MODE         1   // 0 = wave packet, 1 = slower on/off
#define FAN_MODE            1   // 0 = phase control, 1 = on/off control (fan does not support PC), 2 = wave packet

#define ENC_STEPS_PER_NOTCH 4   // steps per notch of the rotary encoder. to cal: set to 1, turn knob slowly, count

#define LCD_ROTATE          2   // 0 or 2 = vertical, 1 or 3 = horizontal
#define LCD_TABTYPE         INITR_BLACKTAB   // lcd type, usually INITR_RED/GREEN/BLACKTAB

#define NAMED_PROFILES      1   // 1 = allow named profiles, 0 = numbers only

#define IDLE_SAFE_TEMP      50  // temp in degC that the oven is considered safe/done cooling

#define GRAPH_DRAW_LINES    1   // unset = draw with pixels (sometimes has gaps), set = draw with lines (no gaps, nicer). +102bytes
#define GRAPH_HAS_TEMPS     1   // unset = nothing, set = print temperature values on right edge of graph line. +74bytes
#define GRAPH_STOP_ON_DONE  1   // unset = keep looping graph after done, set = stop timer/graphing after idle safe temp reached. +42bytes

#define SHOW_TEMP_MAIN_PAGE 1   // unset = no temp shown. set = temp shown. +138bytes

#define INVERT_HEATER       1   // invert heater pin
#define INVERT_FAN          1   // invert fan pin

//#define FAKE_HW             1
//#define PIDTUNE             1   // autotune wouldn't fit in the 28k available on my arduino pro micro.

// run a calibration loop that measures how many timer ticks occur between 2 zero corssings
// FIXME: does not work reliably at the moment, so a oscilloscope-determined value is used.
//#define WITH_CALIBRATION 1 // loop timing calibration


// --INCL----------------------------------------------------------------------
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include <SPI.h>
#include <WString.h>

#include <Menu.h>

#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

#include <PID_v1.h>
#include <TimerOne.h>


#ifdef FAKE_HW
 #ifdef __AVR_ATmega32U4__
  #include <TimerThree.h>
 #endif
#endif

#ifdef PIDTUNE
 #include <PID_AutoTune_v0.h>
#endif


// --VER-----------------------------------------------------------------------
const char *ver = "3.1_tr02";


// --FREQ----------------------------------------------------------------------
#define TIMER1_ISR_CYCLE      600     // usec per timer1 tick. DEFAULT_LOOP_DELAY depends on this. fyi, TimerOne runs in mode 8.
                                      // at 600, we are divisible by 100 or 120, and get a counter value of 4800, prescale 1.

#define SINES_PER_SEC         (LINE_FREQ * 2)

#define DEFAULT_LOOP_DELAY    (((1000000 / 600) / SINES_PER_SEC) / 2)     // 50Hz = 16.6, 60Hz = 13.3


// --CLICK ENCODER-------------------------------------------------------------
#define ENC_CUSTOM_INTERVAL   1
#define ENC_BUTTONINTERVAL    20       // 12ms
#define ENC_DOUBLECLICKTIME   1000     // 600ms
#define ENC_HOLDTIME          2000     // 1.2s

#define ENC_CUSTOM_ACCEL      1
#define ENC_ACCEL_TOP         5000
#define ENC_ACCEL_INC         40
#define ENC_ACCEL_DEC         3

#include <ClickEncoder.h>


// --HARDWARE CONFIG-----------------------------------------------------------
// 1.8" TFT via SPI -> breadboard
#if   (BOARD_TYPE == 0)  // kp's design
 #define LCD_CS       10
 #define LCD_DC       9
 #define LCD_RST      8
#elif (BOARD_TYPE == 1)  // true's design
 #define LCD_CS       6
 #define LCD_DC       9
 #define LCD_RST      8
#else
 #error "Please set a board type"
#endif

// Maxim TC
#define TCOUPLE1_CS  3
#define TCOUPLE2_CS  4

// SSR
#ifdef __AVR_ATmega32U4__
 #define PIN_HEATER   1
 #define PIN_FAN      0
#else
 #define PIN_HEATER   0  // SSR output for the heater
 #define PIN_FAN      1  // SSR output for the fan
#endif

// ZX Circuit
#define PIN_ZX       2 // pin for zero crossing detector

#if defined(__AVR_ATmega32U4__)
 #define INT_ZX      1 // interrupt for ZX detector (pro micro)
#elif defined(__AVR_ATmega328P__)
 #define INT_ZX      0 // interrupt for ZX detector (pro mini/nano 3.0)
#endif
                       // Leonardo == Pro Micro:
                       //   Pin: 3 2 0 1 7
                       //   Int: 0 1 2 3 4
                      
 
// --SPLASH SCREEN-------------------------------------------------------------
#define WITH_SPLASH 1


// --LOCAL INCL----------------------------------------------------------------
#include "temperature.h"
#include "helpers.h"


// --GLOBALS-------------------------------------------------------------------
volatile uint32_t timerTicks     = 0;
volatile uint32_t zeroCrossTicks = 0;
volatile uint8_t  heaterTicks    = 0;
volatile uint8_t  fanTicks       = 0;
volatile uint8_t  phaseCounter   = 0;

uint32_t lastUpdate              = 0;
uint32_t lastDisplayUpdate       = 0;

#ifdef GRAPH_STOP_ON_DONE
  bool processCompleted = false;
#endif

char buf[20]; // generic char buffer


// --PID-----------------------------------------------------------------------
uint8_t fanValue;
uint8_t heaterValue;

double Setpoint;
double Input;
double Output;

typedef struct {
  double Kp;
  double Ki;
  double Kd;
} PID_t;

PID_t heaterPID;
PID_t fanPID    = { 1.00, 0.03, 10.00 };

PID PID(&Input, &Output, &Setpoint, 4.00, 0.05, 2.00, DIRECT);  // these PID values are ignored

#ifdef PIDTUNE
PID_ATune PIDTune(&Input, &Output);

#define AUTOTUNE_STEP            50
#define AUTOTUNE_NOISE            1
#define AUTOTUNE_START_VALUE     50
#define AUTOTUNE_LOOKBACK        30
#endif


// --PROFILES------------------------------------------------------------------
// data type for the values used in the reflow profile
typedef struct profileValues_s {
  int16_t soakTemp;
  int16_t soakDuration;
  int16_t peakTemp;
  int16_t peakDuration;
  int16_t rampUpRate;
  int16_t rampDownRate;
} Profile_t;

#define MAX_PROFILES               30
Profile_t activeProfile; // the one and only instance
uint8_t activeProfileId = 0;

uint8_t fanAssistSpeed = 33; // default fan speed

// EEPROM offsets
#define E2OFFSET_PID_CONFIG         (void *)    (E2END - 16 - sizeof(PID_t))  // sizeof(PID_t)
#define E2OFFSET_FAN_SPEED          (uint8_t *) (E2END - 16)  // one byte
#define E2OFFSET_PROFILE_NUMBER     (uint8_t *) (E2END - 15)  // one byte
#define E2OFFSET_RUNCOUNT_SUCCESS   (uint16_t *)(E2END - 12)  // 2 bytes
#define E2OFFSET_RUNCOUNT_CANCEL    (uint16_t *)(E2END - 10)  // 2 bytes
#define E2OFFSET_CHECKSUM           (uint8_t *) (E2END)       // 1 byte


// --UI------------------------------------------------------------------------
// NB: Adafruit GFX ASCII-Table is bogous:
//     https://github.com/adafruit/Adafruit-GFX-Library/issues/22

#define ST7735_LTGRAY       0xe71c
#define ST7735_STDGRAY      0x9492

Adafruit_ST7735 tft = Adafruit_ST7735(LCD_CS, LCD_DC, LCD_RST);

#if (LCD_ROTATE % 2 == 1)   // landscape view
 #define TFT_LEFTCOL        10
 #define TFT_WIDTH          160
 #define TFT_HEIGHT         128
#else                       // portrait view
 #define TFT_LEFTCOL        4
 #define TFT_WIDTH          128
 #define TFT_HEIGHT         160
#endif

ClickEncoder Encoder(A1, A0, A2, ENC_STEPS_PER_NOTCH);

Menu::Engine Engine;

int16_t encMovement;
int16_t encAbsolute;
int16_t encLastAbsolute = -1;


#define MENU_ITEMS_VISIBLE   5
#define MENU_ITEM_HEIGHT     12

bool menuUpdateRequest = true;
bool initialProcessDisplay = false;

#ifdef SHOW_TEMP_MAIN_PAGE
  bool menuTempUpdateRequest;
#endif


// --STATE MACHINE-------------------------------------------------------------
typedef enum {
  None     = 0,
  Idle     = 1,
  Settings = 2,
  Edit     = 3,

  UIMenuEnd = 9,

  RampToSoak = 10,
  Soak,
  RampUp,
  Reflow,
  RampDown,
  CoolDown,

  Complete = 20,

  Tune = 30
} State;

State currentState  = Idle;
State previousState = Idle;
bool stateChanged = false;
uint32_t stateChangedTicks = 0;

// track menu item state to improve render preformance
typedef struct {
  const Menu::Item_t *mi;
  uint8_t pos;
  bool current;
} LastItemState_t;

LastItemState_t currentlyRenderedItems[MENU_ITEMS_VISIBLE];

void clearLastMenuItemRenderState()
{
  // memset(&currentlyRenderedItems, 0xff, sizeof(LastItemState_t) * MENU_ITEMS_VISIBLE);
  for (uint8_t i = 0; i < MENU_ITEMS_VISIBLE; i++) {
    currentlyRenderedItems[i].mi = NULL;
    currentlyRenderedItems[i].pos = 0xff;
    currentlyRenderedItems[i].current = false;
  }
  
  // clear the area below the menu
  // TODO: find out why ADDING this line saves 82 bytes of flash (!!)
  tft.fillRect(0, (MENU_ITEMS_VISIBLE * MENU_ITEM_HEIGHT) + 2, TFT_WIDTH, TFT_HEIGHT, ST7735_WHITE);
}


// ----------------------------------------------------------------------------
extern const Menu::Item_t miRampUpRate, miRampDnRate, miSoakTime, 
                          miSoakTemp, miPeakTime, miPeakTemp,
                          miLoadProfile, miSaveProfile,
                          miPidSettingP, miPidSettingI, miPidSettingD,
                          miFanSettings;


// --HELPERS-------------------------------------------------------------------
#define FS(x)      (x)     // define as F(x) to store strings in flash.
                           // at this time this costs 70bytes, and we don't need the SRAM

void printDouble(double val, uint8_t precision = 1)
{
  ftoa(buf, val, precision);
  tft.print(buf);
}

void printAtPos(const __FlashStringHelper *str, uint8_t x, uint8_t y)
{
  tft.setCursor(x, y);
  tft.print(str);
}

void printAtPos(const char *str, uint8_t x, uint8_t y)
{
  tft.setCursor(x, y);
  tft.print(str);
}

void printCentered(const __FlashStringHelper *str, uint8_t y)
{
  tft.setCursor((TFT_WIDTH >> 1) - (strlen((const prog_char *)str) * (tft.getTextSize() * 3)), y);
  tft.print(str);
}

void printCentered(const char *str, uint8_t y)
{
  tft.setCursor((TFT_WIDTH >> 1) - (strlen(str) * (tft.getTextSize() * 3)), y);
  tft.print(str);
}

void alignRightPrefix(uint16_t v)
{
  if (v < 1e2) tft.print(" "); 
  if (v < 1e1) tft.print(" ");
}

bool isPidSetting(const Menu::Item_t *mi)
{
  return mi == &miPidSettingP || mi == &miPidSettingI || mi == &miPidSettingD;
}

bool isRampSetting(const Menu::Item_t *mi)
{
  return mi == &miRampUpRate || mi == &miRampDnRate;
}


// --MENU FN-------------------------------------------------------------------
bool menuExit(const Menu::Action_t a)
{
  clearLastMenuItemRenderState();
  Engine.lastInvokedItem = &Menu::NullItem;
  menuUpdateRequest = false;
  return false;
}

bool menuDummy(const Menu::Action_t a)
{
  return true;
}

void * getItemValuePointer(const Menu::Item_t *mi)
{
  if (mi == &miRampUpRate)  return &activeProfile.rampUpRate;
  if (mi == &miRampDnRate)  return &activeProfile.rampDownRate;
  if (mi == &miSoakTime)    return &activeProfile.soakDuration;
  if (mi == &miSoakTemp)    return &activeProfile.soakTemp;
  if (mi == &miPeakTime)    return &activeProfile.peakDuration;
  if (mi == &miPeakTemp)    return &activeProfile.peakTemp;
  if (mi == &miPidSettingP) return &heaterPID.Kp;
  if (mi == &miPidSettingI) return &heaterPID.Ki;
  if (mi == &miPidSettingD) return &heaterPID.Kd; 
  if (mi == &miFanSettings) return &fanAssistSpeed;
  return NULL;
}

bool getItemValueLabel(const Menu::Item_t *mi, char *label)
{
  char work[2];
  void *val;

  val = getItemValuePointer(mi);
  
  if (isPidSetting(mi)) {
    ftoa(label, *(double *)val, 2); // need greater precision with pid values
  }
  if (isRampSetting(mi)) {
    // set integer
    itoa10((*((int16_t *)val) / 10), label, 1);
    // set decimal
    work[0] = (*((int16_t *)val) % 10) + 0x30;
    work[1] = 0x00;
    strcat(label, ".");
    strcat(label, work);
    // set degC/s
    strcat(label, "\367C/s");
  }
  if (mi == &miPeakTemp || mi == &miSoakTemp) {
    itostr(label, *(int16_t *)val, "\367C");
  }
  if (mi == &miPeakTime || mi == &miSoakTime) {
    itostr(label, *(int16_t *)val, "s");
  }
  if (mi == &miFanSettings) {
    itostr(label, *(uint8_t *)val, "%");
  }

  return val;
}

bool editNumericalValue(const Menu::Action_t action)
{
  if (action == Menu::actionDisplay) {
    bool initial = currentState != Edit;
    currentState = Edit;

    if (initial) {
      tft.setTextColor(ST7735_BLACK, ST7735_WHITE);

      // need spaces here to overwrite potential stale text
      printAtPos(FS("Rotate to edit      "), TFT_LEFTCOL, 80);
      printAtPos(FS("Click to save"), TFT_LEFTCOL, 90);
      
      Encoder.setAccelerationEnabled(true);
    }

    for (uint8_t i = 0; i < MENU_ITEMS_VISIBLE; i++) {
      if (currentlyRenderedItems[i].mi == Engine.currentItem) {
        uint8_t y = currentlyRenderedItems[i].pos * MENU_ITEM_HEIGHT + 2;

        if (initial) {
          tft.fillRect(TFT_LEFTCOL + 59, y - 1, (TFT_WIDTH - (TFT_LEFTCOL + 59) - TFT_LEFTCOL) + 1, MENU_ITEM_HEIGHT - 2, ST7735_RED);
        }

        tft.setCursor(TFT_LEFTCOL + 60, y);
        break;
      }
    }

    tft.setTextColor(ST7735_WHITE, ST7735_RED);

    void *val;
    val = getItemValuePointer(Engine.currentItem);

    // none of the items we can edit happen to go negative.
    encAbsolute = (encAbsolute < 0) ? 0 : encAbsolute;
    
    if (isPidSetting(Engine.currentItem)) {      
      double tmp;
      if (initial) {
        tmp = *(double *)val;
        tmp *= 100;
        encAbsolute = (int16_t)tmp;
      }
      else {
        tmp = encAbsolute;
        tmp /= 100;
        *(double *)val = tmp;
      }      
    }
    else {
      if (initial) encAbsolute = *(int16_t *)val;
      else *(int16_t *)val = encAbsolute;
    }
    
    // TODO: clamp upper values
    /*
    switch (Engine.currentItem) {
      case  
    }
    */

    getItemValueLabel(Engine.currentItem, buf);
    tft.print(buf);
    tft.print("   ");
    tft.setTextColor(ST7735_BLACK, ST7735_WHITE);
  }

  if (action == Menu::actionParent || action == Menu::actionTrigger) {
    clearLastMenuItemRenderState();
    menuUpdateRequest = true;
    Engine.lastInvokedItem = &Menu::NullItem;

    if (currentState == Edit) { // leave edit mode, return to menu
      if (isPidSetting(Engine.currentItem)) {
        savePID();
      }
      else if (Engine.currentItem == &miFanSettings) {
        saveFanSpeed();
      }
      // don't autosave profile, so that one can do "save as" without overwriting the current profile

      currentState = Settings;
      Encoder.setAccelerationEnabled(false);
      return false;
    }

    return true;
  }
}


// --PROFILE PROTOTYPE---------------------------------------------------------
void saveProfile(uint8_t targetProfile, bool quiet = false);
bool saveLoadProfile(const Menu::Action_t action);
bool factoryReset(const Menu::Action_t action);


// ----------------------------------------------------------------------------
void toggleAutoTune();

bool cycleStart(const Menu::Action_t action)
{
  if (action == Menu::actionDisplay) {
    zeroCrossTicks = 0;
    menuExit(action);

#ifndef PIDTUNE    
    currentState = RampToSoak;
#else
    toggleAutoTune();
#endif
    initialProcessDisplay = false;
    menuUpdateRequest = false;
#ifdef GRAPH_STOP_ON_DONE
    processCompleted = false;
#endif
  }

  return true;
}


// --RENDER--------------------------------------------------------------------
void renderMenuItem(const Menu::Item_t *mi, uint8_t pos)
{
  bool isCurrent = Engine.currentItem == mi;
  uint8_t y = pos * MENU_ITEM_HEIGHT + 2;

  if (currentlyRenderedItems[pos].mi == mi 
      && currentlyRenderedItems[pos].pos == pos 
      && currentlyRenderedItems[pos].current == isCurrent) 
  {
    return; // don't render the same item in the same state twice
  }

  // menu cursor bar
  tft.fillRect(TFT_LEFTCOL - 2, y - 2, TFT_WIDTH - (TFT_LEFTCOL << 1) + 4, MENU_ITEM_HEIGHT, isCurrent ? ST7735_BLUE : ST7735_WHITE);
  if (isCurrent) tft.setTextColor(ST7735_WHITE, ST7735_BLUE);
  else tft.setTextColor(ST7735_BLACK, ST7735_WHITE);

  printAtPos(Engine.getLabel(mi), TFT_LEFTCOL, y);

  // show values if in-place editable items
  if (getItemValueLabel(mi, buf)) {
    tft.print(" "); tft.print(buf); tft.print("   ");
  }

  // mark items that have children
  if (Engine.getChild(mi) != &Menu::NullItem) {
    tft.print(FS(" \x10  ")); // 0x10 -> filled right arrow
  }

  currentlyRenderedItems[pos].mi = mi;
  currentlyRenderedItems[pos].pos = pos;
  currentlyRenderedItems[pos].current = isCurrent;
}

// --MENU ITEMS----------------------------------------------------------------
// Name, Label, Next, Previous, Parent, Child, Callback
MenuItem(miExit, "", Menu::NullItem, Menu::NullItem, Menu::NullItem, miCycleStart, menuExit);
MenuItem(miEditable, "", Menu::NullItem, Menu::NullItem, Menu::NullItem, miCycleStart, menuExit);

#ifndef PIDTUNE
MenuItem(miCycleStart,  "Start Cycle",  miEditProfile, Menu::NullItem, miExit, Menu::NullItem, cycleStart);
#else
MenuItem(miCycleStart,  "Start Autotune",  miEditProfile, Menu::NullItem, miExit, Menu::NullItem, cycleStart);
#endif
MenuItem(miEditProfile, "Edit Profile", miLoadProfile, miCycleStart,   miExit, miRampUpRate, menuDummy);
  MenuItem(miRampUpRate, "Ramp up  ",   miSoakTemp,      Menu::NullItem, miEditProfile, Menu::NullItem, editNumericalValue);
  MenuItem(miSoakTemp,   "Soak temp", miSoakTime,      miRampUpRate,   miEditProfile, Menu::NullItem, editNumericalValue);
  MenuItem(miSoakTime,   "Soak time", miPeakTemp,      miSoakTemp,     miEditProfile, Menu::NullItem, editNumericalValue);
  MenuItem(miPeakTemp,   "Peak temp", miPeakTime,      miSoakTime,     miEditProfile, Menu::NullItem, editNumericalValue);
  MenuItem(miPeakTime,   "Peak time", miRampDnRate,    miPeakTemp,     miEditProfile, Menu::NullItem, editNumericalValue);
  MenuItem(miRampDnRate, "Ramp down", Menu::NullItem,  miPeakTime,     miEditProfile, Menu::NullItem, editNumericalValue);
MenuItem(miLoadProfile,  "Load Profile",  miSaveProfile,  miEditProfile, miEditable, Menu::NullItem, saveLoadProfile);
MenuItem(miSaveProfile,  "Save Profile",  miFanSettings,  miLoadProfile, miEditable, Menu::NullItem, saveLoadProfile);
MenuItem(miFanSettings,  "Fan Speed",  miPidSettings,  miSaveProfile, miExit, Menu::NullItem, editNumericalValue);
MenuItem(miPidSettings,  "PID Settings",  miFactoryReset, miFanSettings, miExit, miPidSettingP,  menuDummy);
  MenuItem(miPidSettingP,  "Heater Kp",  miPidSettingI, Menu::NullItem, miPidSettings, Menu::NullItem, editNumericalValue);
  MenuItem(miPidSettingI,  "Heater Ki",  miPidSettingD, miPidSettingP,  miPidSettings, Menu::NullItem, editNumericalValue);
  MenuItem(miPidSettingD,  "Heater Kd",  Menu::NullItem, miPidSettingI, miPidSettings, Menu::NullItem, editNumericalValue);
MenuItem(miFactoryReset, "Factory Reset", Menu::NullItem, miPidSettings, miEditable, Menu::NullItem, factoryReset);


// --TC------------------------------------------------------------------------
#define TC_COUNT                 2
#define TC_NUMREADINGS           10
#define TC_ERROR_TOLERANCE       5  // allow for n consecutive errors before bailing out

typedef struct {
  double temp;
  uint16_t ticks;
} Temp_t;

typedef struct {
  double readings[TC_NUMREADINGS];
  double total;
  double average;
} TempLog_t;

Temp_t airTemp[TC_NUMREADINGS];

double runningTotalRampRate;
double rampRate = 0;

Thermocouple tc[TC_COUNT];
TempLog_t tcLog[TC_COUNT];
uint8_t tcIndex = 0;                // the index of the current reading

// --SSR-----------------------------------------------------------------------
// Ensure that relay outputs are off (low) when starting
void setupRelayPins(void)
{
#if (INVERT_HEATER != 0)
  digitalWrite(PIN_HEATER, HIGH);
#else
  digitalWrite(PIN_HEATER, LOW);
#endif
  pinMode(PIN_HEATER, OUTPUT);
  
#if (INVERT_FAN != 0)
  digitalWrite(PIN_FAN, HIGH);
#else
  digitalWrite(PIN_FAN, LOW);
#endif
  pinMode(PIN_FAN, OUTPUT);
}

void killRelayPins(void)
{
  Timer1.stop();
#ifndef FAKE_HW
  detachInterrupt(INT_ZX);
#endif
  digitalWrite(PIN_HEATER, LOW);
  digitalWrite(PIN_FAN, LOW);
}


// --WAVE PACKET---------------------------------------------------------------
// only turn the solid state relays on for a percentage 
// of complete sinusoids (i.e. 1x 360°)

#define CHANNELS       2

#define CHANNEL_HEATER 0
#define CHANNEL_FAN    1

typedef struct Channel_s {
  volatile uint8_t target; // percentage of on-time
  uint8_t state;           // current state counter
  int32_t next;            // when the next change in output shall occur  
  bool action;             // hi/lo active
  bool enable;             // channel active
} Channel_t;

Channel_t Channels[] = {
  { 0, 0, 0, false },
  { 0, 0, 0, false }
};

// delay to align relay activation with the actual zero crossing
uint16_t zxLoopDelay;

#ifdef WITH_CALIBRATION
// calibrate zero crossing: how many timerIsr happen within one zero crossing
#define zxCalibrationLoops 128
struct {
  volatile int8_t iterations;
  volatile uint8_t measure[zxCalibrationLoops];
} zxLoopCalibration = {
  0, {}
};
#endif


// --ZX ISR--------------------------------------------------------------------
// per ZX, process one channel per interrupt only
// NB: use native port IO instead of digitalWrite for better performance
// true: using digitalWrite again for better compatibility
void zeroCrossingIsr(void)
{
  static uint8_t ch = 0;

  zeroCrossTicks++;
    
  // reset phase control timer
  phaseCounter = 0;
  TCNT1 = 0;
  
  // update mode 1 tick counters
  if (--fanTicks == 0) {
    fanTicks = 25;
  }  
  if (--heaterTicks == 0) {
    heaterTicks = 50;
  }
  
  // update root menu temperature display
#ifdef SHOW_TEMP_MAIN_PAGE
  if ((uint8_t)zeroCrossTicks == 0) {
    menuTempUpdateRequest = true; 
  }
#endif

  // calculate wave packet parameters
  Channels[ch].state += Channels[ch].target;
  if (Channels[ch].state >= 100) {
    Channels[ch].state -= 100;
    Channels[ch].action = false;
  }
  else {
    Channels[ch].action = true;
  }
  Channels[ch].next = (timerTicks + zxLoopDelay) - 1; // delay added to reach the next zx

  ++ch %= CHANNELS; // next channel

#ifdef WITH_CALIBRATION
  if (zxLoopCalibration.iterations < zxCalibrationLoops) {
    zxLoopCalibration.iterations++;
  }
#endif
}

#ifdef FAKE_HW
 #if defined(__AVR_ATmega328P__)
ISR(TIMER2_COMPA_vect) {    
  zeroCrossingIsr();
  // interrupt flag is cleared automatically by hardware
}
 #endif
#endif

// --TIMER ISR-----------------------------------------------------------------
void timerIsr(void)
{
  static uint32_t lastHtrTicks = 0;
  
#if (FAN_MODE == 0)
  // phase control for the fan 
  phaseCounter += (TIMER1_ISR_CYCLE / SINES_PER_SEC);
  if (phaseCounter > 90) {
    phaseCounter = 0;
  }
  
  bool fanSet = (phaseCounter > Channels[CHANNEL_FAN].target) ? LOW : HIGH;
#elif (FAN_MODE == 1)
  // toggling control for the fan, useful for picky fans and low code space
  bool fanSet = ((fanValue >> 2) >= fanTicks) ? LOW : HIGH;
#elif (FAN_MODE == 2)
  // wave packet control for the fan, implemented because it works on my fan and phase control doesn't
  static uint32_t lastFanTicks = 0;
  static bool fanSet;
  
  if (Channels[CHANNEL_FAN].next > lastFanTicks // FIXME: this loses ticks when overflowing
          && timerTicks > Channels[CHANNEL_FAN].next)
  {
    fanSet = Channels[CHANNEL_FAN].action ? HIGH : LOW;
    lastFanTicks = timerTicks;
  }
#else
  #error ("Please set FAN_MODE to 0, 1 or 2")
#endif

#if (INVERT_FAN != 0)
  digitalWrite(PIN_FAN, !fanSet);
#else
  digitalWrite(PIN_FAN, fanSet);
#endif
  
#if (HEATER_MODE == 0)  
  // wave packet control for heater
  static bool heaterSet;
  if (Channels[CHANNEL_HEATER].next > lastHtrTicks // FIXME: this loses ticks when overflowing
          && timerTicks > Channels[CHANNEL_HEATER].next)
  {
    heaterSet = Channels[CHANNEL_HEATER].action ? HIGH : LOW;
    lastHtrTicks = timerTicks;
  }
#elif (HEATER_MODE == 1)
  bool heaterSet = ((heaterValue >> 1) >= heaterTicks) ? LOW : HIGH;
#else
  #error ("Please set HEATER_MODE to 0 or 1");
#endif
 
#if (INVERT_HEATER != 0)
    digitalWrite(PIN_HEATER, !heaterSet);
#else
    digitalWrite(PIN_HEATER, heaterSet);
#endif

  
  // handle encoder + button
  Encoder.service();

  // update timer1 calls
  timerTicks++;


#ifdef WITH_CALIBRATION
  if (zxLoopCalibration.iterations < zxCalibrationLoops) {
    zxLoopCalibration.measure[zxLoopCalibration.iterations]++;
  }
#endif
}


// ----------------------------------------------------------------------------
void abortWithError(int error)
{
  killRelayPins();

  tft.setTextColor(ST7735_WHITE, ST7735_RED);
  tft.fillScreen(ST7735_RED);
  
  if (error < 9) {
    printAtPos(FS("Thermocouple Error"), TFT_LEFTCOL, 10);
    switch (error) {
      case 0b001:
        printAtPos(FS("Open Circuit"), TFT_LEFTCOL, 30);
        break;
      case 0b010:
        printAtPos(FS("GND Short"), TFT_LEFTCOL, 30);
        break;
      case 0b100:
        printAtPos(FS("VCC Short"), TFT_LEFTCOL, 30);
        break;
    }
    printAtPos(FS("Power Off,"), TFT_LEFTCOL, 60);
    printAtPos(FS("check connections"), TFT_LEFTCOL, 75);
  }
  else {
    printAtPos(FS("Temperature"), TFT_LEFTCOL, 10); 
    printAtPos(FS("following error"), TFT_LEFTCOL, 30);
    printAtPos(FS("during "), TFT_LEFTCOL, 45);
    tft.print((error == 10) ? F("heating") : F("cooling"));
  }

  while (1) { //  stop
    ;
  }
}


// ----------------------------------------------------------------------------
void displayThermocoupleData(struct Thermocouple* input)
{
  switch (input->stat) {
    case 0:
      printDouble(input->temperature);
      tft.print(FS("\367C"));
      break;
    case 1:
      tft.print(FS("---"));
      break;
  }
}


// --PROCESS GRAPH-------------------------------------------------------------
uint16_t pxPerS;
uint16_t pxPerC;
uint16_t xOffset; // used for wraparound on x axis

void updateProcessDisplay()
{
  const uint8_t h       = 86;
  const uint8_t w       = TFT_WIDTH;
  const uint8_t yOffset = TFT_HEIGHT - 98; // space not available for graph

  static uint8_t lastState = 0;

#ifdef GRAPH_DRAW_LINES
  static uint16_t old_dx, old_dy_sp, old_dy_tc;
#endif

  uint16_t dx, dy;
  uint8_t y = 2;
  double tmp;

  // header & initial view
  tft.setTextColor(ST7735_WHITE, ST7735_BLUE);

  if (!initialProcessDisplay) {
    initialProcessDisplay = true;

    tft.fillScreen(ST7735_WHITE);
    tft.fillRect(0, 0, TFT_WIDTH, MENU_ITEM_HEIGHT, ST7735_BLUE);
#ifndef PIDTUNE
    printAtPos(FS("Profile "), 2, y);
    tft.print(activeProfileId);
#else
    printAtPos(FS("Tuning "), 2, y);
#endif

    tmp = h / (activeProfile.peakTemp * 1.10) * 100.0;
    pxPerC = (uint16_t)tmp;
    
#if 0 // pxPerS should be calculated from the selected profile, wint fit in flash right now
    double estimatedTotalTime = 60 * 12;
    // estimate total run time for current profile
    estimatedTotalTime = activeProfile.soakDuration + activeProfile.peakDuration;
    estimatedTotalTime += (activeProfile.soakTemp - 20.0) / (activeProfile.rampUpRate / 10);
    estimatedTotalTime += (activeProfile.peakTemp - activeProfile.soakTemp) / (activeProfile.rampUpRate / 10);
    estimatedTotalTime += (activeProfile.peakTemp - 20.0) / (activeProfile.rampDownRate  / 10);
    //estimatedTotalTime *= 2; // add some spare
    Serial.print("total est. time: ");
    Serial.println((uint16_t)estimatedTotalTime);
#endif
    tmp = 60 * 8;
    tmp = w / tmp * 10.0; 
    pxPerS = (uint16_t)tmp;

    // 50°C grid lines horizontal
    int16_t t = (uint16_t)(activeProfile.peakTemp * 1.10);
    for (uint16_t tg = 0; tg < t; tg += 50) {
      uint16_t line = h - (tg * pxPerC / 100) + yOffset;
      tft.drawFastHLine(0, line, TFT_WIDTH, (tg == 0) ? ST7735_STDGRAY : ST7735_LTGRAY);
#ifdef GRAPH_HAS_TEMPS
      uint8_t xshift;
#if (LCD_ROTATE % 2 == 1)   // landscape view
      if (tg > 0 && tg != 250) {
#else
      if (tg > 0) {
#endif
        itoa10(tg, buf, 1);
        xshift = (tg == 50) ? 6 : 0;
  
        tft.setTextColor(ST7735_STDGRAY, ST7735_WHITE);      
        printAtPos(buf, TFT_WIDTH - (6 * 4) - 4 + xshift, line - 3);   // lcdwidth - 4 digits - right side buffer + maybe 1 digit
        // tft.print("\367");
      }
#endif
    }

#ifdef GRAPH_DRAW_LINES    
    old_dx = 0;
    old_dy_sp = old_dy_tc = TFT_HEIGHT - 11;
#endif
    
#ifdef GRAPH_VERBOSE
    Serial.print("Calc pxPerC/S: ");
    Serial.print(pxPerC);
    Serial.print("/");
    Serial.println(pxPerS);
#endif
  }

  // elapsed time
  uint16_t elapsed;
#ifdef GRAPH_STOP_ON_DONE
  if (processCompleted == false) {
#endif
  elapsed = zeroCrossTicks / SINES_PER_SEC;
  tft.setCursor(TFT_WIDTH - 24, y);
  alignRightPrefix(elapsed); 
  tft.print(elapsed);
  tft.print("s");
#ifdef GRAPH_STOP_ON_DONE
  }
#endif

  y += MENU_ITEM_HEIGHT + 2;

#if (LCD_ROTATE % 2 == 0)
  ftoa(buf, tc[0].temperature, 1);
  tft.setCursor((TFT_WIDTH >> 1) - (6 * strlen(buf)) - 12 - 24, y + 2);
#else
  tft.setCursor(2, y);
#endif

  tft.setTextColor(ST7735_BLACK, ST7735_WHITE);

  // temperature
  tft.setTextSize(2);

#if (LCD_ROTATE % 2 == 0)
  tft.print("  ");  // clear any preceding crap
#else
  alignRightPrefix((int)tc[0].temperature);
#endif

  displayThermocoupleData(&tc[0]);

#if (LCD_ROTATE % 2 == 0)
  tft.print("  ");  // clear any post crap
#endif

  tft.setTextSize(1);
  
#ifdef GRAPH_STOP_ON_DONE
  if (currentState == Complete) {
    if (processCompleted == true) return;
    processCompleted = true;
  }
#endif

#ifndef PIDTUNE
  // current state
  y -= 2;
#if (LCD_ROTATE % 2 == 0)
  y += 24;
  #define casePrintState(state) case state: { printCentered(#state, y); break; }
#else
  tft.setCursor(94, y);
  #define casePrintState(state) case state: { tft.print(#state); break; }
#endif
  
  tft.setTextColor(ST7735_BLACK, ST7735_GREEN);
  
  if (lastState != currentState) {
    lastState = currentState;
#if (LCD_ROTATE % 2 == 0)
    tft.fillRect(20, y, TFT_WIDTH - 40, 8, ST7735_GREEN);
#else
    tft.fillRect(94, y, TFT_WIDTH - 88, 8, ST7735_GREEN);
#endif  
    switch (currentState) {
      casePrintState(RampToSoak);
      casePrintState(Soak);
      casePrintState(RampUp);
      casePrintState(Reflow);
      casePrintState(RampDown);
      casePrintState(CoolDown);
      casePrintState(Complete);
      default: break; // tft.print((uint8_t)currentState); break;
    }
  }

  tft.setTextColor(ST7735_BLACK, ST7735_WHITE);
#endif

  // set point
  y += 10;

#if (LCD_ROTATE % 2 == 0)
  tft.setCursor((TFT_WIDTH >> 1) - (3 * 10), y);
#else
  tft.setCursor(94, y);
#endif

  tft.print(FS("Sp:")); 
  alignRightPrefix((int)Setpoint); 
  printDouble(Setpoint);
  tft.print(FS("\367C  "));

  // draw temperature curves
  //

  if (xOffset >= elapsed) {
    xOffset = 0;
  }

  do { // x with wrap around
    dx = ((elapsed - xOffset) * pxPerS) / 10;
    if (dx > w) {
      xOffset = elapsed;
    }
  } while(dx > w);

  // temperature setpoint
  dy = h - ((uint16_t)Setpoint * pxPerC / 100) + yOffset;
#ifdef GRAPH_DRAW_LINES
  tft.drawLine(old_dx, old_dy_sp, dx, dy, ST7735_BLUE);
  old_dy_sp = dy;
#else
  tft.drawPixel(dx, dy, ST7735_BLUE);
#endif

  // actual temperature
  dy = h - ((uint16_t)tc[0].temperature * pxPerC / 100) + yOffset;
#ifdef GRAPH_DRAW_LINES
  tft.drawLine(old_dx, old_dy_tc, dx, dy, ST7735_RED);
  old_dy_tc = dy;
  old_dx = dx;
#else
  tft.drawPixel(dx, dy, ST7735_RED);
#endif

  // bottom line
  y = TFT_HEIGHT - 9;

  // set values
  printAtPos("\xef", 2, y);
  alignRightPrefix((int)heaterValue); 
  tft.print((int)heaterValue);
  tft.print("%");

#if (LCD_ROTATE % 2 == 1)   // landscape view
  tft.print(" ");
#endif

  tft.print(FS(" \x2a"));
  alignRightPrefix((int)fanValue); 
  tft.print((int)fanValue);
  tft.print("%");

#if (LCD_ROTATE % 2 == 1)   // landscape view
  // move last item to the right; looks nicer
  tft.setCursor(TFT_WIDTH - (10 * 6), y);
#endif

  tft.print(FS(" \x12")); // alternative: \x7f
  if (rampRate >= 0) {
    tft.print(" ");   // space is filled with negative sign otherwise
  }
  printDouble(rampRate);
  tft.print(FS("\367C/s    "));
}


// --RESET---------------------------------------------------------------------
bool factoryReset(const Menu::Action_t action)
{
#ifndef PIDTUNE
  if (action == Menu::actionDisplay) {
    bool initial = currentState != Edit;
    currentState = Edit;

    if (initial) { // TODO: add eyecandy: colors or icons
      tft.setTextColor(ST7735_BLACK, ST7735_WHITE);
      printAtPos(FS("Doubleclick to RESET"), TFT_LEFTCOL, 80);
      printAtPos(FS("Click to cancel"), TFT_LEFTCOL, 90);
    }
  }
  
  if ((currentState == Edit) && (action == Menu::actionParent)) { // do it
    factoryReset();
  }
  
  if (action == Menu::actionTrigger || action == Menu::actionParent) {
    currentState = Settings;
    tft.fillScreen(ST7735_WHITE);
    Engine.currentItem = Engine.lastInvokedItem = &Menu::NullItem;     // prevent infinite loops
    Engine.navigate(&miCycleStart);                                    // and just go back to the beginning
    return false;  
  }
#endif // PIDTUNE
}


// --PROFILE LOAD/SAVE---------------------------------------------------------
bool saveLoadProfile(const Menu::Action_t action)
{
#ifndef PIDTUNE
  bool isLoad = Engine.currentItem == &miLoadProfile;

  if (action == Menu::actionDisplay) {
    bool initial = currentState != Edit;
    currentState = Edit;

    tft.setTextColor(ST7735_BLACK, ST7735_WHITE);

    if (initial) {
      encAbsolute = activeProfileId;  
      printAtPos(FS("Doubleclick to exit"), TFT_LEFTCOL, 90);
    }

    if (encAbsolute > MAX_PROFILES) encAbsolute = MAX_PROFILES;
    if (encAbsolute <  0) encAbsolute =  0;

    printAtPos(FS("Click to "), TFT_LEFTCOL, 80);
    tft.print((isLoad) ? FS("load ") : FS("save "));
    tft.setTextColor(ST7735_WHITE, ST7735_RED);
    tft.print(encAbsolute);

    // fix garbage potentially showing up after digit
    tft.setTextColor(ST7735_BLACK, ST7735_WHITE);
    tft.print("   ");
  }

  if (action == Menu::actionTrigger) {
    (isLoad) ? loadProfile(encAbsolute) : saveProfile(encAbsolute);
    tft.fillScreen(ST7735_WHITE);
    Engine.navigate(Engine.getParent());
    return false;
  }

  if (action == Menu::actionParent) {
    currentState = Settings;
    clearLastMenuItemRenderState();
    return false;
  }
#endif // PIDTUNE
}


// --INIT----------------------------------------------------------------------
void setup()
{
  // configure SSR outputs to initial state
  setupRelayPins();
  
  // start up LCD
  tft.initR(LCD_TABTYPE);
  tft.setTextWrap(false);
  tft.setTextSize(1);
  tft.setRotation(LCD_ROTATE);

  if (firstRun()) {
    factoryReset();
    loadParameters(0);
  } 
  else {
    loadActiveProfileId();
  }

  tft.fillScreen(ST7735_WHITE);
  tft.setTextColor(ST7735_BLACK, ST7735_WHITE);

#ifdef WITH_SPLASH
  tft.setTextSize(2);
  printCentered(FS("Open"), 8);
  printCentered(FS("Reflow"), 26);
  printCentered(FS("Controller"), 44);

  tft.setTextSize(1);
  strcpy(buf, "version ");
  strcat(buf, ver);
#ifdef FAKE_HW
  strcat(buf, "sw");  // sw = software only? it's shorter than the previous "-fake"
#endif
  printCentered(buf, 66);
  

  printCentered(FS("updates by true"), TFT_HEIGHT - 41);
  printCentered(FS("Copyright (c) 2014"), TFT_HEIGHT - 26);
  printCentered(FS("karl@pitrich.com"), TFT_HEIGHT - 16);
  delay(950);
#endif

  loadFanSpeed();
  loadPID();
  
  PID.SetOutputLimits(0, 100);

  // mainline timer
  Timer1.initialize(TIMER1_ISR_CYCLE);
  Timer1.attachInterrupt(timerIsr);
  
#ifndef FAKE_HW
  pinMode(PIN_ZX, INPUT_PULLUP);
  attachInterrupt(INT_ZX, zeroCrossingIsr, RISING);
  delay(100);
#else
 #if defined(__AVR_ATmega328P__)
  TCCR2B = 0x00;         // disable timer2
  TCCR2A = 0x02;         // set as CTC timer
  TCNT2  = 0x00;         // reset counter
  OCR2A  = (15625 / SINES_PER_SEC);

  TIFR2  = 0x00;         // clear all interrupt flags
  TIMSK2 = _BV(OCIE2A);  // enable compare match interrupt
  
  TCCR2B = 0x07;         // enable, set prescaler /1024
 #elif defined(__AVR_ATmega32U4__)
  Timer3.initialize(SINES_PER_SEC * FAKE_HW); // set speed multiplier by setting FAKE_HW to value
  Timer3.attachInterrupt(zeroCrossingIsr);
 #endif
#endif

  // setup /CS line for thermocouple and read initial temperature
  tc[0].chipSelect = TCOUPLE1_CS;
  digitalWrite(TCOUPLE1_CS, HIGH);
  pinMode(TCOUPLE1_CS, OUTPUT);

  // disable second TC; not used yet
  tc[1].chipSelect = TCOUPLE2_CS;
  digitalWrite(TCOUPLE2_CS, HIGH);
  pinMode(TCOUPLE2_CS, OUTPUT);

#ifndef FAKE_HW
  readThermocouple(&tc[0]);

  if (tc[0].stat != 0) {
    abortWithError(tc[0].stat);
  }
#endif

  // initialize moving average filter
  runningTotalRampRate = tc[0].temperature * TC_NUMREADINGS;
  for(int i = 0; i < TC_NUMREADINGS; i++) {
    airTemp[i].temp = tc[0].temperature;
  }

#ifdef WITH_CALIBRATION
  printAtPos(FS("Calibrating... "), 7, 99);
  delay(400);

  // FIXME: does not work reliably
  while (zxLoopDelay == 0) {
    if (zxLoopCalibration.iterations == zxCalibrationLoops) { // average tick measurements, dump 1st value
      for (int8_t l = 0; l < zxCalibrationLoops; l++) {
        zxLoopDelay += zxLoopCalibration.measure[l];
      }
      zxLoopDelay /= zxCalibrationLoops;
      zxLoopDelay -= 10; // compensating loop runtime
    }
  }
  tft.print(zxLoopDelay);
#else
  zxLoopDelay = DEFAULT_LOOP_DELAY;
#endif

  delay(1000);

  menuExit(Menu::actionDisplay); // reset to initial state
  Engine.navigate(&miCycleStart);
  currentState = Settings;
  menuUpdateRequest = true;
}

// ----------------------------------------------------------------------------
/* moving average
    int samples[8];

    total -= samples[i];
    samples[i] = tc[0].temperature; // new value
    total += samples[i];

    i = (i + 1) % 8; // next position
    average = total >> 3; // == div by 8 */
// ----------------------------------------------------------------------------

uint32_t lastRampTicks;

void updateRampSetpoint(bool down = false)
{
  if (zeroCrossTicks > lastRampTicks + SINES_PER_SEC) {
    double rate = (down) ? activeProfile.rampDownRate : activeProfile.rampUpRate;
    rate /= 10;
    Setpoint += (rate / SINES_PER_SEC * (zeroCrossTicks - lastRampTicks)) * ((down) ? -1 : 1);
    lastRampTicks = zeroCrossTicks;
  }
}

// ----------------------------------------------------------------------------

#ifdef PIDTUNE
void toggleAutoTune()
{
 if(currentState != Tune) { //Set the output to the desired starting frequency.
    currentState = Tune;

    Output = AUTOTUNE_START_VALUE;
    PIDTune.SetNoiseBand(AUTOTUNE_NOISE);
    PIDTune.SetOutputStep(AUTOTUNE_STEP);
    PIDTune.SetLookbackSec(AUTOTUNE_LOOKBACK);
  }
  else { // cancel autotune
    PIDTune.Cancel();
    currentState = CoolDown;
  }
}
#endif // PIDTUNE

// ----------------------------------------------------------------------------

uint8_t thermocoupleErrorCount;


// --MAINLINE------------------------------------------------------------------
void loop(void) 
{
  bool menuUpdateLocal = false;
  static bool menuIsRoot = false;
  
  // --------------------------------------------------------------------------
  // handle encoder
  //
  encMovement = Encoder.getValue();
  if (encMovement) {
    encAbsolute += encMovement;
    if (currentState == Settings) {
      Engine.navigate((encMovement > 0) ? Engine.getNext() : Engine.getPrev());
      menuUpdateLocal = true;
    }
  }

  // --------------------------------------------------------------------------
  // handle button
  //
  switch (Encoder.getButton()) {
    case ClickEncoder::Clicked: {
      if (currentState == Complete) {   // at end of cycle; reset at click
        menuExit(Menu::actionDisplay);  // reset to initial state
        Engine.navigate(&miCycleStart);
        currentState = Settings;
        menuUpdateLocal = true;
      }
      else if (currentState < UIMenuEnd) {
        menuUpdateLocal = true;
        Engine.invoke();
      }
      else if (currentState > UIMenuEnd) {
        currentState = CoolDown;
      }
      break;
    }
   
    case ClickEncoder::DoubleClicked: {
      if (currentState < UIMenuEnd) {
        if (Engine.getParent() != &miExit) {
          Engine.navigate(Engine.getParent());
          menuUpdateLocal = true;
        }
      }
      break;
    }
  }

  // --------------------------------------------------------------------------
  // update current menu item while in edit mode
  //
  if (currentState == Edit) {
    if (Engine.currentItem != &Menu::NullItem) {
      Engine.executeCallbackAction(Menu::actionDisplay);      
    }
  }

  // --------------------------------------------------------------------------
  // handle menu update
  //
  if (menuUpdateRequest || menuUpdateLocal) {
    menuUpdateRequest = false;
    menuIsRoot = false;
    
    // clear menu on child/parent navigation
    if (currentState < UIMenuEnd && !encMovement && currentState != Edit && previousState != Edit) {
      tft.fillScreen(ST7735_WHITE);
    }

    // print menu entries
    Engine.render(renderMenuItem, MENU_ITEMS_VISIBLE);
    
    // separate menu entries from bottom area
    tft.drawFastHLine(0, (MENU_ITEMS_VISIBLE * MENU_ITEM_HEIGHT), TFT_WIDTH, ST7735_LTGRAY);

    // are we navigating in a submenu? if so, print information on how to exit the submenu
    if (currentState == Settings) {
      tft.setTextColor(ST7735_BLACK);  // should always be safe to do this?
      
      if (!(Engine.getParent() == &miExit || Engine.getParent() == &miEditable)) {
        // we must be in a submenu...
        printAtPos(FS("Doubleclick to exit"), TFT_LEFTCOL, 80);
      }
      else if (Engine.lastInvokedItem == &Menu::NullItem) {
        // we are at root menu; show the currently loaded profile
        menuIsRoot = true;
        
        printAtPos(FS("Using Profile "), TFT_LEFTCOL, 80);       
        tft.print(activeProfileId);
        
#ifdef SHOW_TEMP_MAIN_PAGE
        // also show oven temp line
        printAtPos(FS("Oven Temp: "), TFT_LEFTCOL, TFT_HEIGHT - 12);
#endif
      }
    }
  }
  
#ifdef SHOW_TEMP_MAIN_PAGE
  if (menuTempUpdateRequest) {
    menuTempUpdateRequest = false;
    if (currentState == Settings && menuIsRoot)
    {
      tft.fillRect(TFT_LEFTCOL + 66, TFT_HEIGHT - 12, TFT_LEFTCOL + 96, TFT_HEIGHT - 6, ST7735_WHITE);
      tft.setCursor(TFT_LEFTCOL + 66, TFT_HEIGHT - 12);
      displayThermocoupleData(&tc[0]);
    }
  }
#endif

  // --------------------------------------------------------------------------
  // track state changes
  //
  if (currentState != previousState) {
    stateChangedTicks = zeroCrossTicks;
    stateChanged = true;
    previousState = currentState;
  }

  // --------------------------------------------------------------------------

  if (zeroCrossTicks - lastUpdate >= 10) {
    uint32_t deltaT = zeroCrossTicks - lastUpdate;
    lastUpdate = zeroCrossTicks;

#ifndef FAKE_HW
    readThermocouple(&tc[0]); // should be sufficient to read it every 250ms or 500ms
    
    if (tc[0].stat > 0) {
      thermocoupleErrorCount++;
    }
    else {
      thermocoupleErrorCount = 0;
    }

    if (thermocoupleErrorCount > TC_ERROR_TOLERANCE) {
      abortWithError(tc[0].stat);
    }
#else
    tc[0].temperature = encAbsolute;
#endif

#if 0 // verbose thermocouple error bits
    for (uint8_t mask = B111; mask; mask >>= 1) {
      printAtPos(mask & tc[0].stat ? '1' : '0', TFT_LEFTCOL, 40);
    }
#endif
      
    // rolling average of the temp T1 and T2
    tcLog[0].total -= tcLog[0].readings[tcIndex];        // subtract the last reading
    tcLog[0].readings[tcIndex] = tc[0].temperature;      // copy the temperature
    tcLog[0].total += tcLog[0].readings[tcIndex];        // add the reading to the total
    tcIndex = (tcIndex + 1) % TC_NUMREADINGS;            // next position
    tcLog[0].average = tcLog[0].total / TC_NUMREADINGS;  // calculate the average temp

    // need to keep track of a few past readings in order to work out rate of rise
    for (int i = 1; i < TC_NUMREADINGS; i++) { // iterate over all previous entries, moving them backwards one index
      airTemp[i - 1].temp = airTemp[i].temp;
      airTemp[i - 1].ticks = airTemp[i].ticks;     
    }

    airTemp[TC_NUMREADINGS - 1].temp = tcLog[0].average; // update the last index with the newest average
    airTemp[TC_NUMREADINGS - 1].ticks = deltaT;

    // calculate rate of temperature change
    uint32_t collectTicks;
    for (int i = 0; i < TC_NUMREADINGS; i++) {
      collectTicks += airTemp[i].ticks;
    }

    rampRate = (airTemp[TC_NUMREADINGS - 1].temp - airTemp[0].temp) / collectTicks * SINES_PER_SEC;

    Input = airTemp[TC_NUMREADINGS - 1].temp; // update the variable the PID reads

    // display update
    if (zeroCrossTicks - lastDisplayUpdate > 33) {
      lastDisplayUpdate = zeroCrossTicks;
      if (currentState > UIMenuEnd) {
        updateProcessDisplay();
      }
    }

    switch (currentState) {
#ifndef PIDTUNE
      case RampToSoak:
        if (stateChanged) {
          lastRampTicks = zeroCrossTicks;
          stateChanged = false;
          Output = 80;
          
          PID.SetControllerDirection(DIRECT);
          PID.SetTunings(heaterPID.Kp, heaterPID.Ki, heaterPID.Kd);
          PID.SetMode(AUTOMATIC);
          
          Setpoint = airTemp[TC_NUMREADINGS - 1].temp;
        }

        updateRampSetpoint();

        if (Setpoint >= activeProfile.soakTemp - 1) {
          currentState = Soak;
        }
        break;

      case Soak:
        if (stateChanged) {
          stateChanged = false;
          Setpoint = activeProfile.soakTemp;
        }

        if (zeroCrossTicks - stateChangedTicks >= (uint32_t)activeProfile.soakDuration * SINES_PER_SEC) {
          currentState = RampUp;
        }
        break;

      case RampUp:
        if (stateChanged) {
          stateChanged = false;
          lastRampTicks = zeroCrossTicks;
        }

        updateRampSetpoint();

        if (Setpoint >= activeProfile.peakTemp - 1) {
          Setpoint = activeProfile.peakTemp;
          currentState = Reflow;
        }
        break;

      case Reflow:
        if (stateChanged) {
          stateChanged = false;
          Setpoint = activeProfile.peakTemp;
        }

        if (zeroCrossTicks - stateChangedTicks >= (uint32_t)activeProfile.peakDuration * SINES_PER_SEC) {
          currentState = RampDown;
        }
        break;

      case RampDown:
        if (stateChanged) {
          stateChanged = false;
          lastRampTicks = zeroCrossTicks;
          PID.SetControllerDirection(REVERSE);
          PID.SetTunings(fanPID.Kp, fanPID.Ki, fanPID.Kd);
          Setpoint = activeProfile.peakTemp - 15; // get it all going with a bit of a kick! v sluggish here otherwise, too hot too long
        }

        updateRampSetpoint(true);

        if (Setpoint <= IDLE_SAFE_TEMP) {
          currentState = CoolDown;
        }
        break;
#endif
      case CoolDown:
        if (stateChanged) {
          stateChanged = false;
          PID.SetControllerDirection(REVERSE);
          PID.SetTunings(fanPID.Kp, fanPID.Ki, fanPID.Kd);
          Setpoint = IDLE_SAFE_TEMP;
        }

        if (Input < (IDLE_SAFE_TEMP + 5)) {
          currentState = Complete;
          PID.SetMode(MANUAL);
          Output = 0;
        }

#ifdef PIDTUNE
      case Tune:
        {
          Setpoint = 210.0;
          int8_t val = PIDTune.Runtime();
          PIDTune.setpoint = 210.0;

          if (val != 0) {
            currentState = CoolDown;
          }

          if (currentState != Tune) { // we're done, set the tuning parameters
            heaterPID.Kp = PIDTune.GetKp();
            heaterPID.Ki = PIDTune.GetKi();
            heaterPID.Kd = PIDTune.GetKd();
            
            savePID();

            printAtPos(FS("Kp: ")); tft.print((uint32_t)(heaterPID.Kp * 100), 40, 40);
            printAtPos(FS("Ki: ")); tft.print((uint32_t)(heaterPID.Ki * 100), 40, 52);
            printAtPos(FS("Kd: ")); tft.print((uint32_t)(heaterPID.Kd * 100), 40, 64);
          }
        }
        break;
#endif
    }
  }

  // safety check that we're not doing something stupid. 
  // if the thermocouple is wired backwards, temp goes DOWN when it increases
  // during cooling, the t962a lags a long way behind, hence the hugely lenient cooling allowance.
  // both of these errors are blocking and do not exit!
  //if (Setpoint > Input + 50) abortWithError(10); // if we're 50 degree cooler than setpoint, abort
  //if (Input > Setpoint + 50) abortWithError(20); // or 50 degrees hotter, also abort
  
#ifndef PIDTUNE
  PID.Compute();

  // decides which control signal is fed to the output for this cycle
  if (   currentState == RampDown
      || currentState == CoolDown
      || currentState == Settings
      || currentState == Complete
      || currentState == Idle
      || currentState == Settings
      || currentState == Edit)
  {
    heaterValue = 0;
    fanValue = Output;
  } else {
    heaterValue = Output;
    fanValue = fanAssistSpeed;
  }
#else
  heaterValue = Output;
  fanValue = fanAssistSpeed;
#endif

  Channels[CHANNEL_HEATER].target = heaterValue;

  // 0-100% -> 0-90° phase control
#if (FAN_MODE == 0)
  Channels[CHANNEL_FAN].target = 90 - ((90 * fanValue) / 100);
#endif
}

void memoryFeedbackScreen(uint8_t profileId, bool loading)
{
  tft.fillScreen(ST7735_GREEN);
  tft.setTextColor(ST7735_BLACK);
  printAtPos(loading ? FS("Loading") : FS("Saving"), TFT_LEFTCOL + 8, 50);
  tft.print(FS(" profile "));
  tft.print(profileId);
}


// --EEPROM------------------------------------------------------------------
void saveProfile(uint8_t targetProfile, bool quiet)
{
#ifndef PIDTUNE
  activeProfileId = targetProfile;

  if (!quiet) {
    memoryFeedbackScreen(activeProfileId, false);
  }
  saveParameters(activeProfileId); // activeProfileId is modified by the menu code directly, this method is called by a menu action

  if (!quiet) delay(500);
#endif
}

void loadProfile(uint8_t targetProfile)
{
  memoryFeedbackScreen(targetProfile, true);
  loadParameters(targetProfile);

  // save in any way, as we have no undo
  activeProfileId = targetProfile;
  saveActiveProfileId();

  delay(500);
}

bool saveParameters(uint8_t profile)
{
#ifndef PIDTUNE
  uint16_t offset = profile * sizeof(Profile_t);

  do {} while (!(eeprom_is_ready()));
  eeprom_write_block(&activeProfile, (void *)offset, sizeof(Profile_t));
#endif
  return true;
}

void loadParameters(uint8_t profile)
{
  uint16_t offset = profile * sizeof(Profile_t);

  do {} while (!(eeprom_is_ready()));
  eeprom_read_block(&activeProfile, (void *)offset, sizeof(Profile_t));
}

bool loadPID()
{
  do {} while (!(eeprom_is_ready()));
  eeprom_read_block(&heaterPID, E2OFFSET_PID_CONFIG, sizeof(PID_t));
  return true;  
}

bool savePID()
{
  do {} while (!(eeprom_is_ready()));
  eeprom_write_block(&heaterPID, E2OFFSET_PID_CONFIG, sizeof(PID_t));
  UpdateChecksum();
  return true;
}

uint8_t E2Checksum()
{
  uint8_t checksum = 0;
  
  for (uint16_t addr = 0; addr < E2END; addr++)
    checksum += eeprom_read_byte((uint8_t *)addr);
    
  return checksum;
}

void UpdateChecksum()
{
  eeprom_write_byte((uint8_t *)E2END, E2Checksum());
}

bool firstRun()
{
#ifndef PIDTUNE
  if (eeprom_read_byte((uint8_t *)E2END) == E2Checksum())
    return false;
#endif

  return true;
}

void makeDefaultProfile()
{
  activeProfile.soakTemp     = 145;
  activeProfile.soakDuration =  80;
  activeProfile.peakTemp     = 240;
  activeProfile.peakDuration =  45;
  activeProfile.rampUpRate   =   6;
  activeProfile.rampDownRate =  18;
}

void factoryReset()
{
#ifndef PIDTUNE
  makeDefaultProfile();

  tft.fillScreen(ST7735_RED);
  tft.setTextColor(ST7735_WHITE);
  printAtPos(FS("Resetting..."), TFT_LEFTCOL, 50);

  // then save the same profile settings into all slots
  for (uint8_t i = 0; i < MAX_PROFILES; i++) {
    saveParameters(i);
  }

  fanAssistSpeed = 33;
  saveFanSpeed();

  savePID();

  activeProfileId = 0;
  saveActiveProfileId();

  delay(500);
#endif
}

void loadFanSpeed()
{
  fanAssistSpeed = eeprom_read_byte(E2OFFSET_FAN_SPEED);
}

void saveFanSpeed()
{
  eeprom_write_byte(E2OFFSET_FAN_SPEED, fanAssistSpeed);
  UpdateChecksum();
}

void updateRunCounter()
{
  
}

void saveRunCounter()
{
  // EEPROM.write( 
}

void loadActiveProfileId()
{
  activeProfileId = eeprom_read_byte(E2OFFSET_PROFILE_NUMBER);
  loadParameters(activeProfileId);
}

void saveActiveProfileId()
{
  eeprom_write_byte(E2OFFSET_PROFILE_NUMBER, activeProfileId);
  UpdateChecksum();
}

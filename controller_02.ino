/* -----------------------------------------------------------------------------------

                   Control program for the
                      Saw Mill Lodge
                    Pool/Spa Controller


   This is a custom-built combination pool and spa controller for a system that has:
    - two circulation pumps (pool, spa)
    - a bubbler pump
    - an electronically-controlled heater (Raypack 266A)
    - three electric Jandy valves that provide four configurations:
       - heat spa
       - heat pool
       - fill spa
       - empty spa
    - pool lights

   The controller contains the following components:
    - an Arduino MEGA 2560 microcontroller
    - a 4-line by 20 character LCD display
    - 8 waterproof lighted pushbuttons
    - a rotary encoder for temperature control
    - a Maxim DS18B20 thermometer
    - a Chronodot battery backed-up realtime clock with a Maxim DS1307 clock chip
    - 10 optically-isolated relays for controlling pumps, heater, valves, and lights

   For more information including photos and a schematic, see
   https://github.com/LenShustek/PoolSpaController

   See notes in the Watchdog section about how you need to program the processor.
    (It requires disabling the bootloader entirely by unprogramming the BOOTRST fuse to 1, so
    the application program starts instead of the boot loader, and then using
    AVRDUDE and an external programmer like USBTinyISP to load new software.)
   ----------------------------------------------------------------------------------------*/

// 13 Jan 2014, V1.0, L. Shustek
//              - initial version
//  2 Feb 2014, V2.0, L. Shustek
//              - add rotary encoder for temperature control
// 19 Mar 2014, V2.1, L. Shustek
//              - allow either "heat pool" or "heat spa" configuration for filtering
//  9 Apr 2014, V2.2, L. Shustek
//              - fix noon/midnight confusion in displaying filter hour;
//                see http://en.wikipedia.org/wiki/12-hour_clock
//              - reorganize buttons; add separate filter spa/pool, spa water level
// 20 Jul 2014, V2.3, L. Shustek
//              - Don't display temp if filtering spa with "heat pool" valve config, or
//                filtering pool with "heat spa" valve config. (No flow past temp sensor.)
//              - Fix filter auto-start hour again: was ignoring am/pm flag.
//                Simplify by recording 12-hour clock format in the EPROM!
// 10 Aug 2014, V2.4, L. Shustek
//              - Validity-check realtime clock data, and reset if bad.
//              - Add a special test mode enabled by left/right arrows in "spa fill/empty".
//  5 Jan 2016, V2.5, L. Shustek
//              - Add a "heater out of service" mode that avoids sending water to it.
//              - Fix if_zero() to fetch with interrupts disabled.
//                (Dunno if we ever encountered a race condition because of that.)
//  3 Mar 2016, V2.6, L. Shustek
//               - Add event logging.
// 25 Mar 2017, V2.7, L. Shustek
//               - Log an initial "IDLE" event at power on.
//               - Reformat the source in a more condensed indentation style.
// 06 Jan 2020, V2.8, L. Shustek
//               - Enable the processor's Watchdog Timer to reset us if we become catatonic.
//                 (Requires the use of an external programmer.)
//               - Move the log to EEPROM: more history, non-volatile, and saves RAM.
//                 Also, use it for general interesting events, not just mode changes.
//
//-----------------------------------------------------------------------------------------

#define VERSION "2.8"
#define TITLE "Saw Mill Lodge"

#define DEBUG false
#define DEBUG_TIMES false
#define ALLOW_SPECIAL_TEST_MODE false
#define SHOWMEM false

#include <arduino.h>
#include <stdarg.h>
#include <EEPROM.h>         // for non-volatile configuration memory
#include <LiquidCrystal.h>  // for LCD display
#include <Wire.h>           // for realtime clock
#include <avr/wdt.h>        // for watchdog timer
#include "OneWire.h"        // for temperature sensor
#include "controller_02.h"  // our stuff

#define NULLP ((char *)0)   // null pointer

// pushbuttons

#define NUM_BUTTONS 8
const byte button_pins[NUM_BUTTONS] = { // button pin numbers
   HEAT_SPA_BUTTON_PIN, HEAT_POOL_BUTTON_PIN, SPA_JETS_BUTTON_PIN, POOL_LIGHT_BUTTON_PIN,
   FILTER_SPA_BUTTON_PIN, FILTER_POOL_BUTTON_PIN, SPA_WATER_LEVEL_BUTTON_PIN, PROGRAM_BUTTON_PIN };

enum button_indexes { // define button indices, 0..7
   HEAT_SPA_BUTTON, HEAT_POOL_BUTTON, SPA_JETS_BUTTON, POOL_LIGHT_BUTTON,
   FILTER_SPA_BUTTON, FILTER_POOL_BUTTON, SPA_WATER_LEVEL_BUTTON, PROGRAM_BUTTON };
#define  LEFTARROW_BUTTON  HEAT_SPA_BUTTON
#define  RIGHTARROW_BUTTON  HEAT_POOL_BUTTON
#define  DOWNARROW_BUTTON  SPA_JETS_BUTTON
#define  UPARROW_BUTTON  POOL_LIGHT_BUTTON

// State information

enum mode_t mode = MODE_IDLE;                  // currrent global mode
enum heater_t heater_mode = HEATING_NONE;      // current heater setting: none, pool, spa
boolean heater_on = false;                     // is the heater currently on?
volatile byte heater_cooldown_secs_left = 0;   // cooldown seconds left after heater off
enum vconfig_t valve_config = VALVES_UNDEFINED;// current valve configuration
enum pump_status_t pump_status = PUMP_NONE;    // current status of pumps
byte target_temp = 102;                        // target temperature in degrees F
boolean button_awaiting_release[NUM_BUTTONS] = {
   false };                                     // button pushed but awaiting release?
uint16_t leds_on = 0;                          // mask for which LEDs are currently on
#define LED_OFF 0x0000
#define LED_ON  0xffff
boolean pool_light_on = false;
boolean spa_jets_on = false;
boolean have_tempsensor = false;
boolean no_clock = false;

volatile boolean do_title1 = true;
volatile boolean do_title2 = false;
volatile byte title_timer = 0;           // count seconds for titles

volatile unsigned int mode_timer = 0;     // minutes left in the current mode
volatile unsigned int spa_jets_timer = 0; // minutes left to aerator shutoff
volatile unsigned int light_timer = 0;    // minutes left to light shutoff
boolean filter_autostarted = false;       // did we autostart filtering?
byte last_filter_date = 0xff;             // what day we last did filtering

LiquidCrystal lcd(LCD_RS, LCD_ENB, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
#define CONFIG_ROW 1  // use second row for configuration strings

#define DS1307_CTRL_ID B1101000  //DS1307 realtime clock id
struct datetime {
   byte sec, min, hour /*1-12*/, ampm, day, date /*1-7*/, month, year; }
now,
clock_init = {
   50, 10, 8, 1, 3, 21, 1, 14 }; // (when we first wrote the code)

static const char *months[] = { // index 1..12 from realtime clock
   "???", "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};
static const byte days_in_month []  = {
   99, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

OneWire tempsensor(TEMPSENSOR_PIN); // Maxim DS18B20 temperature sensor
byte tempsensor_addr[8];  // its discovered address

//****  map of EEPROM storage for configuration info and the event log

struct { // local copy of the configuration data that is in EEPROM
   char hdr_id[6]; // "SMLnn" // unique header ID w/ version number
   byte filter_pool_mins;     // how many minutes to filter pool
   byte filter_spa_mins;      // how many minutes to filter spa
   byte filter_start_hour;    // what hour 1-12 to start filtering each day
   byte filter_start_ampm;    // whether AM or PM, 0=am, 1=pm
   byte heater_allowed;       // whether heater is allowed to be used
   // We could add the last set spa and pool temperatures, I suppose.
   // But that might just be confusing when there are multiple pool/spa users.
} config_data = { // default configuration data
   // change the hdr_id when the format or event list changes, to force reinitialization
   "SML04", FILTER_POOL_TIME, FILTER_SPA_TIME, FILTER_START_HOUR, FILTER_START_AMPM, TRUE};

struct {  // local copy of the event log  header that is in EEPROM
   unsigned short count;    // how many log entries are in use
   unsigned short next;     // the next location to write a new entry to
} eventlog_hdr = {0, 0 };

#define LOG_MSGSIZE 20
struct { /*logentry_t*/  // the log entries immediately following the log header
   struct datetime timestamp;
   byte event_type;
   char event_msg[LOG_MSGSIZE]; // optional message, maybe NOT 0-terminated
} logentry; // we keep only one entry here in RAM

#define EEPROM_SIZE 4096      // for ATMega2560
#define CONFIG_HDR_LOC 0      // where the config data starts in EEPROM
#define LOG_HDR_LOC sizeof(config_data) // where the event log header starts in EEPROM
#define LOG_LOC (LOG_HDR_LOC+sizeof(eventlog_hdr)) // where the log entries start
#define LOG_SIZE ((EEPROM_SIZE - LOG_LOC) / sizeof(logentry)) // how many entries, max

enum event_t {    // event types
   EV_BAD,              // (to catch uninitialized log entries)
   EV_STARTUP,          // we started from power up
   EV_WATCHDOG_RESET,   // we were reset by watchdog timer
   EV_ASSERTION_FAILED, // assertion failed
   EV_CLOCK_BAD,        // can't find realtime clock
   EV_INIT_CONFIG,      // initialized the config data
   EV_UPDATED_CONFIG,   // updated the configuration data
   EV_IDLE,             // entered these various modes...
   EV_HEAT_SPA,
   EV_HEAT_POOL,
   EV_FILL_SPA,
   EV_EMPTY_SPA,
   EV_FILTER_POOL,
   EV_FILTER_SPA,
   EV_NUM_EVENTS };

static const char *event_names[] = {
   "???",
   "power on restart", "watchdog restart", "assertion failed", "clock bad",
   "init config", "updated config",
   "idle", "heat spa", "heat pool", "fill spa", "empty spa", "filter pool", "filter spa" };
typedef char event_name_error[sizeof(event_names) / sizeof(event_names[0]) == EV_NUM_EVENTS ? 1 : -1]; // compiler check

#if SHOWMEM
//************************************************************************
//*	http://www.nongnu.org/avr-libc/user-manual/malloc.html
//*	thanks to John O for the pointer to this info and the inspiration to do it
void	Ram_TableDisplay(void) {
   char stack = 1;
   extern char *__data_start;
   extern char *__data_end;
   extern char *__brkval;
   extern char *__bss_start;
   extern char *__bss_end;
   //extern char *__brkval;
   extern char *__heap_start;
   extern char *__heap_end;
   //extern char *__malloc_heap_end;
   //extern size_t __malloc_margin;
   int	data_size	=	(int)&__data_end - (int)&__data_start;
   int	bss_size	=	(int)&__bss_end - (int)&__data_end;
   int	heap_end	=	(int)&stack - (int)&__malloc_margin;
   //	int	heap_size	=	(int)__brkval - (int)&__bss_end;
   int	heap_size	=	heap_end - (int)&__bss_end;
   int	stack_size	=	RAMEND - (int)&stack + 1;
   int	available	=	(RAMEND - (int)&__data_start + 1);
   available	-=	data_size + bss_size + heap_size + stack_size;
   Serial.print("+----------------+  __data_start  =");
   Serial.println((int)&__data_start);
   Serial.println("+      data      +");
   Serial.print("+    variables   +  data_size     =");
   Serial.println(data_size);
   Serial.print("+----------------+  __data_end    =");
   Serial.println((int)&__data_end);
   Serial.print("+----------------+  __bss_start   =");
   Serial.println((int)&__bss_start);
   Serial.println("+       bss      +");
   Serial.print("+    variables   +  bss_size      =");
   Serial.println(bss_size);
   Serial.print("+----------------+  __bss_end     =");
   Serial.println((int)&__bss_end);
   Serial.print("+----------------+  __heap_start  =");
   Serial.println((int)&__heap_start);
   Serial.print("+       heap     +  heap_size     =");
   Serial.println(heap_size);
   Serial.print("+----------------+  heap_end      =");
   Serial.println(heap_end);
   Serial.print("+----------------+  Current STACK =");
   Serial.println((int)&stack);
   Serial.print("+      stack     +  stack_size    =");
   Serial.println(stack_size);
   Serial.print("+----------------+  RAMEND        =");
   Serial.println(RAMEND);
   Serial.print("__brkval      ="); Serial.println((int)__brkval);
   Serial.print("available ="); Serial.println(available);
   Serial.println(); }
#endif //DEBUG

//-----------------------------------------------------------------
// event log routines
//-----------------------------------------------------------------

void eeprom_write(int addr, int length, byte *srcptr) {
   while (length--)
      EEPROM.write(addr++, *srcptr++); }

void eeprom_read(int addr, int length, byte *dstptr) {
   while (length--)
      *dstptr++ = EEPROM.read(addr++); }

void log_event(enum event_t event_type, const char *msg) {
   #if DEBUG
   Serial.print("log event: "); Serial.print(event_names[event_type]);
   if (msg) {
      Serial.print(", "); Serial.print(msg); }
   Serial.println();
   #endif
   logentry.timestamp = now;
   logentry.event_type = event_type;
   if (msg) strncpy(logentry.event_msg, msg, LOG_MSGSIZE);
   else memset(logentry.event_msg, 0, LOG_MSGSIZE);
   eeprom_write(LOG_LOC + eventlog_hdr.next * sizeof(logentry),
                sizeof(logentry), (byte *) &logentry);
   if (eventlog_hdr.count < LOG_SIZE) ++eventlog_hdr.count;
   if (++eventlog_hdr.next >= LOG_SIZE) eventlog_hdr.next = 0;
   eeprom_write(LOG_HDR_LOC, sizeof(eventlog_hdr), (byte *)&eventlog_hdr); }

void log_event(enum event_t event_type) {
   log_event(event_type, NULLP); }


//-------------------------------------------------------
// Utility routines
//-------------------------------------------------------

void assert(boolean test, const char *msg) {
   if (!test) {
      #if DEBUG
      Serial.print("failed assertion: "); Serial.println(msg);
      #endif
      log_event(EV_ASSERTION_FAILED, msg);
      lcd.clear(); lcd.print("** INTERNAL ERROR **");
      lcd.setCursor(0, 1); lcd.print("Assertion failed:");
      lcd.setCursor(0, 2); lcd.print(msg);
      while (true) ; // wait for watchdog to reset us
   } }

void center_message (byte row, const char *msg) {
   byte len, nblanks;
   len = strlen(msg);
   assert(len <= 20, "bad center_message");
   #if DEBUG
   Serial.print(":: "); Serial.println(msg);
   #endif
   nblanks = (20 - len) >> 1;
   lcd.setCursor(0, row);
   for (byte i = 0; i < nblanks; ++i) lcd.print(" ");
   lcd.print(msg);
   nblanks = (20 - nblanks) - len;
   for (byte i = 0; i < nblanks; ++i) lcd.print(" "); }

void center_messagef (byte row, const char *msg, ...) {
   char buf[40];
   va_list arg_ptr;
   va_start(arg_ptr, msg);
   vsnprintf(buf, sizeof(buf), msg, arg_ptr);
   center_message(row, buf);
   va_end(arg_ptr); }

void mode_message (const char *msg) {
   lcd.clear();
   if (msg != NULLP) {  // new mode starting
      center_message(1, msg);
      do_title1 = true;  // restart top title
      do_title2 = false;
      title_timer = 0; }
   else {  // changing mode
      center_message(0, "changing mode"); } }

void setLED(uint16_t mask, uint16_t onoff) {  // turn LEDs on or off
   uint16_t bits;
   leds_on = (leds_on & ~mask) | (mask & onoff); // copy new bit(s) to merged mask
   assert(leds_on != 0xffff, "err in setLED");
   bits = leds_on;
   for (byte i = 0; i < 16; ++i) { //shift out all bits to a pair of TI TLC5916 drivers
      digitalWrite(LED_DRIVER_SDI, bits & 1 ? HIGH : LOW);
      bits >>= 1;
      digitalWrite(LED_DRIVER_CLK, HIGH); // create clock pulse to shift one bit
      digitalWrite(LED_DRIVER_CLK, LOW); }
   digitalWrite(LED_DRIVER_LE, HIGH); // all done: create latch pulse
   digitalWrite(LED_DRIVER_LE, LOW); }

byte check_for_button (void) {  // check for a button push, return button or 0xff if none
   byte button;
   watchdog_poke(); // a good place to reset the watchdog timer
   for (button = 0; button < NUM_BUTTONS; ++button) {
      if (digitalRead(button_pins[button])) {  // button is released
         if (button_awaiting_release[button]) {
            button_awaiting_release[button] = false;
            delay (DEBOUNCE_DELAY); } }
      else { // button is pushed
         if (!button_awaiting_release[button]) { // not already acted on
            delay (DEBOUNCE_DELAY);
            button_awaiting_release[button] = true; // setup to await release later
            return button; // return this button
         } } }
   return 0xff; }

byte wait_for_button (void) { // wait for a button to be pushed
   byte button;
   while ((button = check_for_button()) == 0xff);
   return button; }

void heater_off (void) {
   if (heater_mode != HEATING_NONE) {
      center_message(2, " "); // remove time left display
      center_message(3, " "); // remove temperature display
      digitalWrite(HEAT_SPA_RELAY, RELAY_OFF);
      digitalWrite(HEAT_POOL_RELAY, RELAY_OFF);
      setLED(TEMPCTL_RED_LED | TEMPCTL_BLUE_LED, LED_OFF);
      if (heater_on) {
         heater_cooldown_secs_left = DELAY_HEATER_OFF;
         heater_on = false; }
      heater_mode = HEATING_NONE;
      if (heater_cooldown_secs_left) {
         do {
            char msg[25];
            sprintf(msg, "heater cooling...%d", heater_cooldown_secs_left);
            center_message(2, msg);
            watchdog_poke(); }
         while (heater_cooldown_secs_left); // interrupt routine decrements this
         center_message(2, " "); } } }

void spa_heater_mode(void) {
   target_temp = 102;  // initial target temperature
   digitalWrite(HEAT_POOL_RELAY, RELAY_OFF);
   digitalWrite(HEAT_SPA_RELAY, RELAY_ON);
   setLED(TEMPCTL_RED_LED, LED_ON);
   heater_on = true;
   heater_mode = HEATING_SPA; }

void pool_heater_mode(void) {
   target_temp = 80;  // initial target temperature
   digitalWrite(HEAT_SPA_RELAY, RELAY_OFF);
   digitalWrite(HEAT_POOL_RELAY, RELAY_ON);
   setLED(TEMPCTL_RED_LED, LED_ON);
   heater_on = true;
   heater_mode = HEATING_POOL; }

void temp_change (int8_t direction) {
   // WARNING: this is called from the interrupt routine,
   // so don't do anything heavy-duty!
   if (heater_mode != HEATING_NONE) { //heater is running
      if (direction < 0) {  // decreasing temp
         if (target_temp > 60) --target_temp; }
      else {  // increasing temp
         if (heater_mode == HEATING_SPA && target_temp < 104
               || heater_mode == HEATING_POOL && target_temp < 90)
            ++target_temp; } } }

void pumps_off (void) {
   heater_off();
   if (pump_status != PUMP_NONE) {
      digitalWrite(POOL_PUMP_RELAY, RELAY_OFF);
      digitalWrite(SPA_PUMP_RELAY, RELAY_OFF);
      pump_status = PUMP_NONE;
      center_message(3, " "); // remove temperature display
      center_message(2, "stopping pump");
      longdelay(1000 * DELAY_PUMP_OFF);
      center_message(2, " "); } }

void pump_on (pump_status_t pump) {
   if (pump == PUMP_SPA)
      digitalWrite(SPA_PUMP_RELAY, RELAY_ON);
   else if (pump == PUMP_POOL)
      digitalWrite(POOL_PUMP_RELAY, RELAY_ON);
   else assert(false, "Bad call to pump_on");
   pump_status = pump;
   center_message(2, "starting pump");
   longdelay(1000 * DELAY_PUMP_ON);
   center_message(2, " "); }

void setvalveconfig(vconfig_t config) {
   if (valve_config != config) {
      pumps_off();
      switch (config) {
         case VALVES_HEAT_SPA:
            digitalWrite(POOL_VALVE_RELAY, VALVE_RIGHT);
            digitalWrite(SPA_VALVE_RELAY, VALVE_RIGHT);
            digitalWrite(HEATER_VALVE_RELAY, VALVE_RIGHT);
            break;
         case VALVES_HEAT_POOL:
            digitalWrite(POOL_VALVE_RELAY, VALVE_LEFT);
            digitalWrite(SPA_VALVE_RELAY, VALVE_LEFT);
            digitalWrite(HEATER_VALVE_RELAY, VALVE_LEFT);
            break;
         case VALVES_FILL_SPA:
            digitalWrite(POOL_VALVE_RELAY, VALVE_LEFT);
            digitalWrite(SPA_VALVE_RELAY, VALVE_RIGHT);
            digitalWrite(HEATER_VALVE_RELAY, VALVE_RIGHT);
            break;
         case VALVES_EMPTY_SPA:
            digitalWrite(POOL_VALVE_RELAY, VALVE_RIGHT);
            digitalWrite(SPA_VALVE_RELAY, VALVE_RIGHT);
            digitalWrite(HEATER_VALVE_RELAY, VALVE_LEFT);
            break;
         default:
            assert(false, "Bad call to setconfig"); }
      for (byte timeleft = DELAY_VALVE_CHANGE; timeleft; --timeleft) {
         center_messagef(2, "setting valves...%d", timeleft);
         delay(1000);  // seconds countdown
         watchdog_poke(); }
      center_message(2, " ");
      valve_config = config; } }

void enter_idle_mode (void) {
   setLED (HEAT_SPA_LED | HEAT_POOL_LED | FILTER_SPA_LED | FILTER_POOL_LED | SPA_WATER_LEVEL_LED, LED_OFF);  // turn off all the mode LEDs
   mode_message (NULLP); // "changing"
   pumps_off();  // turn off the heater and pumps
   mode_timer = 0;
   if (mode != MODE_IDLE) log_event(EV_IDLE);
   mode = MODE_IDLE;
   filter_autostarted = false;
   mode_message(" "); // blank mode message resets title line timing
}

byte read_temp (void) {  // read temperature from DS18B20 one-wire temp sensor
   byte data[12];
   unsigned int temp;
   tempsensor.reset();
   tempsensor.select(tempsensor_addr);
   tempsensor.write(0x44, 1); // start conversion w/ parasite power on at the end
   delay(250);  // 10-bit resolution takes 187 msec
   tempsensor.reset();
   tempsensor.select(tempsensor_addr);
   tempsensor.write(0xBE);  // read scratchpad
   for (byte i = 0; i < 9; ++i)
      data[i] = tempsensor.read();
   temp = (data[1] << 8) | (data[0] & 0xfc ); // zero low 2 bits for 10-bit resolution
   // temp is 16*celsius; do limited-range conversion to fahrenheit
   return (temp * 9) / (5 * 16) + 32; }

//------------------------------------------------------------------------------
//    watchdog timer routines, which cause a hard reset if we become catatonic
//------------------------------------------------------------------------------

/*  This will only work if the bootloader on the chip is bypassed, because it too
    uses the watchdog timer for various purposes including starting an application
    program after powerup. But it delays too long, so if the reset was caused by
    our watchdog timer going off, we wind up in an an infinite reset loop.
    See a discussion at  https://www.avrfreaks.net/forum/watch-dog-disable-bootloader.
    or https://github.com/Optiboot/optiboot/issues/97
    or https://github.com/MCUdude/MightyCore/issues/95

    There are two solutions:
    (1) Reflash a newer bootloader that is the latest 2018 Optiboot with the Adafruit
    "Adaboot no-wait mode", so that if it's not a power-on reset the application
    is started immediately. See https://github.com/Optiboot/optiboot
    and https://github.com/Optiboot/optiboot/wiki/HowOptibootWorks
    (Obsolete: https://github.com/Optiboot/optiboot/wiki/GoodQuestions about ATMega2560)
    That seems hard.

    (2) Disable the bootloader entirely by unprogramming the BOOTRST fuse to 1, so
    the application program starts instead of the boot loader. But then you have
    to flash new programs using AVRDUDE and an external programmer like USBTinyISP.
    The ATmega2560 comes with the high fuse byte set to 0xd8, which has BOOTRST=0.
    To set BOOTRST=1, use avrdude -c usbtiny -p m2560 -U hfuse:w:0xd9:m
    That is what we do.
*/

//uint8_t mcusr_mirror __attribute__ ((section (".noinit")));
//void get_mcusr(void) \   // THIS WAS AN ATTEMPT THAT NEVER WORKED!
//__attribute__((naked)) \
//__attribute__((section(".init3")));
//void get_mcusr(void) { // this runs soon after reset
//   mcusr_mirror = MCUSR;
//   MCUSR = 0;
//   wdt_disable(); }

bool watchdog_setup(void) { // enable the watchdog timer
   // return TRUE if the watchdog triggered the last startup
   noInterrupts();
   __asm__ volatile ("WDR\n\t"); // poke the watchdog, just in case it's about to trigger
   bool triggered = (MCUSR & (1 << WDRF)) != 0; // remember the "watchdog triggered" bit
   MCUSR &= ~(1 << WDRF); // clear the "watchdog triggered" for next time
   WDTCSR = (1 << WDCE) | (1 << WDE); // change enable (for the next 4 cycles!)
   WDTCSR = (1 << WDE) | (1 << WDP3) | (1 << WDP0); // enable with the maximum 8 second timeout
   interrupts();
   return triggered; }

void watchdog_poke(void) { // poke the watchdog to prevent a reset
   if (!DEBUG || digitalRead(TESTPIN) != 0) // in DEBUG mode, test pin prevents watchdog poking
      __asm__ volatile ("WDR\n\t"); }

void longdelay(unsigned long msec) { // a long delay during which we poke the watchdog
   while (msec > 100) {
      watchdog_poke();
      delay(100);
      msec -= 100; }
   if (msec > 0) delay(msec); }

//-------------------------------------------------------
//  Realtime clock routines
//-------------------------------------------------------

byte bcd2bin (byte val) {
   return val - 6 * (val >> 4); }
byte bin2bcd (byte val) {
   return val + 6 * (val / 10); }

// N.B.: The realtime clock is in 12-hour mode

// Read realtime clock data
void rtc_read(struct datetime * dt) {
   byte hour;
   Wire.beginTransmission(DS1307_CTRL_ID);
   Wire.write(0x00);
   Wire.endTransmission();
   // request the 7 bytes of data    (secs, min, hr, day, date. mth, yr)
   Wire.requestFrom(DS1307_CTRL_ID, 7);
   dt->sec = bcd2bin(Wire.read());
   dt->min = bcd2bin(Wire.read());
   hour = Wire.read();
   dt->hour = bcd2bin(hour & 0x1f);
   dt->ampm = (hour >> 5) & 1; // 0=AM, 1=PM
   dt->day = bcd2bin(Wire.read());
   dt->date = bcd2bin(Wire.read());
   dt->month = bcd2bin(Wire.read());
   if (dt->month > 12) dt->month = 12; // careful: we use as subscript
   dt->year = bcd2bin(Wire.read()); }

// Write realtime clock data
void rtc_write(struct datetime * dt) {
   Wire.beginTransmission(DS1307_CTRL_ID);
   Wire.write(0x00); // reset register pointer
   Wire.write(bin2bcd(dt->sec));
   Wire.write(bin2bcd(dt->min));
   Wire.write(bin2bcd(dt->hour) | ((dt->ampm) << 5) | 0x40); // 12 hour mode
   Wire.write(bin2bcd(dt->day));
   Wire.write(bin2bcd(dt->date));
   Wire.write(bin2bcd(dt->month));
   Wire.write(bin2bcd(dt->year));
   Wire.endTransmission(); }

bool rtc_invalid(void) {
   return  now.sec > 59  || now.min > 59 || now.hour == 0 || now.hour > 12
           || now.day  == 0 || now.day > 7 || now.date == 0 || now.date > 31
           || now.month == 0 || now.month > 12 || now.year > 99; }

void show_time(byte row, struct datetime * dt) {
   char string[25];
   if (no_clock)
      strcpy(string, "?? ??? 20?? ??:?? ??");
   else sprintf(string, "%2d %s 20%02d %2d:%02d %s",
                   dt->date, months[dt->month], dt->year, dt->hour, dt->min, dt->ampm ? "PM" : "AM");
   lcd.setCursor(0, row);
   lcd.print(string);
   #if DEBUG
   Serial.print(":: "); Serial.println(string);
   #endif
}
void show_current_time (byte row) {
   rtc_read(&now);
   show_time(row, &now); }

//-------------------------------------------------------
//  special test routine
//  triggered by SPA WATER LEVEL followed by PROGRAM
//-------------------------------------------------------

#if ALLOW_SPECIAL_TEST_MODE
void do_special_test(void) {
   byte button, relay;
   int count;

   //*** This test continually operates any one of the 24V relays.
   //*** I used it to diagnose the LCD display noise problem, which was
   //***  eventually solved by using snubbers across the 5V relay contacts.

   enter_idle_mode();
   mode_message("special relay test");
   center_message(2, "press a relay button");
   relay = 0xff;
   button = wait_for_button();
   if (button == HEAT_SPA_BUTTON) relay = POOL_PUMP_RELAY;
   else if (button == HEAT_POOL_BUTTON) relay = SPA_PUMP_RELAY;
   else if (button == SPA_JETS_BUTTON) relay = SPA_JETS_PUMP_RELAY;
   else if (button == POOL_LIGHT_BUTTON) relay = POOL_LIGHT_RELAY;
   count = 0;
   if (relay != 0xff) do {
         center_messagef(2, "%d", ++count);
         digitalWrite(relay, RELAY_ON);
         delay(250);
         digitalWrite(relay, RELAY_OFF);
         delay(250); }
      while (check_for_button() == 0xff);
   enter_idle_mode(); }
#endif

//-------------------------------------------------------
//  Button action routines
//-------------------------------------------------------

void heat_spa_pushed(void) {
   if (mode == MODE_HEAT_SPA)  // turning off spa
      enter_idle_mode();
   else { // turning on spa
      if (mode != MODE_IDLE) enter_idle_mode();
      if (config_data.heater_allowed) {
         setLED(HEAT_SPA_LED, LED_ON);
         mode_message(NULLP); // "changing"
         setvalveconfig(VALVES_HEAT_SPA);
         pump_on(PUMP_SPA);
         spa_heater_mode();
         mode_timer = MODE_SPA_TIMEOUT;
         mode = MODE_HEAT_SPA;
         log_event(EV_HEAT_SPA);
         mode_message("heating spa"); }
      else {
         mode_message("heater disabled!");
         longdelay(2000);
         mode_message(" "); } } }

void heat_pool_pushed (void) {
   if (mode == MODE_HEAT_POOL)  // turning off pool
      enter_idle_mode();
   else { // turning on pool
      if (mode != MODE_IDLE) enter_idle_mode();
      if (config_data.heater_allowed) {
         setLED(HEAT_POOL_LED, LED_ON);
         mode_message(NULLP); // "changing"
         setvalveconfig(VALVES_HEAT_POOL);
         pump_on(PUMP_POOL);
         pool_heater_mode();
         mode_timer = MODE_POOL_TIMEOUT;
         log_event(EV_HEAT_POOL);
         mode = MODE_HEAT_POOL;
         mode_message("heating pool"); }
      else {
         mode_message("heater disabled!");
         longdelay(2000);
         mode_message(" "); } } }

void spa_water_level_pushed (void) {
   byte button;

   if (mode == MODE_FILL_SPA || mode == MODE_EMPTY_SPA)   // stopping water level change
      enter_idle_mode();
   else { // starting water level change
      if (mode != MODE_IDLE) enter_idle_mode();
      if (config_data.heater_allowed) { // need heater plumbing path to fill or empty spa
         setLED(SPA_WATER_LEVEL_LED, LED_ON);
         lcd.clear(); lcd.print("press \x02 to fill spa"); // uparrow
         lcd.setCursor(0, 1); lcd.print("press \x01 to empty spa"); // downarrow
         lcd.setCursor(0, 2); lcd.print("any other cancels");
         #if DEBUG
         Serial.println(":: press up/down to fill/empty spa");
         #endif
         button = wait_for_button();
         mode_message(NULLP); // "changing"
         if (button == UPARROW_BUTTON) {
            setvalveconfig(VALVES_FILL_SPA);
            pump_on(PUMP_POOL);
            mode_timer = MODE_FILL_TIMEOUT;
            log_event(EV_FILL_SPA);
            mode = MODE_FILL_SPA;
            mode_message("filling spa"); }
         else if (button == DOWNARROW_BUTTON) {
            setvalveconfig(VALVES_EMPTY_SPA);
            pump_on(PUMP_SPA);
            mode_timer = MODE_EMPTY_TIMEOUT;
            mode = MODE_EMPTY_SPA;
            log_event(EV_EMPTY_SPA);
            mode_message("emptying spa"); }
         #if ALLOW_SPECIAL_TEST_MODE
         else if (button == PROGRAM_BUTTON)
            do_special_test();
         #endif
      }
      else {
         mode_message("heater disabled!");
         longdelay(2000); } } }

void filter_spa_pushed (void) {
   if (mode == MODE_FILTER_SPA)   // turning off spa filtering
      enter_idle_mode();
   else { // turning on spa filtering
      if (mode != MODE_IDLE) enter_idle_mode();
      setLED(FILTER_SPA_LED, LED_ON);
      mode_message(NULLP); // "changing"
      // If we can't use the heater, we must filter the spa in the "heat pool" configuration.
      // Otherwise we can use either the "heat pool" or "heat spa" configuration
      if (!config_data.heater_allowed || (valve_config != VALVES_HEAT_POOL && valve_config != VALVES_HEAT_SPA))
         setvalveconfig(VALVES_HEAT_POOL);
      pump_on(PUMP_SPA);
      mode_timer = config_data.filter_spa_mins;
      log_event(EV_FILTER_SPA);
      mode = MODE_FILTER_SPA;
      mode_message("filtering spa"); } }

void filter_pool_pushed (void) {
   if (mode == MODE_FILTER_POOL)   // turning off pool filtering
      enter_idle_mode();
   else { // turning on spa filtering
      if (mode != MODE_IDLE) enter_idle_mode();
      setLED(FILTER_POOL_LED, LED_ON);
      mode_message(NULLP); // "changing"
      // If we can't use the heater, we must filter the pool in the "heat spa" configuration.
      // Otherwise we can use either the "heat pool" or "heat spa" configuration
      if (!config_data.heater_allowed || (valve_config != VALVES_HEAT_POOL && valve_config != VALVES_HEAT_SPA))
         setvalveconfig(VALVES_HEAT_SPA);
      pump_on(PUMP_POOL);
      mode_timer = config_data.filter_pool_mins;
      mode = MODE_FILTER_POOL;
      log_event(EV_FILTER_POOL);
      mode_message("filtering pool"); } }

void spa_jets_pushed (void) {
   if (spa_jets_on) {  // turning off
      setLED(SPA_JETS_LED, LED_OFF);
      digitalWrite(SPA_JETS_PUMP_RELAY, RELAY_OFF);
      spa_jets_timer = 0;
      spa_jets_on = false; }
   else { // turning on
      setLED(SPA_JETS_LED, LED_ON);
      digitalWrite(SPA_JETS_PUMP_RELAY, RELAY_ON);
      spa_jets_timer = SPA_JETS_TIMEOUT;
      spa_jets_on = true; } }

void pool_light_pushed (void) {
   if (pool_light_on) {  // turning off
      setLED(POOL_LIGHT_LED, LED_OFF);
      digitalWrite(POOL_LIGHT_RELAY, RELAY_OFF);
      light_timer = 0;
      pool_light_on = false; }
   else { // turning on
      setLED(POOL_LIGHT_LED, LED_ON);
      digitalWrite(POOL_LIGHT_RELAY, RELAY_ON);
      light_timer = POOL_LIGHT_TIMEOUT;
      pool_light_on = true; } }

//-------------------------------------------------------
//  EEPROM configuration routines
//-------------------------------------------------------

bool config_changed;

void write_config (void) {
   #if DEBUG
   Serial.println("writing config...");
   #endif
   eeprom_write(CONFIG_HDR_LOC, sizeof(config_data), (byte *) &config_data); }

void init_config(void) {
   char hdr_id[6];
   eeprom_read(CONFIG_HDR_LOC, sizeof(hdr_id), (byte *) hdr_id);
   if (memcmp(hdr_id, config_data.hdr_id, sizeof(hdr_id)) != 0) {//check for matching id string
      write_config(); // not there: write our default config
      log_event(EV_INIT_CONFIG); // logging an event forces writing of log header
      center_message(2, "initialized config"); }
   else {
      // header id seems ok; read the whole config block and log header
      eeprom_read(CONFIG_HDR_LOC, sizeof(config_data), (byte *) &config_data);
      eeprom_read(LOG_HDR_LOC, sizeof(eventlog_hdr), (byte *) &eventlog_hdr);
      center_messagef(2, "%d of %d events", eventlog_hdr.count, LOG_SIZE); }
   delay(1000); }

//-------------------------------------------------------
// Configuration programming
//-------------------------------------------------------

// define the position of configurable fields in each programming message
// rightmost character of the field, 0-origin

byte config_time_columns[] = { // if setting date and time:
   1, 5, 10, 13, 16, 19, 0xff }; // day, month, year, hour, minute, am/pm

byte config_filterhour_columns [] = { // if setting hour to start filtering
   7 + 1, 7 + 4, 0xff }; // hour, am/pm

byte config_poolfiltertime_columns [] = { // if setting pool filtering time
   4 + 2, 0xff }; // minutes

byte config_spafiltertime_columns [] = { // if setting spa filtering time
   4 + 2, 0xff }; // minutes

byte config_heater_enable_columns [] = { // if setting heater enable disable
   11, 0xff }; // enable/disable

int8_t get_config_changes (byte columns[], byte * field) {
   // process arrow keys and return field value change (+1, -1), or 0 to stop
   while (1) {
      lcd.setCursor(columns[*field], CONFIG_ROW);
      lcd.blink();
      switch (wait_for_button()) {  // wait for a button push
         case PROGRAM_BUTTON:
            lcd.noBlink();
            return 0;  // done
         case UPARROW_BUTTON:
            config_changed = true;
            return +1;  // return "increment"
         case DOWNARROW_BUTTON:
            config_changed = true;
            return -1;  // return "decrement"
         case RIGHTARROW_BUTTON:
            if (columns[++*field] == 0xff) *field = 0;
            break;
         case LEFTARROW_BUTTON:
            if (*field == 0)
               while (columns[++*field] != 0xff) ;
            --*field;
            break;
         default: ; // ignore all other buttons
      } } }

// adjust a field up or down, within specified bounds
byte bound (byte value, int8_t delta /* only +-1 */ , byte min, byte max) {
   if (value <= min && delta < 0) return value;
   if (value >= max && delta > 0) return value;
   return value + delta; }

void show_event(int index) {
   char string[25];
   int num; // figure out what event number this is
   if (eventlog_hdr.count < LOG_SIZE) num = index + 1;
   else if ((num = index - eventlog_hdr.next + 1) <= 0) num += LOG_SIZE;
   center_messagef(0, "event %d of %d", num, eventlog_hdr.count);
   eeprom_read(LOG_LOC + index * sizeof(logentry), sizeof(logentry), (byte *) &logentry);
   show_time(1, &logentry.timestamp);
   center_message(2, event_names[logentry.event_type]);
   memset(string, 0, sizeof(string)); // in case the message isn't zero-terminated
   if (logentry.event_msg) memcpy(string, logentry.event_msg, LOG_MSGSIZE);
   center_message(3, string); }

void show_eventlog(void) { //******** display the event log
   char string[25];
   int index, index_newest, index_oldest;

   if (eventlog_hdr.count == 0) return; // skip if the log is empty
   if ((index_newest = eventlog_hdr.next - 1) < 0) index_newest = LOG_SIZE - 1;
   index_oldest = (eventlog_hdr.count < LOG_SIZE ? 0 : eventlog_hdr.next);
   index = -1; // haven't started yet
   while (1) {
      switch (wait_for_button()) {
         case PROGRAM_BUTTON:
            return;  // done
         case RIGHTARROW_BUTTON: // go forward in time
            if (index < 0) index = index_oldest; // start with oldest
            else if (index != index_newest && ++index >= LOG_SIZE) index = 0; // otherwise do next newer
            show_event(index);
            break;
         case LEFTARROW_BUTTON: // go backward in time
            if (index < 0) index = index_newest; // start with newest
            else if (index != index_oldest && --index < 0) index = LOG_SIZE - 1; // otherwise do next older
            show_event(index);
            break;
         default: ;// ignore all other buttons
      } } }

void set_time(void) {  //********* change the current date and time
   int8_t delta;
   byte field;

   rtc_read(&now);
   field = 0; // start with first field
   while (true) {
      show_time(CONFIG_ROW, &now); // show it in row 1
      delta = get_config_changes(config_time_columns, &field);
      if (delta == 0) break;
      switch (field) {
         case 0: // day
            now.date = bound (now.date, delta, 1, days_in_month[now.month]);
            break;
         case 1: // month
            now.month = bound (now.month, delta, 1, 12);
            if (now.date > days_in_month[now.month])
               now.date = days_in_month[now.month];
            break;
         case 2: // year
            now.year = bound (now.year, delta, 0, 99);
            break;
         case 3:  // hour
            now.hour = bound (now.hour, delta, 1, 12);
            break;
         case 4:  // minutes
            now.min = bound (now.min, delta, 0, 59);
            break;
         case 5: // am/pm switch
            now.ampm = now.ampm ^ 1;  // just reverse
            break; } }
   rtc_write(&now);  // write it into the realtime clock
}

void set_filter_hour (void) {  //****** change when filtering starts each day
   int8_t delta;
   byte field;

   field = 0; // start with first field
   while (true) {
      center_messagef(CONFIG_ROW, "%2d %s", config_data.filter_start_hour,
                      config_data.filter_start_ampm ? "PM" : "AM"); // " 3 pm"
      // get change to hour or am/pm
      delta = get_config_changes(config_filterhour_columns, &field);
      if (delta == 0) break;
      switch (field) {
         case 0:  // hour
            config_data.filter_start_hour = bound (config_data.filter_start_hour, delta, 1, 12);
            break;
         case 1: // am/pm switch
            config_data.filter_start_ampm = config_data.filter_start_ampm ^ 1;  // just reverse
            break; } } }

void set_pool_filter_time (void) {  //******** change how long to filter the pool
   int8_t delta;
   byte field;

   field = 0; // start with first (and only) field
   while (true) {
      center_messagef(CONFIG_ROW, "%3d minutes", config_data.filter_pool_mins);
      delta = get_config_changes(config_poolfiltertime_columns, &field);
      if (delta == 0) break;
      switch (field) {
         case 0:  // minutes from 5 to 120
            config_data.filter_pool_mins = bound (config_data.filter_pool_mins, delta, 5, 120);
            break; } } };

void set_spa_filter_time (void) {  //********* change how long to filter the spa
   int8_t delta;
   byte field;

   field = 0; // start with first (and only) field
   while (true) {
      center_messagef(CONFIG_ROW, "%2d minutes", config_data.filter_spa_mins);
      delta = get_config_changes(config_spafiltertime_columns, &field);
      if (delta == 0) break;
      switch (field) {
         case 0:  // minutes from 5 to 60
            config_data.filter_spa_mins = bound (config_data.filter_spa_mins, delta, 5, 60);
            break; } } };

void enable_heater(void) { //********* enable or disable the use of the heater
   int8_t delta;
   byte field;

   field = 0; // start with first (and only) field
   while (true) {
      center_messagef(CONFIG_ROW, "heater is %s", config_data.heater_allowed ? "enabled" : "disabled");
      delta = get_config_changes(config_heater_enable_columns, &field);
      if (delta == 0) break;
      config_data.heater_allowed ^= 1; // reverse
   } }

const static struct  {  // configuration programming action routines
   const char *title;
   void (*fct)(void); }
#define NUM_CONFIGS 6
config_cmds [] = {
   {"show event log", show_eventlog },
   {"set time", set_time },
   {"set filter hour", set_filter_hour },
   {"set pool filter time", set_pool_filter_time },
   {"set spa filter time", set_spa_filter_time },
   {"enable heater", enable_heater } };

void program_pushed (void) {
   setLED(PROGRAM_LED, LED_ON);
   enter_idle_mode();   // stop everything
   config_changed = false;
   for (byte cmd = 0; cmd < NUM_CONFIGS; ++cmd) { // do all config settings
      lcd.clear();
      lcd.print(config_cmds[cmd].title); // show instruction on top line
      lcd.setCursor(0, 2);
      lcd.print("Arrows view/change");
      lcd.setCursor(0, 3);
      lcd.print("Then press \"program\"");
      #if DEBUG
      Serial.print(":: "); Serial.println(config_cmds[cmd].title);
      #endif
      (config_cmds[cmd].fct)(); // execute the configuration routine
   }
   lcd.clear();
   if (config_changed) {
      write_config();  // write configuration into EPROM
      log_event(EV_UPDATED_CONFIG);
      center_message(0, "changes recorded");
      longdelay(1500); }
   setLED(PROGRAM_LED, LED_OFF); }

//-------------------------------------------------------
//  Initialization
//-------------------------------------------------------

void start_timer(void) {  // start a timer that interrupts every second
   TCCR1A = 0;  // mode 0100: CTC
   TCCR1B = (1 << WGM12) | (1 << CS10) | (1 << CS12); // clock select 101: clk/1024 prescaling
   OCR1A = F_CPU / 1024; // (freq in MHz)
   TIMSK1 = (1 << OCIE1A); // turn on timer 1 interrupts
   interrupts();  // enable interrupts
}

void outpin(byte pinnum, byte starting_state) {  // configure an output pin
   digitalWrite(pinnum, starting_state);
   pinMode(pinnum, OUTPUT); }

void inpin(byte pinnum) {  // configure an input pin
   pinMode(pinnum, INPUT);
   digitalWrite(pinnum, HIGH); // turn pullup resistor on
}

void setup (void) {  // initialization starts here

   static uint8_t downarrow_char[8] = {
      B00000,
      B00100,
      B00100,
      B00100,
      B10101,
      B01110,
      B00100,
      B00000 };
   static uint8_t uparrow_char[8] = {
      B00100,
      B01110,
      B10101,
      B00100,
      B00100,
      B00100,
      B00100,
      B00000 };

   bool watchdog_triggered = watchdog_setup();  // start the watchdog timer

   #if DEBUG
   Serial.begin(57600);
   while (!Serial) ;
   Serial.println("Debugging...");
   #if SHOWMEM
   Ram_TableDisplay();
   #endif
   #endif

   lcd.begin(20, 4); // start LCD display
   lcd.createChar(0x01, downarrow_char);
   lcd.createChar(0x02, uparrow_char);
   lcd.noCursor();
   center_messagef(0, "initializing v%s", VERSION);

   //lighted pushbuttons and LEDs
   for (byte i = 0; i < NUM_BUTTONS; ++i)
      inpin(button_pins[i]);
   outpin(LED_DRIVER_SDI, LOW);
   outpin(LED_DRIVER_CLK, LOW);
   outpin(LED_DRIVER_LE, LOW);
   outpin(LED_DRIVER_OD, LOW);
   setLED(0, LED_OFF);  // turn off all LEDs

   // relays
   outpin(POOL_PUMP_RELAY, RELAY_OFF);
   outpin(SPA_PUMP_RELAY, RELAY_OFF);
   outpin(SPA_JETS_PUMP_RELAY, RELAY_OFF);
   outpin(HEAT_SPA_RELAY, RELAY_OFF);
   outpin(HEAT_POOL_RELAY, RELAY_OFF);
   outpin(POOL_VALVE_RELAY, RELAY_OFF);
   outpin(SPA_VALVE_RELAY, RELAY_OFF);
   outpin(HEATER_VALVE_RELAY, RELAY_OFF);
   outpin(POOL_LIGHT_RELAY, RELAY_OFF);
   outpin(SPARE_RELAY, RELAY_OFF);
   inpin(TESTPIN);

   init_config();  // get or set configuration data from EEPROM
   start_timer();  // start timer interrupts
   Wire.begin();   // start onewire for temperature sensor access

   // realtime clock
   rtc_read(&now);  // check for valid realtime clock data
   if (rtc_invalid()) {
      rtc_write (&clock_init); // reset if bad
      if (rtc_invalid()) {// if still invalid, it's broken or not present
         no_clock = true; } }

   // temperature sensor
   outpin(TEMPSENSOR_PIN, HIGH);
   delay(250); // wait for parasitic power capacitor to charge?
   have_tempsensor = tempsensor.search(tempsensor_addr)  // find temp sensor
                     && (OneWire::crc8(tempsensor_addr, 7) == tempsensor_addr[7]); // with ok CRC
   if (have_tempsensor) {
      const byte set_10_bit_resolution[4] = {
         0x4e, 0, 0, 0x20 };   // use 10-bit (Centigrate/16) resolution
      tempsensor.reset();
      tempsensor.select(tempsensor_addr);
      tempsensor.write_bytes(set_10_bit_resolution, 4, 1); }

   // rotary encoder for temperature control
   inpin(TEMPCTL_INPUT_A);
   inpin(TEMPCTL_INPUT_B);
   PCMSK0 |= 0x03;  // enable PB1 (PCINT1) and PB0 (PCINT0) pin change interrupts
   PCICR |= (1 << PCIE0); // enable pin change interrupts PCINT7-PCINT0

   log_event( // make an initial "power on" event log entry
      watchdog_triggered ? EV_WATCHDOG_RESET :  EV_STARTUP);
   if (watchdog_triggered) {
      center_message(2, "Watchdog triggered!");
      delay(1000); }

   if (no_clock) {
      log_event(EV_CLOCK_BAD);
      center_message(2, "no clock!");
      delay(1000); }

   setvalveconfig(VALVES_HEAT_SPA); // put the valves into a known state
}

//-------------------------------------------------------
//  Interrupt routines
//-------------------------------------------------------

ISR(TIMER1_COMPA_vect) {  // TIMER 1 interrupt routine, once a second
   // Don't do anything heavy-duty here, and beware of race conditions for
   // multi-byte variables that might be accessed by the background process.

   static byte minute_timer = 60;  // prescale seconds into minutes

   if (++title_timer == TITLE_LINE_TIME)  // changing title line
      do_title2 = true;
   else if (title_timer == 2 * TITLE_LINE_TIME) {
      do_title1 = true;
      title_timer = 0; }

   if (heater_cooldown_secs_left) --heater_cooldown_secs_left;

   if (minute_timer) --minute_timer;
   else {                          // countdown minutes
      minute_timer = 60;
      if (mode_timer) --mode_timer;
      if (spa_jets_timer) --spa_jets_timer;
      if (light_timer) --light_timer; } }

ISR(PCINT0_vect) {  // pin change interrupts PCINT7 to PCINT 0, for rotary encoder for temperature
   static const int8_t encoder_states [] = { //encoder lookup table
      // a clever encoding scheme from Oleg Mazurov
      0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 };
   static byte old_AB = 3; // last value of the encoder bits
   static int8_t count_clicks = 0;
   byte new_AB;
   int8_t direction;

   delay(3); // 3 msec debounce -- not such a great idea in an interrupt routine, but it works!
   old_AB <<= 2;  // shift old encoder bits over
   new_AB = TEMPCTL_READ & 0x03; // read new encoder bits
   old_AB = (old_AB | new_AB) & 0x0f; // create 4-bit index
   direction = encoder_states[old_AB];
   if (direction /*&& (new_AB == 3)*/) {  // [at detent and] transition is valid
      count_clicks += direction; // force more rotation for each change
#define ENCODER_CLICKS 3  // number of detent clicks per change we report
      if (count_clicks < -ENCODER_CLICKS || count_clicks > ENCODER_CLICKS) {
         count_clicks = 0;
         temp_change(direction); } } }

//-------------------------------------------------------
//  Main loop
//-------------------------------------------------------

// table of button action routines
static void (*button_actions[NUM_BUTTONS]) (void) = {
   heat_spa_pushed,
   heat_pool_pushed,
   spa_jets_pushed,
   pool_light_pushed,
   filter_spa_pushed,
   filter_pool_pushed,
   spa_water_level_pushed,
   program_pushed };

// Race-safe test of variables that are changed in the interrupt routine
// (This probably isn't necessary, since int decrements are most likely atomic.)
boolean if_zero (volatile unsigned int *counter) {
   boolean test;
   noInterrupts();
   test = *counter == 0;
   interrupts();
   return test; }

// The main polling loop for activities

void loop (void) {
   char string[25];
   unsigned int timer;
   byte temp_now, button;

   watchdog_poke();

   // Alternate the top-line title
   if (do_title1) {
      center_message(0, TITLE);
      do_title1 = false; }
   else if (do_title2) {
      show_current_time(0);
      do_title2 = false; }

   // Check for button pushes
   if ((button = check_for_button()) != 0xFF)
      (button_actions[button])(); // do the action routine

   // Check for the time to autostart daily filtering routine
   if (now.hour == config_data.filter_start_hour
         && now.ampm == config_data.filter_start_ampm
         && last_filter_date != now.date
         && mode == MODE_IDLE) {
      filter_autostarted = true;
      last_filter_date = now.date; // only once per day
      filter_pool_pushed();  // simulate pushing the "filter pool" buttton
   }

   // display time left in this mode
   noInterrupts();
   timer = mode_timer;  // get timer; watch for race conditions
   interrupts();
   if (timer) {   // display the current mode's "time left" message
      if (timer >= 60)
         center_messagef(2, "%d hr %d min left ", timer / 60, timer % 60);
      else center_messagef(2, "%d min left", timer % 60); }

   // check if this mode has timed out
   if (if_zero(&mode_timer) && mode != MODE_IDLE) { // timed out
      if (mode == MODE_FILTER_POOL && filter_autostarted)  // if it's "filter pool" autostarted
         filter_spa_pushed(); // then switch to "filter spa"
      else   // otherwise go into idle mode
         enter_idle_mode(); }

   // have the spa jets timed out?
   if (spa_jets_on && if_zero(&spa_jets_timer)) {
      setLED(SPA_JETS_LED, LED_OFF);
      digitalWrite(SPA_JETS_PUMP_RELAY, LOW);
      spa_jets_on = false; }

   // has the pool light timed out?
   if (pool_light_on && if_zero(&light_timer)) {
      setLED(POOL_LIGHT_LED, LED_OFF);
      digitalWrite(POOL_LIGHT_RELAY, LOW);
      pool_light_on = false; }

   // display the water temperature and turn the heater on or off

   if (pump_status != PUMP_NONE) { //pump is on
      if (have_tempsensor) {
         temp_now = read_temp();

         if (heater_mode == HEATING_NONE) {
            // We're not heating, but one of the pumps is running.
            // Display the temperature if water is flowing through the heater,
            // since the temp sensor is in the return inlet to the heater
            if (mode == MODE_FILTER_SPA && valve_config == VALVES_HEAT_POOL
                  || mode == MODE_FILTER_POOL && valve_config == VALVES_HEAT_SPA)
               sprintf(string, " ");
            else sprintf(string, "temperature: %dF", temp_now); }

         else {  // either HEATING_SPA or HEATING_POOL
            // (Do we need to enforce a minimum time between heater changes
            //  here, to avoid damage? Or can the heater cope?)
            if (heater_on) {
               if (temp_now >= target_temp) { // turn heater off
                  digitalWrite(HEAT_SPA_RELAY, RELAY_OFF);
                  digitalWrite(HEAT_POOL_RELAY, RELAY_OFF);
                  setLED(TEMPCTL_BLUE_LED, LED_ON);
                  setLED(TEMPCTL_RED_LED, LED_OFF);
                  heater_cooldown_secs_left = DELAY_HEATER_OFF;
                  heater_on = false; } }
#define TEMP_HYSTERESIS 2  // hysteresis in degrees fahrenheit
            else { // heater off
               if (temp_now <= target_temp - TEMP_HYSTERESIS) { // turn heater on
                  digitalWrite(heater_mode == HEATING_SPA ? HEAT_SPA_RELAY : HEAT_POOL_RELAY, RELAY_ON);
                  setLED(TEMPCTL_BLUE_LED, LED_OFF);
                  setLED(TEMPCTL_RED_LED, LED_ON);
                  heater_on = true; } }
            center_messagef(3, "temp set %d, is %d", target_temp, temp_now); } }
      else center_message(3, "no temp sensor!"); }

} // repeat loop

//*

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
    - an Adafruit HUZZAH32 ESP32 Feather Wifi microcontroller
      https://learn.adafruit.com/adafruit-huzzah32-esp32-feather
    - a 4-line by 20 character I2C LCD display
    - 8 waterproof lighted pushbuttons
    - a rotary encoder with 3 lights for temperature control
    - a Maxim DS18B20 thermometer
    - a Maxim DS3231 realtime clock with battery backup
    - 10 relays for controlling pumps, heater, valves, and lights

   For more information including photos and a schematic, see
   https://github.com/LenShustek/PoolSpaController

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
// 27 Dec 2021, V3.0, L. Shustek
//               - Major change for version 3.0 hardware using the ESP32 WiFi microcontroller
//                 instead of the Arduino MEGA 2560. Add heater simulator for testing.
//                 Add temperature history.
//
//---------------------------------------------------------------------------------------------

#define VERSION "3.0"
#define TITLE "Saw Mill Lodge"

#define DEBUG true
#define DEBUG_TIMES true
#define DEBUG_LCD false // show LCD output in debug window?
#define WATCHDOG true
#define WEBSERVER true
#define ALLOW_SPECIAL_TEST_MODE false
#define ROTARY_ENCODER true

/* The 4MB (0x400000) of FLASH memory in the ESP32 is divided into partitions that are
   defined by the "partitions.csv" file in the sketch directory. We add two partitions
   at the end from space taken out of the default "spiffs" filesystem partition:
      "log", a 60K area for holding the event log
      "config", a 4K area for holding the configuration information
   Our partitions.csv file is thus:
      # Name,   Type, SubType, Offset,   Size,      Flags
      nvs,      data, nvs,     0x9000,   0x5000,
      otadata,  data, ota,     0xe000,   0x2000,
      app0,     app,  ota_0,   0x10000,  0x140000,
      app1,     app,  ota_1,   0x150000, 0x140000,
      spiffs,   data, spiffs,  0x290000, 0x160000,
      log,      0x4D, 0x00,    0x3f0000, 0xf000,
      config,   0x4E, 0x00,    0x3ff000, 0x1000,
*/

#include <Wire.h>                   // for I2C devices
#include <Adafruit_LiquidCrystal.h> // for LCD display
#include <OneWire.h>                // for temperature sensor
#include "controller_03.h"          // our stuff
#include "esp32_flashlogs.h"        // logging routines

#define NULLP ((char *)0)       // null pointer

// pushbuttons

const byte button_masks[NUM_BUTTONS] = { // button masks for ADC28 mux
   HEAT_SPA_BUTTON_MASK, HEAT_POOL_BUTTON_MASK, SPA_JETS_BUTTON_MASK, POOL_LIGHT_BUTTON_MASK,
   FILTER_SPA_BUTTON_MASK, FILTER_POOL_BUTTON_MASK, SPA_WATER_LEVEL_BUTTON_MASK, MENU_BUTTON_MASK };

enum button_indexes { // define button indices, 0..7
   HEAT_SPA_BUTTON, HEAT_POOL_BUTTON, SPA_JETS_BUTTON, POOL_LIGHT_BUTTON,
   FILTER_SPA_BUTTON, FILTER_POOL_BUTTON, SPA_WATER_LEVEL_BUTTON, MENU_BUTTON };
#define  LEFTARROW_BUTTON  HEAT_SPA_BUTTON
#define  RIGHTARROW_BUTTON  HEAT_POOL_BUTTON
#define  DOWNARROW_BUTTON  SPA_JETS_BUTTON
#define  UPARROW_BUTTON  POOL_LIGHT_BUTTON

// State information

enum global_mode_t mode = MODE_IDLE;           // current global mode
enum heater_t heater_mode = HEATING_NONE;      // current heater setting: none, pool, spa
boolean heater_on = false;                     // is the heater currently on?
volatile byte heater_cooldown_secs_left = 0;   // cooldown seconds left after heater off
enum vconfig_t valve_config = VALVES_UNDEFINED;// current valve configuration
enum pump_status_t pump_status = PUMP_NONE;    // current status of pumps
byte target_temp = 102;                        // target temperature in degrees F
byte simulated_temp = 72;                      // simulated temperature in F if we have no temp sensor
boolean button_awaiting_release[NUM_BUTTONS] = {
   false };                                     // button pushed but awaiting release?
bool button_webpushed[NUM_BUTTONS] = {false }; // buttons with pending "pushes" from the web
uint16_t leds_on = 0;                           // mask for which LEDs are currently on
uint16_t led_masks[NUM_LEDS] = {                // mask for individual LEDs
   HEAT_SPA_LED, HEAT_POOL_LED, SPA_JETS_LED, POOL_LIGHT_LED,
   FILTER_SPA_LED, FILTER_POOL_LED, SPA_WATER_LEVEL_LED, MENU_LED,
   TEMPCTL_RED_LED, TEMPCTL_GREEN_LED, TEMPCTL_BLUE_LED };
boolean pool_light_on = false;
boolean spa_jets_on = false;
boolean have_tempsensor = false;
boolean no_clock = false;
BaseType_t cpu_core;                      // which CPU core we're running on

volatile boolean do_title1 = true;
volatile boolean do_title2 = false;
volatile byte title_timer = 0;            // count seconds for titles

volatile unsigned int mode_timer = 0;     // minutes left in the current mode
volatile unsigned int spa_jets_timer = 0; // minutes left to aerator shutoff
volatile unsigned int light_timer = 0;    // minutes left to light shutoff
boolean filter_autostarted = false;       // did we autostart filtering?
byte last_filter_date = 0xff;             // what date we last did filtering


struct datetime now,
          clock_init = {
   50, 10, 8, 1, 3, 21, 1, 14 }; // (when we first wrote the code)

static const char *months[] = { // index 1..12 from realtime clock
   "???", "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};
static const byte days_in_month []  = {
   99, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

OneWire tempsensor(TEMPSENSOR_PIN); // Maxim DS18B20 temperature sensor
byte tempsensor_addr[8];  // its discovered address
byte temp_now;            // the most recently read temperature
bool temp_valid = false;  // if the temp is valid: pump running water through heater


//****  map of non-volatile storage for configuration info

struct { // a local copy of the configuration data
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
   "SML04", FILTER_POOL_TIME, FILTER_SPA_TIME, FILTER_START_HOUR, FILTER_START_AMPM, true};

#define LOG_DATASIZE 60 // total size of a log entry: must be 4 less than a power of 2
#define LOG_MSGSIZE (LOG_DATASIZE-8-2) // the remainder not including time stamp and event type
struct logentry_t {  // the log entries
   struct datetime timestamp;
   uint16_t event_type;
   char event_msg[LOG_MSGSIZE]; }; // optional message, maybe NOT 0-terminated

struct flashlog_state_t log_state;

enum event_t {    // event types
   EV_BAD,              // (to catch uninitialized log entries)
   EV_STARTUP,          // we started from power up
   EV_WATCHDOG_RESET,   // we were reset by watchdog timer
   EV_ASSERTION_FAILED, // assertion failed
   EV_CLOCK_BAD,        // can't find realtime clock
   EV_TEMPSENSOR_BAD,   // can't find temperature sensor
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
   "power on restart", "watchdog restart", "assertion failed", "clock bad", "tempsensor bad",
   "init config", "updated config",
   "idle", "heat spa", "heat pool", "fill spa", "empty spa", "filter pool", "filter spa" };
typedef char event_name_error[sizeof(event_names) / sizeof(event_names[0]) == EV_NUM_EVENTS ? 1 : -1]; // compiler check

void watchdog_poke(void);

//-----------------------------------------------------------------
// event log routines
//-----------------------------------------------------------------

void log_event(enum event_t event_type, const char *msg) {
   dprint("log event: %s %s\n",
          event_names[event_type],
          msg ? msg : "");
   struct logentry_t *plog = (struct logentry_t *) log_state.logdata;
   plog->timestamp = now;
   plog->event_type = event_type;
   if (msg) strncpy(plog->event_msg, msg, LOG_MSGSIZE);
   else memset(plog->event_msg, 0, LOG_MSGSIZE);
   assert_that(flashlog_add(&log_state) == FLASHLOG_ERR_OK, "can't add log entry"); }

void log_event(enum event_t event_type) {
   log_event(event_type, NULLP); }

void log_dump(void * parm, void (*print)(void * parm, const char *line)) {
   if (log_state.numinuse == 0)
      print(parm, "log empty\n");
   else {
      char buf[200];
      snprintf(buf, sizeof(buf), "%d of %d entries", log_state.numinuse, log_state.numslots);
      print(parm, buf);
      flashlog_goto_newest(&log_state);
      do {
         assert_that(flashlog_read (&log_state) == FLASHLOG_ERR_OK,
                     "can't read log entry");
         struct logentry_t *plog = (struct logentry_t *) log_state.logdata;
         if (datetime_invalid(plog->timestamp))
            snprintf(buf, sizeof(buf), " -- --- 20-- --:-- -- ");
         else snprintf(buf, sizeof(buf), " %2d %s 20%02d %2d:%02d %s ",
                          plog->timestamp.date, months[plog->timestamp.month], plog->timestamp.year,
                          plog->timestamp.hour, plog->timestamp.min, plog->timestamp.ampm ? "PM" : "AM");
         int sofar = strlen(buf);
         snprintf(buf + sofar, sizeof(buf) - sofar, " %s %.17s", event_names[plog->event_type], plog->event_msg);
         print(parm, buf); }
      while (flashlog_goto_prev(&log_state) == FLASHLOG_ERR_OK); } }

//-----------------------------------------------------------------------
//    LCD display routines
//-----------------------------------------------------------------------
Adafruit_LiquidCrystal lcdhw(0);  // LCD_DISPLAY, 0x20
#define CONFIG_ROW 1            // use second row for configuration strings
// We keep a software simulation of the LCD for web access.
char lcdbuf[4][21]; // our simulated LCD buffer, with 0 string terminators
int lcdrow /* 0..3 */, lcdcol /* 0..19 */;
bool lcd_cursorblinking = false;

void lcd_start(void) {
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

   lcdhw.begin(20, 4); // start LCD display
   lcdhw.createChar(DOWNARROW[0], downarrow_char);
   lcdhw.createChar(UPARROW[0], uparrow_char);
   lcdhw.noCursor(); }

void lcd_restart(void) {
   lcd_start(); // restart the hardware
   for (int row = 0; row < 4; ++row) // copy from our internal buffer to the display
      for (int col = 0; col < 20; ++col) {
         lcdhw.setCursor(col, row);
         lcdhw.print(lcdbuf[row][col]); }
   lcdhw.setCursor(lcdcol, lcdrow); }

void lcdsetrow( byte row) {
   assert_that(row <= 3, "bad lcdsetrow");
   lcdhw.setCursor(0, row);
   lcdrow = row; lcdcol = 0; }

void lcdsetCursor(byte col, byte row) {
   assert_that(row <= 3 && col <= 19, "bad lcdsetcursor");
   lcdhw.setCursor(col, row);
   lcdcol = col; lcdrow = row; }

void lcdclear(void) {
   lcdhw.clear();
   memset(lcdbuf, ' ', sizeof(lcdbuf)); // blank the buffer
   for (int row = 0; row < 4; ++row) lcdbuf[row][20] = 0; // insert string terminators
   lcdrow = lcdcol = 0; }

void lcdblink(void) {
   lcd_cursorblinking = true;
   lcdhw.blink(); }

void lcdnoBlink(void) {
   lcd_cursorblinking = false;
   lcdhw.noBlink(); }

void lcdprint(char ch) {
   assert_that(lcdcol < 20 && lcdrow < 4, "bad lcdprint ch");
   lcdhw.print(ch);
   /* if (lcdcol < 20)*/ lcdbuf[lcdrow][lcdcol] = ch;
   if (lcdcol < 19) ++lcdcol; }

void lcdprint(const char *msg) {
   // if not on last row, allow message to overflow onto a second line
   int length = strlen(msg);
   int fits = 20 - lcdcol; // how much fits on the first row
   if (lcdrow < 3 && length > fits && length <= fits + 20) {
      char firstline[21];
      strncpy(firstline, msg, fits);
      firstline[fits] = 0;
      lcdprint(firstline); // recursively do first part
      lcdsetCursor(0, lcdrow + 1); // move to start of next row
      lcdprint(msg  + fits); } // recursively do second line
   else {
      assert_that(length <= 20 - lcdcol, "bad lcdprint");
      lcdhw.print(msg);
      for (int ndx = 0; length--; ++ndx ) {
         if (lcdcol < 20) lcdbuf[lcdrow][lcdcol] = msg[ndx];
         if (lcdcol < 19) ++lcdcol; } } }

void lcdprint(byte row, const char *msg) {
   lcdsetrow(row);
   lcdprint(msg); }

void lcdprintf(byte row, const char *msg, ...) {
   char buf[40];
   va_list arg_ptr;
   va_start(arg_ptr, msg);
   vsnprintf(buf, sizeof(buf), msg, arg_ptr);
   lcdprint(row, buf);
   va_end(arg_ptr); }

//-------------------------------------------------------
// Utility routines
//-------------------------------------------------------

void dprint(const char *format, ...) {
   #if DEBUG
   char buf[200];
   va_list argptr;
   va_start(argptr, format);
   vsnprintf(buf, sizeof(buf), format, argptr);
   Serial.print(buf);
   va_end(argptr);
   #endif
}

void assert_that(boolean test, const char *msg, ...) {
   // N.B.: "assert()" is a macro somewhere else in the ESP32 ecosystem.
   if (!test) {
      char buf[50];
      va_list argptr;
      va_start(argptr, msg);
      vsnprintf(buf, sizeof(buf), msg, argptr);
      #if DEBUG
      Serial.print("failed assertion : "); Serial.println(buf);
      #endif
      log_event(EV_ASSERTION_FAILED, buf);
      lcdclear(); lcdprint("** INTERNAL ERROR **");
      lcdsetCursor(0, 1); lcdprint("Assertion failed : ");
      lcdsetCursor(0, 2); lcdprint(buf);
      while (true) ; // wait for watchdog to reset us
   } }

//-----------------------------------------------------------------------------------------
//  message routines
//-----------------------------------------------------------------------------------------

void center_message (byte row, const char *msg) {
   int len = strlen(msg);
   assert_that(len <= 20 && row < 4, "bad center_message");
   #if DEBUG && DEBUG_LCD
   Serial.print(":: "); Serial.println(msg);
   #endif
   int nblanks = (20 - len) >> 1;
   lcdsetCursor(0, row);
   for (byte i = 0; i < nblanks; ++i) lcdprint(" ");
   lcdprint(msg);
   nblanks = (20 - nblanks) - len;
   for (byte i = 0; i < nblanks; ++i) lcdprint(" "); }

void center_messagef (byte row, const char *msg, ...) {
   char buf[40];
   va_list arg_ptr;
   va_start(arg_ptr, msg);
   vsnprintf(buf, sizeof(buf), msg, arg_ptr);
   center_message(row, buf);
   va_end(arg_ptr); }

void mode_message (const char *msg) {
   lcdclear();
   if (msg != NULLP) {  // new mode starting
      center_message(1, msg);
      do_title1 = true;  // restart top title
      do_title2 = false;
      title_timer = 0; }
   else {  // changing mode
      center_message(0, "changing mode"); } }

//------------------------------------------------------------------------------
//    temperature history routines
//------------------------------------------------------------------------------

struct temphist_t temphist[TEMPHIST_ENTRIES];
int temphist_count = 0;
int temphist_next = 0;
int temphist_minute_count = 0;

void temphistory_add(void) { // this is called once a minute from the interrupt routine
   if (temp_valid && ++temphist_minute_count >= TEMPHIST_DELTA_MINS) {
      temphist_minute_count = 0;
      temphist[temphist_next].timestamp = now;
      temphist[temphist_next].temphist_temp = temp_now;
      if (temphist_count < TEMPHIST_ENTRIES) ++temphist_count;
      if (++temphist_next >= TEMPHIST_ENTRIES) temphist_next = 0; } }

void temphistory_dump(void * parm, void (*print)(void * parm, const char *line)) {
   if (temphist_count > 0) {
      int ndx = temphist_next - temphist_count;
      if (ndx < 0) ndx += TEMPHIST_ENTRIES;
      for (int cnt = 0; cnt < temphist_count; ++cnt) {
         char str[100];
         int hour = temphist[ndx].timestamp.hour;
         if (temphist[ndx].timestamp.ampm == 0 ) { // AM
            if (hour == 12) hour = 0; }
         else { // PM
            if (hour < 12) hour += 12; }
         snprintf(str, sizeof(str), "%4d-%02d-%02d %02d:%02d:%02d, %d",
                  temphist[ndx].timestamp.year + 2000, temphist[ndx].timestamp.month, temphist[ndx].timestamp.date,
                  hour, temphist[ndx].timestamp.min, temphist[ndx].timestamp.sec,
                  temphist[ndx].temphist_temp);
         print(parm, str);
         if (++ndx >= TEMPHIST_ENTRIES) ndx = 0; } }
   else print(parm, "no temperature history"); }

void temphistory_dprint(void *parm, const char *msg) {
   Serial.println(msg); }

//------------------------------------------------------------------------------
//    light/button/relay routines
//------------------------------------------------------------------------------

void setLED(uint16_t mask, uint16_t onoff) {  // turn LEDs on or off
   uint16_t bits;
   leds_on = (leds_on & ~mask) | (mask & onoff); // copy new bit(s) to merged mask
   assert_that(leds_on != 0xffff, "err in setLED");
   bits = leds_on;
   for (byte i = 0; i < 16; ++i) { //shift out all bits to the STP16CP05 driver
      digitalWrite(LED_DRIVER_SDI, bits & 0x8000 ? HIGH : LOW);
      digitalWrite(LED_DRIVER_CLK, HIGH); // create clock pulse to shift one bit
      bits <<= 1; // shift to next, and delay for pulse width
      digitalWrite(LED_DRIVER_CLK, LOW); }
   digitalWrite(LED_DRIVER_LE, HIGH); // all done: create latch pulse
   delayMicroseconds(2);
   digitalWrite(LED_DRIVER_LE, LOW); }

static bool doing_light_button_tests = false;
void do_light_button_tests(void);

byte check_for_button (void) {  // check for a button push, return button or 0xff if none
   byte button;
   watchdog_poke(); // a good place to reset the watchdog timer
   for (button = 0; button < NUM_BUTTONS; ++button) {
      Wire.beginTransmission(PUSHBUTTONS); // configure the ADG728 analog mux to read the button
      Wire.write(button_masks[button]);
      Wire.endTransmission();
      if (digitalRead(PUSHBUTTON_IN)) {  // button is released
         if (button_awaiting_release[button]) {
            button_awaiting_release[button] = false;
            delay (DEBOUNCE_DELAY); } }
      else { // button is pushed
         if (!button_awaiting_release[button]) { // not already acted on
            delay (DEBOUNCE_DELAY);
            button_awaiting_release[button] = true; // setup to await release later
            return button; // return this button
         } }
      if (button_webpushed[button]) { // if we gueued a button "push" from the web
         dprint("web client pushed button %d\n", button);
         button_webpushed[button] = false;
         return button; } }
   #if DEBUG  // check for characters pressed on the serial monitor keyboard
   if (Serial.available()) {
      char ch = Serial.read();
      if (ch >= '1' && ch <= '8')  // '1' to '8' are simulated button pushes
         return ch - '1';
      if (ch == 'd')               // 'd': dump the temperature history
         temphistory_dump(NULL, &temphistory_dprint);
      else if (ch == 't') {        // 't': start/stop light and button test
         if (!doing_light_button_tests) {
            doing_light_button_tests = true;
            do_light_button_tests(); }
         else {
            doing_light_button_tests = false;
            lcdclear(); } } }
   #endif
   return 0xff; }

byte wait_for_button (void) { // wait for a button to be pushed
   byte button;
   while ((button = check_for_button()) == 0xff);
   return button; }

bool menu_button_pushed;
bool yesno(byte row, bool return_if_menu_button, const char *msg) {
   assert_that(strlen(msg) <= 20, "bad msg in yesno", strlen(msg));
   center_message(row, msg);
   assert_that(row < 3, "bad row in yesno", row);
   center_message(row + 1, UPARROW " if yes, " DOWNARROW " if no");
   byte button;
   do {
      button = wait_for_button();
      menu_button_pushed = button == MENU_BUTTON; }
   while (button != UPARROW_BUTTON && button != DOWNARROW_BUTTON
          && (!return_if_menu_button || !menu_button_pushed));
   return button == UPARROW_BUTTON; }

void setrelay(uint16_t relay_mask, bool whichway) {
   static uint16_t relay_status = 0;  // which relays are on
   //dprint("relays %04X %s, from %04X ", relay_mask, whichway == RELAY_ON ? "on" : "off", relay_status);
   if (whichway == RELAY_ON) relay_status |= relay_mask;
   else relay_status &= ~relay_mask;
   //dprint("to %04X\n", relay_status);
   Wire.beginTransmission(RELAYS1to8); // ADG728 analog mux #1
   Wire.write(relay_status >> 8);
   Wire.endTransmission();
   Wire.beginTransmission(RELAYS9to10); // ADG728 analog mux #2
   Wire.write(relay_status & 0xff);
   Wire.endTransmission(); }

#if DEBUG
void do_light_button_tests(void) {
   // note that this causes one level of recursion into check_for_button()
   dprint("light/button test; %d msec per RTOS tick\n", portTICK_PERIOD_MS);
   static uint16_t relay_masks [11] = {
      POOL_PUMP_RELAY     ,
      SPA_PUMP_RELAY      ,
      SPA_JETS_PUMP_RELAY ,
      HEAT_POOL_RELAY     ,
      HEAT_SPA_RELAY      ,
      POOL_VALVE_RELAY    ,
      SPA_VALVE_RELAY     ,
      HEATER_VALVE_RELAY  ,
      POOL_LIGHT_RELAY    ,
      SPARE_RELAY         ,
      0 };
   bool gotbutton = false;
   int button;
   while (doing_light_button_tests) {
      for (int ndx = 0; ndx < 11; ++ndx) { // step through all 11 lights and 10 relays
         watchdog_poke();
         center_messagef(0, "light/relay %d", ndx + 1);
         show_current_time(3);
         setrelay(relay_masks[ndx], RELAY_ON);
         int mask = (1 << ndx);
         setLED(mask, LED_ON);
         //dprint("LEDs on: %02X\n", mask);
         unsigned long starttime = millis();
         while (millis() - starttime < 1000) { // change relays and lights once a second
            if (gotbutton) {
               check_for_button();
               if (!button_awaiting_release[button]) {
                  center_message(1, "");
                  gotbutton = false; } }
            else {
               button = check_for_button();
               if (button != 0xff) {
                  center_messagef(1, "button %d", button);
                  gotbutton = true; } } }
         setrelay(relay_masks[ndx], RELAY_OFF);
         setLED(mask, LED_OFF); } } }
#endif

//------------------------------------------------------------------------------
//    heater and pump control routines
//------------------------------------------------------------------------------

void heater_off (void) {
   if (heater_mode != HEATING_NONE) {
      center_message(2, " "); // remove time left display
      center_message(3, " "); // remove temperature display
      setrelay(HEAT_SPA_RELAY + HEAT_POOL_RELAY, RELAY_OFF);
      setLED(TEMPCTL_RED_LED + TEMPCTL_BLUE_LED, LED_OFF);
      if (heater_on) {
         heater_cooldown_secs_left = DELAY_HEATER_OFF;
         heater_on = false; }
      heater_mode = HEATING_NONE;
      if (heater_cooldown_secs_left) {
         do {
            char msg[25];
            sprintf(msg, "heater cooling... %d", heater_cooldown_secs_left);
            center_message(2, msg);
            watchdog_poke(); }
         while (heater_cooldown_secs_left); // interrupt routine decrements this
         center_message(2, " "); } } }

void spa_heater_mode(void) {
   target_temp = 102;  // initial target temperature
   setrelay(HEAT_POOL_RELAY, RELAY_OFF);
   setrelay(HEAT_SPA_RELAY, RELAY_ON);
   setLED(TEMPCTL_RED_LED, LED_ON);
   heater_on = true;
   heater_mode = HEATING_SPA; }

void pool_heater_mode(void) {
   target_temp = 80;  // initial target temperature
   setrelay(HEAT_SPA_RELAY, RELAY_OFF);
   setrelay(HEAT_POOL_RELAY, RELAY_ON);
   setLED(TEMPCTL_RED_LED, LED_ON);
   heater_on = true;
   heater_mode = HEATING_POOL; }

void pumps_off (void) {
   heater_off();
   if (pump_status != PUMP_NONE) {
      setrelay(POOL_PUMP_RELAY + SPA_PUMP_RELAY, RELAY_OFF);
      pump_status = PUMP_NONE;
      center_message(3, " "); // remove temperature display
      center_message(2, "stopping pump");
      longdelay(1000 * DELAY_PUMP_OFF);
      center_message(2, " "); } }

void pump_on (pump_status_t pump) {
   if (pump == PUMP_SPA)
      setrelay(SPA_PUMP_RELAY, RELAY_ON);
   else if (pump == PUMP_POOL)
      setrelay(POOL_PUMP_RELAY, RELAY_ON);
   else assert_that(false, "Bad call to pump_on");
   pump_status = pump;
   center_message(2, "starting pump");
   longdelay(1000 * DELAY_PUMP_ON);
   center_message(2, " "); }

void setvalveconfig(vconfig_t config) {
   if (valve_config != config) {
      pumps_off();
      switch (config) {
         case VALVES_HEAT_SPA:
            setrelay(POOL_VALVE_RELAY, VALVE_RIGHT);
            setrelay(SPA_VALVE_RELAY, VALVE_RIGHT);
            setrelay(HEATER_VALVE_RELAY, VALVE_RIGHT);
            break;
         case VALVES_HEAT_POOL:
            setrelay(POOL_VALVE_RELAY, VALVE_LEFT);
            setrelay(SPA_VALVE_RELAY, VALVE_LEFT);
            setrelay(HEATER_VALVE_RELAY, VALVE_LEFT);
            break;
         case VALVES_FILL_SPA:
            setrelay(POOL_VALVE_RELAY, VALVE_LEFT);
            setrelay(SPA_VALVE_RELAY, VALVE_RIGHT);
            setrelay(HEATER_VALVE_RELAY, VALVE_RIGHT);
            break;
         case VALVES_EMPTY_SPA:
            setrelay(POOL_VALVE_RELAY, VALVE_RIGHT);
            setrelay(SPA_VALVE_RELAY, VALVE_RIGHT);
            setrelay(HEATER_VALVE_RELAY, VALVE_LEFT);
            break;
         default:
            assert_that(false, "Bad call to setconfig"); }
      for (byte timeleft = DELAY_VALVE_CHANGE; timeleft; --timeleft) {
         center_messagef(2, "setting valves... %d", timeleft);
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
   if (have_tempsensor) {
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
      // temp is 16*Celsius; do limited-range conversion to Fahrenheit
      return (temp * 9) / (5 * 16) + 32; }
   else return simulated_temp; }


//------------------------------------------------------------------------------
//    watchdog timer routines, which cause a hard reset if we become catatonic
//------------------------------------------------------------------------------

#include <esp_task_wdt.h>
#define WATCHDOG_TIMEOUT_SECS 10

bool watchdog_setup(void) { // enable the watchdog timer
   // return TRUE if the watchdog triggered the last startup
   esp_reset_reason_t reason = esp_reset_reason();
   esp_task_wdt_init(WATCHDOG_TIMEOUT_SECS, WATCHDOG); // enable so ESP32 restarts, maybe
   esp_task_wdt_add(NULL); // add current thread to WDT watch
   return reason == ESP_RST_INT_WDT || reason == ESP_RST_TASK_WDT || reason == ESP_RST_WDT; }

void watchdog_poke(void) {
   esp_err_t error = esp_task_wdt_reset();  // routine to poke the watchdog to prevent a reset
   yield(); // and do a yield to FreeRTOS
   return; }

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
   Wire.beginTransmission(REALTIME_CLOCK);
   Wire.write(0x00);
   Wire.endTransmission();
   // request the 7 bytes of data    (secs, min, hr, day, date. mth, yr)
   Wire.requestFrom(REALTIME_CLOCK, 7);
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
   Wire.beginTransmission(REALTIME_CLOCK);
   Wire.write(0x00); // reset register pointer
   Wire.write(bin2bcd(dt->sec));
   Wire.write(bin2bcd(dt->min));
   Wire.write(bin2bcd(dt->hour) | ((dt->ampm) << 5) | 0x40); // 12 hour mode
   Wire.write(bin2bcd(dt->day));
   Wire.write(bin2bcd(dt->date));
   Wire.write(bin2bcd(dt->month));
   Wire.write(bin2bcd(dt->year));
   Wire.endTransmission(); }

bool datetime_invalid(struct datetime dt) {
   return  dt.sec > 59  || dt.min > 59 || dt.hour == 0 || dt.hour > 12
           || dt.day  == 0 || dt.day > 7 || dt.date == 0 || dt.date > 31
           || dt.month == 0 || dt.month > 12 || dt.year > 99; }

char *format_datetime(struct datetime *dt, char *string) {
   // string should be at least 25 characters long
   if (datetime_invalid(*dt))
      strcpy(string, "-- --- 20-- --:-- --");
   else sprintf(string, "%2d %s 20%02d %2d:%02d %s",
                   dt->date, months[dt->month], dt->year, dt->hour, dt->min, dt->ampm ? "PM" : "AM");
   return string; }

void show_datetime(byte row, struct datetime * dt) {
   char string[25];
   format_datetime(dt, string);
   lcdsetCursor(0, row);
   lcdprint(string);
   #if DEBUG && DEBUG_LCD
   Serial.print(":: "); Serial.println(string);
   #endif
}

int compare_datetime(struct datetime *dt1, struct datetime *dt2) {
#define compare(f) if (dt1->f < dt2->f) return -1; else if (dt1->f > dt2->f) return +1;
   compare(year);
   compare(month);
   compare(day);
   compare(ampm);
   compare(hour);
   compare(min);
   compare(sec);
   return 0;
#undef compare
}

void show_current_time (byte row) {
   rtc_read(&now);
   show_datetime(row, &now); }

//-------------------------------------------------------
//  special test routine
//  triggered by SPA WATER LEVEL followed by MENU
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
         center_messagef(2, " %d", ++count);
         setrelay(relay, RELAY_ON);
         delay(250);
         setrelay(relay, RELAY_OFF);
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
         lcdclear(); lcdprint("press \x02 to fill spa"); // uparrow
         lcdsetCursor(0, 1); lcdprint("press \x01 to empty spa"); // downarrow
         lcdsetCursor(0, 2); lcdprint("any other cancels");
         #if DEBUG
         Serial.println(":: press up / down to fill / empty spa");
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
         else if (button == MENU_BUTTON)
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
      setrelay(SPA_JETS_PUMP_RELAY, RELAY_OFF);
      spa_jets_timer = 0;
      spa_jets_on = false; }
   else { // turning on
      setLED(SPA_JETS_LED, LED_ON);
      setrelay(SPA_JETS_PUMP_RELAY, RELAY_ON);
      spa_jets_timer = SPA_JETS_TIMEOUT;
      spa_jets_on = true; } }

void pool_light_pushed (void) {
   if (pool_light_on) {  // turning off
      setLED(POOL_LIGHT_LED, LED_OFF);
      setrelay(POOL_LIGHT_RELAY, RELAY_OFF);
      light_timer = 0;
      pool_light_on = false; }
   else { // turning on
      setLED(POOL_LIGHT_LED, LED_ON);
      setrelay(POOL_LIGHT_RELAY, RELAY_ON);
      light_timer = POOL_LIGHT_TIMEOUT;
      pool_light_on = true; } }

//-------------------------------------------------------
//  FLASH memory configuration routines
//-------------------------------------------------------

#include <esp_partition.h>
#define ESP_PARTITION_TYPE_CONFIG (esp_partition_type_t)0x4E

const esp_partition_t *config_partition = NULL;
bool config_changed = false;

void write_config (void) {
   #if DEBUG
   Serial.println("writing config...");
   #endif
   assert_that(esp_partition_erase_range(config_partition, 0, 4096) == ESP_OK,
               "can't erase config partition");
   assert_that(esp_partition_write(config_partition, 0, &config_data, sizeof(config_data)) == ESP_OK,
               "can't write to config partition"); }

void init_config(void) {
   char hdr_id[6];
   assert_that((config_partition = esp_partition_find_first(ESP_PARTITION_TYPE_CONFIG, ESP_PARTITION_SUBTYPE_ANY, NULLP)) != NULL,
               "can't find FLASH config partition");
   assert_that(esp_partition_read(config_partition, 0, &hdr_id, sizeof(hdr_id)) == ESP_OK,
               "can't read config header from FLASH");
   if (memcmp(hdr_id, config_data.hdr_id, sizeof(hdr_id)) != 0) {//check for matching id string
      write_config(); // not there: write our default compiled-in config
      log_event(EV_INIT_CONFIG);
      center_message(2, "initialized config"); }
   else { // ID seems good; read the whole header into RAM
      assert_that(esp_partition_read(config_partition, 0, &config_data, sizeof(config_data)) == ESP_OK,
                  "can't read config data from FLASH"); }
   delay(1000); }

//-------------------------------------------------------
// menu commands, including configuration programming
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
      lcdsetCursor(columns[*field], CONFIG_ROW);
      lcdblink();
      switch (wait_for_button()) {  // wait for a button push
         case MENU_BUTTON:
            lcdnoBlink();
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

void show_event(int eventnum) {
   char string[25];
   center_messagef(0, "event %d of %d", eventnum, log_state.numinuse);
   assert_that(flashlog_read (&log_state) == FLASHLOG_ERR_OK,
               "can't read log entry");
   struct logentry_t *plog = (struct logentry_t *) log_state.logdata;
   show_datetime(1, &plog->timestamp);
   center_message(2, event_names[plog->event_type]);
   memset(string, 0, sizeof(string)); // in case the message isn't zero-terminated
   if (plog->event_msg[0]) memcpy(string, plog->event_msg, sizeof(string) - 1);
   center_message(3, string); }

bool show_eventlog(void) { //******** display the event log
   char string[25];
   int eventnum = -1; // haven't started yet
   center_messagef(0, "%d of %d entries", log_state.numinuse, log_state.numslots);
   center_message(1, "");
   center_message(2, LEFTARROW " and " RIGHTARROW " moves");
   center_message(3, "MENU exits");
   if (log_state.numinuse == 0) return false; // skip if the log is empty
   while (1) {
      switch (wait_for_button()) {
         case MENU_BUTTON:
            return eventnum != -1;  // done; return true if we showed something
         case RIGHTARROW_BUTTON: // go forward in time
            if (eventnum < 0) {
               flashlog_goto_oldest(&log_state); // start with oldest
               eventnum = 1; }
            else if (flashlog_goto_next(&log_state) == FLASHLOG_ERR_OK)   // otherwise do next newer
               ++eventnum;
            show_event(eventnum);
            break;
         case LEFTARROW_BUTTON: // go backward in time
            if (eventnum < 0) {
               flashlog_goto_newest(&log_state); // start with newest
               eventnum = log_state.numinuse; }
            else if (flashlog_goto_prev(&log_state) == FLASHLOG_ERR_OK)   // otherwise do next older
               --eventnum;
            show_event(eventnum);
            break;
         default: ;// ignore all other buttons
      } } }

bool set_time(void) {  //********* change the current date and time
   int8_t delta;
   byte field;

   rtc_read(&now);
   field = 0; // start with first field
   while (true) {
      show_datetime(CONFIG_ROW, &now); // show it in row 1
      delta = get_config_changes(config_time_columns, &field);
      if (delta == 0) break;
      switch (field) {
         case 0: // date
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
   return false; }

bool set_filter_hour (void) {  //****** change when filtering starts each day
   int8_t delta;
   byte field;

   field = 0; // start with first field
   while (true) {
      center_messagef(CONFIG_ROW, " %2d %s", config_data.filter_start_hour,
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
            break; } }
   return false; }

bool set_pool_filter_time (void) {  //******** change how long to filter the pool
   int8_t delta;
   byte field;

   field = 0; // start with first (and only) field
   while (true) {
      center_messagef(CONFIG_ROW, " %3d minutes", config_data.filter_pool_mins);
      delta = get_config_changes(config_poolfiltertime_columns, &field);
      if (delta == 0) break;
      switch (field) {
         case 0:  // minutes from 5 to 120
            config_data.filter_pool_mins = bound (config_data.filter_pool_mins, delta, 5, 120);
            break; } }
   return false; };

bool set_spa_filter_time (void) {  //********* change how long to filter the spa
   int8_t delta;
   byte field;

   field = 0; // start with first (and only) field
   while (true) {
      center_messagef(CONFIG_ROW, " %2d minutes", config_data.filter_spa_mins);
      delta = get_config_changes(config_spafiltertime_columns, &field);
      if (delta == 0) break;
      switch (field) {
         case 0:  // minutes from 5 to 60
            config_data.filter_spa_mins = bound (config_data.filter_spa_mins, delta, 5, 60);
            break; } }
   return false; };

bool enable_heater(void) { //********* enable or disable the use of the heater
   int8_t delta;
   byte field;

   field = 0; // start with first (and only) field
   while (true) {
      center_messagef(CONFIG_ROW, "heater is %s", config_data.heater_allowed ? "enabled" : "disabled");
      delta = get_config_changes(config_heater_enable_columns, &field);
      if (delta == 0) break;
      config_data.heater_allowed ^= 1; // reverse
   }
   return false; }

const static struct  {  // configuration programming action routines
   const char *title;
   bool (*fct)(void); }
config_cmds [] = {
   {"set time", set_time },
   {"set filter hour", set_filter_hour },
   {"set pool filter time", set_pool_filter_time },
   {"set spa filter time", set_spa_filter_time },
   {"enable heater", enable_heater },
   {NULL, NULL } };

bool do_configuration (void) {
   enter_idle_mode();   // stop everything
   config_changed = false;
   for (byte cmd = 0; config_cmds[cmd].title; ++cmd) { // do all config settings
      lcdclear();
      lcdprint(config_cmds[cmd].title); // show instruction on top line
      lcdsetCursor(0, 2);
      lcdprint("Arrows view, change");
      lcdsetCursor(0, 3);
      lcdprint("Then press \"menu\"");
      if ((config_cmds[cmd].fct)()) // execute the configuration routine
         break; // don't continue (only for "program" if it showed something)
   }
   lcdclear();
   if (config_changed) {
      write_config();  // write configuration into EPROM
      log_event(EV_UPDATED_CONFIG);
      center_message(0, "changes recorded");
      longdelay(1500); }
   return config_changed; }

bool show_wifi_info(void) {
   lcdclear();
   center_message(3, "press MENU");
   if (webserver_address[0]) {
      lcdprint(0, webserver_address);
      lcdprintf(1, "%d ok, %d bad", connect_successes, connect_failures);
      lcdprintf(2, "%d requests", client_requests);
      while (wait_for_button() != MENU_BUTTON) ;
      center_message(0, ""); center_message(1, ""); center_message(2, "");
      do {
         lcdprintf(0, "RSSI = %d   ", wifi_get_rssi()); // need mutex?
         delay(250); }
      while (check_for_button() != MENU_BUTTON);
      return true; }
   else center_message(0, "not connected");
   while (wait_for_button() != MENU_BUTTON) ;
   return true; }

void menu_pushed (void) {
   const static struct  {  // menu action routines
      const char *title;
      bool (*fct)(void); }
   menu_cmds [] = {
      {"show event log?", show_eventlog },
      {"show WiFi info?", show_wifi_info },
      {"configure?", do_configuration },
      {NULL, NULL } };
   setLED(MENU_LED, LED_ON);
   for (byte cmd = 0; ; ) { //cycle through menu items
      lcdclear();
      center_message(3, "MENU exits");
      bool doit = yesno(0, true, menu_cmds[cmd].title);
      if (menu_button_pushed) break;
      if (doit) {
         (menu_cmds[cmd].fct)(); // execute the command routine
         break; } // and exit
      if (menu_cmds[++cmd].title == NULL) cmd = 0; } // go to the next operation
   lcdclear();
   setLED(MENU_LED, LED_OFF); }



//-------------------------------------------------------
//  Interrupt routines
//-------------------------------------------------------

// once-a-second timer interrupt routine
// Don't do anything heavy-duty here, and beware of race conditions for
// multi-byte variables that might be accessed by the background process.

hw_timer_t *secondtimer = NULL;

void IRAM_ATTR timerint() {
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
      if (light_timer) --light_timer;
      temphistory_add(); }  // maybe add to temp history

   if (!have_tempsensor) { // no temp sensor: simulate the heater
      static int simulation_secs = 0;
      if (heater_on) {
         if (++simulation_secs >= 60) { // +1 degree every minute
            if (simulated_temp < 150) ++simulated_temp;
            simulation_secs = 0; } }
      else { // heater off
         if (++simulation_secs >= 2 * 60) { // -1 degree every 2 minutes
            if (simulated_temp >= 60) --simulated_temp;
            simulation_secs = 0; } } } }

// rotary encoder interrupt: one of the pins changed

#if ROTARY_ENCODER
static TaskHandle_t rotary_encoder_task = NULL;
static const UBaseType_t rotary_encoder_notifyarrayindex = 0;
static bool rotary_encoder_processing_change = false;
static char *rotary_encoder_stackpointer = NULL;
static char *rotary_encoder_stackstart = NULL;

void rotary_encoder_pin_change_ISR(void) { // the pin change interrupt
   BaseType_t xHigherPriorityTaskWoken = pdFALSE;
   if (rotary_encoder_task != NULL  // if the task is created,
         && !rotary_encoder_processing_change) {  // and we're not already processing a change
      rotary_encoder_processing_change = true; // then start processing the change
      vTaskNotifyGiveIndexedFromISR( // by unblocking the pin change task
         rotary_encoder_task,
         rotary_encoder_notifyarrayindex,
         &xHigherPriorityTaskWoken );
      //not a good idea?  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
   } }

// The following routine runs as a separate FreeRTOS task and waits to be
// unblocked by the interrupt service routine above. We do it this way
// so that we can wait for the switch debounce before processing
// the new encoder state.
void rotary_encoder_change_task(void *parm) {
   static const int8_t encoder_states [] = { //encoder lookup table
      // a clever encoding scheme from Oleg Mazurov
      0, +1, -1, 0, -1, 0, 0, +1, +1, 0, 0, -1, 0, -1, +1, 0 };
   static byte old_AB = 3; // last value of the encoder bits
   static int8_t count_clicks = 0;
   // debugging stack stuff
   char local;
   rotary_encoder_stackpointer = &local;
   if (rotary_encoder_stackstart == NULL)
      rotary_encoder_stackstart = rotary_encoder_stackpointer;
   while (1) { // loop waiting for the pin change interrupt to unblock us
      ulTaskNotifyTakeIndexed(rotary_encoder_notifyarrayindex, pdTRUE, portMAX_DELAY );
      if (rotary_encoder_processing_change) {
         vTaskDelay(3 /*DEBOUNCE_DELAY*/ / portTICK_PERIOD_MS); // short debounce delay
         old_AB <<= 2;  // shift old encoder bits over
         byte new_AB = (digitalRead(TEMPCTL_INPUT_A) << 1) | digitalRead(TEMPCTL_INPUT_B); // read new encoder bits
         //dprint("encoder new AB: %d\n", new_AB);
         old_AB = (old_AB | new_AB) & 0x0f; // create a 4-bit index with the new and old bits
         int8_t direction = encoder_states[old_AB];
         if (direction /*&& (new_AB == 3)*/) {  // [at detent and] transition is valid
            count_clicks += direction; // force more rotation for each change
#define ENCODER_CLICKS 3  // number of detent clicks per change we report
            if (count_clicks < -ENCODER_CLICKS || count_clicks > ENCODER_CLICKS) {
               count_clicks = 0;
               temp_change(direction); } }
         rotary_encoder_processing_change = false; } } }
#endif

void temp_change (int8_t direction) {
   UBaseType_t new_highwater = uxTaskGetStackHighWaterMark(rotary_encoder_task);
   dprint("temp change: %d, stack used %d, highwater %d\n", direction,
          rotary_encoder_stackpointer - rotary_encoder_stackstart, new_highwater);
   if (heater_mode != HEATING_NONE) { //heater is running
      if (direction < 0) {  // decreasing temp
         if (target_temp > TEMP_MIN) --target_temp; }
      else {  // increasing temp
         if (heater_mode == HEATING_SPA && target_temp < TEMP_MAX_SPA
               || heater_mode == HEATING_POOL && target_temp < TEMP_MAX_POOL)
            ++target_temp; } } }

//-------------------------------------------------------
//  Initialization
//-------------------------------------------------------

void webserver_task(void *parm);  // our webserver task, in webserver.cpp
TaskHandle_t webserver_task_handle = NULL;

void outpin(byte pinnum, byte starting_state) {  // configure an output pin
   digitalWrite(pinnum, starting_state);
   pinMode(pinnum, OUTPUT); }

void inpin(byte pinnum) {  // configure an input pin
   pinMode(pinnum, INPUT_PULLUP);
   //digitalWrite(pinnum, HIGH); // turn pullup resistor on
}

void setup (void) {  // initialization starts here

   bool watchdog_triggered = watchdog_setup();  // start the watchdog timer

   #if DEBUG
   Serial.begin(115200);
   while (!Serial) ;
   Serial.println("Debugging log for Pool/Spa controller");
   #endif

   cpu_core = xPortGetCoreID(); // record which CPU core we're running on
   lcd_start();
   #if DEBUG
   Serial.print("LCD started, running on core ");
   Serial.println(cpu_core);
   #endif
   center_messagef(0, "initializing v%s", VERSION);

   // hardware initialization
   outpin(LED_DRIVER_SDI, LOW);
   outpin(LED_DRIVER_CLK, LOW);
   outpin(LED_DRIVER_LE, LOW);
   outpin(LED_DRIVER_OD, LOW);
   setLED(0, LED_OFF);  // turn off all LEDs
   setrelay(0, RELAY_OFF); // make sure all relays are off
   inpin(PUSHBUTTON_IN);
   Wire.begin();   // start onewire for temperature sensor and realtime clock

   // initialize the log
   assert_that(flashlog_open(NULL, LOG_DATASIZE, &log_state) == FLASHLOG_ERR_OK, "can't open log");
   center_messagef(2, "%d of %d events", log_state.numinuse, log_state.numslots);
   #if DEBUG
   //dump_log();
   #endif
   delay(1000);
   init_config();  // get or set configuration data from FLASH

   // start a timer that interrupts once a second
   secondtimer = timerBegin(0, 80, true);
   timerAttachInterrupt(secondtimer, &timerint, true);
   timerAlarmWrite(secondtimer, 1000000, true);
   timerAlarmEnable(secondtimer);

   // realtime clock
   rtc_read(&now);  // check for valid realtime clock data
   if (datetime_invalid(now)) {
      rtc_write (&clock_init); // reset if bad
      if (datetime_invalid(now)) {// if still invalid, it's broken or not present
         no_clock = true; } }

   // temperature sensor
   outpin(TEMPSENSOR_PIN, HIGH);
   delay(250); // wait for parasitic power capacitor to charge?
   have_tempsensor = tempsensor.search(tempsensor_addr)  // find temp sensor
                     && (OneWire::crc8(tempsensor_addr, 7) == tempsensor_addr[7]); // with ok CRC
   if (have_tempsensor) {
      const byte set_10_bit_resolution[4] = {
         0x4e, 0, 0, 0x20 };   // use 10-bit (Centigrade/16) resolution
      tempsensor.reset();
      tempsensor.select(tempsensor_addr);
      tempsensor.write_bytes(set_10_bit_resolution, 4, 1); }

   // rotary encoder for temperature control
   #if ROTARY_ENCODER
   assert_that(xTaskCreatePinnedToCore( // start the task to process pin changes
                  rotary_encoder_change_task,
                  "rotary encoder",
                  4096, // stack size (leave room for stacked interrupts!)
                  NULL, // parameter
                  2, // priority
                  &rotary_encoder_task, // where to put the task handle
                  cpu_core) // which CPU core it should run on: ours
               == pdPASS, "can't create rotary encoder task");
   inpin(TEMPCTL_INPUT_A);
   inpin(TEMPCTL_INPUT_B);
   attachInterrupt(TEMPCTL_INPUT_A, rotary_encoder_pin_change_ISR, CHANGE);
   attachInterrupt(TEMPCTL_INPUT_B, rotary_encoder_pin_change_ISR, CHANGE);
   #endif

   // make log entries

   log_event( // make an initial "power on" event log entry
      watchdog_triggered ? EV_WATCHDOG_RESET :  EV_STARTUP);
   if (watchdog_triggered) {
      center_message(2, "Watchdog triggered!");
      delay(1000); }

   if (no_clock) {
      log_event(EV_CLOCK_BAD);
      center_message(2, "no clock!");
      delay(1000); }

   if (!have_tempsensor) {
      log_event(EV_TEMPSENSOR_BAD);
      center_message(2, "no temp sensor!");
      delay(1000); }

   #if WEBSERVER
   // start the web server
   esp_err_t err = xTaskCreatePinnedToCore( // start the webserver task
                      webserver_task,
                      "webserver",
                      32768, // stack size
                      NULL, // parameter
                      0, // priority
                      &webserver_task_handle, // where to put the task handle
                      1 - cpu_core); // which CPU core it should run on: not ours
   assert_that(err == pdPASS, "can't create webserver task, err %d", err);
   #if WATCHDOG
   err = esp_task_wdt_add(webserver_task_handle);
   assert_that(err == ESP_OK, "can't add webserver task to WDT, err %d", err);
   // disableCore0WDT(); // doesn't work
   #endif
   #endif

   setvalveconfig(VALVES_HEAT_SPA); // put the valves into a known state

}


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
   menu_pushed };

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
   byte button;

   watchdog_poke();

   //show our IP address when we first become connected
   static bool ip_address_shown = false;
   if (!ip_address_shown && webserver_address[0]) {
      center_message(0, "IP address");
      lcdprint(1, webserver_address);
      ip_address_shown = true;
      longdelay(3000);
      center_message(1, ""); }

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
         center_messagef(2, " %d hr %d min left ", timer / 60, timer % 60);
      else center_messagef(2, " %d min left", timer % 60); }

   // check if this mode has timed out
   if (if_zero(&mode_timer) && mode != MODE_IDLE) { // timed out
      if (mode == MODE_FILTER_POOL && filter_autostarted)  // if it's "filter pool" autostarted
         filter_spa_pushed(); // then switch to "filter spa"
      else   // otherwise go into idle mode
         enter_idle_mode(); }

   // have the spa jets timed out?
   if (spa_jets_on && if_zero(&spa_jets_timer)) {
      setLED(SPA_JETS_LED, LED_OFF);
      setrelay(SPA_JETS_PUMP_RELAY, RELAY_OFF);
      spa_jets_on = false; }

   // has the pool light timed out?
   if (pool_light_on && if_zero(&light_timer)) {
      setLED(POOL_LIGHT_LED, LED_OFF);
      setrelay(POOL_LIGHT_RELAY, RELAY_OFF);
      pool_light_on = false; }

   // display the water temperature and turn the heater on or off

   if (pump_status == PUMP_NONE)  //pump is off
      temp_valid = false;
   else { //pump is on
      temp_now = read_temp();

      if (heater_mode == HEATING_NONE) {
         // We're not heating, but one of the pumps is running.
         // Display the temperature if water is flowing through the heater,
         // since the temp sensor is in the return inlet to the heater
         if (mode == MODE_FILTER_SPA && valve_config == VALVES_HEAT_POOL
               || mode == MODE_FILTER_POOL && valve_config == VALVES_HEAT_SPA) {
            sprintf(string, " ");
            temp_valid = false; }
         else {
            sprintf(string, "temperature: %dF", temp_now);
            temp_valid = true; } }

      else {  // either HEATING_SPA or HEATING_POOL
         // (Do we need to enforce a minimum time between heater changes
         //  here, to avoid damage? Or can the heater cope?)
         if (heater_on) {
            if (temp_now >= target_temp) { // turn heater off
               setrelay(HEAT_SPA_RELAY + HEAT_POOL_RELAY, RELAY_OFF);
               setLED(TEMPCTL_BLUE_LED, LED_ON);
               setLED(TEMPCTL_RED_LED, LED_OFF);
               heater_cooldown_secs_left = DELAY_HEATER_OFF;
               heater_on = false; } }
#define TEMP_HYSTERESIS 2  // hysteresis in degrees Fahrenheit
         else { // heater off
            if (temp_now <= target_temp - TEMP_HYSTERESIS) { // turn heater on
               setrelay(heater_mode == HEATING_SPA ? HEAT_SPA_RELAY : HEAT_POOL_RELAY, RELAY_ON);
               setLED(TEMPCTL_BLUE_LED, LED_OFF);
               setLED(TEMPCTL_RED_LED, LED_ON);
               heater_on = true; } }
         center_messagef(3, "temp set %d, is %d", target_temp, temp_now);
         temp_valid = true; } }

} // repeat loop

//*

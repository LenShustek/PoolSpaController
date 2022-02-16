//----------------------------------------------------
// Declarations for Saw Mill Lodge Pool Controller
//----------------------------------------------------

#include <stdint.h>
typedef unsigned char byte;
#define stringify(x) stringifier(x)  // for turning #defined macros into strings
#define stringifier(x) #x

struct datetime { // Maxim DS1307 format: 8 bytes
   byte sec /*0-59*/, min /*0.59*/, hour /*1-12*/, ampm/*0-1*/,
        day/*1-7*/, date /*1-31*/, month/*1-12*/, year/*00-99*/; };
extern struct datetime now;
char *format_datetime(struct datetime *dt, char *string);
int compare_datetime(struct datetime *dt1, struct datetime *dt2);
#define AM false
#define PM true

#define MAXLINE 500
#define DOWNARROW   "\x01"    // glyphs we define
#define UPARROW     "\x02"
#define RIGHTARROW  "\x7e"    // other non-ASCII glyphs in the character generator
#define LEFTARROW   "\x7f"

// I/O  configuration

// ten relays: the bits for ADG728 drivers 0x4c and 0x4d
#define POOL_PUMP_RELAY      0b0000000100000000 // relay 1
#define SPA_PUMP_RELAY       0b0000001000000000 // relay 2
#define SPA_JETS_PUMP_RELAY  0b0000010000000000 // relay 3
#define HEAT_POOL_RELAY      0b0000100000000000 // relay 4
#define HEAT_SPA_RELAY       0b0001000000000000 // relay 5
#define POOL_VALVE_RELAY     0b0010000000000000 // relay 6
#define SPA_VALVE_RELAY      0b0100000000000000 // relay 7
#define HEATER_VALVE_RELAY   0b1000000000000000 // relay 8
#define POOL_LIGHT_RELAY     0b0000000000000001 // relay 9
#define SPARE_RELAY          0b0000000000000010 // relay 10

#define RELAY_ON HIGH
#define RELAY_OFF LOW
// valve "left" means the water block is on the left side viewed from the input port
// valve "right" means the water block is on the right side viewed from the input port
#define VALVE_LEFT RELAY_ON   // depends on Jandy valve wiring and switch position
#define VALVE_RIGHT RELAY_OFF // depends on Jandy valve wiring and switch position

// Eight pushbuttons for ADG728 mux at I2C 0x4e
#define NUM_BUTTONS 8
#define HEAT_SPA_BUTTON_MASK        0b00000001
#define HEAT_POOL_BUTTON_MASK       0b00000010
#define SPA_JETS_BUTTON_MASK        0b00000100
#define POOL_LIGHT_BUTTON_MASK      0b00001000
#define FILTER_SPA_BUTTON_MASK      0b00010000
#define FILTER_POOL_BUTTON_MASK     0b00100000
#define SPA_WATER_LEVEL_BUTTON_MASK 0b01000000
#define MENU_BUTTON_MASK            0b10000000
#define PUSHBUTTON_IN 12     // I/O pin where the muxed button appears
extern bool button_webpushed[NUM_BUTTONS];

// Maxim DS18B20 temperature sensor
#define TEMPSENSOR_PIN 13

// I2C addresses
#define LCD_DISPLAY 0x20     // 20x4 LCD display Adafruit I2C backpack
#define RELAYS1to8  0x4c     // Analog Devices ADG728
#define RELAYS9to10 0x4d     // Analog Devices ADG728
#define PUSHBUTTONS 0x4e     // Analog Devices ADG728
#define REALTIME_CLOCK 0x68  // Maxim DS3231

// temperature control rotary encoder
#define TEMPCTL_INPUT_A 14
#define TEMPCTL_INPUT_B 32 

// pins for two TI TLC5916 LED driver chips in series
#define LED_DRIVER_SDI 27  // serial data in
#define LED_DRIVER_CLK 33  // clock
#define LED_DRIVER_LE  21  // data latch enable
#define LED_DRIVER_OD  15  // output disable

// masks for 11 LEDs
#define NUM_LEDS 11
#define HEAT_SPA_LED         0x0001
#define HEAT_POOL_LED        0x0002
#define SPA_JETS_LED         0x0004
#define POOL_LIGHT_LED       0x0008
#define FILTER_SPA_LED       0x0010
#define FILTER_POOL_LED      0x0020
#define SPA_WATER_LEVEL_LED  0x0040
#define MENU_LED             0x0080
#define TEMPCTL_RED_LED      0x0400
#define TEMPCTL_GREEN_LED    0x0200
#define TEMPCTL_BLUE_LED     0x0100
extern uint16_t leds_on;              // mask for which LEDs are currently on
extern uint16_t led_masks[NUM_LEDS];  // mask for the individual LEDs
#define LED_OFF 0x0000
#define LED_ON  0xffff

// modes
// (need to be in a .h file because of an Arduino IDE compiler bug)

enum  global_mode_t {  // current global mode: "mode"
   MODE_IDLE,
   MODE_HEAT_SPA,
   MODE_HEAT_POOL,
   MODE_FILL_SPA,
   MODE_EMPTY_SPA,
   MODE_FILTER_POOL,
   MODE_FILTER_SPA };

enum heater_t {  // current heater setting: "heater_mode"
   HEATING_NONE,
   HEATING_SPA,
   HEATING_POOL };

enum vconfig_t {  // current valve configuration: "valve_config"
   VALVES_UNDEFINED,
   VALVES_HEAT_SPA,
   VALVES_HEAT_POOL,
   VALVES_FILL_SPA,
   VALVES_EMPTY_SPA };

enum pump_status_t { // current pump status: "pump_status"
   PUMP_NONE,
   PUMP_SPA,
   PUMP_POOL };

// temperature history stuff

#define TEMPHIST_DELTA_MINS 1     // how many minutes between entries
#define TEMPHIST_TOTAL_HOURS 20   // how many hours of data to record
#define TEMPHIST_ENTRIES (TEMPHIST_TOTAL_HOURS*60/TEMPHIST_DELTA_MINS) // how many entries to record
struct temphist_t {
  struct datetime timestamp;
  #define temphist_temp timestamp.day // save space: put temp where day (Mon-Sun) is
  // write it to a CSV file as "yyyy-mm-dd hh:mm:ss, temp"
};

// Timing parameters

#if DEBUG_TIMES
   #define DELAY_HEATER_OFF 5      // seconds of wait after heater off  **DEBUG
   #define DELAY_VALVE_CHANGE 5    // seconds of wait for valve change  **DEBUG
#else
   #define DELAY_HEATER_OFF 60     // seconds of wait after heater off
   #define DELAY_VALVE_CHANGE 45   // seconds of wait for valve change
#endif
#define DELAY_PUMP_OFF 3        // seconds of wait after pump off
#define DELAY_PUMP_ON 3         // seconds of wait after pump on
#define TITLE_LINE_TIME 2       // seconds for each different title line

#define FILTER_POOL_TIME 20      // minutes to filter pool, by default
#define FILTER_SPA_TIME 10       // minutes to filter spa, by default
#define FILTER_START_HOUR 01     // clock hour to start filter, by default
#define FILTER_START_AMPM AM     // 

#define MODE_SPA_TIMEOUT 3*60    // minutes before spa turns off
#define MODE_POOL_TIMEOUT 24*60  // minutes before pool turns off
#define MODE_FILL_TIMEOUT 5      // minutes before spa fill turns off
#define MODE_EMPTY_TIMEOUT 5     // minutes before spa empty turns off

#define POOL_LIGHT_TIMEOUT 3*60  // minutes before pool light turns off
#define SPA_JETS_TIMEOUT 1*60    // minutes before spa aerator turns off

#define DEBOUNCE_DELAY 50        // milliseconds for debounce delay

// temperature limits

#define TEMP_MIN 60
#define TEMP_MAX_POOL 92
#define TEMP_MAX_SPA 105

void dprint(const char *format, ...);
void assert_that(bool test, const char *msg, ...);
void lcdprintf(byte row, const char *msg, ...);
void watchdog_poke(void);
void log_dump(void * parm, void (*print)(void * parm, const char *line));
void temphistory_dump(void *parm, void (*print)(void * parm, const char *line));
void temp_change (int8_t direction);
int wifi_get_rssi(void);

extern char lcdbuf[4][21];
extern int lcdrow, lcdcol;
extern bool lcd_cursorblinking;
extern enum heater_t heater_mode;
extern bool heater_on; 
extern char webserver_address[]; 
extern int connect_successes;
extern int connect_failures;
extern int client_requests;

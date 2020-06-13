//----------------------------------------------------
// Declarations for Saw Mill Lodge Pool Controller
//----------------------------------------------------

#define AM false
#define PM true

// I/O pin numbers

// ten relays
#define POOL_PUMP_RELAY 2     // relay 1
#define SPA_PUMP_RELAY 3      // relay 2
#define SPA_JETS_PUMP_RELAY 4 // relay 3
#define HEAT_POOL_RELAY 5     // relay 4
#define HEAT_SPA_RELAY 6      // relay 5
#define POOL_VALVE_RELAY 7    // relay 6
#define SPA_VALVE_RELAY 8     // relay 7
#define HEATER_VALVE_RELAY 9  // relay 8
#define POOL_LIGHT_RELAY 10   // relay 9
#define SPARE_RELAY 11        // relay 10

#define RELAY_ON LOW
#define RELAY_OFF HIGH
// valve "left" means the water block is on the left side viewed from the input port
// valve "right" means the water block is on the right side viewed from the input port
#define VALVE_LEFT RELAY_ON   // depends on Jandy valve wiring and switch position
#define VALVE_RIGHT RELAY_OFF // depends on Jandy valve wiring and switch position

// Maxim DS18B20 temperature sensor
#define TEMPSENSOR_PIN 12

// Maxim DS1307 realtime clock
#define REALTIME_CLOCK_SDA
#define REALTIME_CLOCK_SCL

// Hitachi HD44780 LCD controller
#define LCD_D4 24
#define LCD_D5 25
#define LCD_D6 26
#define LCD_D7 27
#define LCD_ENB 23
#define LCD_RS 22

// temperature control rotary encoder
#define TEMPCTL_INPUT_A 53 // B0, pin change interrupt 0
#define TEMPCTL_INPUT_B 52 // B1, pin change interrupt 0
#define TEMPCTL_READ PINB  // encoder port read

// Eight pushbuttons 
#define HEAT_SPA_BUTTON_PIN        51
#define HEAT_POOL_BUTTON_PIN       50
#define SPA_JETS_BUTTON_PIN        49
#define POOL_LIGHT_BUTTON_PIN      48
#define FILTER_SPA_BUTTON_PIN      47
#define FILTER_POOL_BUTTON_PIN     46
#define SPA_WATER_LEVEL_BUTTON_PIN 45
#define PROGRAM_BUTTON_PIN         44

// pins for two TI TLC5916 LED driver chips in series
#define LED_DRIVER_SDI 43  // serial data in
#define LED_DRIVER_CLK 42  // clock
#define LED_DRIVER_LE  41  // data latch enable
#define LED_DRIVER_OD  40  // output disable
// masks for 11 LEDs
#define HEAT_SPA_LED         0x0001
#define HEAT_POOL_LED        0x0002
#define SPA_JETS_LED         0x0004
#define POOL_LIGHT_LED       0x0008
#define FILTER_SPA_LED       0x0010
#define FILTER_POOL_LED      0x0020
#define SPA_WATER_LEVEL_LED  0x0040
#define PROGRAM_LED          0x0080
#define TEMPCTL_RED_LED      0x0100
#define TEMPCTL_GREEN_LED    0x0200
#define TEMPCTL_BLUE_LED     0x0400

// Test pins

#define TESTPIN 14

// modes
// (need to be in a .h file because of an Arduino IDE compiler bug)

enum  mode_t {  // current global mode: "mode"
  MODE_IDLE,
  MODE_HEAT_SPA,
  MODE_HEAT_POOL,
  MODE_FILL_SPA,
  MODE_EMPTY_SPA,
  MODE_FILTER_POOL,
  MODE_FILTER_SPA};

enum heater_t {  // current heater setting: "heater_mode"
  HEATING_NONE,
  HEATING_SPA,
  HEATING_POOL};

enum vconfig_t {  // current valve configuration: "valve_config"
  VALVES_UNDEFINED,
  VALVES_HEAT_SPA,
  VALVES_HEAT_POOL,
  VALVES_FILL_SPA,
  VALVES_EMPTY_SPA};

enum pump_status_t { // current pump status: "pump_status"
  PUMP_NONE,
  PUMP_SPA,
  PUMP_POOL};

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

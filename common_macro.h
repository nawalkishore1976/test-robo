
// Debug configuration - set to 1 to enable debug output
#define DEBUG_ENABLED 0

#if DEBUG_ENABLED
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_PRINTF(x, y) Serial.print(x); Serial.println(y)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(x, y)
#endif

// Store strings in program memory to save RAM
const char MOTOR_INIT_MSG[] PROGMEM = "[RoboMotor] Initialized - Fwd pin: ";
const char MOTOR_REV_MSG[] PROGMEM = ", Rev pin: ";
const char MOTOR_PWM_MSG[] PROGMEM = ", PWM pin: ";
const char MOTOR_FWD_STR[] PROGMEM = "FWD";
const char MOTOR_REV_STR[] PROGMEM = "REV";
const char MOTOR_DEBUG_MSG[] PROGMEM = "Motor: ";
const char MOTOR_SPEED_MSG[] PROGMEM = ", Speed: ";
const char MOTOR_PWM_VAL_MSG[] PROGMEM = "%, PWM: ";


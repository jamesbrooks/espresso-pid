#include <EEPROM.h>
#include <max6675.h>
#include <PID_v1.h>

// Thermocouple
#define TC_CLK 7
#define TC_CS 6
#define TC_MISO 5
#define TC_DELAY_BETWEEN_READS 250
#define TC_NUM_READINGS 4

// PID
#define kP 1000
#define kI 0
#define kD 3000
#define WINDOW_SIZE 1000  // milliseconds

// Misc
#define ESPRESSO_MODE_BUTTON_PIN 4
#define DECREASE_TEMPERATURE_BUTTON_PIN 3
#define INCREASE_TEMPERATURE_BUTTON_PIN 2
#define RELAY_PIN A1
#define RELAY_MINIMUM_CYCLE_TIME 20 // milliseconds

// Thermocouple
unsigned long tc_last_read_time = 0;
double tc_readings[TC_NUM_READINGS];
int tc_reading_index = 0;
double tc_readings_total = 0;
double tc_average_reading = 0;
MAX6675 thermocouple = MAX6675(TC_CLK, TC_CS, TC_MISO);

// PID
double pid_input = 0; // we will set this to tc_average_reading
double pid_output = 0;
double pid_setpoint = 0;
unsigned long window_start_time = 0;
PID pid(&pid_input, &pid_output, &pid_setpoint, kP, kI, kD, DIRECT);

// State
bool relay_on = false;
bool espresso_mode = true;
byte target_temp_espresso = 0;
byte target_temp_steam = 0;
unsigned long target_temp_last_checked;

// Loop time tracking
unsigned long time_now = 0;
unsigned long loop_delta = 0;

// EEPROM addresses
#define EEPROM_ADDR_TARGET_TEMP_ESPRESSO 0
#define EEPROM_ADDR_TARGET_TEMP_STEAM 1

// Forward declarations
void readThermocoupleTemperature();
void updatePIDOutput();
void updateRelayState();
void setRelay(bool enabled);
void updateEspressoMode();

void setup()
{
  pinMode(ESPRESSO_MODE_BUTTON_PIN, INPUT);
  pinMode(DECREASE_TEMPERATURE_BUTTON_PIN, INPUT);
  pinMode(INCREASE_TEMPERATURE_BUTTON_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);

  // Write EEPROM defaults
  //EEPROM.update(EEPROM_ADDR_TARGET_TEMP_ESPRESSO, 93);
  //EEPROM.update(EEPROM_ADDR_TARGET_TEMP_STEAM, 125);

  // Read EEPROM values
  target_temp_espresso = EEPROM.read(EEPROM_ADDR_TARGET_TEMP_ESPRESSO);
  target_temp_steam = EEPROM.read(EEPROM_ADDR_TARGET_TEMP_STEAM);

  // PID setup
  pid_setpoint = espresso_mode ? target_temp_espresso : target_temp_steam;
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(0, WINDOW_SIZE);
  pid.SetSampleTime(WINDOW_SIZE);
  window_start_time = millis();

  // Clear recent temps (for averaging)
  for (int reading = 0; reading < TC_NUM_READINGS; reading++)
  {
    tc_readings[reading] = 0;
  }

  target_temp_last_checked = millis();
}

void loop()
{
  unsigned long current_time = millis();
  loop_delta = current_time - time_now;
  time_now = current_time;

  readThermocoupleTemperature();
  updatePIDOutput();
  updateRelayState();
  updateEspressoMode();
}

void readThermocoupleTemperature()
{
  // Don't read from the thermocouple too often (it will lock up)
  if (tc_last_read_time + TC_DELAY_BETWEEN_READS > time_now)
  {
    return;
  }

  double current_reading = thermocouple.readCelsius();
  if (isnan(current_reading) || current_reading == 0 || current_reading > 170)
  {
    // Set current_reading to a high non-relay enabling temp
    current_reading = 170;
  }

  tc_last_read_time = time_now;
  tc_readings_total -= tc_readings[tc_reading_index];
  tc_readings[tc_reading_index] = current_reading;
  tc_readings_total += tc_readings[tc_reading_index];
  tc_reading_index = (tc_reading_index + 1) % TC_NUM_READINGS;
  tc_average_reading = tc_readings_total / TC_NUM_READINGS;

  #ifndef SIMULATION
  pid_input = tc_average_reading;
  #endif
}

void updatePIDOutput()
{
  if (time_now - window_start_time >= WINDOW_SIZE)
  {
    // Start new window
    window_start_time = time_now;

    // Force PID to re-compute
    pid.Compute(true);

    // Clamp PID output to a minimum to ensure SSR doesnt cycle on->off too quickly
    if (pid_output > 0 && pid_output < RELAY_MINIMUM_CYCLE_TIME)
    {
      pid_output = RELAY_MINIMUM_CYCLE_TIME;
    }
  }
}

void updateRelayState()
{
  // Dirty fail safe checks
  if (pid_input < 1 || pid_input > 140)
  {
    setRelay(false);
    return;
  }

  if (pid_output > time_now - window_start_time)
  {
    setRelay(true);
  }
  else
  {
    setRelay(false);
  }
}

void setRelay(bool enabled)
{
  relay_on = enabled;
  digitalWrite(RELAY_PIN, enabled);
}

void updateEspressoMode()
{
  bool espresso_mode_button_state = digitalRead(ESPRESSO_MODE_BUTTON_PIN) == LOW;

  if (espresso_mode_button_state != espresso_mode)
  {
    // Toggle between espresso and steam modes
    espresso_mode = espresso_mode_button_state;
    pid_setpoint = espresso_mode ? target_temp_espresso : target_temp_steam;
  }
}

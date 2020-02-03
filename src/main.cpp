#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <max6675.h>
#include <PID_v1.h>

//#define SIMULATION
//#define SERIAL_GRAPH

// Display
#define TFT_CS 10
#define TFT_RST 9
#define TFT_DC 8

// Thermocouple
#define TC_CLK 4
#define TC_CS 5
#define TC_MISO 6
#define TC_DELAY_BETWEEN_READS 250
#define TC_NUM_READINGS 4

// PID
#define kP 800
#define kI 0
#define kD 0
#define WINDOW_SIZE 1000  // milliseconds

// Misc
#define ESPRESSO_MODE_BUTTON_PIN 2
#define RELAY_PIN A5
#define RELAY_MINIMUM_CYCLE_TIME 20 // milliseconds
#define ESPRESSO_MODE_COLOR 0x02B3
#define STEAM_MODE_COLOR 0xC011
#define TARGET_TEMP_ESPRRESSO 93
#define TARGET_TEMP_STEAM 93

// Display
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

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
bool espresso_mode = true;
bool last_espresso_mode_button_state = false;

// Loop time tracking
unsigned long time_now = 0;
unsigned long loop_delta = 0;

// Misc
char temp_str[100];  // output formatting

// Forward declarations
void readThermocoupleTemperature();
void updatePIDOutput();
void updateRelayState();
void setRelay(bool enabled);

void updateEspressoMode();
void updateDisplay();
void updateEspressoModeDisplay();

void setup()
{
  Serial.begin(9600);
  #ifdef SERIAL_GRAPH
  delay(100);
  Serial.println("temp,power,target");
  #endif

  pinMode(ESPRESSO_MODE_BUTTON_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);

  // Init display
  tft.initR(INITR_GREENTAB);
  tft.fillScreen(ST7735_BLACK);
  tft.setTextWrap(false);

  // PID setup
  pid_setpoint = espresso_mode ? TARGET_TEMP_ESPRRESSO : TARGET_TEMP_STEAM;
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(0, WINDOW_SIZE);
  pid.SetSampleTime(WINDOW_SIZE);
  window_start_time = millis();

  // Clear recent temps (for averaging)
  for (int reading = 0; reading < TC_NUM_READINGS; reading++)
  {
    tc_readings[reading] = 0;
  }

  updateEspressoModeDisplay();
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
  updateDisplay();
}

void readThermocoupleTemperature()
{
  // Don't read from the thermocouple too often (it will lock up)
  if (tc_last_read_time + TC_DELAY_BETWEEN_READS > time_now)
  {
    return;
  }

  tc_last_read_time = time_now;
  tc_readings_total -= tc_readings[tc_reading_index];
  tc_readings[tc_reading_index] = thermocouple.readCelsius();
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
  if (pid_output > time_now - window_start_time)
  {
    setRelay(true);

    #ifdef SIMULATION
    pid_input += ((0.9 + (random(3) / 10.0)) * loop_delta / 1000.0);
    #endif
  }
  else
  {
    setRelay(false);

#ifdef SIMULATION
    pid_input += (-0.1 * loop_delta / 1000.0);
    if (pid_input < 22.0) {
      pid_input = 22.0;  // minimum temp
    }
    #endif
  }

  #ifdef SERIAL_GRAPH
  Serial.print(pid_input);
  Serial.print(",");
  Serial.print(pid_output * 100.0 / WINDOW_SIZE);  // scale
  Serial.print(",");
  Serial.println(pid_setpoint);
  #endif
}

void setRelay(bool enabled)
{
  digitalWrite(RELAY_PIN, enabled);

  if (enabled)
  {
    tft.setCursor(43, 118);
    tft.setTextColor(0xF9E7);
    tft.setTextSize(1);
    tft.print("HEATING");
  }
  else
  {
    tft.fillRect(1, 100, 126, 27, 0x0000);
  }
}

void updateEspressoMode()
{
  bool espresso_mode_button_state = digitalRead(ESPRESSO_MODE_BUTTON_PIN) == HIGH;

  if (espresso_mode_button_state && !last_espresso_mode_button_state)
  {
    // Toggle between espresso and steam modes
    espresso_mode = !espresso_mode;
    pid_setpoint = espresso_mode ? TARGET_TEMP_ESPRRESSO : TARGET_TEMP_STEAM;

    updateEspressoModeDisplay();

    #ifdef SIMULATION
    // Suddenly drop temp in simulation mode
    pid_input -= 20.0;
    #endif
  }

  last_espresso_mode_button_state = espresso_mode_button_state;
}

void updateEspressoModeDisplay()
{
  // force clear away old content
  tft.drawRect(0, 0, 128, 128, espresso_mode ? ESPRESSO_MODE_COLOR : STEAM_MODE_COLOR);
  tft.fillRect(0, 0, 128, 24, espresso_mode ? ESPRESSO_MODE_COLOR : STEAM_MODE_COLOR);

  tft.setCursor(0, 4);
  tft.setTextColor(0xffff);
  tft.setTextSize(1);
  tft.print(" Mode: ");

  if (espresso_mode)
  {
    tft.setTextColor(0xffff);
    tft.println("  ESPRESSO");
  }
  else
  {
    tft.setTextColor(0xffff);
    tft.println("  STEAM");
  }

  tft.setTextColor(0xffff);
  tft.setTextSize(1);
  tft.print(" Target: ");
  dtostrf(pid_setpoint, (pid_setpoint < 100 ? 4 : 5), 1, temp_str);
  tft.print(temp_str);
  tft.print(" deg");
}

void updateDisplay()
{
  tft.setCursor(28, 60);
  tft.setTextColor(0xffff, 0x0000);
  tft.setTextSize(3);

  dtostrf(pid_input, 4, 1, temp_str);
  tft.print(temp_str);
}

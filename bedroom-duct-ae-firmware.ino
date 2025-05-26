/*
 * This program is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by the 
 * Free Software Foundation, either version 3 of the License, or (at your 
 * option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY 
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for 
 * more details.
 * 
 * You should have received a copy of the GNU General Public License along 
 * with this program. If not, see <https://www.gnu.org/licenses/>.
 */

 /*
  Duct firmware for my house. Moves air from the AC unit to other rooms. Handles 
  the HRV/air exchanger in the bedroom. 

  Author: Andrew Somerville <andy16666@gmail.com> 
  GitHub: andy16666
  */

#include <FreeRTOS.h>
#include <task.h>
#include <LEAmDNS.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <stdint.h>
#include <float.h>
#include <math.h>
#include "DS18B20.h"
#include "config.h"
#include <CPU.h>
#include <string.h>

#define STACK_SIZE 200
#define CORE_0 (1 << 0)
#define CORE_1 (1 << 1)

#define LED_PIN CYW43_WL_GPIO_LED_PIN

#define TEMP_SENSOR_PIN 2
#define MAX_TEMP_C 60.0
#define MIN_TEMP_C -40.0

#define PING_INTERVAL_MS 5000
#define SENSOR_READ_INTERVAL_MS 2000
#define MAX_CONSECUTIVE_FAILED_PINGS 5


#define HRV_LOW_EXHAUST  6
#define HRV_LOW_INTAKE   7
#define HRV_HIGH_EXHAUST 8
#define HRV_HIGH_INTAKE  9

#define HRV_EXHAUST_BOOST 10
#define EXHAUST_BYPASS    11
#define ENTRYWAY_BYPASS   12
#define LR_DUCT_BOOST     13

#define HRV_LOW_EXHAUST_IDX  0
#define HRV_LOW_INTAKE_IDX   1
#define HRV_HIGH_EXHAUST_IDX 2
#define HRV_HIGH_INTAKE_IDX  3

#define HRV_EXHAUST_BOOST_IDX 4
#define EXHAUST_BYPASS_IDX    5
#define ENTRYWAY_BYPASS_IDX   6
#define LR_DUCT_BOOST_IDX     7


#define NUM_BLOWER_PINS   8

// Transition time for changes to fan speed and compressor state. This must be long enough
// for the circuit breaker to recover. 1s is much too short. 5s is close to the factory
// setting.
#define TRANSITION_TIME_MS 2000

#define HRV_INTAKE_INLET_TEMP_ADDR  139
#define HRV_INTAKE_OUTLET_TEMP_ADDR 131
#define HRV_EXHAUST_INLET_TEMP_ADDR 191
#define HRV_EXHAUST_OUTLET_TEMP_ADDR 205
#define INTAKE_TEMP_ADDR 12
#define LR_OUTLET_TEMP_ADDR 235
#define FEW_OUTLET_TEMP_ADDR 176

#define INVALID_TEMP FLT_MIN

#define HRV_INTAKE_INLET_IDX   0
#define HRV_INTAKE_OUTLET_IDX  1
#define HRV_EXHAUST_INLET_IDX  2
#define HRV_EXHAUST_OUTLET_IDX 3
#define INTAKE_IDX 4
#define LR_OUTLET_IDX 5
#define FEW_OUTLET_IDX 6
#define NUM_SENSORS 7

DS18B20 ds(TEMP_SENSOR_PIN);
CPU cpu; 

typedef enum {
  BLOWER_OFF = '0', 
  BLOWER_ON  = '1'
} blower_state_t;

const char* STATUS_JSON_FORMAT = 
  "{\n"\
  "  \"hrvIntakeInletTempC\":\"%f\",\n" \
  "  \"hrvIntakeOutletTempC\":\"%f\",\n" \
  "  \"hrvExhaustInletTempC\":\"%f\",\n" \
  "  \"hrvExhaustOutletTempC\":\"%f\",\n" \
  "  \"intakeTempC\":\"%f\",\n" \
  "  \"lrOutletTempC\":\"%f\",\n" \
  "  \"fewOutletTempC\":\"%f\",\n" \
  "  \"cpuTempC\":\"%f\",\n" \
  "  \"hrvCommand\":\"%c\",\n" \
  "  \"lrDuctCommand\":\"%c\",\n" \
  "  \"fewDuctCommand\":\"%c\",\n" \
  "  \"tempErrors\":%d,\n" \
  "  \"lastPing\":\"%fms\",\n" \
  "  \"consecutiveFailedPings\":%d,\n" \
  "  \"freeMem\":%d,\n" \
  "  \"powered\":\"%s\",\n" \
  "  \"booted\":\"%s\",\n" \
  "  \"connected\":\"%s\",\n" \
  "  \"numRebootsPingFailed\":%d,\n" \
  "  \"numRebootsDisconnected\":%d,\n" \
  "  \"wiFiStatus\":%d\n" \
  "}\n";


const uint8_t TEMP_SENSOR_ADDRESSES[(NUM_SENSORS)] = {
  HRV_INTAKE_INLET_TEMP_ADDR, 
  HRV_INTAKE_OUTLET_TEMP_ADDR, 
  HRV_EXHAUST_INLET_TEMP_ADDR,
  HRV_EXHAUST_OUTLET_TEMP_ADDR, 
  INTAKE_TEMP_ADDR, 
  LR_OUTLET_TEMP_ADDR, 
  FEW_OUTLET_TEMP_ADDR
};

float            TEMPERATURES_C[(NUM_SENSORS)]    = {INVALID_TEMP, INVALID_TEMP, INVALID_TEMP, INVALID_TEMP, INVALID_TEMP, INVALID_TEMP, INVALID_TEMP};
const char*      SENSOR_NAMES[(NUM_SENSORS)] = {"HRV Intake Inlet", "HRV Intake Outlet", "HRV Exhaust Inlet", "HRV Exhaust Outlet", "Intake", "Living Room Outlet", "Front Entryway Outlet"}; 

const pin_size_t BLOWER_PINS[(NUM_BLOWER_PINS)]  = {
  HRV_LOW_EXHAUST, 
  HRV_LOW_INTAKE, 
  HRV_HIGH_EXHAUST, 
  HRV_HIGH_INTAKE, 
  HRV_EXHAUST_BOOST, 
  EXHAUST_BYPASS, 
  ENTRYWAY_BYPASS,
  LR_DUCT_BOOST
}; 

const char*      BLOWER_NAMES[(NUM_BLOWER_PINS)] = {
  "HRV Low Exhaust", 
  "HRV Low Intake", 
  "HRV High Exhaust",
  "HRV High Intake", 
  "HRV Exhaust Boost",
  "Exhaust Bypass",
  "Front Entryway Bypass",
  "Living Room Duct Boost"
}; 

blower_state_t   BLOWER_STATES[(NUM_BLOWER_PINS)] = {
  BLOWER_OFF,
  BLOWER_OFF,
  BLOWER_OFF,
  BLOWER_OFF,
  BLOWER_OFF,
  BLOWER_OFF,
  BLOWER_OFF,
  BLOWER_OFF
}; 

blower_state_t   BLOWER_COMMANDS[(NUM_BLOWER_PINS)] = {
  BLOWER_OFF,
  BLOWER_OFF,
  BLOWER_OFF,
  BLOWER_OFF,
  BLOWER_OFF,
  BLOWER_OFF,
  BLOWER_OFF,
  BLOWER_OFF
};

WebServer server(80);

typedef enum {
  CMD_VENTILATE_HIGH = 'H',
  CMD_VENTILATE_MED  = 'M',
  CMD_VENTILATE_LOW  = 'L',
  CMD_VENTILATE_OFF  = 'O'
} ventilate_cmd_t;

typedef enum {
  CMD_LR_DUCT_HIGH = 'H',
  CMD_LR_DUCT_MED  = 'M',
  CMD_LR_DUCT_LOW  = 'L',
  CMD_LR_DUCT_OFF  = 'O'
} lr_duct_cmd_t;

typedef enum {
  CMD_ENTRYWAY_DUCT_HIGH = 'H',
  CMD_ENTRYWAY_DUCT_OFF  = 'O'
} few_duct_cmd_t;

static volatile long            initialize             __attribute__((section(".uninitialized_data")));
static volatile long            timeBaseMs             __attribute__((section(".uninitialized_data")));
static volatile long            powerUpTime            __attribute__((section(".uninitialized_data")));
static volatile long            tempErrors             __attribute__((section(".uninitialized_data")));
static volatile long            numRebootsPingFailed   __attribute__((section(".uninitialized_data")));
static volatile long            numRebootsDisconnected __attribute__((section(".uninitialized_data")));
static volatile ventilate_cmd_t ventilateCommand       __attribute__((section(".uninitialized_data")));
static volatile lr_duct_cmd_t   lrDuctCommand          __attribute__((section(".uninitialized_data")));
static volatile few_duct_cmd_t  fewDuctCommand         __attribute__((section(".uninitialized_data"))); 

volatile unsigned long lastStateChangeTimeMs = millis();
volatile unsigned long startupTime = millis();
volatile unsigned long connectTime = millis();
volatile int  lastPingMicros = 0; 
volatile unsigned long nextPingTimeMs = 0; 
volatile unsigned long nextSensorReadTime = 0;
volatile unsigned int consecutiveFailedPings = 0;  

void setup() 
{
  if (initialize)
  {
    timeBaseMs = 0; 
    powerUpTime = millis(); 
    tempErrors = 0; 
    ventilateCommand = CMD_VENTILATE_OFF; 
    lrDuctCommand = CMD_LR_DUCT_OFF; 
    fewDuctCommand = CMD_ENTRYWAY_DUCT_OFF; 
    numRebootsPingFailed = 0; 
    numRebootsDisconnected = 0; 
    initialize = 0; 
  }

  cpu.begin(); 

  pinMode(LED_PIN, OUTPUT); 

  Serial.begin();
  Serial.setDebugOutput(false);
  
  wifi_connect();
  server.begin();

  if (MDNS.begin(HOSTNAME)) {
    Serial.println("MDNS responder started");
  }

  MDNS.update();

  server.on("/", []() {
    for (uint8_t i = 0; i < server.args(); i++) {
      String argName = server.argName(i);
      String arg = server.arg(i);

      if (argName.equals("hrv") && arg.length() == 1) {
        switch(arg.charAt(0))
        {
          case 'O':  ventilateCommand = CMD_VENTILATE_OFF;  break;
          case 'L':  ventilateCommand = CMD_VENTILATE_LOW; break;  
          case 'M':  ventilateCommand = CMD_VENTILATE_MED; break;  
          case 'H':  ventilateCommand = CMD_VENTILATE_HIGH; break;  
          default:  ; 
        }
      }

      if (argName.equals("lr") && arg.length() == 1) {
        switch(arg.charAt(0))
        {
          case 'O':  lrDuctCommand = CMD_LR_DUCT_OFF;  break;
          case 'L':  lrDuctCommand = CMD_LR_DUCT_LOW; break;  
          case 'M':  lrDuctCommand = CMD_LR_DUCT_MED; break;  
          case 'H':  lrDuctCommand = CMD_LR_DUCT_HIGH; break;  
          default:   ;  
        }
      }

      if (argName.equals("ew") && arg.length() == 1) {
        switch(arg.charAt(0))
        {
          case 'O':  fewDuctCommand = CMD_ENTRYWAY_DUCT_OFF;  break;
          case 'H':  fewDuctCommand = CMD_ENTRYWAY_DUCT_HIGH; break;  
          default:   ;  
        }
      }

      if (argName.equals("bootloader") && arg.length() == 1) {
        switch(arg.charAt(0))
        {
          case 'A':  rp2040.rebootToBootloader();  break;
          default:   ;  
        }
      }

      if (argName.equals("reboot") && arg.length() == 1) {
        switch(arg.charAt(0))
        {
          case 'A':  reboot(); break;
          default:   ;  
        }
      }
    }

    unsigned long timeMs = millis(); 
    char* buffer = (char*)malloc(1024 * sizeof(char));
    char* poweredTimeStr = msToHumanReadableTime((timeBaseMs + timeMs) - powerUpTime); 
    char* bootedTimeStr = msToHumanReadableTime(timeMs - startupTime); 
    char* connectedTimeStr = msToHumanReadableTime(timeMs - connectTime); 
    sprintf(buffer, STATUS_JSON_FORMAT,
            TEMPERATURES_C[HRV_INTAKE_INLET_IDX],
            TEMPERATURES_C[HRV_INTAKE_OUTLET_IDX],
            TEMPERATURES_C[HRV_EXHAUST_INLET_IDX], 
            TEMPERATURES_C[HRV_EXHAUST_OUTLET_IDX], 
            TEMPERATURES_C[INTAKE_IDX],
            TEMPERATURES_C[LR_OUTLET_IDX],
            TEMPERATURES_C[FEW_OUTLET_IDX],
            cpu.getTemperature(),
            ventilateCommand,
            lrDuctCommand,
            fewDuctCommand,
            tempErrors,
            lastPingMicros/1000.0,
            consecutiveFailedPings,
            getFreeHeap(),
            poweredTimeStr, 
            bootedTimeStr,
            connectedTimeStr,
            numRebootsPingFailed,
            numRebootsDisconnected, 
            WiFi.status());
    free(poweredTimeStr); 
    free(bootedTimeStr); 
    free(connectedTimeStr); 
    server.send(200, "text/json", buffer);
    free(buffer);
  });

  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started");
  MDNS.update(); 
}

void loop() 
{
  if (!is_wifi_connected()) 
  {
    Serial.println("WiFi disconnected, rebooting.");
    numRebootsDisconnected++; 
    reboot();  
  }

  if (millis() > nextPingTimeMs)
  {
    unsigned long timeMicros = micros(); 
    int pingStatus = WiFi.ping(WiFi.gatewayIP(), 255); 
    lastPingMicros = micros() - timeMicros; 
    if (pingStatus < 0) 
    {
      if (++consecutiveFailedPings >= MAX_CONSECUTIVE_FAILED_PINGS)
      {
        Serial.println("Gateway unresponsive, rebooting.");
        numRebootsPingFailed++; 
        reboot(); 
      }
    }
    else 
    {
      consecutiveFailedPings = 0; 
    }

    nextPingTimeMs = millis() + PING_INTERVAL_MS; 
  }

  digitalWrite(LED_PIN, HIGH);
  server.handleClient();
  digitalWrite(LED_PIN, LOW);
  MDNS.update();
}

void setup1()
{
  for (int i = 0; i < NUM_BLOWER_PINS; i++)
  {
    pinMode(BLOWER_PINS[i], OUTPUT); 
  }

  for (int i = 0; i < NUM_BLOWER_PINS; i++)
  {
    digitalWrite(BLOWER_PINS[i], HIGH);
  }
}

void loop1()
{
  if (millis() > nextSensorReadTime)
  {
    readTemperatures(); 

    for (int i = 0; i < NUM_SENSORS; i++)
    {
      Serial.printf("%s: %fC\r\n", SENSOR_NAMES[i], TEMPERATURES_C[i]);
    }

    nextSensorReadTime = millis() + SENSOR_READ_INTERVAL_MS; 
  }

  processCommands(); 
}

void reboot()
{
  timeBaseMs += millis(); 
  rp2040.reboot(); 
}

unsigned long millisSincePowerOn()
{
  return timeBaseMs + millis(); 
}

bool is_wifi_connected() 
{
  return WiFi.status() == WL_CONNECTED;
}

void wifi_connect() 
{
  WiFi.noLowPowerMode();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.noLowPowerMode();
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int attempts = 20;
  while (!is_wifi_connected() && (attempts--)) 
  {
    delay(1000);
  }

  if (!is_wifi_connected()) {
    Serial.print("Connect failed! Rebooting.");
    numRebootsDisconnected++; 
    reboot(); 
  }

  Serial.println("Connected");
  Serial.print("Connected to ");
  Serial.println(WiFi.SSID());

  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  Serial.print("Hostname: ");
  Serial.print(HOSTNAME);
  Serial.println(".local");

  connectTime = millis();
}

void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";

  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }

  server.send(404, "text/plain", message);
}

void processCommands()
{
  setBlower(HRV_HIGH_INTAKE_IDX, 
      ventilateCommand == CMD_VENTILATE_HIGH 
      || ventilateCommand == CMD_VENTILATE_MED
      ? BLOWER_ON : BLOWER_OFF); 

  setBlower(HRV_LOW_INTAKE_IDX, 
      ventilateCommand != CMD_VENTILATE_OFF 
      ? BLOWER_ON : BLOWER_OFF); 

  setBlower(HRV_HIGH_EXHAUST_IDX, 
      ventilateCommand == CMD_VENTILATE_HIGH 
      || ventilateCommand == CMD_VENTILATE_MED
      || (
        ventilateCommand == CMD_VENTILATE_OFF 
        && (lrDuctCommand == CMD_LR_DUCT_HIGH || lrDuctCommand == CMD_LR_DUCT_MED)
      )
      ? BLOWER_ON : BLOWER_OFF); 

  setBlower(HRV_LOW_EXHAUST_IDX, 
      ventilateCommand != CMD_VENTILATE_OFF
      || (
        ventilateCommand == CMD_VENTILATE_OFF 
        && (lrDuctCommand == CMD_LR_DUCT_HIGH || lrDuctCommand == CMD_LR_DUCT_MED)
      )
      ? BLOWER_ON : BLOWER_OFF); 
    
  setBlower(HRV_EXHAUST_BOOST_IDX, 
      ventilateCommand == CMD_VENTILATE_HIGH 
      || (
        ventilateCommand == CMD_VENTILATE_OFF 
        && (lrDuctCommand == CMD_LR_DUCT_HIGH || lrDuctCommand == CMD_LR_DUCT_MED)
      )
      ? BLOWER_ON : BLOWER_OFF); 
    
  setBlower(EXHAUST_BYPASS_IDX, 
      ventilateCommand == CMD_VENTILATE_HIGH 
      || lrDuctCommand != CMD_LR_DUCT_OFF
      ? BLOWER_ON : BLOWER_OFF); 

  setBlower(LR_DUCT_BOOST_IDX, 
      ventilateCommand == CMD_VENTILATE_HIGH 
      || lrDuctCommand == CMD_LR_DUCT_HIGH
      ? BLOWER_ON : BLOWER_OFF); 

  setBlower(FEW_OUTLET_IDX, 
      ventilateCommand == CMD_VENTILATE_HIGH 
      || fewDuctCommand == CMD_ENTRYWAY_DUCT_HIGH
      ? BLOWER_ON : BLOWER_OFF); 
  
  if (updateBlowers())
    printBlowerStates(); 
}

int setBlower(int blowerIndex, blower_state_t state)
{
  if (BLOWER_COMMANDS[blowerIndex] != state)
  {
    BLOWER_COMMANDS[blowerIndex] = state; 
    return 1; 
  }

  return 0; 
}

int updateBlowers()
{
  int updated = 0; 

  for (int i = 0; i < NUM_BLOWER_PINS; i++)
  {
    if (BLOWER_COMMANDS[i] != BLOWER_STATES[i]) {
      updated++; 
      digitalWrite(BLOWER_PINS[i], BLOWER_COMMANDS[i] == BLOWER_OFF ? HIGH : LOW); 
      BLOWER_STATES[i] = BLOWER_COMMANDS[i]; 
    }
  }

  if (updated)
  {
    delay(TRANSITION_TIME_MS);
  }

  return updated; 
}

void printBlowerStates()
{
  for (int i = 0; i < NUM_BLOWER_PINS; i++)
  {
    Serial.printf("[%s=%c>%c]", BLOWER_NAMES[i], BLOWER_COMMANDS[i], BLOWER_STATES[i]); 
  }
  Serial.println(""); 
}


// TEMP SENSORS

int isTempCValid(float tempC) 
{
  return tempC != INVALID_TEMP
    && tempC == tempC 
    && tempC < MAX_TEMP_C 
    && tempC > MIN_TEMP_C; 
}

void readTemperatures() 
{
  printSensorAddresses(); 

  int nSensors = ds.getNumberOfDevices(); 
  float newTempC[256]; 

  for (int i = 0; i < 256; i++) { newTempC[i] = INVALID_TEMP; }
  
  while (ds.selectNext()) {
    uint8_t sensorAddress[8];
    ds.getAddress(sensorAddress);
    float reading1C = ds.getTempC(); 
    float reading2C = ds.getTempC(); 
    float diff = fabs(reading1C - reading2C); 
    if (diff < 0.1 && isTempCValid(reading1C))
    {
      newTempC[sensorAddress[7]] = reading1C; 
    }
    else 
    {
      Serial.printf("TEMP ERROR: %f %f\r\n", reading1C, reading2C); 
      tempErrors++; 
    }
  }
  
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    if (newTempC[TEMP_SENSOR_ADDRESSES[i]] != INVALID_TEMP)
    {
      TEMPERATURES_C[i] = newTempC[TEMP_SENSOR_ADDRESSES[i]]; 
    }
  }
}

/* Reads temperature sensors */
void printSensorAddresses()
{
  int nSensors = ds.getNumberOfDevices();
  if (nSensors == 0)
  {
    Serial.println("No sensors detected"); 
    return; 
  }

  char buffer[1024]; 
  buffer[0] = 0; 
  while (ds.selectNext()) {
    uint8_t sensorAddress[8];
    ds.getAddress(sensorAddress);
    sprintf(buffer + strlen(buffer), "%d ", sensorAddress[7]); 
  }
  Serial.println(buffer); 
}

// Heap 

uint32_t getTotalHeap(void) {
  extern char __StackLimit, __bss_end__;

  return &__StackLimit - &__bss_end__;
}

uint32_t getFreeHeap(void) {
  struct mallinfo m = mallinfo();

  return getTotalHeap() - m.uordblks;
}

// Util 

char *msToHumanReadableTime(long timeMs)
{
  char buffer[1024]; 

  if (timeMs < 1000)
  {
    sprintf(buffer, "%dms", timeMs);
  }
  else if (timeMs < 60 * 1000)
  {
    sprintf(buffer, "%5.2fs", timeMs / 1000.0);
  }
  else if (timeMs < 60 * 60 * 1000)
  {
    sprintf(buffer, "%5.2fm", timeMs / (60.0 * 1000.0));
  }
  else if (timeMs < 24 * 60 * 60 * 1000)
  {
    sprintf(buffer, "%5.2fh", timeMs / (60.0 * 60.0 * 1000.0));
  }
  else
  {
    sprintf(buffer, "%5.2fd", timeMs / (24.0 * 60.0 * 60.0 * 1000.0));
  }

  int allocChars = strlen(buffer) + 1; 

  char *formattedString = (char *)malloc(allocChars * sizeof(char)); 

  strcpy(formattedString, buffer); 

  return formattedString; 
}


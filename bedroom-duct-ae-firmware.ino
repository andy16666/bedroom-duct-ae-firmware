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
#include <Arduino_JSON.h>
#include <LEAmDNS.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <stdint.h>
#include <float.h>
#include <math.h>
#include "config.h"
#include <CPU.h>
#include <string.h>
#include <GPIOOutputs.h>
#include <Thermostats.h>
#include <TemperatureSensors.h>
#include <GPIOOutputs.h>
#include <util.h>
#include <SimplicityAC.h> 
extern "C" {
#include <threadkernel.h>
};

using namespace AOS; 
using AOS::Thermostat; 
using AOS::Thermostats;

#define TEMP_SENSOR_PIN 2

#define PING_INTERVAL_MS 5000
#define SENSOR_READ_INTERVAL_MS 6000
#define MAX_CONSECUTIVE_FAILED_PINGS 5

#define HRV_LOW_EXHAUST  6
#define HRV_LOW_INTAKE   7
#define HRV_HIGH_EXHAUST 8
#define HRV_HIGH_INTAKE  9

#define HRV_EXHAUST_BOOST 10
#define EXHAUST_BYPASS    11
#define ENTRYWAY_BYPASS   12
#define LR_DUCT_BOOST     13

#define CORE_0_ACT   18
#define CORE_1_ACT     19

#define TRANSITION_TIME_MS 2000

#define HRV_INTAKE_INLET_TEMP_ADDR  139
#define HRV_INTAKE_OUTLET_TEMP_ADDR 131
#define HRV_EXHAUST_INLET_TEMP_ADDR 191
#define HRV_EXHAUST_OUTLET_TEMP_ADDR 205
//#define INTAKE_TEMP_ADDR 83
//#define LR_OUTLET_TEMP_ADDR 226
//#define FEW_OUTLET_TEMP_ADDR 181
#define INTAKE_TEMP_ADDR 12
#define LR_OUTLET_TEMP_ADDR 235
#define FEW_OUTLET_TEMP_ADDR 176

CPU cpu; 
TemperatureSensors TEMPERATURES(TEMP_SENSOR_PIN);
GPIOOutputs BLOWERS("Blowers");
WebServer server(80);
AOS::Thermostats thermostats;
SimplicityAC acData(AC_HOSTNAME); 

const char* STATUS_JSON_FORMAT = 
  "{\n"\
  "  \"acData\":{\n" \
  "    \"evapTempC\":\"%f\",\n" \
  "    \"outletTempC\":\"%f\",\n" \
  "    \"lastUpdated\":\"%s\",\n" \
  "    \"command\":\"%c\",\n" \
  "    \"state\":\"%c\",\n" \
  "    \"fanState\":\"%c\",\n" \
  "    \"compressorState\":\"%d\"\n" \
  "  },\n" \
  "  \"hrvIntakeInletTempC\":\"%s\",\n" \
  "  \"hrvIntakeOutletTempC\":\"%s\",\n" \
  "  \"hrvExhaustInletTempC\":\"%s\",\n" \
  "  \"hrvExhaustOutletTempC\":\"%s\",\n" \
  "  \"intakeTempC\":\"%s\",\n" \
  "  \"lrOutletTempC\":\"%s\",\n" \
  "  \"fewOutletTempC\":\"%s\",\n" \
  "  \"hrvCommand\":\"%c\",\n" \
  "  \"lrDuctCommand\":\"%c\",\n" \
  "  \"fewDuctCommand\":\"%c\",\n" \
  "  \"cpuTempC\":\"%f\",\n" \
  "  \"tempErrors\":%d,\n" \
  "  \"thermostats\":%s,\n" \
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

static volatile ventilate_cmd_t hrvCommand             __attribute__((section(".uninitialized_data")));
static volatile lr_duct_cmd_t   lrDuctCommand          __attribute__((section(".uninitialized_data")));
static volatile few_duct_cmd_t  fewDuctCommand         __attribute__((section(".uninitialized_data"))); 
static volatile ac_cmd_t        acCommand              __attribute__((section(".uninitialized_data"))); 

volatile unsigned long startupTime = millis();
volatile unsigned long connectTime = millis();

volatile unsigned long lastPingMicros = 0; 
volatile unsigned int consecutiveFailedPings = 0; 

threadkernel_t* CORE_0_KERNEL; 
threadkernel_t* CORE_1_KERNEL;

volatile char *httpJsonOutput; 

void setup() 
{
  CORE_0_KERNEL = create_threadkernel(&millis); 

  if (initialize)
  {
    timeBaseMs = 0; 
    powerUpTime = millis(); 
    tempErrors = 0; 
    hrvCommand = CMD_VENTILATE_OFF; 
    lrDuctCommand = CMD_LR_DUCT_OFF; 
    fewDuctCommand = CMD_ENTRYWAY_DUCT_OFF; 
    numRebootsPingFailed = 0; 
    numRebootsDisconnected = 0; 
    initialize = 0; 
    acCommand = CMD_AC_OFF; 
  }

  pinMode(CORE_0_ACT, OUTPUT); 

  httpJsonOutput = 0; 

  thermostats.add("lr");  
  thermostats.add("mb"); 
  thermostats.add("few"); 

  cpu.begin(); 

  Serial.begin();
  Serial.setDebugOutput(false);
  
  wifi_connect();
  server.begin();

  if (MDNS.begin(HOSTNAME)) {
    Serial.println("MDNS responder started");
  }

  server.on("/", []() {
    bool success = true; 

    for (uint8_t i = 0; i < server.args(); i++) 
    {
      String argName = server.argName(i);
      String arg = server.arg(i);

      if (argName.equals("hrv") && arg.length() == 1) 
      {
        switch(arg.charAt(0))
        {
          case 'O':  hrvCommand = CMD_VENTILATE_OFF;  break;
          case 'L':  hrvCommand = CMD_VENTILATE_LOW; break;  
          case 'M':  hrvCommand = CMD_VENTILATE_MED; break;  
          case 'H':  hrvCommand = CMD_VENTILATE_HIGH; break;  
          default:   success = false; 
        }
      }

      if (argName.equals("lr") && arg.length() == 1) 
      {
        switch(arg.charAt(0))
        {
          case 'O':  lrDuctCommand = CMD_LR_DUCT_OFF;  break;
          case 'L':  lrDuctCommand = CMD_LR_DUCT_LOW; break;  
          case 'M':  lrDuctCommand = CMD_LR_DUCT_MED; break;  
          case 'H':  lrDuctCommand = CMD_LR_DUCT_HIGH; break;  
          default:   success = false;  
        }
      }

      if (argName.equals("ew") && arg.length() == 1) 
      {
        switch(arg.charAt(0))
        {
          case 'O':  fewDuctCommand = CMD_ENTRYWAY_DUCT_OFF;  break;
          case 'H':  fewDuctCommand = CMD_ENTRYWAY_DUCT_HIGH; break;  
          default:   success = false;  
        }
      }

      if (argName.equals("bootloader") && arg.length() == 1) 
      {
        switch(arg.charAt(0))
        {
          case 'A':  rp2040.rebootToBootloader();  break;
          default:   success = false;  
        }
      }

      if (argName.equals("reboot") && arg.length() == 1) 
      {
        switch(arg.charAt(0))
        {
          case 'A':  reboot(); break;
          default:   success = false;  
        }
      }

      if (argName.equals("thset") || argName.equals("thcur") || argName.equals("thcmd")) 
      {
        char buffer[strlen(arg.c_str()) + 1]; 
        strcpy(buffer, arg.c_str()); 

        char* roomName = strtok(buffer, "_");
        char* argumentStr = strtok(NULL, "_");
    
        if (roomName && argumentStr && thermostats.contains(roomName))
        {
          Serial.printf("%s: %s %s\r\n", argName.c_str(), roomName, argumentStr); 

          if (argName.equals("thset"))
            thermostats.get(roomName).setSetPointC(atoff(argumentStr)); 
          
          if (argName.equals("thcur"))
            thermostats.get(roomName).setCurrentTemperatureC(atoff(argumentStr));
          
          if (argName.equals("thcmd"))
            thermostats.get(roomName).setCommand(argumentStr[0] == '1');  
        }
        else 
        {
          success = false; 
        }
      }
    }

    if (success)
    {
      char * buffer = (char *)httpJsonOutput; 
      server.send(200, "text/json", buffer);
    }
    else 
    {
      server.send(500, "text/plain", "Failed to parse request.");
    }
  });

  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started");
  
  CORE_0_KERNEL->addImmediate(CORE_0_KERNEL, task_mdnsUpdate); 
  CORE_0_KERNEL->addImmediate(CORE_0_KERNEL, task_handleClient); 

  CORE_1_KERNEL->add(CORE_1_KERNEL, task_core0ActOn, 10); 
  CORE_0_KERNEL->add(CORE_0_KERNEL, task_testPing, PING_INTERVAL_MS); 
  CORE_0_KERNEL->add(CORE_0_KERNEL, task_testWiFiConnection, 5000); 
  CORE_0_KERNEL->add(CORE_0_KERNEL, task_processThermostatData, 15000); 
  //CORE_0_KERNEL->add(CORE_0_KERNEL, task_readTemperatures, SENSOR_READ_INTERVAL_MS); 
  //CORE_0_KERNEL->add(CORE_0_KERNEL, task_bufferJsonOutput, 2000); 
  CORE_1_KERNEL->add(CORE_1_KERNEL, task_core0ActOff, 10); 
}

void loop() 
{
  CORE_0_KERNEL->run(CORE_0_KERNEL); 
}

void setup1()
{
  CORE_1_KERNEL = create_threadkernel(&millis); 
  pinMode(CORE_1_ACT, OUTPUT); 

  TEMPERATURES.add("HRV Intake Inlet", HRV_INTAKE_INLET_TEMP_ADDR); 
  TEMPERATURES.add("HRV Intake Outlet", HRV_INTAKE_OUTLET_TEMP_ADDR); 
  TEMPERATURES.add("HRV Exhaust Inlet", HRV_EXHAUST_INLET_TEMP_ADDR); 
  TEMPERATURES.add("HRV Exhaust Outlet", HRV_EXHAUST_OUTLET_TEMP_ADDR); 
  TEMPERATURES.add("Intake", INTAKE_TEMP_ADDR); 
  TEMPERATURES.add("Living Room Outlet", LR_OUTLET_TEMP_ADDR); 
  TEMPERATURES.add("Front Entryway Outlet", FEW_OUTLET_TEMP_ADDR);
  TEMPERATURES.discoverSensors();

  BLOWERS.add("HRV Low Exhaust", HRV_LOW_EXHAUST, false);
  BLOWERS.add("HRV Low Intake", HRV_LOW_INTAKE, false);
  BLOWERS.add("HRV High Exhaust", HRV_HIGH_EXHAUST, false);
  BLOWERS.add("HRV High Intake", HRV_HIGH_INTAKE, false);
  BLOWERS.add("HRV Exhaust Boost", HRV_EXHAUST_BOOST, false);
  BLOWERS.add("Exhaust Bypass", EXHAUST_BYPASS, false);
  BLOWERS.add("Entryway Bypass", ENTRYWAY_BYPASS, false);
  BLOWERS.add("Livingroom Duct Boost", LR_DUCT_BOOST, false);
  BLOWERS.setAll();

  CORE_1_KERNEL->addImmediate(CORE_1_KERNEL, task_processCommands);  
  CORE_1_KERNEL->add(CORE_1_KERNEL, task_core1ActOn, 10); 
  CORE_1_KERNEL->add(CORE_1_KERNEL, task_readTemperatures, 10000); 
  CORE_1_KERNEL->add(CORE_1_KERNEL, task_updateNextBlower, TRANSITION_TIME_MS); 
  CORE_1_KERNEL->add(CORE_1_KERNEL, task_bufferJsonOutput, 2000); 
  CORE_1_KERNEL->add(CORE_1_KERNEL, task_core1ActOff, 10);
}

void loop1()
{
  CORE_1_KERNEL->run(CORE_1_KERNEL); 
}

void task_core0ActOn()
{
  digitalWrite(CORE_0_ACT, 1);
}

void task_core1ActOn()
{
  digitalWrite(CORE_1_ACT, 1);
}

void task_core0ActOff()
{
  digitalWrite(CORE_0_ACT, 0);
}

void task_core1ActOff()
{
  digitalWrite(CORE_1_ACT, 0);
}

void task_bufferJsonOutput()
{
  char *oldJsonBuffer = (char *)httpJsonOutput; 
  char *jsonBuffer = (char *)malloc(sizeof(char) * 2 * 1024); 
  unsigned long timeMs = millis(); 
  char* poweredTimeStr = msToHumanReadableTime((timeBaseMs + timeMs) - powerUpTime); 
  char* bootedTimeStr = msToHumanReadableTime(timeMs - startupTime); 
  char* connectedTimeStr = msToHumanReadableTime(timeMs - connectTime); 
  char* acDataUpdatedTimeStr = msToHumanReadableTime(timeMs - acData.updateTimeMs); 
  char* formattedThermostats = thermostats.toString(); 
  sprintf(jsonBuffer, STATUS_JSON_FORMAT,
          acData.evapTempC,
          acData.outletTempC, 
          acDataUpdatedTimeStr,
          acData.command, 
          acData.state, 
          acData.fanState, 
          acData.compressorState,
          TEMPERATURES.formatTempC(HRV_INTAKE_INLET_TEMP_ADDR).c_str(),
          TEMPERATURES.formatTempC(HRV_INTAKE_OUTLET_TEMP_ADDR).c_str(),
          TEMPERATURES.formatTempC(HRV_EXHAUST_INLET_TEMP_ADDR).c_str(),
          TEMPERATURES.formatTempC(HRV_EXHAUST_OUTLET_TEMP_ADDR).c_str(),
          TEMPERATURES.formatTempC(INTAKE_TEMP_ADDR).c_str(),
          TEMPERATURES.formatTempC(LR_OUTLET_TEMP_ADDR).c_str(),
          TEMPERATURES.formatTempC(FEW_OUTLET_TEMP_ADDR).c_str(),
          hrvCommand,
          lrDuctCommand,
          fewDuctCommand,
          cpu.getTemperature(),
          tempErrors,
          formattedThermostats, 
          lastPingMicros/1000.0,
          consecutiveFailedPings,
          getFreeHeap(),
          poweredTimeStr, 
          bootedTimeStr,
          connectedTimeStr,
          numRebootsPingFailed,
          numRebootsDisconnected, 
          3);
  free(poweredTimeStr); 
  free(bootedTimeStr); 
  free(connectedTimeStr); 
  free(formattedThermostats);  
  free(acDataUpdatedTimeStr);
  httpJsonOutput = jsonBuffer; 
  free(oldJsonBuffer); 
}

void task_processThermostatData()
{
  if (acData.isExpired() && !acData.execute())
  {
    return; 
  }

  acCommand = acData.command; 

  Thermostat lr  = thermostats.get("lr"); 
  Thermostat mb  = thermostats.get("mb"); 
  Thermostat few = thermostats.get("few"); 

  if (!lr.isCurrent() || !mb.isCurrent() || !few.isCurrent() || !TEMPERATURES.ready())
    return;

  if (few.coolingCalledFor())
  {
    acCommand = mb.heatCalledFor() 
      ? ((mb.getMagnitude() >= 0.5) 
        ? CMD_AC_COOL_LOW : CMD_AC_COOL_MED
      ) 
      : CMD_AC_COOL_HIGH; 
    fewDuctCommand = CMD_ENTRYWAY_DUCT_HIGH; 
  }
  else
  {
    fewDuctCommand = CMD_ENTRYWAY_DUCT_OFF; 
  }

  if (lr.coolingCalledFor())
  {
    if (mb.heatCalledFor())
    {
      acCommand = (mb.getMagnitude() >= 0.5) ? CMD_AC_COOL_LOW : CMD_AC_COOL_MED; 
      lrDuctCommand = CMD_LR_DUCT_HIGH; 
    }
    else 
    {
      acCommand = CMD_AC_COOL_HIGH; 
      if (lr.getMagnitude() >= 0.5)
      {
        lrDuctCommand = CMD_LR_DUCT_HIGH; 
      }
      else if (lr.getMagnitude() >= 0.25)
      {
        lrDuctCommand = CMD_LR_DUCT_MED; 
      }
    }
  }
  else 
  {
    lrDuctCommand = CMD_LR_DUCT_OFF; 
  }

  if (mb.coolingCalledFor()) 
  {
    if(mb.getMagnitude() >= 1.0 || lr.coolingCalledFor())
    {
      acCommand = CMD_AC_COOL_HIGH; 
    }
    else if (mb.getMagnitude() >= 0.5)
    {
      acCommand = CMD_AC_COOL_MED; 
    }
    else if (mb.getMagnitude() >= 0.25)
    {
      acCommand = CMD_AC_COOL_LOW; 
    }
  }

  bool coolOff = !lr.coolingCalledFor() && !mb.coolingCalledFor() && !few.coolingCalledFor(); 

  if (coolOff)
  {
    if (acData.isOutletCold())
    {
      acCommand = acData.isCooling() ? CMD_AC_OFF : CMD_AC_FAN; 
    }
    else if (acData.isEvapCold())
    {
      acCommand = acData.isCooling() ? CMD_AC_OFF : CMD_AC_FAN; 
    }
    else
    {
      acCommand = CMD_AC_KILL; 
    }

    if (!acData.isOutletCold() && !acData.isCooling())
    {
      if (TEMPERATURES.getTempC(INTAKE_TEMP_ADDR) < 15)
      {
        lrDuctCommand = CMD_LR_DUCT_LOW; 
      }
      else if (TEMPERATURES.getTempC(LR_OUTLET_TEMP_ADDR) < 15 && TEMPERATURES.getTempC(INTAKE_TEMP_ADDR) > 15)
      {
        lrDuctCommand = CMD_LR_DUCT_LOW; 
      }
      else 
      {
        lrDuctCommand = CMD_LR_DUCT_OFF;
      }

      if (TEMPERATURES.getTempC(FEW_OUTLET_TEMP_ADDR) < 15 && TEMPERATURES.getTempC(INTAKE_TEMP_ADDR) > 15)
      {
        fewDuctCommand = CMD_ENTRYWAY_DUCT_HIGH; 
      }
    }
  }

  if (!acData.isCommand(acCommand))
  {
    acData.execute(acCommand); 
  }
}

void task_handleClient()
{
  server.handleClient(); 
}

void task_mdnsUpdate()
{
  MDNS.update(); 
}

void task_testWiFiConnection()
{
  if (!is_wifi_connected()) 
  {
    Serial.println("WiFi disconnected, rebooting.");
    numRebootsDisconnected++; 
    reboot();  
  }
}

void task_testPing()
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
}

void task_processCommands()
{
  processCommands(hrvCommand, lrDuctCommand, fewDuctCommand);
}

void task_readTemperatures()
{
  TEMPERATURES.readSensors();
}

void task_printTemperatures()
{
  TEMPERATURES.printSensors(); 
}

void task_updateNextBlower()
{
  BLOWERS.setNext();
}

void reboot()
{
  timeBaseMs += millis(); 
  rp2040.reboot(); 
}

void processCommands(ventilate_cmd_t hrvCommand, lr_duct_cmd_t lrDuctCommand, few_duct_cmd_t fewDuctCommand)
{
  BLOWERS.setCommand(HRV_HIGH_INTAKE, 
      hrvCommand == CMD_VENTILATE_HIGH 
      || hrvCommand == CMD_VENTILATE_MED); 

  BLOWERS.setCommand(HRV_LOW_INTAKE, 
      hrvCommand != CMD_VENTILATE_OFF); 

  BLOWERS.setCommand(HRV_HIGH_EXHAUST, 
      hrvCommand == CMD_VENTILATE_HIGH 
      || hrvCommand == CMD_VENTILATE_MED
      || (
        hrvCommand == CMD_VENTILATE_OFF 
        && (lrDuctCommand == CMD_LR_DUCT_HIGH || lrDuctCommand == CMD_LR_DUCT_MED)
      )); 

  BLOWERS.setCommand(HRV_LOW_EXHAUST, 
      hrvCommand != CMD_VENTILATE_OFF
      || (
        hrvCommand == CMD_VENTILATE_OFF 
        && (lrDuctCommand == CMD_LR_DUCT_HIGH || lrDuctCommand == CMD_LR_DUCT_MED)
      )); 
    
  BLOWERS.setCommand(HRV_EXHAUST_BOOST, 
      hrvCommand == CMD_VENTILATE_HIGH 
      || (
        hrvCommand == CMD_VENTILATE_OFF 
        && (lrDuctCommand == CMD_LR_DUCT_HIGH || lrDuctCommand == CMD_LR_DUCT_MED)
      )); 
    
  BLOWERS.setCommand(EXHAUST_BYPASS, 
      hrvCommand == CMD_VENTILATE_HIGH 
      || lrDuctCommand != CMD_LR_DUCT_OFF); 

  BLOWERS.setCommand(LR_DUCT_BOOST, 
      hrvCommand == CMD_VENTILATE_HIGH 
      || lrDuctCommand == CMD_LR_DUCT_HIGH); 

  BLOWERS.setCommand(ENTRYWAY_BYPASS, 
      hrvCommand == CMD_VENTILATE_HIGH 
      || fewDuctCommand == CMD_ENTRYWAY_DUCT_HIGH); 
}

// WIFI

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

// AC
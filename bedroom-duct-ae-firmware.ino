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

#include <stdint.h>
#include <float.h>
#include <math.h>
#include <string.h>

#include <CPU.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClient.h>

#include <aos.h>
#include <config.h>
#include <GPIOOutputs.h>
#include <Thermostats.h>
#include <SimplicityAC.h> 

using namespace AOS;

using AOS::Thermostat; 
using AOS::Thermostats;
using AOS::TemperatureSensors;
using AOS::Ping; 

#define HRV_LOW_EXHAUST  6
#define HRV_LOW_INTAKE   7
#define HRV_HIGH_EXHAUST 8
#define HRV_HIGH_INTAKE  9

#define HRV_EXHAUST_BOOST 10
#define EXHAUST_BYPASS    11
#define ENTRYWAY_BYPASS   12
#define LR_DUCT_BOOST     13

#define TRANSITION_TIME_MS 2000

#define HRV_INTAKE_INLET_TEMP_ADDR  139
#define HRV_INTAKE_OUTLET_TEMP_ADDR 131
#define HRV_EXHAUST_INLET_TEMP_ADDR 191
#define HRV_EXHAUST_OUTLET_TEMP_ADDR 205
#define INTAKE_TEMP_ADDR 12
#define LR_OUTLET_TEMP_ADDR 235
#define FEW_OUTLET_TEMP_ADDR 176

GPIOOutputs BLOWERS("Blowers");
WebServer server(80);
SimplicityAC acData(AC_HOSTNAME); 
AOS::Thermostats thermostats;

const char* LIVING_ROOM    = "lr"; 
const char* MASTER_BEDROOM = "mb"; 
const char* FRONT_ENTRYWAY = "few"; 

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

static volatile ventilate_cmd_t hrvCommand             __attribute__((section(".uninitialized_data")));
static volatile lr_duct_cmd_t   lrDuctCommand          __attribute__((section(".uninitialized_data")));
static volatile few_duct_cmd_t  fewDuctCommand         __attribute__((section(".uninitialized_data"))); 
static volatile ac_cmd_t        acCommand              __attribute__((section(".uninitialized_data"))); 
static volatile double          outletHumidity         __attribute__((section(".uninitialized_data")));
static volatile double          ductIntakeHumidity     __attribute__((section(".uninitialized_data")));
static volatile double          roomHumidity           __attribute__((section(".uninitialized_data")));

volatile long coolingOffMs = millis(); 

const char* generateHostname()
{
  return "hrv1"; 
}

void aosInitialize()
{
  hrvCommand = CMD_VENTILATE_OFF; 
  lrDuctCommand = CMD_LR_DUCT_OFF; 
  fewDuctCommand = CMD_ENTRYWAY_DUCT_OFF; 
  acCommand = CMD_AC_OFF; 
  outletHumidity = 0; 
  ductIntakeHumidity = 0;
  roomHumidity = 0; 
}

void aosSetup()
{
  thermostats.add(LIVING_ROOM);  
  thermostats.add(MASTER_BEDROOM); 
  thermostats.add(FRONT_ENTRYWAY); 

  server.begin();

  server.on("/", []() {
    bool success = true; 

    for (uint8_t i = 0; i < server.args(); i++) 
    {
      String argName = server.argName(i);
      String arg = server.arg(i);
      bool success = false; 

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

      if (argName.equals(LIVING_ROOM) && arg.length() == 1) 
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
      
      server.send(200, "text/json", httpResponseString);
    }
    else 
    {
      server.send(500, "text/plain", "Failed to parse request.");
    }
  });

  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started");
  
  CORE_0_KERNEL->addImmediate(CORE_0_KERNEL, task_handleClient); 

  CORE_0_KERNEL->add(CORE_0_KERNEL, task_processThermostatData, 15000); 
}

void aosSetup1()
{
  TEMPERATURES.add("HRV Intake Inlet", "hrvIntakeInletTempC", HRV_INTAKE_INLET_TEMP_ADDR); 
  TEMPERATURES.add("HRV Intake Outlet", "hrvIntakeOutletTempC", HRV_INTAKE_OUTLET_TEMP_ADDR); 
  TEMPERATURES.add("HRV Exhaust Inlet", "hrvExhaustInletTempC", HRV_EXHAUST_INLET_TEMP_ADDR); 
  TEMPERATURES.add("HRV Exhaust Outlet", "hrvExhaustOutletTempC", HRV_EXHAUST_OUTLET_TEMP_ADDR); 
  TEMPERATURES.add("Intake", "intakeTempC", INTAKE_TEMP_ADDR); 
  TEMPERATURES.add("Living Room Outlet", "lrOutletTempC", LR_OUTLET_TEMP_ADDR); 
  TEMPERATURES.add("Front Entryway Outlet", "fewOutletTempC", FEW_OUTLET_TEMP_ADDR);

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
  CORE_1_KERNEL->add(CORE_1_KERNEL, task_updateNextBlower, TRANSITION_TIME_MS); 
}

void populateHttpResponse(JSONVar& document) 
{
  document["intakeHumidity"] = ductIntakeHumidity; 

  document["hrvCommand"] = String((char)hrvCommand).c_str();
  document["lrDuctCommand"] = String((char)lrDuctCommand).c_str();
  document["fewDuctCommand"] = String((char)fewDuctCommand).c_str();
  document["acCommand"] = String((char)acCommand).c_str();

  acData.addTo(AC_HOSTNAME, document); 
  document[AC_HOSTNAME]["outletHumidity"] = outletHumidity; 
  document[AC_HOSTNAME]["roomHumidity"]   = roomHumidity; 

  thermostats.addTo("thermostats", document); 
}

void task_processThermostatData()
{
  if (acData.isExpired() && !acData.execute())
  {
    return; 
  }

  acCommand = acData.command; 

  Thermostat lr  = thermostats.get(LIVING_ROOM); 
  Thermostat mb  = thermostats.get(MASTER_BEDROOM); 
  Thermostat few = thermostats.get(FRONT_ENTRYWAY); 

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
    if (acData.isCooling())
    {
      coolingOffMs = millis(); 
    }

    boolean coolingOffForMs = millis() - coolingOffMs; 

    bool cmdACFanOn = acData.isOutletCold() || acData.isCooling(); 
    bool cmdDuctFanOn = !acData.isOutletCold() && !acData.isCooling(); 
    if (!acData.isCooling() && acData.isFanRunning() && TEMPERATURES.isTempValid(INTAKE_TEMP_ADDR))
    {
      double ductIntakeTempC = TEMPERATURES.getTempC(INTAKE_TEMP_ADDR);
      if (acData.isEvapTempValid())
      {
        double evapTempC = acData.getEvapTempC(); 
        ductIntakeHumidity = calculate_relative_humidity(ductIntakeTempC, evapTempC); 
        cmdDuctFanOn = cmdDuctFanOn || ductIntakeHumidity > 75.0;  
        if (acData.isOutletTempValid())
        {
          double outletTempC = acData.getOutletTempC(); 
          outletHumidity = calculate_relative_humidity(outletTempC, evapTempC);
          
          cmdACFanOn   = cmdACFanOn || outletTempC < evapTempC || outletHumidity > 75.0; 
        }

        if (thermostats.contains(MASTER_BEDROOM) && thermostats.get(MASTER_BEDROOM).isCurrent())
        {
          double roomTemperatureC = thermostats.get(MASTER_BEDROOM).getCurrentTemperatureC(); 
          roomHumidity = calculate_relative_humidity(roomTemperatureC, evapTempC); 
        }
      } 
    }

    cmdACFanOn = cmdACFanOn 
        || coolingOffForMs < 30000 
        || ductIntakeHumidity <= 0 
        || outletHumidity <= 0 
        || roomHumidity <= 0;
    cmdDuctFanOn = cmdDuctFanOn 
        || cmdACFanOn
        || (TEMPERATURES.getTempC(LR_OUTLET_TEMP_ADDR) < 15 && TEMPERATURES.getTempC(INTAKE_TEMP_ADDR) > 15)
        || TEMPERATURES.getTempC(INTAKE_TEMP_ADDR) < 15;

    if (cmdACFanOn)
    {
      acCommand = acData.isCooling() ? CMD_AC_OFF : CMD_AC_FAN_LOW; 
    }
    else
    {
      acCommand = CMD_AC_KILL; 
    }

    if (cmdDuctFanOn)
    {
      lrDuctCommand = CMD_LR_DUCT_LOW; 
    }

    if (TEMPERATURES.getTempC(FEW_OUTLET_TEMP_ADDR) < 15 && TEMPERATURES.getTempC(INTAKE_TEMP_ADDR) > 15)
    {
      fewDuctCommand = CMD_ENTRYWAY_DUCT_HIGH; 
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

void task_processCommands()
{
  processCommands(hrvCommand, lrDuctCommand, fewDuctCommand);
}

void task_updateNextBlower()
{
  BLOWERS.setNext();
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
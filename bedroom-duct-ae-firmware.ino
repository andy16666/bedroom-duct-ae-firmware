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
#include <Arduino.h>
#include <OneWire.h>
#include <DS18B20.h>

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

#define COOLING_TIME_BEFORE_ACCLIMATE 30 * 60 * 1000
#define ACCLIMATE_TIME_WHEN_COOLING    2 * 60 * 1000

#define HRV_LOW_EXHAUST  6
#define HRV_LOW_INTAKE   7
#define HRV_HIGH_EXHAUST 8
#define HRV_HIGH_INTAKE  9

#define HRV_EXHAUST_BOOST 10
#define EXHAUST_BYPASS    11
#define ENTRYWAY_BYPASS   12
#define LR_DUCT_BOOST     13

#define TRANSITION_TIME_MS 5000

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

extern SimplicityACResponse& responseRef; 

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

typedef enum {
  DST_IDLE = 'I', 
  DST_COOL_FEW = 'F', 
  DST_COOL_MB_HIGH = 'h', 
  DST_COOL_MB_MED  = 'm', 
  DST_COOL_MB_LOW  = 'l', 
  DST_COOL_MB_LR_HIGH = 'H', 
  DST_COOL_MB_LR_MED  = 'M', 
  DST_COOL_MB_LR_LOW  = 'L',
  DST_ACCLIMATE_HIGH = 'c',
  DST_ACCLIMATE_MED  = 'b',
  DST_ACCLIMATE_LOW  = 'a',
  DST_ACCLIMATE_DONE  = 'd'
} system_cooling_state_t; 

static volatile ventilate_cmd_t         hrvCommand          __attribute__((section(".uninitialized_data")));
static volatile lr_duct_cmd_t           lrDuctCommand       __attribute__((section(".uninitialized_data")));
static volatile few_duct_cmd_t          fewDuctCommand      __attribute__((section(".uninitialized_data"))); 
static volatile system_cooling_state_t  state               __attribute__((section(".uninitialized_data"))); 
static volatile ac_cmd_t                acCommand           __attribute__((section(".uninitialized_data"))); 
static volatile double                  outletHumidity      __attribute__((section(".uninitialized_data")));
static volatile double                  ductIntakeHumidity  __attribute__((section(".uninitialized_data")));
static volatile double                  roomHumidity        __attribute__((section(".uninitialized_data")));

volatile long coolOnTime = millis(); 
volatile long acclimateOnTimeMs = millis(); 

unsigned long coolingForMs()
{
  return acData.isSet() && acData.isCooling() ? millis() - coolOnTime : 0; 
}

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
  state = DST_IDLE; 
}

void aosSetup()
{
  thermostats.add(LIVING_ROOM);  
  thermostats.add(MASTER_BEDROOM); 
  thermostats.add(FRONT_ENTRYWAY); 
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
  BLOWERS.init(); 
  BLOWERS.setAll();

  server.on("/", []() {
    bool success = true; 
    //Serial.println("Enter / handler"); Serial.flush(); 

    for (uint8_t i = 0; i < server.args(); i++) 
    {
      String argName = server.argName(i);
      String arg = server.arg(i);
      bool success = false; 

      //Serial.println("/ handler: parse hrv"); Serial.flush(); 

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

      //Serial.println("/ handler: parse lr"); Serial.flush(); 

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

      //Serial.println("/ handler: parse ew"); Serial.flush(); 

      if (argName.equals("ew") && arg.length() == 1) 
      {
        switch(arg.charAt(0))
        {
          case 'O':  fewDuctCommand = CMD_ENTRYWAY_DUCT_OFF;  break;
          case 'H':  fewDuctCommand = CMD_ENTRYWAY_DUCT_HIGH; break;  
          default:   success = false;  
        }
      }

      //Serial.println("/ handler: parse bootloader"); Serial.flush(); 

      if (argName.equals("bootloader") && arg.length() == 1) 
      {
        switch(arg.charAt(0))
        {
          case 'A':  rp2040.rebootToBootloader();  break;
          default:   success = false;  
        }
      }

      //Serial.println("/ handler: parse reboot"); Serial.flush(); 


      if (argName.equals("reboot") && arg.length() == 1) 
      {
        switch(arg.charAt(0))
        {
          case 'A':  reboot(); break;
          default:   success = false;  
        }
      }

      //Serial.println("/ handler: parse th"); Serial.flush(); 


      if (argName.equals("thset") || argName.equals("thcur") || argName.equals("thcmd")) 
      {
        char buffer[strlen(arg.c_str()) + 1]; 
        strcpy(buffer, arg.c_str()); 

        char* roomName = strtok(buffer, "_");
        char* argumentStr = strtok(NULL, "_");
    
        if (roomName && argumentStr && thermostats.contains(roomName))
        {
          //Serial.printf("%s: %s %s\r\n", argName.c_str(), roomName, argumentStr); 

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
      //Serial.println("/ handler: success, get lock"); Serial.flush(); 

      while(xSemaphoreTakeRecursive( networkMutex, portMAX_DELAY ) != pdTRUE)
      {
        //Serial.println("httpAccept waiting");  Serial.flush(); 
      }
      String responseString = httpResponseString[0] ? String((char *)httpResponseString) : String("Loading..."); 
      
      //Serial.println("/ handler: drop lock"); Serial.flush(); 
      xSemaphoreGiveRecursive(networkMutex); 

      //Serial.println("/ handler: send 200"); Serial.flush(); 

      server.send(200, "text/json", responseString.length() > 0 ? responseString.c_str() : "{ 'status':\"Loading...\" }");

      //Serial.println("/ handler: success sent 200"); Serial.flush(); 
    }
    else 
    {
      //Serial.println("/ handler: fail send 500"); Serial.flush(); 

      server.send(500, "text/plain", "Failed to parse request.");

      //Serial.println("/ handler: fail sent 500"); Serial.flush(); 
    }

    //Serial.println("Leave / handler"); Serial.flush(); 
  });

  server.onNotFound(handleNotFound);
  server.begin();
  //Serial.println("HTTP server started");

  CORE_0_KERNEL->addImmediate(CORE_0_KERNEL, task_handleClient); 
  CORE_0_KERNEL->add(CORE_0_KERNEL, task_pollACData, 5000); 
}

void aosSetup1()
{
  CORE_1_KERNEL->add(CORE_1_KERNEL, task_parseACData, 2000);  
  CORE_1_KERNEL->add(CORE_1_KERNEL, task_processCommands, 2000);  
  CORE_1_KERNEL->add(CORE_1_KERNEL, task_updateNextBlower, TRANSITION_TIME_MS); 
}

void populateHttpResponse(JsonDocument& document) 
{
  document["intakeHumidity"] = ductIntakeHumidity; 
  document["hrvCommand"] = String((char)hrvCommand).c_str();
  document["lrDuctCommand"] = String((char)lrDuctCommand).c_str();
  document["fewDuctCommand"] = String((char)fewDuctCommand).c_str();
  document["acCommand"] = String((char)acCommand).c_str();
  document["state"] = String((char)state).c_str();
  document["coolingForMs"] = coolingForMs(); 
  document["acclimateOnForMs"] = acclimateOnForMs(); 
  document["timeMs"] = millis(); 
  if (acData.isSet())
  {
    acData.addTo(AC_HOSTNAME, document); 
    document[AC_HOSTNAME]["outletHumidity"] = outletHumidity; 
    document[AC_HOSTNAME]["roomHumidity"]   = roomHumidity; 
  }
  if (thermostats.isCurrent())
  {
    thermostats.addTo("thermostats", document); 
  }
}

void task_parseACData()
{
  //Serial.println("                             Enter task_parseACData"); Serial.flush(); 
  acData.parse();
  //Serial.println("                             Leave task_parseACData"); Serial.flush(); 
}

void task_pollACData()
{
  //Serial.println("Enter task_pollACData"); Serial.flush(); 
  acData.execute(acCommand);
  //Serial.println("Leave task_pollACData"); Serial.flush(); 
}

void task_handleClient()
{
  //Serial.println("Enter task_handleClient"); Serial.flush(); 
  server.handleClient(); 
  //Serial.println("Leave task_handleClient"); Serial.flush(); 
  delay(10); 
}

void task_processCommands()
{
  //Serial.println("                             Enter task_processCommands");Serial.flush(); 
  state = computeState(); 
  //Serial.printf("                             Leave computeState: %c\r\n", state);Serial.flush(); 
  if (stateIsAcclimateDone())
  { 
    state = computeState(); 
    //Serial.printf("                             Leave computeState: %c\r\n", state);Serial.flush(); 
  }
  //Serial.println("                             Enter computeACCommand");Serial.flush(); 
  computeACCommand(state); 
  //Serial.println("                             Leave computeACCommand");Serial.flush(); 
  //Serial.println("                             Enter processCommands");Serial.flush(); 
  processCommands(state, hrvCommand, lrDuctCommand, fewDuctCommand);
  //Serial.println("                             Leave processCommands");
  //Serial.println("                             Leave task_processCommands"); Serial.flush(); 
}

void task_updateNextBlower()
{
  //Serial.println("                             Enter task_updateNextBlower"); Serial.flush(); 
  BLOWERS.setNext();
  //Serial.println("                             Leave task_updateNextBlower"); Serial.flush(); 
}

bool stateIsAcclimate()
{
  return state == DST_ACCLIMATE_HIGH || state == DST_ACCLIMATE_MED || state == DST_ACCLIMATE_LOW; 
}

bool stateIsAcclimateDone()
{
  return state == DST_ACCLIMATE_DONE; 
}

unsigned long acclimateOnForMs()
{
  return stateIsAcclimate() ? millis() - acclimateOnTimeMs : 0;
}

system_cooling_state_t computeState()
{
  //Serial.printf("computeState(): enter\r\n"); 

  if (!acData.isSet() || !TEMPERATURES.ready())
  {
    return state; 
  }

  if (!thermostats.contains(LIVING_ROOM) || !thermostats.contains(MASTER_BEDROOM) || !thermostats.contains(FRONT_ENTRYWAY))
  {
    return state; 
  }

  if (!acData.isEvapTempValid() || !acData.isOutletTempValid())
    return state;

  Thermostat& lr  = thermostats.get(LIVING_ROOM); 
  Thermostat& mb  = thermostats.get(MASTER_BEDROOM); 
  Thermostat& few = thermostats.get(FRONT_ENTRYWAY); 

  if (!lr.isCurrent() || !mb.isCurrent() || !few.isCurrent())
  {
    return state; 
  }

  if (!TEMPERATURES.isTempValid(INTAKE_TEMP_ADDR))
  {
    return state; 
  }

  unsigned long timeMs = millis(); 
  if (!acData.isCooling())
  {
    coolOnTime = timeMs; 
  }

  if (!stateIsAcclimate())
  {
    acclimateOnTimeMs = timeMs; 
  }

  //Serial.printf("computeState(): proceeding\r\n"); 

  double evapTempC = acData.getEvapTempC(); 
  double outletTempC = acData.getOutletTempC(); 
  double ductIntakeTempC = TEMPERATURES.getTempC(INTAKE_TEMP_ADDR);
  double roomTemperatureC = thermostats.get(MASTER_BEDROOM).getCurrentTemperatureC(); 

  //Serial.printf("computeState(): evapTempC=%f outletTempC=%f ductIntakeTempC=%f\r\n", evapTempC, outletTempC, ductIntakeTempC); 

  bool needsAcclimation = 
    ductIntakeHumidity <= 0 || outletHumidity <= 0 || roomHumidity <= 0
    || (coolingForMs() > COOLING_TIME_BEFORE_ACCLIMATE) || (state == DST_IDLE && outletTempC < 15); 

  if (stateIsAcclimate())
  {
    ductIntakeHumidity = calculate_relative_humidity(ductIntakeTempC, evapTempC); 
    outletHumidity = calculate_relative_humidity(outletTempC, evapTempC);
    //Serial.printf("computeState(): ductIntakeHumidity=%f outletHumidity=%f\r\n", ductIntakeHumidity, outletHumidity); 
    
    roomHumidity = calculate_relative_humidity(roomTemperatureC, evapTempC); 
    //Serial.printf("computeState(): roomHumidity=%f roomTemperatureC=%f\r\n", roomHumidity, roomTemperatureC); 
    
    if (thermostats.coolingCalledFor() && acclimateOnForMs() > ACCLIMATE_TIME_WHEN_COOLING)
    {
      return DST_ACCLIMATE_DONE; 
    }
    else if (
      ductIntakeHumidity < 80.0 && ductIntakeHumidity > 20.0
      && outletHumidity < 80.0 && outletHumidity > 20.0
    )
    {
      return DST_ACCLIMATE_DONE; 
    }
    else 
    {
      return state; 
    }
  }
  else if (needsAcclimation)
  {
    if (thermostats.coolingCalledFor() && thermostats.getMaxCoolingMagnitude() > 2.0)
    {
      return DST_ACCLIMATE_HIGH;
    }

    switch(acCommand)
    {
      case CMD_AC_COOL_HIGH: return DST_ACCLIMATE_HIGH; break; 
      case CMD_AC_COOL_MED:  return DST_ACCLIMATE_MED;  break; 
      case CMD_AC_COOL_LOW:  return DST_ACCLIMATE_LOW;  break; 
      default: return DST_ACCLIMATE_LOW; break; 
    }
  }
  else if (mb.coolingCalledFor()) 
  {
    if(mb.getMagnitude() >= 1.0 || lr.coolingCalledFor())
    {
      return DST_COOL_MB_HIGH; 
    }
    else if (mb.getMagnitude() >= 0.25)
    {
      return DST_COOL_MB_MED; 
    }
    else
    {
      return DST_COOL_MB_LOW; 
    }
  }
  else if (lr.coolingCalledFor())
  {
    if (mb.heatCalledFor())
    {
      return (mb.getMagnitude() >= 0.5) ? DST_COOL_MB_LR_LOW : DST_COOL_MB_LR_MED; 
    }
    else 
    {
      if (lr.getMagnitude() >= 0.5)
      {
        return DST_COOL_MB_LR_HIGH; 
      }
      else if (lr.getMagnitude() >= 0.25)
      {
        return DST_COOL_MB_LR_MED; 
      }
      else
      {
        return DST_COOL_MB_LR_LOW;
      }
    }
  }
  else if (few.coolingCalledFor())
  {
    return DST_COOL_FEW; 
  }
  else 
  {
    return DST_IDLE; 
  }
}



void computeACCommand(system_cooling_state_t state)
{
  switch (state) 
  {
    case DST_IDLE:           
      acCommand = (acCommand == CMD_AC_OFF ? CMD_AC_KILL : CMD_AC_OFF); 
      break; 
    case DST_ACCLIMATE_DONE:  acCommand = CMD_AC_OFF;       break; 
    case DST_COOL_FEW:        acCommand = CMD_AC_COOL_MED;  break; 
    case DST_COOL_MB_HIGH:    acCommand = CMD_AC_COOL_HIGH; break; 
    case DST_COOL_MB_MED:     acCommand = CMD_AC_COOL_MED;  break; 
    case DST_COOL_MB_LOW:     acCommand = CMD_AC_COOL_LOW;  break; 
    case DST_COOL_MB_LR_HIGH: acCommand = CMD_AC_COOL_HIGH; break; 
    case DST_COOL_MB_LR_MED:  acCommand = CMD_AC_COOL_HIGH; break; 
    case DST_COOL_MB_LR_LOW:  acCommand = CMD_AC_COOL_MED;  break; 
    case DST_ACCLIMATE_HIGH:  acCommand = CMD_AC_FAN_HIGH;  break;     
    case DST_ACCLIMATE_MED:   acCommand = CMD_AC_FAN_MED;   break;     
    case DST_ACCLIMATE_LOW:   acCommand = CMD_AC_FAN_LOW;   break;   
  }
}

void processCommands(system_cooling_state_t state, ventilate_cmd_t hrvCommand, lr_duct_cmd_t lrDuctCommand, few_duct_cmd_t fewDuctCommand)
{
  switch (state) 
  {
    case DST_COOL_FEW:        break; 
    case DST_COOL_MB_HIGH:    hrvCommand = CMD_VENTILATE_HIGH; break; 
    case DST_COOL_MB_MED:     hrvCommand = CMD_VENTILATE_MED;  break; 
    case DST_COOL_MB_LOW:     break; 
    case DST_COOL_MB_LR_HIGH: hrvCommand = CMD_VENTILATE_OFF; break; 
    case DST_COOL_MB_LR_MED:  hrvCommand = CMD_VENTILATE_OFF; break; 
    case DST_COOL_MB_LR_LOW:  hrvCommand = CMD_VENTILATE_OFF; break; 
    case DST_ACCLIMATE_HIGH:  hrvCommand = CMD_VENTILATE_OFF; break;     
    case DST_ACCLIMATE_MED:   break;     
    case DST_ACCLIMATE_LOW:   break;   
  }

  switch (state) 
  {
    case DST_COOL_FEW:        break; 
    case DST_COOL_MB_HIGH:    break; 
    case DST_COOL_MB_MED:     break; 
    case DST_COOL_MB_LOW:     break; 
    case DST_COOL_MB_LR_HIGH: lrDuctCommand = CMD_LR_DUCT_HIGH;  break; 
    case DST_COOL_MB_LR_MED:  lrDuctCommand = CMD_LR_DUCT_HIGH;  break; 
    case DST_COOL_MB_LR_LOW:  lrDuctCommand = CMD_LR_DUCT_MED;   break; 
    case DST_ACCLIMATE_HIGH:  lrDuctCommand = CMD_LR_DUCT_MED;   break;     
    case DST_ACCLIMATE_MED:   lrDuctCommand = CMD_LR_DUCT_MED;   break;     
    case DST_ACCLIMATE_LOW:   lrDuctCommand = CMD_LR_DUCT_LOW;   break;   
  }

  switch (state) 
  {
    case DST_COOL_FEW:        fewDuctCommand = CMD_ENTRYWAY_DUCT_HIGH; break; 
    case DST_COOL_MB_HIGH:    break; 
    case DST_COOL_MB_MED:     break; 
    case DST_COOL_MB_LOW:     break; 
    case DST_COOL_MB_LR_HIGH: fewDuctCommand = CMD_ENTRYWAY_DUCT_HIGH; break; 
    case DST_COOL_MB_LR_MED:  break; 
    case DST_COOL_MB_LR_LOW:  break; 
    case DST_ACCLIMATE_HIGH:  fewDuctCommand = CMD_ENTRYWAY_DUCT_HIGH; break;     
    case DST_ACCLIMATE_MED:   break;     
    case DST_ACCLIMATE_LOW:   break;   
  }

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
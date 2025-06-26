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

#include <stdint.h>
#include <float.h>
#include <math.h>
#include <string.h>

#include <aos.h>
#include <config.h>
#include <GPIOOutputs.h>
#include <Thermostats.h>
#include <SimplicityAC.h> 
#include <WebServer.h>

using namespace AOS;

using AOS::Thermostat; 
using AOS::Thermostats;
using AOS::TemperatureSensors;
using AOS::Ping; 

#define COOLING_TIME_BEFORE_ACCLIMATE 60 * 60
#define ACCLIMATE_TIME_WHEN_COOLING   15 * 60
#define FAN_TIME_BEFORE_VALID_HUMIDITIES_SECONDS 30

#define AC_HOSTNAME "ac1"
#define HOSTNAME "hrv1"

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
SimplicityAC AC(AC_HOSTNAME); 
Thermostats THERMOSTATS;

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
  DST_UNINIT = 'U', 
  DST_IDLE = 'I', 
  DST_COOL_FEW = 'F', 
  DST_COOL_FEW_MB_LR = 'A', 
  DST_HEAT_FEW = 'U',
  DST_COOL_MB_HIGH = 'h', 
  DST_COOL_MB_MED  = 'm', 
  DST_COOL_MB_LOW  = 'l', 
  DST_COOL_MB_LR_HIGH = 'H', 
  DST_COOL_MB_LR_MED  = 'M', 
  DST_COOL_MB_LR_LOW  = 'L',
  // Push: Used to minimize cooling in the room with the A/C by pushing it elsewhere. 
  DST_COOL_LR_PUSH  = 'P',
  DST_COOL_DONE = 'i', 
  DST_HEAT_LR = 'T', 
  DST_ACCLIMATE_HIGH = 'c',
  DST_ACCLIMATE_MED  = 'b',
  DST_ACCLIMATE_LOW  = 'a',
  DST_ACCLIMATE_DONE  = 'd'
} system_cooling_state_t; 

static volatile ventilate_cmd_t         hrvCommand          __attribute__((section(".uninitialized_data")));
static volatile lr_duct_cmd_t           lrDuctCommand       __attribute__((section(".uninitialized_data")));
static volatile few_duct_cmd_t          fewDuctCommand      __attribute__((section(".uninitialized_data"))); 
static volatile double                  outletHumidity      __attribute__((section(".uninitialized_data")));
static volatile double                  ductIntakeHumidity  __attribute__((section(".uninitialized_data")));
static volatile double                  roomHumidity        __attribute__((section(".uninitialized_data")));

volatile system_cooling_state_t  state; 
volatile ac_cmd_t                acCommand;


volatile double acclimateOnTimeSeconds; 
volatile double coolOnTimeSeconds; 
volatile double coolOffTimeSeconds; 

double coolingForSeconds()
{
  return AC.isSet() && AC.isCooling() ? seconds() - coolOnTimeSeconds : 0; 
}

double coolingOffForSeconds()
{
  return !AC.isSet() || !AC.isCooling() ? seconds() - coolOffTimeSeconds : 0; 
}

double acclimateOnForSeconds()
{
  return stateIsAcclimate() ? seconds() - acclimateOnTimeSeconds : 0;
}

const char* generateHostname()
{
  return HOSTNAME; 
}

void aosInitialize()
{
  hrvCommand = CMD_VENTILATE_OFF; 
  lrDuctCommand = CMD_LR_DUCT_OFF; 
  fewDuctCommand = CMD_ENTRYWAY_DUCT_OFF; 
  outletHumidity = 0; 
  ductIntakeHumidity = 0;
  roomHumidity = 0; 
}

void aosSetup()
{
  acCommand = CMD_AC_OFF; 
  state = DST_UNINIT; 
  coolOnTimeSeconds = seconds();
  acclimateOnTimeSeconds = seconds(); 
  coolOffTimeSeconds = seconds(); 

  THERMOSTATS.add(LIVING_ROOM);  
  THERMOSTATS.add(MASTER_BEDROOM); 
  THERMOSTATS.add(FRONT_ENTRYWAY); 

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

  setupFrontEnd("/hrv.html"); 

  CORE_0_KERNEL->add(CORE_0_KERNEL, task_pollAC, 15000);   
}

void aosSetup1()
{
  CORE_1_KERNEL->addImmediate(CORE_1_KERNEL, task_parseAC);  
  CORE_1_KERNEL->addImmediate(CORE_1_KERNEL, task_processCommands);  
  CORE_1_KERNEL->add(CORE_1_KERNEL, task_updateNextBlower, TRANSITION_TIME_MS); 
}

bool handleHttpArg(String argName, String arg) 
{
  bool success = true; 

  DPRINTLN("/ handler: parse hrv"); 

  if (argName.equals("hrv") && arg.length() == 1) 
  {
    switch(arg.charAt(0))
    {
      case 'O':  hrvCommand = CMD_VENTILATE_OFF;  break;
      case 'L':  hrvCommand = CMD_VENTILATE_LOW;  break;  
      case 'M':  hrvCommand = CMD_VENTILATE_MED;  break;  
      case 'H':  hrvCommand = CMD_VENTILATE_HIGH; break;  
      default:   success = false; 
    }
  }

  DPRINTLN("/ handler: parse lr"); 

  if (argName.equals("lr") && arg.length() == 1) 
  {
    switch(arg.charAt(0))
    {
      case 'O':  lrDuctCommand = CMD_LR_DUCT_OFF;  break;
      case 'L':  lrDuctCommand = CMD_LR_DUCT_LOW;  break;  
      case 'M':  lrDuctCommand = CMD_LR_DUCT_MED;  break;  
      case 'H':  lrDuctCommand = CMD_LR_DUCT_HIGH; break;  
      default:   success = false;  
    }
  }

  DPRINTLN("/ handler: parse ew"); 

  if (argName.equals("ew") && arg.length() == 1) 
  {
    switch(arg.charAt(0))
    {
      case 'O':  fewDuctCommand = CMD_ENTRYWAY_DUCT_OFF;  break;
      case 'H':  fewDuctCommand = CMD_ENTRYWAY_DUCT_HIGH; break;  
      default:   success = false;  
    }
  }

  DPRINTLN("/ handler: parse th"); 

  if (argName.equals("thset") || argName.equals("thcur") || argName.equals("thcmd") || argName.equals("theat")) 
  {
    char buffer[strlen(arg.c_str()) + 1]; 
    strcpy(buffer, arg.c_str()); 

    char* roomName = strtok(buffer, "_");
    char* argumentStr = strtok(NULL, "_");

    if (roomName && argumentStr && THERMOSTATS.contains(roomName))
    {
      DPRINTF("%s: %s %s\r\n", argName.c_str(), roomName, argumentStr); 

      if (argName.equals("thset"))
        THERMOSTATS.get(roomName).setSetPointC(atoff(argumentStr)); 
      
      if (argName.equals("thcur"))
        THERMOSTATS.get(roomName).setCurrentTemperatureC(atoff(argumentStr));
      
      if (argName.equals("thcmd"))
        THERMOSTATS.get(roomName).setCommand(argumentStr[0] == '1');  

      if (argName.equals("theat"))
        THERMOSTATS.get(roomName).setHeatOn(argumentStr[0] == '1');  
    }
    else 
    {
      success = false; 
    }
  }

  return success; 
}

void populateHttpResponse(JsonDocument& document) 
{
  document["intakeHumidity"] = ductIntakeHumidity; 
  document["hrvCommand"] = String((char)hrvCommand).c_str();
  document["lrDuctCommand"] = String((char)lrDuctCommand).c_str();
  document["fewDuctCommand"] = String((char)fewDuctCommand).c_str();
  document["acCommand"] = String((char)acCommand).c_str();
  document["state"] = String((char)state).c_str();
  document["coolingFor"] = secondsToHMS(coolingForSeconds()); 
  document["coolingOffFor"] = secondsToHMS(coolingOffForSeconds()); 
  document["acclimateOnFor"] = secondsToHMS(acclimateOnForSeconds()); 
  document["time"] = secondsToHMS(seconds()); 
  BLOWERS.addTo("blowers", document); 
  if (AC.isSet())
  {
    AC.addTo(AC_HOSTNAME, document); 
    document[AC_HOSTNAME]["outletHumidity"] = outletHumidity; 
    document[AC_HOSTNAME]["roomHumidity"]   = roomHumidity; 
  }
  if (THERMOSTATS.isCurrent())
  {
    THERMOSTATS.addTo("thermostats", document); 
  }
}

void task_parseAC()
{
  DPRINTLN("                             Enter task_parseAC"); 
  AC.parse();
  DPRINTLN("                             Leave task_parseAC"); 
}

void task_pollAC()
{
  DPRINTLN("Enter task_pollAC"); 
  if (state != DST_UNINIT)
  {
    AC.execute(acCommand);
  }
  else 
  {
    AC.execute(); 
  }
  delay(10); 
}

void task_processCommands()
{
  state = computeState(); 
  if (stateIsAcclimateDone())
  { 
    state = computeState(); 
    DPRINTF("                             Leave computeState: %c\r\n", state);Serial.flush(); 
  }
  DPRINTLN("                             Enter computeACCommand");Serial.flush(); 
  computeACCommand(state); 
  DPRINTLN("                             Leave computeACCommand");Serial.flush(); 
  DPRINTLN("                             Enter processCommands");Serial.flush(); 
  processCommands(state, hrvCommand, lrDuctCommand, fewDuctCommand);
  DPRINTLN("                             Leave processCommands");
  DPRINTLN("                             Leave task_processCommands"); 
}

void task_updateNextBlower()
{
  DPRINTLN("                             Enter task_updateNextBlower"); 
  BLOWERS.setNext();
  DPRINTLN("                             Leave task_updateNextBlower"); 
}

bool stateIsAcclimate()
{
  return state == DST_ACCLIMATE_HIGH || state == DST_ACCLIMATE_MED || state == DST_ACCLIMATE_LOW; 
}

bool stateIsCool()
{
  return state == DST_COOL_FEW 
    || state == DST_COOL_MB_HIGH
    || state == DST_COOL_MB_MED
    || state == DST_COOL_MB_LOW
    || state == DST_COOL_MB_LR_HIGH
    || state == DST_COOL_MB_LR_MED
    || state == DST_COOL_MB_LR_LOW
    || state == DST_COOL_LR_PUSH
    || state == DST_COOL_FEW_MB_LR; 
}

bool stateIsAcclimateDone()
{
  return state == DST_ACCLIMATE_DONE; 
}

bool needsAcclimation()
{
  double evapTempC = AC.getEvapTempC(); 
  double outletTempC = AC.getOutletTempC(); 
  double ductIntakeTempC = TEMPERATURES.getTempC(INTAKE_TEMP_ADDR);
  double roomTemperatureC = THERMOSTATS.get(MASTER_BEDROOM).getCurrentTemperatureC(); 
  
  return
    ductIntakeHumidity <= 0 || outletHumidity <= 0 || roomHumidity <= 0
    || (coolingForSeconds() > COOLING_TIME_BEFORE_ACCLIMATE) 
    || (!THERMOSTATS.coolingCalledFor() && !THERMOSTATS.heatCalledFor() && outletTempC < 15)
    || ((state == DST_COOL_DONE || state == DST_IDLE) && (ductIntakeHumidity > 80.0 || outletHumidity > 80.0)); 
}

bool humiditiesValid()
{
  return coolingOffForSeconds() > FAN_TIME_BEFORE_VALID_HUMIDITIES_SECONDS || stateIsAcclimate(); 
}

bool updateHumidities()
{
  double evapTempC = AC.getEvapTempC(); 
  double outletTempC = AC.getOutletTempC(); 
  double ductIntakeTempC = TEMPERATURES.getTempC(INTAKE_TEMP_ADDR);
  double roomTemperatureC = THERMOSTATS.get(MASTER_BEDROOM).getCurrentTemperatureC(); 

  if (humiditiesValid())
  {
    ductIntakeHumidity = calculate_relative_humidity(ductIntakeTempC, evapTempC); 
    outletHumidity = calculate_relative_humidity(outletTempC, evapTempC);    
    roomHumidity = calculate_relative_humidity(roomTemperatureC, evapTempC); 
    return true; 
  }
  else 
  {
    return false; 
  }
}

system_cooling_state_t computeState()
{
  DPRINTF("computeState(): enter\r\n"); 

  if (!AC.isSet() || !TEMPERATURES.ready())
  {
    return state; 
  }

  if (state == DST_UNINIT)
  {
    if (AC.isSet())
    {
      acCommand = AC.getCommand(); 
    }
  }

  if (!THERMOSTATS.contains(LIVING_ROOM) || !THERMOSTATS.contains(MASTER_BEDROOM) || !THERMOSTATS.contains(FRONT_ENTRYWAY))
  {
    return state; 
  }

  if (!AC.isEvapTempValid() || !AC.isOutletTempValid())
    return state;

  Thermostat& lr  = THERMOSTATS.get(LIVING_ROOM); 
  Thermostat& mb  = THERMOSTATS.get(MASTER_BEDROOM); 
  Thermostat& few = THERMOSTATS.get(FRONT_ENTRYWAY); 

  if (!lr.isCurrent() || !mb.isCurrent() || !few.isCurrent())
  {
    return state; 
  }

  if (!TEMPERATURES.isTempValid(INTAKE_TEMP_ADDR))
  {
    return state; 
  }

  if (!AC.isCooling())
  {
    coolOnTimeSeconds = seconds(); 
  }

  if (AC.isCooling())
  {
    coolOffTimeSeconds = seconds(); 
  }

  if (!stateIsAcclimate())
  {
    acclimateOnTimeSeconds = seconds(); 
  }

  DPRINTF("computeState(): proceeding\r\n"); 

  double evapTempC = AC.getEvapTempC(); 
  double outletTempC = AC.getOutletTempC(); 
  double ductIntakeTempC = TEMPERATURES.getTempC(INTAKE_TEMP_ADDR);
  double roomTemperatureC = THERMOSTATS.get(MASTER_BEDROOM).getCurrentTemperatureC(); 

  DPRINTF("computeState(): evapTempC=%f outletTempC=%f ductIntakeTempC=%f\r\n", evapTempC, outletTempC, ductIntakeTempC); 

  bool humiditiesUpdated = updateHumidities(); 

  // Stay in DST_COOL_DONE until humidities updated. 
  if (state == DST_COOL_DONE && !humiditiesUpdated)
  {
    return DST_COOL_DONE; 
  }

  if (stateIsAcclimate())
  {
    if (THERMOSTATS.coolingCalledFor() && acclimateOnForSeconds() > ACCLIMATE_TIME_WHEN_COOLING)
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
    else if (THERMOSTATS.heatCalledFor())
    {
      return DST_ACCLIMATE_LOW; 
    }
    else if (THERMOSTATS.coolingCalledFor() && THERMOSTATS.getMaxCoolingMagnitude() >= 1.5)
    {
      return DST_ACCLIMATE_HIGH; 
    }
    else if (THERMOSTATS.coolingCalledFor() && THERMOSTATS.getMaxCoolingMagnitude() >= 1.0)
    {
      return DST_ACCLIMATE_MED; 
    }
    else 
    {
      return state; 
    }
  }
  else if (needsAcclimation())
  {
    if (THERMOSTATS.heatCalledFor())
    {
      return DST_ACCLIMATE_LOW;
    }
    else if (THERMOSTATS.coolingCalledFor() && THERMOSTATS.getMaxCoolingMagnitude() >= 1.5)
    {
      return DST_ACCLIMATE_HIGH;
    }

    switch(acCommand)
    {
      case CMD_AC_COOL_HIGH: return DST_ACCLIMATE_HIGH; break; 
      case CMD_AC_COOL_MED:  return DST_ACCLIMATE_MED;  break; 
      case CMD_AC_COOL_LOW:  return DST_ACCLIMATE_LOW;  break; 
      default:               return DST_ACCLIMATE_LOW;  break; 
    }
  }
  else if (mb.coolingCalledFor() && mb.getMagnitude() >= 0.5) 
  {
    if (mb.getMagnitude() >= 1.0 && lr.coolingCalledFor() && lr.getMagnitude() >= 1.0)
    {
      return few.coolingCalledFor() ? DST_COOL_FEW_MB_LR : DST_COOL_MB_LR_HIGH;
    }
    else if(mb.getMagnitude() >= 1.5)
    {
      return lr.coolingCalledFor() ? DST_COOL_MB_LR_HIGH : DST_COOL_MB_HIGH; 
    }
    else if (mb.getMagnitude() >= 1.0)
    {
      return lr.coolingCalledFor() ? DST_COOL_MB_LR_MED : DST_COOL_MB_MED; 
    }
    else if (few.coolingCalledFor())
    {
      return lr.coolingCalledFor() ? DST_COOL_FEW_MB_LR : DST_COOL_FEW; 
    }
    else
    {
      return lr.coolingCalledFor() ? DST_COOL_MB_LR_LOW : DST_COOL_MB_LOW; 
    }
  }
  else if (lr.coolingCalledFor())
  {
    if (mb.heatCalledFor() || (mb.getCommand() && mb.getMagnitude() < 0.5))
    {
      return DST_COOL_LR_PUSH; 
    }
    else if (lr.getMagnitude() >= 1.0)
    {
      return (few.coolingCalledFor() && few.getMagnitude() >= 1.0) 
            ? DST_COOL_FEW_MB_LR 
            : DST_COOL_MB_LR_HIGH; 
    }
    else
    {
      return few.coolingCalledFor() ? DST_COOL_FEW_MB_LR : DST_COOL_MB_LR_MED; 
    }
  }
  else if (few.coolingCalledFor())
  {
    return DST_COOL_FEW; 
  }
  else if (few.heatCalledFor() && TEMPERATURES.getTempC(INTAKE_TEMP_ADDR) > few.getCurrentTemperatureC())
  {
    return DST_HEAT_FEW; 
  }
  else if (lr.heatCalledFor()
      && (TEMPERATURES.getTempC(LR_OUTLET_TEMP_ADDR) > lr.getCurrentTemperatureC() || TEMPERATURES.getTempC(INTAKE_TEMP_ADDR) > lr.getCurrentTemperatureC())
  )
  {
    return DST_HEAT_LR; 
  }
  else if (stateIsCool())
  {
    return DST_COOL_DONE; 
  }
  else
  {
    return DST_IDLE; 
  }
}

void computeACCommand(system_cooling_state_t state)
{
  if (state == DST_UNINIT)
  {
    return;
  }

  switch (state) 
  {
    case DST_COOL_DONE:       acCommand = CMD_AC_OFF;       break; 
    case DST_IDLE:            acCommand = CMD_AC_KILL;      break; 
    case DST_ACCLIMATE_DONE:  acCommand = CMD_AC_OFF;       break; 
    case DST_COOL_FEW:        acCommand = CMD_AC_COOL_MED;  break; 
    case DST_COOL_FEW_MB_LR:  acCommand = CMD_AC_COOL_HIGH; break; 
    case DST_COOL_MB_HIGH:    acCommand = CMD_AC_COOL_HIGH; break; 
    case DST_COOL_MB_MED:     acCommand = CMD_AC_COOL_MED;  break; 
    case DST_COOL_MB_LOW:     acCommand = CMD_AC_COOL_LOW;  break; 
    case DST_COOL_MB_LR_HIGH: acCommand = CMD_AC_COOL_HIGH; break; 
    case DST_COOL_MB_LR_MED:  acCommand = CMD_AC_COOL_MED; break; 
    case DST_COOL_MB_LR_LOW:  acCommand = CMD_AC_COOL_LOW;  break; 
    case DST_COOL_LR_PUSH:    acCommand = CMD_AC_FAN_HIGH;  break; 
    case DST_ACCLIMATE_HIGH:  acCommand = CMD_AC_FAN_HIGH;  break;     
    case DST_ACCLIMATE_MED:   acCommand = CMD_AC_FAN_MED;   break;     
    case DST_ACCLIMATE_LOW:   acCommand = CMD_AC_FAN_LOW;   break;   
  }
}

void processCommands(system_cooling_state_t state, ventilate_cmd_t hrvCommand, lr_duct_cmd_t lrDuctCommand, few_duct_cmd_t fewDuctCommand)
{
  if (state == DST_UNINIT)
  {
    return;
  }

  switch (state) 
  {
    case DST_COOL_MB_HIGH:    
    case DST_COOL_MB_LR_HIGH: 
    case DST_COOL_MB_LR_MED:  
    case DST_COOL_MB_LR_LOW:  
    case DST_COOL_LR_PUSH:  
    case DST_COOL_FEW_MB_LR:   
    case DST_ACCLIMATE_HIGH:  
        hrvCommand = CMD_VENTILATE_OFF; break;     
  }

  switch (state) 
  {
    case DST_COOL_FEW_MB_LR: 
    case DST_COOL_MB_LR_HIGH: 
    case DST_COOL_MB_LR_MED:  
    case DST_COOL_LR_PUSH:
    case DST_ACCLIMATE_HIGH:  
      lrDuctCommand = CMD_LR_DUCT_HIGH;  break; 
    case DST_COOL_MB_LR_LOW:  
    case DST_ACCLIMATE_MED:  
      lrDuctCommand = CMD_LR_DUCT_MED;   break; 
    case DST_HEAT_LR:         
    case DST_ACCLIMATE_LOW:   
      lrDuctCommand = CMD_LR_DUCT_LOW;   break;   
  }

  switch (state) 
  {
    case DST_COOL_FEW_MB_LR: 
    case DST_HEAT_FEW:        
    case DST_COOL_FEW:        
    case DST_ACCLIMATE_HIGH:  
    case DST_COOL_LR_PUSH:    
        fewDuctCommand = CMD_ENTRYWAY_DUCT_HIGH; break;
  }

  BLOWERS.setCommand(
    HRV_HIGH_INTAKE, 
    hrvCommand == CMD_VENTILATE_HIGH 
    || hrvCommand == CMD_VENTILATE_MED
  ); 

  BLOWERS.setCommand(
    HRV_LOW_INTAKE, 
    hrvCommand != CMD_VENTILATE_OFF
  ); 

  BLOWERS.setCommand(
    HRV_HIGH_EXHAUST, 
    hrvCommand == CMD_VENTILATE_HIGH 
    || hrvCommand == CMD_VENTILATE_MED
  ); 

  BLOWERS.setCommand(
    HRV_LOW_EXHAUST, 
    hrvCommand != CMD_VENTILATE_OFF
  ); 
    
  BLOWERS.setCommand(HRV_EXHAUST_BOOST, 
    hrvCommand == CMD_VENTILATE_HIGH 
    || lrDuctCommand == CMD_LR_DUCT_HIGH 
    || lrDuctCommand == CMD_LR_DUCT_MED
  
  ); 
    
  BLOWERS.setCommand(
    EXHAUST_BYPASS, 
    hrvCommand == CMD_VENTILATE_HIGH || lrDuctCommand != CMD_LR_DUCT_OFF
  ); 

  BLOWERS.setCommand(
    LR_DUCT_BOOST, 
    hrvCommand == CMD_VENTILATE_HIGH || lrDuctCommand == CMD_LR_DUCT_HIGH
  ); 

  BLOWERS.setCommand(
    ENTRYWAY_BYPASS, 
    hrvCommand == CMD_VENTILATE_HIGH || fewDuctCommand == CMD_ENTRYWAY_DUCT_HIGH
  ); 
}
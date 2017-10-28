/**
*  @file    IOTesterino
*  @author  peter c
*  @date    10/23/2017
*  @version 0.1
*
*
*  @section DESCRIPTION
*  Test program to help test arduino nano IO circuit boards
*/
#include <HardwareSerial.h>
#include <TimeAlarms.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h> // for RTC
#include <Streaming.h>
#include <DA_Analoginput.h>
#include <DA_Discreteinput.h>
#include <DA_DiscreteOutput.h>
#include <DA_DiscreteOutputTmr.h>
#include <DA_HOASwitch.h>
#define POLL_CYCLE_SECONDS 2
#define PROCESS_TERMINAL
#define LIGHT_REFRESH_INTERVAL 100
#define TEMPERATURE1 4  // 22 pin arduino mega as host
#define ONE_TEMPERATURE_PRECISION 9

#define IOTesterRS485 
#ifdef IOTesterRS485
  DA_DiscreteOutput MAX485TX_ENABLE = DA_DiscreteOutput(6, HIGH); // 1 is enable, 0 rx enabled

  #define activate485Tx() MAX485TX_ENABLE.activate()
  #define activate485Rx() tracePort->flush();delay(20); MAX485TX_ENABLE.reset();
#else
  #define activate485Tx()
  #define activate485Rx()
#endif

OneWire oneWireBus1(TEMPERATURE1);
DallasTemperature sensors1(& oneWireBus1);
struct _AlarmEntry
{
  time_t epoch;
  AlarmId id = dtINVALID_ALARM_ID;
  bool firstTime = true;
};

typedef _AlarmEntry AlarmEntry;
AlarmEntry onRefreshAnalogs; // sonar and 1-wire read refresh
AlarmEntry onFlowCalc; // flow calculations
// Analog Inputs
// 
// 
struct _DiscreteIOEntry
{
  bool dir; // true output, false input
  DA_DiscreteInput * dInput;
  DA_DiscreteOutput * dOutput;
  bool isOneWire = false;
  // OneWire *oneWireBus;
  // DallasTemperature *sensors;
};

typedef _DiscreteIOEntry DiscreteIOEntry;
#define MAX_DISCRETE_PINS 14 // 0-13 ardiuino nano
DiscreteIOEntry DiscreteIO[MAX_DISCRETE_PINS];
// Serial port to trace to
HardwareSerial *tracePort = &Serial;
void onFT_002_PulseIn()
{
  // FT_002.handleFlowDetection();
}

void onEdgeDetect(bool state, int pin)
{
  activate485Tx();
  *tracePort << "Edge Detection:" << state << " on pin " << pin << endl;
  activate485Rx();
}

void removeObjectsFor(int aPin)
{
  if (DiscreteIO[aPin].dInput)
    delete(DiscreteIO[aPin].dInput);
  if (DiscreteIO[aPin].dOutput)
    delete(DiscreteIO[aPin].dOutput);
  /*
  if( DiscreteIO[aPin].oneWireBus)
  delete( DiscreteIO[aPin].oneWireBus );
  if( DiscreteIO[aPin].sensors)
  delete( DiscreteIO[aPin].sensors );
  */
  DiscreteIO[aPin].dInput = NULL;
  DiscreteIO[aPin].dOutput = NULL;
  /*
  DiscreteIO[aPin].oneWireBus = NULL;
  DiscreteIO[aPin].sensors = NULL;
  */
}

void initAs1Wire(int aPin)
{
  DiscreteIO[aPin].dir = false;
  DiscreteIO[aPin].isOneWire = true;
  removeObjectsFor(aPin);
  // TODO
}

void initAsInput(int aPin)
{
  DiscreteIO[aPin].dir = false;
  DiscreteIO[aPin].isOneWire = false;
  removeObjectsFor(aPin);
  DiscreteIO[aPin].dInput = new DA_DiscreteInput(aPin, DA_DiscreteInput::ToggleDetect, true);
  DiscreteIO[aPin].dInput->setOnEdgeEvent(& onEdgeDetect);
  activate485Tx();
  *tracePort << "Pin " << aPin << " defined as input." << endl;
  activate485Rx();
}

void initAsOutput(int aPin)
{
  DiscreteIO[aPin].dir = true;
  DiscreteIO[aPin].isOneWire = false;
  removeObjectsFor(aPin);
  DiscreteIO[aPin].dOutput = new DA_DiscreteOutput(aPin, LOW);
  activate485Tx();
  *tracePort << "Pin " << aPin << " defined as output." << endl;
  activate485Rx();
}

void initDiscreteIO()
{
  initAsInput(2);
  // initAsInput( 7 );
  // initAsInput( 9 );
  // initAsInput( 12 );
  // initAsOutput(3);
  // initAsOutput(9);
  initAsOutput(10);
  // initAsOutput(11);
}

void printOneWireAddress(HardwareSerial *tracePort, DeviceAddress aDeviceAddress, bool aCR)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (aDeviceAddress[i] < 16)
      *tracePort << '0';
    tracePort->print(aDeviceAddress[i], HEX);
  }
  if (aCR)
    *tracePort << endl;
}

void init1WireTemperatureSensor(DallasTemperature * sensor, int pin)
{
  activate485Tx();
  DeviceAddress address;
  sensor->begin();
  if (sensor->getAddress(address, 0))
  {

#ifdef PROCESS_TERMINAL

    *tracePort << "Channel " << pin << " 1Wire Temperature initialized. Address =  ";
    printOneWireAddress(tracePort, address, true);
#endif

    sensor->setResolution(address, ONE_TEMPERATURE_PRECISION);
  }
  else
  {

#ifdef PROCESS_TERMINAL
    *tracePort << "Unable to find address for 1Wire Temperature Device @ " << pin << endl;
#endif


  }
  activate485Rx();
}

void initOneWireTemps()
{
  init1WireTemperatureSensor(&sensors1, TEMPERATURE1);
}

int polling; // 1=polling analogs, 2=polling digitals, -1 nothing
int currentTogglePin; // -1 none, >0 pin
void setup()
{
  activate485Rx();
  onRefreshAnalogs.id = Alarm.timerRepeat(POLL_CYCLE_SECONDS, doOnPoll);
  randomSeed(analogRead(0));

#ifdef PROCESS_TERMINAL
  tracePort->begin(9600);
  showCommands();
#endif

  initDiscreteIO();
  polling = -1;
  currentTogglePin = -1;
  initOneWireTemps();
}

void loop()
{

#ifdef PROCESS_TERMINAL
  processTerminalCommands();
#endif

  refreshDiscreteInputs(false);
  Alarm.delay(LIGHT_REFRESH_INTERVAL);
}

void refreshDiscreteInputs(bool trace)
{
  for (int i = 0; i < MAX_DISCRETE_PINS; i++)
  {
    if (!DiscreteIO[i].dir && DiscreteIO[i].dInput != NULL)
    {
      DiscreteIO[i].dInput->refresh();
      if (trace)
      {
        activate485Tx();        
        DiscreteIO[i].dInput->serialize(tracePort, true);
        activate485Rx();        
      }
    }
  }
}

void refreshDiscreteOutputs()
{
}

// update sonar and 1-wire DHT-22 readings
void doOnPoll()
{
  refreshDiscreteInputs(false);

#ifdef PROCESS_TERMINAL
  refreshPolling();
  if (currentTogglePin > 0)
    toggleDO(currentTogglePin);
#endif

}

void doReadInputs()
{
}

void doUpdateOutputs()
{
}

#ifdef PROCESS_TERMINAL
#define DI_SETINPUT_HEADER 'I'
#define DO_SETOUTPUT_HEADER 'O'
#define DI_PULLUP_HEADER 'R'
#define DI_READ_HEADER 'D'
#define AI_READ_HEADER 'A'
#define DO_WRITE_HEADER 'W'
#define I_POLL_HEADER 'P'
#define O_TOGGLE_HEADER 'T'
#define HELP_HEADER '?'
#define ONE_WIRE_HEADER '1'
void printDigits(int digits)
{
  *tracePort << ":";
  if (digits < 10)
    *tracePort << '0';
  *tracePort << digits;
}

void showCommands()
{

  activate485Tx();

  *tracePort << "-------------------------------------------------------------------" << endl;
  *tracePort << F("OXX  - Set Discrete Output Pin where XX is pin #") << endl;
  *tracePort << F("IXX  - Set Discrete Input Pin 14 = all as inputs") << endl;
  *tracePort << F("RX   - Read Analog Raw X pin") << endl;
  *tracePort << F("DX   - Read Discrete X Pin") << endl;
  *tracePort << F("WXX 1|0 - Write Discrete output XX is pin ") << endl;
  *tracePort << F("DI   - Dump all discrete Inputs ") << endl;
  *tracePort << F("DO   - Dump all discrete Outputs and toogles output ") << endl;
  *tracePort << F("DA   - Dump all analog Inputs ") << endl;
  *tracePort << F("??   - Display commands") << endl;
  *tracePort << F("PA1|0   - Poll Analogs 1=on 0=off ") << endl;
  *tracePort << F("TR|S X  - Period Toggle DO X = pin # R=run/S=Stop") << endl;
  *tracePort << F("1W    - display one wire temperatures") << endl;
  *tracePort << "------------------------------------------------------------------" << endl;

  activate485Rx();

}

bool toggleDO(int aPin)
{
  if (DiscreteIO[aPin].dir && DiscreteIO[aPin].dOutput != NULL)
  {
    DiscreteIO[aPin].dOutput->toggle();
    activate485Tx();    
    DiscreteIO[aPin].dOutput->serialize(tracePort, true);
    activate485Rx();
    return(true);
  }
  activate485Tx();
  *tracePort << "Pin " << aPin << " is not defined as an output." << endl;
  activate485Rx();
  return(false);
}

void processTogglePin()
{
  char c = tracePort->read();
  if (c == 'S')
    currentTogglePin = -1;
  else
  {
    int pin = tracePort->parseInt();
    if (validate(pin, 0, MAX_DISCRETE_PINS))
    {
      currentTogglePin = (toggleDO(pin) == true? pin:-1);
    }
    else
    {
      activate485Tx();
      *tracePort << "Pin " << pin << " is invalid. Range 0-" << MAX_DISCRETE_PINS << endl;
      activate485Tx();
    }
  }
}

void dumpDOs()
{
  for (int i = 0; i < MAX_DISCRETE_PINS; i++)
  {
    if (DiscreteIO[i].dir && DiscreteIO[i].dOutput != NULL)
    {
      toggleDO(i);
    }
  }
}

bool validate(int aValue, int min, int max)
{
  if (aValue >= min && aValue <= max)
    return(true);
  return(false);
}

void processSetupAsInputs()
{
  int pin = tracePort->parseInt();
  if (validate(pin, 0, MAX_DISCRETE_PINS))
  {
    if (pin == MAX_DISCRETE_PINS)
    {
      for (int i = 0; i < MAX_DISCRETE_PINS; i++)
      {
        initAsInput(i);
      }
    }
    else
    {
      initAsInput(pin);
    }
  }
  else
  {
    activate485Tx();
    *tracePort << "Pin " << pin << " is invalid. Range 0-" << MAX_DISCRETE_PINS << endl;
    activate485Rx();
  }
}

void processSetupAsOutputs()
{
  int pin = tracePort->parseInt();
  if (validate(pin, 0, MAX_DISCRETE_PINS))
  {
    if (pin == MAX_DISCRETE_PINS)
    {
      for (int i = 0; i < MAX_DISCRETE_PINS; i++)
      {
        initAsOutput(i);
      }
    }
    else
    {
      initAsOutput(pin);
    }
  }
  else
  {
    activate485Tx();
    *tracePort << "Pin " << pin << " is invalid. Range 0-" << MAX_DISCRETE_PINS << endl;
    activate485Rx();
  }
}

void processReadDiscretes()
{
  char c = tracePort->read();
  switch (c)
  {
    case 'I':
      refreshDiscreteInputs(true);
      // pinMode( A0, INPUT);
      // *tracePort << "AO=" << digitalRead(A0) << endl;
      break;
    case 'O':
      dumpDOs();
      // 
      break;
    case 'A':
      readAnalogs();
      break;
    default:
      break;
  }
}

void readAnalogs()
{
  for (int i = 0; i < 8; i++)
  {
    int sensorValue = analogRead(i);
    delay(10);
    sensorValue = analogRead(i);
    delay(10);
    sensorValue = map(sensorValue, 0, 1023, 0, 5);
    activate485Tx();
    *tracePort << "Analog  " << i << " = " << sensorValue << endl;
    activate485Rx();
  }
}

void refreshPolling()
{
  if (polling == 1)
    readAnalogs();
}

void processPolling()
{
  char c = tracePort->read();
  int mode = -1;
  switch (c)
  {
    case 'A':
      mode = tracePort->parseInt();
      if (validate(mode, 0, 1))
      {
        polling = (mode == 1? 1:-1);
      }
      else
      {
        activate485Tx();
        *tracePort << mode << " is outside 0-1" << endl;
        activate485Rx();
      }
      break;
    case 'D':
      mode = tracePort->parseInt();
      if (validate(mode, 0, 1))
      {
        polling = (mode == 1? 2:-1);
      }
      else
      {
        activate485Tx();
        *tracePort << mode << " is outside 0-1" << endl;
        activate485Rx();
      }
      // 
      break;
    default:
      break;
  }
}

void poll1WireTemperature(DallasTemperature * sensor, int aPin)
{
  sensor->requestTemperatures();
  activate485Tx();
  *tracePort << "Temperature " << aPin << " = " << sensor->getTempCByIndex(0) << " C" << endl;
  activate485Rx();
}

void processOneWireTemperatures()
{
  char c = tracePort->read();
  poll1WireTemperature(& sensors1, TEMPERATURE1);
}

void processDOWrites()
{
  int pin = tracePort->parseInt();
  if (validate(pin, 0, MAX_DISCRETE_PINS))
  {
    int value = tracePort->parseInt(); // get 1 or 0
    DiscreteIO[pin].dOutput->write(value);
  }
  else
  {
    activate485Tx();
    *tracePort << "Pin " << pin << " is invalid. Range 0-" << MAX_DISCRETE_PINS << endl;
    activate485Rx();
  }
}

void processTerminalCommands()
{
  if (tracePort->available() > 1)
  {
    // wait for at least two characters
    char c = tracePort->read();
    // S*tracePort << "c=" << c << endl;
    // *tracePort << c << endl;
    if (c == DI_SETINPUT_HEADER)
    {
      processSetupAsInputs();
      // processTimeSetMessage();
    }
    else
      if (c == DO_SETOUTPUT_HEADER)
      {
        processSetupAsOutputs();
        // processSetInputs();
      }
    else
      if (c == DI_PULLUP_HEADER)
      {
        // processLightsMessage();
      }
    else
      if (c == HELP_HEADER)
      {
        tracePort->read();
        showCommands();
      }
    else
      if (c == DI_READ_HEADER)
      {
        processReadDiscretes();
      }
    else
      if (c == DO_WRITE_HEADER)
      {
        processDOWrites();
      }
    else
      if (c == I_POLL_HEADER)
      {
        processPolling();
      }
    else
      if (c == O_TOGGLE_HEADER)
      {
        processTogglePin();
      }
    else
      if (c == ONE_WIRE_HEADER)
      {
        processOneWireTemperatures();
      }
  }
}

#endif

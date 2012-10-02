
/********************************************************
 * BeerPID 
 * build from RelayOutput Example
 *
 * Copyright 2012 Ruotger Deecke
 * Pull requests welcome, when put under the same license
 *
 ********************************************************/

/*
  Copyright (c) 2012 Ruotger Deecke.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without 
  modification, are permitted provided that the following conditions are met:
  
  Redistributions of source code must retain the above copyright notice, this 
  list of conditions and the following disclaimer.

  Redistributions in binary form must reproduce the above copyright notice, this 
  list of conditions and the following disclaimer in the documentation and/or 
  other materials provided with the distribution.

  All advertising materials mentioning features or use of this software must 
  display the following acknowledgement: 
  “This product includes software developed by Ruotger Deecke and his 
   contributors.”

  Neither the name of Ruotger Deecke nor the names of his contributors may be 
  used to endorse or promote products derived from this software without 
  specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS “AS IS” AND ANY 
  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
  DISCLAIMED. IN NO EVENT SHALL THE RUOTGER DEECKE OR CONTRIBUTORS BE LIABLE FOR 
  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND 
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


// https://github.com/br3ttb/Arduino-PID-Library
#include <PID_v1.h>

// https://github.com/br3ttb/Arduino-PID-AutoTune-Library
#include <PID_AutoTune_v0.h>

// http://www.pjrc.com/teensy/td_libs_OneWire.html
#include <OneWire.h>

// http://milesburton.com/Dallas_Temperature_Control_Library
#include <DallasTemperature.h>

// comes with Arduino tools
#include <SoftwareSerial.h>


#define DO_LOG_ERRORS false


// --- Pin definitions

// output pin for solid-state relais (LED on the board at the same time)
#define SSR_PIN 13

// OneWire Data 
#define ONE_WIRE_BUS 2

// serial receive / transmit for the Sparkfun serial 7-segment display
// e.g.: https://www.sparkfun.com/products/9766
// the receive pin is unused, but must be defined for the SoftwareSerial library
#define SEVEN_SEGMENT_PIN_RX 11
#define SEVEN_SEGMENT_PIN_TX 10

// two LEDs for displaying status
#define RED_LED_PIN 4
#define GREEN_LED_PIN 3

// three switches for choosing the temperature setpoint
#define SWITCH_0_PIN 5
#define SWITCH_1_PIN 6
#define SWITCH_2_PIN 7

// button to start autotuning
#define TUNING_BUTOON_PIN A5

// Setup a oneWire instance to communicate with any OneWire devices 
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our OneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// since OneWire is a bus you can connect more than one, we limit this here to
// ten. Increase if necessary, decrease to save a couple of bytes
#define MAX_THERMOMETERS 10
DeviceAddress thermAddress[MAX_THERMOMETERS];
int deviceCount = 0;

// -- software serial for 7-segment
SoftwareSerial sevenSegSerial(SEVEN_SEGMENT_PIN_RX, SEVEN_SEGMENT_PIN_TX);

// on-off window
int WindowSize = 2000;

// logging window
int DisplayWindowSize = 1000;

// -- PID: global Variables that the PID and the Autotune use
double Setpoint, Input, Output;

// PID parameters; have fun tuning them apropriately
double kp=17.57, ki=0.5, kd=0;

// PID object
PID myPID(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);

// -- Autotune
double aTuneStep=WindowSize/3, aTuneNoise=1, aTuneStartValue=WindowSize/2;
unsigned int aTuneLookBack=20;
byte ATuneModeRemember=2;

PID_ATune aTune(&Input, &Output);

// tuning state
boolean isTuning = false;

// window start times
unsigned long windowStartTime = 0;
unsigned long displayWindowStartTime = 0;


void changeAutoTune()
{
 if(!isTuning)
  {
    //Set the output to the desired starting frequency.
    Output=aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    isTuning = true;
  }
  else
  { //cancel autotune
    aTune.Cancel();
    isTuning = false;
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(boolean start)
{
  if(start)
    ATuneModeRemember = myPID.GetMode();
  else
    myPID.SetMode(ATuneModeRemember);
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

boolean getTemperature(float *outTemp)
{
  float minTemp = 1000;
  float maxTemp = -1000;
  float meanTemp = 0;
  boolean isTempOK = false;
  
  for (int i=0; i<deviceCount; i++)
  {
    float temp = sensors.getTempC(thermAddress[i]);
    if (temp < -125 || temp == 85)
    {
      if (DO_LOG_ERRORS)
      {
        Serial.println("*******");
        Serial.print("Error in Thermometer ");
        printAddress(thermAddress[i]);      
        Serial.println("*******");
      }
      continue;
    }
    isTempOK = true;
    minTemp = (temp<minTemp) ? temp : minTemp;
    maxTemp = (temp>maxTemp) ? temp : maxTemp;
    meanTemp += temp;
  }
  
  if (deviceCount)
    meanTemp /= deviceCount;

  if (DO_LOG_ERRORS)
  {
    Serial.print(minTemp);
    Serial.print(",");
    Serial.print(meanTemp);
    Serial.print(",");
    Serial.print(maxTemp);
    Serial.print(", ");
  }    
  
  *outTemp = maxTemp;
  return isTempOK;
}


// ============= SETUP ================


void setup()
{
  // start serial port
  Serial.begin(9600);
  Serial.println(" ");
  Serial.println(" ");
  Serial.println("BeerPID");
  Serial.println(" ");
  Serial.println("Copyright 2012 (C) Ruotger Deecke.");
  Serial.println(" ");
  Serial.println("I wouldn't mind tasting your beer but this is");
  Serial.println("not a requisite for using this software.");
  Serial.println("Have fun, good luck and cheers!");
  Serial.println(" ");
  

  // Start up the library
  sensors.begin();

  Serial.print("Locating OneWire devices...");

  deviceCount = sensors.getDeviceCount();
  Serial.print("Found ");
  Serial.print(deviceCount, DEC);
  Serial.print(" devices. (Will use ");
  deviceCount = deviceCount>MAX_THERMOMETERS ? MAX_THERMOMETERS : deviceCount;
  Serial.print(deviceCount, DEC);
  Serial.println(")");

  for (int i=0; i<deviceCount; i++)
  {
    if (!sensors.getAddress(thermAddress[i], i))
    { 
      Serial.print("Unable to find address for Device "); 
      Serial.println(i, DEC);
    }
    else
    {
      Serial.print("Device ");
      Serial.print(i, DEC);
      Serial.print(": ");
      printAddress(thermAddress[i]);      
      Serial.println(" ");
    }
  }

  windowStartTime = millis();
  
  //initialize the variables we're linked to
  Setpoint = 500;

  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  
  sevenSegSerial.begin(9600);

  // reset
  sevenSegSerial.write(0x76);
  
  // decimal point
  sevenSegSerial.write(0x77);
  sevenSegSerial.write(0x04);

  // setup LED pins
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, HIGH);
  digitalWrite(GREEN_LED_PIN, HIGH);

  // setup switch pins
  pinMode(SWITCH_0_PIN, INPUT);
  pinMode(SWITCH_1_PIN, INPUT);
  pinMode(SWITCH_2_PIN, INPUT);

  Serial.println("ready!");
  Serial.println(" ");  
  
  if (readSwitches() == 3)
  {
    Serial.println("you chose 57° at startup, so I will enter autotune!");
    Serial.println(" ");  
    changeAutoTune();  
  }

  // if you want to do a csv import, this is the first line     
  Serial.println("Temp, Output, Setpoint, kp, ki, kd");
}

void displayLEDTemp(float inTemp)
{
  char hundreds = (char)(inTemp/100);
  char tens = (char)floor(inTemp/10 - hundreds*10);
  char ones = (char)floor(inTemp - tens*10 - hundreds*100);
  char tenths = (char)floor(inTemp*10 - ones*10 - tens*100 - hundreds*1000);

  if (hundreds != 0)
  {
    sevenSegSerial.write(hundreds);   
  }
  else
  {
    sevenSegSerial.print("x");    
  }

  if (tens != 0 || hundreds != 0)
  {
    sevenSegSerial.write(tens);   
  }
  else
  {
    sevenSegSerial.print("x");    
  }

  sevenSegSerial.write(ones);
  sevenSegSerial.write(tenths);
}

int readSwitches()
{
  int switch0State = digitalRead(SWITCH_0_PIN);
  int switch1State = digitalRead(SWITCH_1_PIN);
  int switch2State = digitalRead(SWITCH_2_PIN);
  
  int switches = switch0State + switch1State*2 + switch2State*4;

  return switches;
}

// ============= LOOP ================


void loop()
{
  // call sensors.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus
  sensors.requestTemperatures(); // Send the command to get temperatures
  
  float temp = 0;
  
  boolean isTempOK = getTemperature(&temp); 

  float displayTemp = 0;
  if(millis() - windowStartTime < WindowSize/3)
  {
    if (isTuning)
    {
      sevenSegSerial.write(0x77);
      sevenSegSerial.write(0x01);
      sevenSegSerial.print("tUNE");    
    }
    else   
    {
      sevenSegSerial.write(0x77);
      sevenSegSerial.write(0x20);
      displayLEDTemp(Setpoint);
    }
  }
  else
  {
    if (isTempOK)
    {
      sevenSegSerial.write(0x77);
      sevenSegSerial.write(0x04);
      displayLEDTemp(temp);
    }
    else
    {
      sevenSegSerial.write(0x77);
      sevenSegSerial.write(0x04);
      sevenSegSerial.print("Errx");        
    }
  }

  Input = temp;
  
  if(isTuning)
  {
    byte val = (aTune.Runtime());
    if (val!=0)
    {
      isTuning = false;
    }
    if(!isTuning)
    { //we're done, set the tuning parameters
      kp = aTune.GetKp();
      ki = aTune.GetKi();
      kd = aTune.GetKd();
      myPID.SetTunings(kp,ki,kd);
      AutoTuneHelper(false);
    }
  }
  else 
  {
    int switches = readSwitches();
    
    switch(switches)
    {
      case 0:
        Setpoint = 36;
        break;
      case 1:
        Setpoint = 44;
        break;
      case 2:
        Setpoint = 50;
        break;
      case 3:
        Setpoint = 57;
        break;
      case 4:
        Setpoint = 62;
        break;
      case 5:
        Setpoint = 72;
        break;
      case 6:
        Setpoint = 78;
        break;
      case 7:
        Setpoint = 105;
        break;
    }
    myPID.Compute();
  }

  /************************************************
   * turn the output pin on/off based on pid output
   ************************************************/
  if(millis() - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  
  if(millis() - displayWindowStartTime > DisplayWindowSize)
  { //time to shift the Relay Window
    displayWindowStartTime += DisplayWindowSize;

    Serial.print(Input);
    Serial.print(", ");
    Serial.print(Output);
    Serial.print(", ");
    Serial.print(Setpoint);
    
    Serial.print(", ");
    Serial.print(kp);
    Serial.print(", ");
    Serial.print(ki);
    Serial.print(", ");
    Serial.println(kd);
  }
    
  if(Output < millis() - windowStartTime)
  { 
    digitalWrite(SSR_PIN,LOW);
    digitalWrite(GREEN_LED_PIN,LOW);
  }
  else 
  {
    digitalWrite(SSR_PIN,HIGH);
    digitalWrite(GREEN_LED_PIN,HIGH);
  }
  
  int tuneButtonState = analogRead(TUNING_BUTOON_PIN)>20 ? 0 : 1;

  if (tuneButtonState && !isTuning)
  {
    changeAutoTune();  
  }
  
  digitalWrite(RED_LED_PIN,isTuning);

  delay(10);
}




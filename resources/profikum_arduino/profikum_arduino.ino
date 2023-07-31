// Copyright 2023 Moritz Lang
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files (the "Software"), to deal in
// the Software without restriction, including without limitation the rights to
// use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
// the Software, and to permit persons to whom the Software is furnished to do so,
// subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
// IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
// CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "ProfikumDevice.h"
#include "profikum_com.h"
#include <stdint.h>
#include <Wire.h>
#include <PololuBuzzer.h>

// Constants
#define LED_PIN 13

// Uncomment the next line to additionally send any serial data over SERIAL_PORT_DEBUG. 
//#define DEBUG
// Which serial port to use. Set to "Serial" for communication via microUSB, and to "Serial1" for GPIO based com.
#define SERIAL_DEBUG_PORT Serial
#define SERIAL_PORT Serial1

// Forward declactions
void serialOutput(profikum::com::ProfikumOutput command, int value);

// Global controller object.
profikum::arduino::ProfikumDevice controller{};

// Buzzer
PololuBuzzer buzzer;

void setup()
{
  // Initialize the Wire library and join the I2C bus as a master
  Wire.begin();
  
  pinMode(LED_PIN, OUTPUT);
  SERIAL_PORT.begin(38400, SERIAL_8N1);
  
  #ifdef DEBUG
    SERIAL_DEBUG_PORT.begin(38400, SERIAL_8N1);
    while (!SERIAL_PORT && !SERIAL_DEBUG_PORT) 
    {
      // busy wait for serial port to connect.
    }
  #else
    while (!SERIAL_PORT) 
    {
      // busy wait for serial port to connect.
    }
  #endif
  controller.Init(&serialOutput);
}
void receiveSerial();
void loop()
{
  SERIAL_PORT.flush();
  #ifdef DEBUG
    SERIAL_DEBUG_PORT.flush();
  #endif
  controller.Run();
  receiveSerial();
}

const char connectSound[] PROGMEM = "!C8E16";
const char disconnectSound[] PROGMEM = "!E16C4";

/*
  Called whenever new data comes in the hardware serial RX. This
  routine is run between each time loop() runs.
*/
void receiveSerial()//serialEvent() 
{
  static int step = 0;
  static char command = 'e';
  static int firstMessage = 0;
  static int secondMessage = 0;
  static bool online = false;
  static long lastTimeReceived_ms = 0;
  
  bool anyComplete = false; 
  while(SERIAL_PORT.available()>=1)
  { 
    int val = SERIAL_PORT.read();
    bool commandComplete = false;
    switch(step)
    {
      case 0:
        command = (char)val;
        step++;
        break;
      case 1:
        firstMessage = val;
        step++;
        break;
      case 2:
        secondMessage = val;
        commandComplete = true;
        step++;
        break;
      default:
        if(((byte)val) == profikum::com::stopByte)
          step=0;
        break;
    }
    if(commandComplete)
    {
      // get the value (one 16bit int)
      // network endianess is big endian, Arduino uses little endian
      int16_t value = (firstMessage << 8) | secondMessage;
      anyComplete |= controller.ProcessInput(profikum::com::ToProfikumInput(command), value);
    }
  }

  // Check if we are still connected.
  long time_ms = millis();
  if(anyComplete)
  {
    lastTimeReceived_ms = time_ms;
    if(!online)
    {
      online = true;
      buzzer.playFromProgramSpace(connectSound);
    }
  }
  else if(online && time_ms-lastTimeReceived_ms > 500)
  {
    // stop motor
    controller.ProcessInput(profikum::com::ProfikumInput::error, 0);
    
    online = false;
    buzzer.playFromProgramSpace(disconnectSound);
  }
}
/**
 * Send data over serial. Used as feedback for controller.
 */
void serialOutput(profikum::com::ProfikumOutput command, int16_t value)
{
  SERIAL_PORT.write(profikum::com::FromProfikumOutput(command));
  // Little endian to network/big endian conversion
  SERIAL_PORT.write((value >> 8 ) & 0xFF);
  SERIAL_PORT.write((value      ) & 0xFF);
  SERIAL_PORT.write(profikum::com::stopByte);

  #ifdef DEBUG
    SERIAL_DEBUG_PORT.write(profikum::com::FromProfikumOutput(command));
    // Little endian to network/big endian conversion
    SERIAL_DEBUG_PORT.write((value >> 8 ) & 0xFF);
    SERIAL_DEBUG_PORT.write((value      ) & 0xFF);
    SERIAL_DEBUG_PORT.write(profikum::com::stopByte);
  #endif
}

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

#include "ProfikumSupersonic.h"
#include <Arduino.h>

namespace profikum::arduino
{
ProfikumSupersonic::ProfikumSupersonic(int triggerPin_, int echoPin_, long minTimeBetweenPulses_us_, long maxTimeBetweenPulses_us_) : 
    triggerPin{triggerPin_},
    echoPin{echoPin_},
    minTimeBetweenPulses_us{minTimeBetweenPulses_us_}, 
    maxTimeBetweenPulses_us{maxTimeBetweenPulses_us_}
{
  
}
void ProfikumSupersonic::Init()
{
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void ProfikumSupersonic::OnInterrupt()
{  
  bool echo = digitalRead(echoPin);
  if(!lastEcho && echo)
  {
    // start of the echo pulse
    echoEnd_us = 0;                                 // Clear the end time
    echoStart_us = micros();                        // Save the start time
  }
  else if(lastEcho && !echo && echoStart_us != 0)
  {
    // echo received
    echoEnd_us = micros();                          // Save the end time
    long dur = echoEnd_us - echoStart_us;        // Calculate the pulse duration
    if(dur < maxTimeBetweenPulses_us)
      echoDuration_us = dur;
  }
  lastEcho = echo;
}
void ProfikumSupersonic::Run()
{
  long time_us = micros();
  if((echoEnd_us != 0 && time_us - echoStart_us >= minTimeBetweenPulses_us) // last measurement finished
    || time_us - echoStart_us >= maxTimeBetweenPulses_us) // last measurement failed
  {
    // echoStart will be overwritten by interrupt. We nevertheless set it here to prevent triggering
    // multiple pulses before OnInterrupt was called...
    echoStart_us = time_us;
    echoEnd_us = 0;
    // A measurement is triggered by keeping the trigger pin at least 10us up
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(12);
    digitalWrite(triggerPin, LOW);
  }
}
int16_t ProfikumSupersonic::GetLastDistance_mm()
{
  cli();
  long duration = echoDuration_us;
  echoDuration_us = 0;
  sei();
  
  if(duration<=0)
    return -1;
  int16_t distance_mm = duration*durationToDistance;
  // Überprüfung ob gemessener Wert innerhalb der zulässingen Entfernung liegt
  if (distance_mm >= maxDistance_mm || distance_mm <= minDistance_mm) 
    return -1;
  else
    return distance_mm;
}
}

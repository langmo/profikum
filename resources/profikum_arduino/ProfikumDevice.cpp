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
#include "ProfikumDevice.h"
#include "ProfikumMotors.h"
#include "ProfikumImu.h"
#include "ProfikumEncoders.h"

#include <Arduino.h>

#define MAX_SPEED 400

#define RIGHT_SUPERSONIC_ECHO_PIN 11 // Echo pin of the ultrasound distance sensor
#define RIGHT_SUPERSONIC_TRIGGER_PIN 12 // Trigger pin of the ultrasound distance sensor

//TODO: Change to correct pins and configure interrupts
#define LEFT_SUPERSONIC_ECHO_PIN 11 // Echo pin of the ultrasound distance sensor
#define LEFT_SUPERSONIC_TRIGGER_PIN 12 // Trigger pin of the ultrasound distance sensor

namespace profikum::arduino
{
static ProfikumMotors motors{};
// accelerometer, magnetometer, and gyro
static ProfikumImu imu{};
static ProfikumSupersonic rightSuperSonic{RIGHT_SUPERSONIC_TRIGGER_PIN, RIGHT_SUPERSONIC_ECHO_PIN};
static ProfikumSupersonic leftSuperSonic{LEFT_SUPERSONIC_TRIGGER_PIN, LEFT_SUPERSONIC_ECHO_PIN};
static ProfikumEncoders encoders{};
}

/**
 * Interrupt handlers
 */
ISR(PCINT0_vect)
{
  profikum::arduino::rightSuperSonic.OnInterrupt();
  profikum::arduino::encoders.OnLeftInterrupt();
}
// interrupt pin 7
static void interrupt4()
{
  profikum::arduino::encoders.OnRightInterrupt();
}


namespace profikum::arduino
{
ProfikumDevice::ProfikumDevice()
{
}
void ProfikumDevice::Init(void (*outputProcessor_)(com::ProfikumOutput, int16_t))
{
  outputProcessor = outputProcessor_;

  // Unset global interrupt enable bits (I) in the status register (SREG), such that
  // no interrupts will be thrown while changing configuration
  cli();
  
  // Init IMU sensors.
  if (!imu.Init())
  {
    // If initialization failed, send error message in endless loop.
    while(1)
    {
      outputProcessor(profikum::com::ProfikumOutput::error, -1);
      delay(1000);
    }
  }
  imu.EnableDefault();

  rightSuperSonic.Init();
  leftSuperSonic.Init();
  motors.Init();
  encoders.Init();

  // Configure Interrupt 4 (pin 7)
  attachInterrupt(4, interrupt4, CHANGE);

  // Configure hardware interrupt when echo pin changes state
  // Echo pin 11 -> PCINT7, which is at PCMSK0, Bit 7
  // Set PCIE0 bit in the bit change interrupt control register (PCICR)
  PCICR |= (1 << PCIE0);
  // Set PCINT7 bt (=bit 7) in the bit change enable mask (PCMSK). The first
  // 8 bits are in PCMSK0, the next 8 in PCMSK1,...
  PCMSK0 |= (1 << PCINT7);
  
  // set global interrupt enable bits (I) in the status register (SREG), such that
  // interrupts are enabled again
  sei();
}
void ProfikumDevice::ProcessInput(com::ProfikumInput command, int16_t value)
{
  switch(command)
  {
    case com::ProfikumInput::leftMotorSetSpeed:
      leftSpeed = value;
      if(leftSpeed > MAX_SPEED)
        leftSpeed = MAX_SPEED;
      else if(leftSpeed < -MAX_SPEED)
        leftSpeed = -MAX_SPEED;
      break;
    case com::ProfikumInput::rightMotorSetSpeed:
      rightSpeed = value;
      if(rightSpeed > MAX_SPEED)
        rightSpeed = MAX_SPEED;
      else if(rightSpeed < -MAX_SPEED)
        rightSpeed = -MAX_SPEED;
      break;
    case com::ProfikumInput::error:
      leftSpeed = 0;
      rightSpeed = 0;
      break;
  }
}



void ProfikumDevice::Run()
{
  motors.SetLeftSpeed(leftSpeed);
  motors.SetRightSpeed(rightSpeed);

  rightSuperSonic.Run();
  leftSuperSonic.Run();
  encoders.Run();

  if(outputProcessor != nullptr)
  {
    imu.Read();
    // Accelleration
    outputProcessor(com::ProfikumOutput::accelerationX, imu.a.x);
    outputProcessor(com::ProfikumOutput::accelerationY, imu.a.y);
    outputProcessor(com::ProfikumOutput::accelerationZ, imu.a.z);
    // Gyro 
    outputProcessor(com::ProfikumOutput::gyroX, imu.g.x);
    outputProcessor(com::ProfikumOutput::gyroY, imu.g.y);
    outputProcessor(com::ProfikumOutput::gyroZ, imu.g.z);
    // magnetometer 
    outputProcessor(com::ProfikumOutput::magnetometerX, imu.m.x);
    outputProcessor(com::ProfikumOutput::magnetometerY, imu.m.y);
    outputProcessor(com::ProfikumOutput::magnetometerZ, imu.m.z);
    // Ultrasound distance
    outputProcessor(com::ProfikumOutput::rightUltrasoundDistance, rightSuperSonic.GetLastDistance_mm());
    outputProcessor(com::ProfikumOutput::leftUltrasoundDistance, leftSuperSonic.GetLastDistance_mm());
    // encoders
    outputProcessor(com::ProfikumOutput::leftEncoderCounts, encoders.GetCountsLeft());
    outputProcessor(com::ProfikumOutput::rightEncoderCounts, encoders.GetCountsRight());
    outputProcessor(com::ProfikumOutput::leftEncoderCountsPerSecond, encoders.GetCountsPerSecondLeft());
    outputProcessor(com::ProfikumOutput::rightEncoderCountsPerSecond, encoders.GetCountsPerSecondRight());
  }
}
}

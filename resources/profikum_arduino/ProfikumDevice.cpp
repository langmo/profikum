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

#define RIGHT_SUPERSONIC_ECHO_PIN 11 // Echo pin of the ultrasound distance sensor
#define RIGHT_SUPERSONIC_TRIGGER_PIN 12 // Trigger pin of the ultrasound distance sensor

#define LEFT_SUPERSONIC_ECHO_PIN 14 // Echo pin of the ultrasound distance sensor
#define LEFT_SUPERSONIC_TRIGGER_PIN 4 // Trigger pin of the ultrasound distance sensor

namespace profikum::arduino
{
// Calculate times for supersonic sensors
static constexpr const double minDistance = 0.05; //m
static constexpr const double maxDistance = 0.80; //m
static constexpr const double speedSound = 343; // m/s
static constexpr const long minTimeBetweenPulses_us= (long)(2 * minDistance / speedSound * 1e6);
static constexpr const long maxTimeBetweenPulses_us= (long)(2 * maxDistance / speedSound * 1e6);

static ProfikumMotors motors{};
// accelerometer, magnetometer, and gyro
static ProfikumImu imu{};
static ProfikumSupersonic leftSuperSonic{RIGHT_SUPERSONIC_TRIGGER_PIN, RIGHT_SUPERSONIC_ECHO_PIN, minTimeBetweenPulses_us, maxTimeBetweenPulses_us};
static ProfikumSupersonic rightSuperSonic{LEFT_SUPERSONIC_TRIGGER_PIN, LEFT_SUPERSONIC_ECHO_PIN, minTimeBetweenPulses_us, maxTimeBetweenPulses_us};
static ProfikumEncoders encoders{};
}

/**
 * Interrupt handlers
 */
ISR(PCINT0_vect)
{
  profikum::arduino::rightSuperSonic.OnInterrupt();
  profikum::arduino::leftSuperSonic.OnInterrupt();
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

  // Unset global interrupt enable bits (I) in the status register (SREG), such that
  // no interrupts will be thrown while changing configuration
  cli();

  // Configure Interrupt 4 (pin 7)
  attachInterrupt(4, interrupt4, CHANGE);

  // Configure hardware interrupt when echo pin changes state
  // PB7 = Echo pin 11 -> PCINT7, which is at PCMSK0, Bit 7
  // PB3 = Echo pin 14 -> PCINT3, which is at PCMSK0, Bit 3
  // Set PCIE0 bit in the bit change interrupt control register (PCICR)
  PCICR |= (1 << PCIE0);
  // Set PCINT7 bit (=bit 7) and PCINT3 bit (=bit 3) in the bit change enable mask (PCMSK). The first
  // 8 bits are in PCMSK0, the next 8 in PCMSK1,...
  PCMSK0 |= (1 << PCINT7);
  PCMSK0 |= (1 << PCINT3);
  
  // set global interrupt enable bits (I) in the status register (SREG), such that
  // interrupts are enabled again
  sei();
}
bool ProfikumDevice::ProcessInput(com::ProfikumInput command, int16_t value)
{
  switch(command)
  {
    case com::ProfikumInput::rightMotorSetSpeed:
      rightSpeed = value;
      return true;
    case com::ProfikumInput::leftMotorSetSpeed:
      leftSpeed = value;
      return true;
    case com::ProfikumInput::error:
      leftSpeed = 0;
      rightSpeed = 0;
      return false;
    default:
      return false;
  }
}


template <typename T> int sgn(T val) 
{
    return (T(0) < val) - (val < T(0));
}
void ProfikumDevice::Run()
{
  // Anti wind-up: Only learn correction factors for motors if their input isn't saturated.
  static bool lastSaturated{false};
  long time_ms = millis();

  // Read current speed of motors
  int16_t leftObs  = encoders.GetMillimetersPerSecondLeft();
  int16_t rightObs = encoders.GetMillimetersPerSecondRight();

  // Inner control loop ensuring that the motor speed converges to the set speed.
  // We only learn if both motors are active and move into the same direction, and
  // if neither of the motors were saturated.
  double dt = (time_ms - lastTime_ms)*1.0e-3; // in s
  if(lastTime_ms > 0 && !lastSaturated && lastLeftSpeed*lastRightSpeed > 0)
  {
    // Speed of a motor varies depending on hardware, conitions, ...
    // Learn a scaling constant such that motor speeds converge to a constant setpoint
    leftMotorScaling += scalingLearnConstant * (lastLeftSpeed - leftObs)*dt;
    if(leftMotorScaling > maxScaling)
      leftMotorScaling = maxScaling;
    else if(leftMotorScaling < minScaling)
      leftMotorScaling = minScaling;
  
    rightMotorScaling += scalingLearnConstant * (lastRightSpeed - rightObs)*dt;
    if(rightMotorScaling > maxScaling)
      rightMotorScaling = maxScaling;
    else if(rightMotorScaling < minScaling)
      rightMotorScaling = minScaling;
  

    // When motor speeds are different, the robot should turn. However, it turns out
    // that--due to friction--the robot turns less than expected since the faster motor accelerates the slower,
    // and the slower de-accelerates the faster. As a result, the robot is barely turning. Thus,
    // we have to artificially increase the difference of the motor speeds such that the robot turns as
    // expected by the difference in the motor speed setpoints. 
    if(abs(lastRightSpeed - lastLeftSpeed) >=5)
    {
      diffMotorScaling += diffScalingLearnConstant * (abs(lastRightSpeed - lastLeftSpeed) - sgn(lastRightSpeed - lastLeftSpeed)*sgn(rightObs - leftObs)*abs(rightObs - leftObs));
      if(diffMotorScaling > maxDiffMotorScaling)
        diffMotorScaling = maxDiffMotorScaling;
      else if(diffMotorScaling < minDiffMotorScaling)
        diffMotorScaling = minDiffMotorScaling;
    }
  }
  
  // Set motor speed
  lastSaturated = false;
  double meanSpeed = (leftSpeed + rightSpeed) / 2.0;
  lastSaturated |= motors.SetLeftSpeed(leftMotorScaling*(meanSpeed + diffMotorScaling * (leftSpeed-meanSpeed)) * maxMotorRaw / maxMotorSpeed);
  lastSaturated |= motors.SetRightSpeed(rightMotorScaling*(meanSpeed + diffMotorScaling * (rightSpeed-meanSpeed))  * maxMotorRaw / maxMotorSpeed);
  lastLeftSpeed = leftSpeed;
  lastRightSpeed = rightSpeed;
  lastTime_ms = time_ms;

  // Run sensors
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
    outputProcessor(com::ProfikumOutput::leftEncoderMillimeters, encoders.GetMillimetersLeft());
    outputProcessor(com::ProfikumOutput::rightEncoderMillimeters, encoders.GetMillimetersRight());
    outputProcessor(com::ProfikumOutput::leftEncoderMillimetersPerSecond, leftObs);
    outputProcessor(com::ProfikumOutput::rightEncoderMillimetersPerSecond, rightObs);
    // debug
    outputProcessor(com::ProfikumOutput::scalingLeftMotor, leftMotorScaling * 100);
    outputProcessor(com::ProfikumOutput::scalingRightMotor, rightMotorScaling * 100);
    outputProcessor(com::ProfikumOutput::scalingDiffMotor, diffMotorScaling * 100);
  }
}
}

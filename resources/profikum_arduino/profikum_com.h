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

#ifndef PROFIKUM_COM_H
#define PROFIKUM_COM_H
#pragma once

namespace profikum
{
namespace com
{
  constexpr char stopByte{'\n'};
template<typename EnumType, EnumType... Values> class EnumCheck;

template<typename EnumType> class EnumCheck<EnumType>
{
public:
  template<typename IntType> static bool constexpr IsValue(IntType) 
  { 
    return false; 
  }
};

template<typename EnumType, EnumType V, EnumType... Next>
class EnumCheck<EnumType, V, Next...> : private EnumCheck<EnumType, Next...>
{
  using super = EnumCheck<EnumType, Next...>;
public:
  template<typename IntType> static bool constexpr IsValue(IntType v)
  {
      return v == static_cast<IntType>(V) || super::IsValue(v);
  }
};

enum class ProfikumInput: char
{
  error = 'e',
  leftMotorSetSpeed = 'l',
  rightMotorSetSpeed = 'r'
}; 
bool constexpr IsProfikumInput(char v)
{
  using TestCheck = EnumCheck<ProfikumInput, ProfikumInput::error, ProfikumInput::leftMotorSetSpeed, ProfikumInput::rightMotorSetSpeed>;
  return TestCheck::IsValue(v);
}
ProfikumInput constexpr ToProfikumInput(char v)
{
  return IsProfikumInput(v) ? static_cast<ProfikumInput>(v) : ProfikumInput::error;
}
char constexpr FromProfikumInput(ProfikumInput output)
{
  return static_cast<char>(output);
}

enum class ProfikumOutput: char
{
  error = 'e',
  // Acceleration
  accelerationX = 'a',
  accelerationY = 'b',
  accelerationZ = 'c',
  // Gyro 
  gyroX = 'g',
  gyroY = 'h',
  gyroZ = 'i',
  // Magnetometer 
  magnetometerX = 'm',
  magnetometerY = 'n',
  magnetometerZ = 'o',
  // Ultrasound distance
  rightUltrasoundDistance = 'u',
  leftUltrasoundDistance = 'v',
  // Encoder
  leftEncoderMillimeters = 'l',
  rightEncoderMillimeters = 'r',
  leftEncoderMillimetersPerSecond = 's',
  rightEncoderMillimetersPerSecond = 't',
  // Debug
  scalingLeftMotor = '1',
  scalingRightMotor = '2',
  scalingDiffMotor = '3'
}; 
bool constexpr IsProfikumOutput(char v)
{
  using TestCheck = EnumCheck<ProfikumOutput, ProfikumOutput::error,
    ProfikumOutput::accelerationX, ProfikumOutput::accelerationY, ProfikumOutput::accelerationZ,
    ProfikumOutput::gyroX, ProfikumOutput::gyroY, ProfikumOutput::gyroZ, 
    ProfikumOutput::magnetometerX, ProfikumOutput::magnetometerY , ProfikumOutput::magnetometerZ,
    ProfikumOutput::rightUltrasoundDistance, ProfikumOutput::leftUltrasoundDistance,
    ProfikumOutput::leftEncoderMillimeters, ProfikumOutput::rightEncoderMillimeters,
    ProfikumOutput::leftEncoderMillimetersPerSecond, ProfikumOutput::rightEncoderMillimetersPerSecond,
    ProfikumOutput::scalingLeftMotor, ProfikumOutput::scalingRightMotor, ProfikumOutput::scalingDiffMotor>;
  
  return TestCheck::IsValue(v);
}
ProfikumOutput constexpr ToProfikumOutput(char v)
{
  return IsProfikumOutput(v) ? static_cast<ProfikumOutput>(v) : ProfikumOutput::error;
}
char constexpr FromProfikumOutput(ProfikumOutput output)
{
  return static_cast<char>(output);
}
}
}
#endif

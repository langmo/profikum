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

#ifndef PROFIKUMDEVICE_H
#define PROFIKUMDEVICE_H
#pragma once

#include "profikum_com.h"
#include <stdint.h>

namespace profikum::arduino
{
class ProfikumDevice
{
public:
  ProfikumDevice();
  bool ProcessInput(com::ProfikumInput command, int16_t value);
  void Run();
  /**
   * Initializes the controller.
   * outputProcessor is a callback which gets notified whenever sensory data changes.
   */
  void Init(void (*outputProcessor)(com::ProfikumOutput, int16_t));
private:
  //Outputs
  int16_t leftSpeed{0};
  int16_t rightSpeed{0};
  static constexpr int16_t maxMotorRaw{400};
  static constexpr int16_t maxMotorSpeed{480}; // mm/s
  double leftMotorScaling{1.0};
  double rightMotorScaling{1.0};
  static constexpr double maxScaling{2.0};
  static constexpr double minScaling{0.5};
  static constexpr double scalingLearnConstant{1.0/maxMotorSpeed/5}; // in 1/mm
  double diffMotorScaling{1.0};
  static constexpr double maxDiffMotorScaling{2.0};
  static constexpr double minDiffMotorScaling{1.0};
  static constexpr double diffReference{25}; //mm/s
  static constexpr double diffScalingLearnConstant{1.0/diffReference/5}; // in 1/mm
  
  long lastTime_ms{0};
  void (*outputProcessor)(com::ProfikumOutput, int16_t) {nullptr};
};
}
#endif

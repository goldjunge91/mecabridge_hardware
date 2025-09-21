/*
 * Copyright (c) 2024 MecaBridge Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MECABRIDGE_HARDWARE__WHEEL_H_
#define MECABRIDGE_HARDWARE__WHEEL_H_


#include <string>

namespace mecabridge_hardware
{

  class Wheel
  {
public:
    std::string name = "";
    int enc = 0;
    double cmd = 0;
    double pos = 0;
    double vel = 0;
    double eff = 0;
    double velSetPt = 0;
    double rads_per_count = 0;

    Wheel() = default;

    Wheel(const std::string & wheel_name, int counts_per_rev);

    void setup(const std::string & wheel_name, int counts_per_rev);

    double calcEncAngle();
  };

}  // namespace mecabridge_hardware

#endif  // MECABRIDGE_HARDWARE__WHEEL_H_

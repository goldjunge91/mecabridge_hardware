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

#include "mecabridge_hardware/wheel.h"

#include <cmath>

namespace mecabridge_hardware
{

Wheel::Wheel(const std::string & wheel_name, int counts_per_rev)
{
  setup(wheel_name, counts_per_rev);
}

void Wheel::setup(const std::string & wheel_name, int counts_per_rev)
{
  name = wheel_name;
  rads_per_count = (2 * M_PI) / counts_per_rev;
}

double Wheel::calcEncAngle()
{
  return enc * rads_per_count;
}

}  // namespace mecabridge_hardware

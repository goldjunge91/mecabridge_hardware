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

#ifndef MECABRIDGE__MOCK_SERIAL_HPP_
#define MECABRIDGE__MOCK_SERIAL_HPP_

#include <string>
#include <cstdint>

#include <gmock/gmock.h>
#include <serial/serial.h>

// Mock class for serial::Serial to enable unit testing without hardware
class MockSerial
{
public:
  MOCK_METHOD(void, setPort, (const std::string & port));
  MOCK_METHOD(void, setBaudrate, (uint32_t baudrate));
  MOCK_METHOD(void, setTimeout, (const serial::Timeout & timeout));
  MOCK_METHOD(void, open, ());
  MOCK_METHOD(void, close, ());
  MOCK_METHOD(bool, isOpen, (), (const));
  MOCK_METHOD(size_t, write, (const std::string & data));
  MOCK_METHOD(std::string, readline, ());
  MOCK_METHOD(size_t, available, (), (const));
  MOCK_METHOD(std::string, read, (size_t size));

  // Additional methods that might be needed
  MOCK_METHOD(void, flush, ());
  MOCK_METHOD(void, flushInput, ());
  MOCK_METHOD(void, flushOutput, ());
  MOCK_METHOD(size_t, bytesAvailable, (), (const));
  MOCK_METHOD(bool, waitReadable, ());
  MOCK_METHOD(void, waitByteTimes, (size_t count));
};

#endif // MECABRIDGE__MOCK_SERIAL_HPP_

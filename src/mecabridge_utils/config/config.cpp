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

#include "config.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <set>
#include <sstream>
#include <vector>

namespace mecabridge
{
namespace config
{

static inline std::string trim(const std::string & s)
{
  size_t i = 0, j = s.size();
  while (i < j && std::isspace(static_cast<unsigned char>(s[i]))) {++i;}
  while (j > i && std::isspace(static_cast<unsigned char>(s[j - 1]))) {--j;}
  return s.substr(i, j - i);
}

static inline std::string strip_inline_comment(const std::string & s)
{
  auto pos = s.find('#');
  return pos == std::string::npos ? s : s.substr(0, pos);
}

static inline bool ieq(const std::string & a, const std::string & b)
{
  if (a.size() != b.size()) {return false;}
  for (size_t i = 0; i < a.size(); ++i) {
    if (std::tolower(static_cast<unsigned char>(a[i])) !=
      std::tolower(static_cast<unsigned char>(b[i]))) {return false;}
  }
  return true;
}

void Config::validate() const
{
  if (serial_port.empty()) {throw std::runtime_error("serial_port missing");}
  if (state_publish_rate_hz < 10 || state_publish_rate_hz > 200) {
    throw std::runtime_error("state_publish_rate_hz out of range");
  }
  if (wheel_radius <= 0.0) {throw std::runtime_error("wheel_radius must be >0");}
  if (wheel_separation_x <= 0.0 || wheel_separation_y <= 0.0) {
    throw std::runtime_error("wheel separation must be >0");
  }
  if (encoder_ticks_per_rev <= 0) {throw std::runtime_error("encoder_ticks_per_rev must be >0");}

  // Wheels encoder indices must be unique, each 0..3
  auto idx = wheel_encoder_indices();
  std::set<int> uniq;
  for (int v : idx) {
    if (v < 0 || v > 3) {throw std::runtime_error("duplicate or invalid encoder_index");}
    if (!uniq.insert(v).second) {throw std::runtime_error("duplicate or invalid encoder_index");}
  }

  // Wheel joint names unique and non-empty
  auto names = wheel_joint_names();
  std::set<std::string> uniq_names;
  for (const auto & n : names) {
    if (n.empty()) {throw std::runtime_error("duplicate joint_name");}
    if (!uniq_names.insert(n).second) {throw std::runtime_error("duplicate joint_name");}
  }

  // Servo positional min<max
  if (!(servos.positional.min_rad < servos.positional.max_rad)) {
    throw std::runtime_error("servo positional min>=max");
  }

  // ESC ranges must define a valid min/max ordering
  if (!(escs.left.esc_min_pwm < escs.left.esc_max_pwm) ||
    !(escs.right.esc_min_pwm < escs.right.esc_max_pwm))
  {
    throw std::runtime_error("esc pwm range invalid");
  }
}

// Very small YAML-ish line parser helpers
static inline bool parse_kv_scalar(const std::string & line, std::string & key, std::string & value)
{
  auto s = strip_inline_comment(line);
  auto pos = s.find(':');
  if (pos == std::string::npos) {return false;}
  key = trim(s.substr(0, pos));
  value = trim(s.substr(pos + 1));
  return !key.empty();
}

static inline int to_int(const std::string & v)
{
  try {
    return std::stoi(v);
  } catch (...) {
    throw std::runtime_error("invalid integer for: " + v);
  }
}
static inline double to_double(const std::string & v)
{
  try {
    return std::stod(v);
  } catch (...) {
    throw std::runtime_error("invalid float for: " + v);
  }
}
static inline bool to_bool(const std::string & v)
{
  if (ieq(v, "true") || v == "1") {return true;}
  if (ieq(v, "false") || v == "0") {return false;}
  throw std::runtime_error("invalid bool for: " + v);
}
static inline std::string unquote(std::string v)
{
  if (!v.empty() && (v.front() == '"' || v.front() == '\'')) {
    if (v.size() >= 2 && v.back() == v.front()) {v = v.substr(1, v.size() - 2);}
  }
  return v;
}

static void parse_inline_map(
  const std::string & expr, std::vector<std::pair<std::string,
  std::string>> & out)
{
  // expect format: { a: 1, b: x }
  auto s = trim(expr);
  if (s.empty() || s.front() != '{' || s.back() != '}') {return;}
  s = s.substr(1, s.size() - 2);
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, ',')) {
    std::string k, v;
    if (parse_kv_scalar(item, k, v)) {
      out.emplace_back(trim(k), trim(v));
    }
  }
}

Config parse_from_yaml_string(const std::string & yaml)
{
  Config cfg;
  EscConfig * active_esc = nullptr;

  enum class Sect { NONE, ROOT, WHEELS, SERVOS_POS, SERVOS_CONT, ESCS, FEATURES };
  Sect sect = Sect::NONE;

  std::stringstream in(yaml);
  std::string line;
  while (std::getline(in, line)) {
    std::string raw = line;
    line = strip_inline_comment(line);
    line = trim(line);
    if (line.empty()) {continue;}

    // Section headers
    if (line == "mecabridge_hardware:") {sect = Sect::ROOT; active_esc = nullptr; continue;}
    if (line == "wheels:") {sect = Sect::WHEELS; active_esc = nullptr; continue;}
    if (line == "servos:") {sect = Sect::SERVOS_POS; active_esc = nullptr; continue;}
    if (line == "positional:") {sect = Sect::SERVOS_POS; active_esc = nullptr; continue;}
    if (line == "continuous:") {sect = Sect::SERVOS_CONT; active_esc = nullptr; continue;}
    if (line == "escs:") {sect = Sect::ESCS; active_esc = nullptr; continue;}
    if (line == "features:") {sect = Sect::FEATURES; active_esc = nullptr; continue;}

    // Accept nested keys with two-space indent; our simple parser ignores indent and relies on last seen sect.
    std::string key, value;
    if (!parse_kv_scalar(line, key, value)) {continue;}

    // Remove surrounding quotes for strings
    value = unquote(value);

    switch (sect) {
      case Sect::ROOT:
        if (key == "serial_port") {cfg.serial_port = value;} else if (key == "baud_rate") {
          cfg.baud_rate = to_int(value);
        } else if (key == "state_publish_rate_hz") {
          cfg.state_publish_rate_hz = to_int(value);
        } else if (key == "wheel_radius") {
          cfg.wheel_radius = to_double(value);
        } else if (key == "wheel_separation_x") {
          cfg.wheel_separation_x = to_double(value);
        } else if (key == "wheel_separation_y") {
          cfg.wheel_separation_y = to_double(value);
        } else if (key == "encoder_ticks_per_rev") {
          cfg.encoder_ticks_per_rev = to_int(value);
        } else if (key == "wheels") { /* ignore, handled by Sect::WHEELS */} else if (key ==
          "servos") { /* switch sect in following lines */} else if (key == "escs") { /* switch sect */
        } else if (key == "features") { /* switch sect */}
        break;

      case Sect::WHEELS:
        if (key == "front_left" || key == "front_right" || key == "rear_left" ||
          key == "rear_right")
        {
          std::vector<std::pair<std::string, std::string>> kv;
          parse_inline_map(value, kv);
          WheelDef * tgt = nullptr;
          if (key == "front_left") {tgt = &cfg.wheels.front_left;} else if (key == "front_right") {
            tgt = &cfg.wheels.front_right;
          } else if (key == "rear_left") {
            tgt = &cfg.wheels.rear_left;
          } else if (key == "rear_right") {tgt = &cfg.wheels.rear_right;}
          if (tgt) {
            for (auto & p : kv) {
              if (p.first == "encoder_index") {
                tgt->encoder_index = to_int(p.second);
              } else if (p.first == "joint_name") {tgt->joint_name = unquote(p.second);}
            }
          }
        }
        break;

      case Sect::SERVOS_POS:
        if (key == "joint_name") {
          cfg.servos.positional.joint_name = value;
        } else if (key == "min_rad") {
          cfg.servos.positional.min_rad = to_double(value);
        } else if (key == "max_rad") {cfg.servos.positional.max_rad = to_double(value);}
        // Allow switching into continuous via next header
        break;

      case Sect::SERVOS_CONT:
        if (key == "joint_name") {
          cfg.servos.continuous.joint_name = value;
        } else if (key == "max_velocity_rad_s") {
          cfg.servos.continuous.max_velocity_rad_s = to_double(value);
        }
        break;

      case Sect::ESCS:
        if (key == "left" || key == "right") {
          active_esc = (key == "left") ? &cfg.escs.left : &cfg.escs.right;
          if (!value.empty()) {
            std::vector<std::pair<std::string, std::string>> kv;
            parse_inline_map(value, kv);
            for (auto & p : kv) {
              if (p.first == "joint_name") {
                active_esc->joint_name = unquote(p.second);
              } else if (p.first == "esc_min_pwm") {
                active_esc->esc_min_pwm = to_int(p.second);
              } else if (p.first == "esc_max_pwm") {
                active_esc->esc_max_pwm = to_int(p.second);
              } else if (p.first == "esc_deadband") {
                active_esc->esc_deadband = to_int(p.second);
              }
            }
          }
        } else if (active_esc) {
          if (key == "joint_name") {
            active_esc->joint_name = value;
          } else if (key == "esc_min_pwm") {
            active_esc->esc_min_pwm = to_int(value);
          } else if (key == "esc_max_pwm") {
            active_esc->esc_max_pwm = to_int(value);
          } else if (key == "esc_deadband") {
            active_esc->esc_deadband = to_int(value);
          }
        }
        break;

      case Sect::FEATURES:
        if (key == "enable_servos") {
          cfg.features.enable_servos = to_bool(value);
        } else if (key == "enable_escs") {
          cfg.features.enable_escs = to_bool(value);
        } else if (key == "enable_diagnostics") {
          cfg.features.enable_diagnostics = to_bool(value);
        }
        break;

      default: break;
    }
  }

  // Validate at end
  cfg.validate();
  return cfg;
}

Config parse_from_yaml_file(const std::string & path)
{
  std::ifstream in(path);
  if (!in) {throw std::runtime_error("failed to open YAML: " + path);}
  std::stringstream ss; ss << in.rdbuf();
  return parse_from_yaml_string(ss.str());
}

} // namespace config
} // namespace mecabridge

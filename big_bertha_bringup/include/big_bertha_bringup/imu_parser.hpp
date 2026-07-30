// Copyright 2026 Jjateen Gundesha
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BIG_BERTHA_BRINGUP__IMU_PARSER_HPP_
#define BIG_BERTHA_BRINGUP__IMU_PARSER_HPP_

#include <string>
#include <utility>

namespace big_bertha_bringup
{

struct ImuData
{
  double ax{0.0}, ay{0.0}, az{0.0};
  double gx{0.0}, gy{0.0}, gz{0.0};
  double mx{0.0}, my{0.0}, mz{0.0};
  bool mag_ok{false};
};

inline bool parse_imu_json(const std::string & line, ImuData & out)
{
  auto find_val = [&](const std::string & key) -> std::pair<bool, double> {
    auto pos = line.find("\"" + key + "\"");
    if (pos == std::string::npos) {
      return { false, 0.0 }
    }
    auto colon = line.find(':', pos);
    if (colon == std::string::npos) {
      return { false, 0.0 }
    }
    auto start = colon + 1;
    while (start < line.size() && (line[start] == ' ' || line[start] == '\t')) {
      ++start;
    }
    auto end = start;
    while (end < line.size() && line[end] != ',' && line[end] != '}' && line[end] != ']') {
      ++end;
    }
    try {
      return {true, std::stod(line.substr(start, end - start))};
    } catch (...) {
      return {false, 0.0};
    }
  };

  auto r = find_val("ax");
  if (!r.first) {
    return false;
  }
  out.ax = r.second;
  r = find_val("ay");
  if (!r.first) {
    return false;
  }
  out.ay = r.second;
  r = find_val("az");
  if (!r.first) {
    return false;
  }
  out.az = r.second;
  r = find_val("gx");
  if (!r.first) {
    return false;
  }
  out.gx = r.second;
  r = find_val("gy");
  if (!r.first) {
    return false;
  }
  out.gy = r.second;
  r = find_val("gz");
  if (!r.first) {
    return false;
  }
  out.gz = r.second;

  out.mx = out.my = out.mz = 0.0;
  out.mag_ok = false;
  r = find_val("mx");
  if (r.first) {
    out.mx = r.second;
    out.mag_ok = true;
  }
  r = find_val("my");
  if (r.first) {
    out.my = r.second;
    out.mag_ok = true;
  }
  r = find_val("mz");
  if (r.first) {
    out.mz = r.second;
    out.mag_ok = true;
  }
  return true;
}

}  // namespace big_bertha_bringup

#endif  // BIG_BERTHA_BRINGUP__IMU_PARSER_HPP_

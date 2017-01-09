// Copyright (C) 2017  I. Bogoslavskyi, C. Stachniss, University of Bonn

// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.

// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
// more details.

// You should have received a copy of the GNU General Public License along
// with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef SRC_UTILS_TIMER_H_
#define SRC_UTILS_TIMER_H_

#include <chrono>

namespace depth_clustering {

namespace time_utils {

using MicroSecs = std::chrono::microseconds;
using MilliSecs = std::chrono::milliseconds;
using Clock = std::chrono::high_resolution_clock;
using TimePoint = std::chrono::time_point<Clock>;
using std::chrono::duration_cast;

/**
 * @brief      A small utility time measurement class
 */
class Timer {
 public:
  enum class Units { Micro, Milli };
  void start() { _start = Clock::now(); }
  uint64_t measure(const Units& units = Units::Micro) {
    auto end = Clock::now();
    uint64_t res = 0;
    switch (units) {
      case Units::Micro:
        res = duration_cast<MicroSecs>(end - _start).count();
        break;
      case Units::Milli:
        res = duration_cast<MilliSecs>(end - _start).count();
        break;
    }
    _start = Clock::now();
    return res;
  }

 private:
  TimePoint _start = Clock::now();
};

}  // namespace time_utils

}  // namespace depth_clustering

#endif  // SRC_UTILS_TIMER_H_

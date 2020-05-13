// Copyright (C) 2020  I. Bogoslavskyi, C. Stachniss
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

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

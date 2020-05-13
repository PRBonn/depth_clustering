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

#include "communication/identifiable.h"
#include <string>

namespace depth_clustering {

std::string Identifiable::DemangleName(const char* tname) {
  int status;
  std::string demangled_name_str = "type_not_processed";
  char* demangled_name = abi::__cxa_demangle(tname, NULL, NULL, &status);
  if (status == 0) {
    demangled_name_str = demangled_name;
    std::free(demangled_name);
  }
  size_t start_pos = 0;
  while (true) {
    size_t separator_place = demangled_name_str.find_first_of("::");
    if (separator_place == std::string::npos) {
      break;
    }
    demangled_name_str.erase(start_pos, separator_place + 2);
    start_pos = demangled_name_str.find_first_of("<") - 1;
  }
  return demangled_name_str;
}

int Identifiable::UniqueIdCounter = 0;

}  // namespace depth_clustering

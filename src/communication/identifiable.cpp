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

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

#ifndef SRC_COMMUNICATION_IDENTIFIABLE_H_
#define SRC_COMMUNICATION_IDENTIFIABLE_H_

#include <cxxabi.h>
#include <string>
#include <cstdlib>

namespace depth_clustering {

/**
 * @brief      Class for identifiable.
 */
class Identifiable {
 public:
  Identifiable() : _kId(++UniqueIdCounter) {}
  virtual ~Identifiable() {}

  /**
   * @brief      Gets current object id.
   *
   * @return     id of the object.
   */
  inline int id() const { return _kId; }

  /**
   * @brief      Guesses class name from typeid.
   *
   * @return     Class name as string.
   */
  virtual std::string guess_class_name() const {
    return DemangleName(typeid(*this).name());
  }
  /**
   * @brief      Gets the current identifier counter.
   *
   * @return     The current identifier counter.
   */
  static int get_current_id_counter() { return UniqueIdCounter; }

 protected:
  static std::string DemangleName(const char* tname);

  static int UniqueIdCounter;
  const int _kId;
};

}  // namespace depth_clustering

#endif  // SRC_COMMUNICATION_IDENTIFIABLE_H_

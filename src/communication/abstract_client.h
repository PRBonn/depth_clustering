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

#ifndef SRC_COMMUNICATION_ABSTRACT_CLIENT_H_
#define SRC_COMMUNICATION_ABSTRACT_CLIENT_H_

#include "communication/identifiable.h"

namespace depth_clustering {

/**
 * @brief      Class for abstract client.
 *
 * @tparam     ObjType  Object type that the client cares about.
 */
template <class ObjType>
class AbstractClient : public virtual Identifiable {
 public:
  AbstractClient() : Identifiable() {}
  virtual ~AbstractClient() {}
  virtual void OnNewObjectReceived(const ObjType& object,
                                   const int sender_id) = 0;
};

}  // namespace depth_clustering

#endif  // SRC_COMMUNICATION_ABSTRACT_CLIENT_H_

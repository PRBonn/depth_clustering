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

#ifndef SRC_UTILS_FOLDER_READER_H_
#define SRC_UTILS_FOLDER_READER_H_

#include <boost/algorithm/string/predicate.hpp>
#include <boost/range/iterator_range_core.hpp>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>

#include <stdio.h>
#include <string>
#include <vector>
#include <algorithm>

namespace depth_clustering {

int num_from_string(const std::string& query_str);

bool numeric_string_compare(const std::string& s1, const std::string& s2);

/**
 * @brief      Reads a folder and can sort the inputs. Not too efficient.
 */
class FolderReader {
 public:
  enum class Order { SORTED, UNDEFINED };

  explicit FolderReader(const std::string& folder_path,
                        const std::string& ending_with,
                        const Order order = Order::UNDEFINED);

  explicit FolderReader(const std::string& folder_path,
                        const std::string& starting_with,
                        const std::string& ending_with,
                        const Order order = Order::UNDEFINED);

  std::string GetNextFilePath();

  const std::vector<std::string>& GetAllFilePaths() const { return _all_paths; }

  virtual ~FolderReader() {}

 protected:
  std::vector<std::string> _all_paths;
  size_t _path_counter;
};

}  // namespace depth_clustering

#endif  // SRC_UTILS_FOLDER_READER_H_

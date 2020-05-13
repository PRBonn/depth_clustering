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

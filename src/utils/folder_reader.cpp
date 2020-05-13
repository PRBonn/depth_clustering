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

#include "./folder_reader.h"

#include <algorithm>
#include <string>
#include <vector>

namespace depth_clustering {

namespace fs = boost::filesystem;
using std::string;
using std::vector;
using boost::algorithm::ends_with;
using boost::algorithm::starts_with;

int num_from_string(const std::string& query_str) {
  if (query_str.empty()) {
    return 0;
  }
  const char* pattern = "\\d+";
  boost::regex re(pattern);
  boost::match_results<std::string::const_iterator> what;
  auto start = query_str.begin();
  auto end = query_str.end();
  int found_num = 0;
  while (boost::regex_search(start, end, what, re)) {
    start = what[0].second;
    found_num = std::stoi(what[0].str());
  }
  return found_num;
}

bool numeric_string_compare(const std::string& s1, const std::string& s2) {
  return num_from_string(s1) < num_from_string(s2);
}

FolderReader::FolderReader(const string& folder_path, const string& ending_with,
                           const Order order)
    : FolderReader(folder_path, "", ending_with, order) {}

FolderReader::FolderReader(const string& folder_path,
                           const string& starting_with,
                           const string& ending_with, const Order order)
    : _path_counter(0) {
  fs::path folder(folder_path);
  if (!fs::exists(folder)) {
    fprintf(stderr, "ERROR: no such folder: %s\n", folder_path.c_str());
    return;
  }
  if (fs::is_directory(folder)) {
    fprintf(stderr, "INFO: Getting file paths from folder: %s\n",
            folder_path.c_str());
    auto range = boost::iterator_range<fs::directory_iterator>(
        fs::directory_iterator(folder), fs::directory_iterator());
    for (auto& entry : range) {
      auto filename = entry.path().filename().string();
      bool is_of_correct_type = starts_with(filename, starting_with) &&
                                ends_with(filename, ending_with);
      if (is_of_correct_type) {
        _all_paths.push_back(entry.path().string());
      }
    }
    if (order == Order::SORTED) {
      std::sort(_all_paths.begin(), _all_paths.end(), numeric_string_compare);
    }
    fprintf(stderr, "INFO: There are %lu '%s' files in the folder.\n",
            _all_paths.size(), ending_with.c_str());
  }
}

string FolderReader::GetNextFilePath() {
  if (_path_counter < _all_paths.size()) {
    return _all_paths[_path_counter++];
  }
  fprintf(stderr, "There are no more paths stored.\n");
  return "";
}

}  // namespace depth_clustering

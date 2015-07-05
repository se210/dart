/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DART_COMMON_PATHMANAGER_H_
#define DART_COMMON_PATHMANAGER_H_

#include <list>
#include <map>
#include <string>

namespace dart {
namespace common {

class PathManager
{
public:
  /// Constructor
  explicit PathManager(const std::string& name = "PathManager");

  /// Destructor
  ~PathManager() = default;

  /// Set name
  void setName(const std::string& name);

  /// Get name
  const std::string& getName() const;

  /// Add a path to search paths
  bool addSearchPath(const std::string& path);

  /// Add a path to search paths in given category
  bool addSearchPathInCategory(
      const std::string& category, const std::string& path);

  /// Remove a path from the search paths
  void removeSearchPath(const std::string& path);

  /// Remove a path from the search paths in given category
  void removeSearchPathInCategory(
      const std::string& category, const std::string& path);

  /// Remove all the search paths
  void removeAllSearchPaths();

  /// Remove all the search paths in given category
  void removeAllSearchPathsInCategory(const std::string& category);

  /// Get the list of search paths
  const std::list<std::string>& getSearchPaths() const;

  /// Get the list of search paths in given category
  const std::list<std::string>& getSearchPathsInCategory(
      const std::string& category) const;

  /// Get full path given formatted fileName.
  ///
  /// This function parses fileName with following pattern:
  ///   "[CATEGORY_NAME://]PATH_TO_DIRECTORY_OR_FILE"
  /// Example:
  ///   "data://skel/cubes.skel"
  ///
  /// Once fileName is parsed, it searches PATH_TO_DIRECTORY_OR_FILE in the
  /// search paths in specified category and then returns the full path if it
  /// actually exists in the system, or empty string otherwise.
  ///
  /// Note that CATEGORY_NAME is optional so it can be omitted. If category is
  /// not specified then this function searches the path in the uncategorized
  /// search paths that were given by addSearchPath().
  /// Example:
  ///   "skel/cubes.skel"
  ///
  /// For path without category, if fileName is absolute path itself and it
  /// actually exists in the system, then return fileName.
  /// Example:
  ///   "/home/user_name/my_skel_dir/cubes.skel"
  std::string getFullFilePath(
      const std::string& fileName, bool searchLocalPath = false) const;

protected:
  using SearchPaths = std::list<std::string>;

  /// Split category and file name
  std::pair<std::string, std::string> splitCategoryAndFileName(
      const std::string& fileName) const;

  /// Name
  std::string mName;

  /// Search paths that is not categorized
  std::list<std::string> mSearchPaths;

  /// first: category name | second: search paths according to the category
  std::map<std::string, SearchPaths> mCategoryMap;

  /// Delimiter to distinguish category and file name
  const std::string mDelimiter;

  /// Emplty search path list for performance issue
  const std::list<std::string> mEmptySearchPaths;

private:

};

}  // namespace common
}  // namespace dart

#endif  // DART_COMMON_PATHMANAGER_H_

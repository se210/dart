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

#include "dart/common/PathManager.h"

#include <algorithm>
#include <iostream>
#include <boost/filesystem.hpp>

#include "dart/config.h"
#include "dart/common/Console.h"

namespace dart {
namespace common {

//==============================================================================
PathManager::PathManager(const std::string& name)
  : mName(name), mDelimiter("://")
{
}

//==============================================================================
void PathManager::setName(const std::string& name)
{
  mName = name;
}

//==============================================================================
const std::string& PathManager::getName() const
{
  return mName;
}

//==============================================================================
bool addPathToSerchPaths(
    const std::string& pathManagerName,
    const std::string& functionName,
    const std::string& path,
    std::list<std::string>& searchPaths)
{
  assert(!path.empty());

  const bool hasPath
      = (std::find(searchPaths.begin(), searchPaths.end(), path)
         != searchPaths.end());
  if (hasPath)
  {
    dtmsg << "[PathManager::" << functionName<< "] PathManager ["
          << pathManagerName << "] already contains path [" << path << "].\n";
    return false;
  }

  searchPaths.push_back(path);

  return true;
}

//==============================================================================
bool PathManager::addSearchPath(const std::string& path)
{
  if (path.empty())
  {
    dtwarn << "[PathManager::addSearchPath] Empty path is invalid.\n";
    return false;
  }

  return addPathToSerchPaths(mName, "addSearchPath", path, mSearchPaths);
}

//==============================================================================
bool PathManager::addSearchPathInCategory(
    const std::string& category, const std::string& path)
{
  if (category.empty())
  {
    dtwarn << "[PathManager::addSearchPathWithCategory] Empty category name is "
           << "invalid. "
           << "Please use addSearchPath() instead if you want to add path ["
           << path << "] into uncategorized search paths.\n";
    return false;
  }

  if (path.empty())
  {
    dtwarn << "[PathManager::addSearchPathWithCategory] "
           << "Empty path is invalid.\n";
    return false;
  }

  auto categoryItr = mCategoryMap.find(category);
  const bool hasCategory = (categoryItr != mCategoryMap.end());
  if (hasCategory)
  {
    SearchPaths& searchPaths = categoryItr->second;
    return addPathToSerchPaths(
          mName, "addSearchPathInCategory", path, searchPaths);
  }
  else
  {
    mCategoryMap[category] = {path};
  }

  return true;
}

//==============================================================================
void PathManager::removeSearchPath(const std::string& path)
{
  mSearchPaths.remove(path);
}

//==============================================================================
void PathManager::removeSearchPathInCategory(
    const std::string& category, const std::string& path)
{
  const auto& categoryItr = mCategoryMap.find(category);
  bool const hasCategory = (categoryItr != mCategoryMap.end());

  if (hasCategory)
  {
    auto& searchPaths = categoryItr->second;
    searchPaths.remove(path);
  }
}

//==============================================================================
void PathManager::removeAllSearchPaths()
{
  mSearchPaths.clear();
}

//==============================================================================
void PathManager::removeAllSearchPathsInCategory(const std::string& category)
{
  const auto& categoryItr = mCategoryMap.find(category);
  bool const hasCategory = (categoryItr != mCategoryMap.end());

  if (hasCategory)
  {
    auto& searchPaths = categoryItr->second;
    searchPaths.clear();
  }
}

//==============================================================================
const std::list<std::string>& PathManager::getSearchPaths() const
{
  return mSearchPaths;
}

//==============================================================================
const std::list<std::string>& PathManager::getSearchPathsInCategory(
    const std::string& category) const
{
  const auto& categoryItr = mCategoryMap.find(category);
  bool const hasCategory = (categoryItr != mCategoryMap.end());

  if (hasCategory)
    return categoryItr->second;

  return mEmptySearchPaths;
}

//==============================================================================
std::string searchFullPath(
    const std::list<std::string> searchPaths, const std::string fileName)
{
  for (const auto& searchPath : searchPaths)
  {
    boost::filesystem::path fullPath
        = boost::filesystem::path(searchPath) / fileName;

    if (boost::filesystem::exists(fullPath))
      return fullPath.string();
  }

  return std::string();
}

//==============================================================================
std::string PathManager::getFullFilePath(
    const std::string& fileName, bool searchLocalPath) const
{
  if (fileName.empty())
    return fileName;

  auto categoryAndFileName = splitCategoryAndFileName(fileName);
  std::string& category = categoryAndFileName.first;
  std::string& fileNameWithoutCategory = categoryAndFileName.second;

  std::string fullPath;

  if (category.empty())
  {
    boost::filesystem::path path(fileNameWithoutCategory);

    if (boost::filesystem::exists(path))
      fullPath = path.string();
    else
      fullPath = searchFullPath(mSearchPaths, fileNameWithoutCategory);
  }
  else
  {
    const auto& categoryItr = mCategoryMap.find(category);
    bool const hasCategory = (categoryItr != mCategoryMap.end());
    if (hasCategory)
    {
      const auto& searchPaths = categoryItr->second;
      fullPath = searchFullPath(searchPaths, fileNameWithoutCategory);
    }
  }

  if (fullPath.empty() && searchLocalPath)
  {
    boost::filesystem::path path = boost::filesystem::current_path() / fileName;

    if (boost::filesystem::exists(path))
      fullPath = path.string();
  }

  if (fullPath.empty())
  {
    dtwarn << "[PathManager::getFullPath] File or path does not exist ["
           << fileName << "].\n";
  }

  return fullPath;
}

//==============================================================================
std::pair<std::string, std::string> PathManager::splitCategoryAndFileName(
    const std::string& fileName) const
{
  std::size_t index = fileName.find(mDelimiter);

  if (std::string::npos == index)
    return std::make_pair(std::string(), fileName);

  const std::string category = fileName.substr(0, index);

  const std::size_t delimLen = mDelimiter.length();
  std::string pureFileName = fileName.substr(
      index + delimLen, fileName.length() - index - delimLen);

  if (pureFileName.find(mDelimiter) != std::string::npos)
  {
    dtwarn << "[PathManager::splitCategoryAndFileName] File name [" << fileName
           << "] has multiple categories. DART is accepting the first category "
           << "and ignoring others.\n";
    index = pureFileName.find_last_of(mDelimiter);
    pureFileName = pureFileName.substr(
        pureFileName.length() - index + delimLen, index - delimLen);
  }

  return std::make_pair(category, pureFileName);
}

}  // namespace common
}  // namespace dart

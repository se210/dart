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

#include <iostream>
#include <gtest/gtest.h>
#include "TestHelpers.h"

#include "dart/dart.h"

using namespace dart;
using namespace common;
using namespace math;
using namespace dynamics;
using namespace simulation;
using namespace utils;

//==============================================================================
TEST(PathManager, Basic)
{
  PathManager pathManager("TestPathManager");
  EXPECT_EQ(pathManager.getName(), "TestPathManager");

  bool result;
  result = pathManager.addSearchPath(DART_DATA_PATH);
  EXPECT_TRUE(result);
  result = pathManager.addSearchPath(DART_DATA_PATH);
  EXPECT_FALSE(result);

  result = pathManager.addSearchPathInCategory("data", DART_DATA_PATH);
  EXPECT_TRUE(result);
  result = pathManager.addSearchPathInCategory("data", DART_DATA_PATH);
  EXPECT_FALSE(result);

  const auto& searchPathDART = pathManager.getSearchPathsInCategory("data");
  const auto& searchPathNone = pathManager.getSearchPathsInCategory("none");
  EXPECT_EQ(searchPathDART.size(), 1);
  EXPECT_EQ(searchPathNone.size(), 0);

  std::string path1
      = pathManager.getFullFilePath("data://skel/cubes.skel");
  std::string path2
      = pathManager.getFullFilePath(DART_DATA_PATH"skel/cubes.skel");
  std::string path3
      = pathManager.getFullFilePath("skel/cubes.skel");
  std::string path4
      = pathManager.getFullFilePath("://skel/cubes.skel");
  std::string path5
      = pathManager.getFullFilePath("data://data://data://skel/cubes.skel");

  EXPECT_TRUE(!path1.empty());
  EXPECT_TRUE(!path2.empty());
  EXPECT_TRUE(!path3.empty());
  EXPECT_TRUE(!path4.empty());
  EXPECT_TRUE(!path5.empty());
  EXPECT_EQ(path1, path2);
  EXPECT_EQ(path2, path3);
  EXPECT_EQ(path3, path4);
  EXPECT_EQ(path4, path5);
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

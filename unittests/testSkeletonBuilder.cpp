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
using namespace math;
using namespace dynamics;
using namespace simulation;
using namespace utils;

//==============================================================================
TEST(SkeletonBuilder, AddRemove)
{
  SkeletonBuilder builder;
  bool result;

  // -- BodyNode

  std::shared_ptr<BodyNode::Properties> bodyProp1(new BodyNode::Properties());
  bodyProp1->mName = "Link 1";
  result = builder.addBodyNode<BodyNode>(bodyProp1);
  EXPECT_TRUE(result);

  result = builder.addBodyNode<BodyNode>("Link 2");
  EXPECT_TRUE(result);

  std::shared_ptr<SoftBodyNode::Properties> bodyProp3(
        new SoftBodyNode::Properties());
  bodyProp3->mName = "Link 3";
  result = builder.addBodyNode<SoftBodyNode>(bodyProp3);
  EXPECT_TRUE(result);

  result = builder.addBodyNode<SoftBodyNode>("Link 4");
  EXPECT_TRUE(result);

  // Attempt to add a BodyNode with repeated name -- should return false
  result = builder.addBodyNode<BodyNode>("Link 4");
  EXPECT_FALSE(result);
  EXPECT_EQ(builder.getNumBodyNodes(), 4);

  // Attempt to remove a BodyNode with unregistered name -- should return false
  result = builder.removeBodyNode("Link 5");
  EXPECT_FALSE(result);
  EXPECT_EQ(builder.getNumBodyNodes(), 4);

  result = builder.removeBodyNode("Link 4");
  EXPECT_TRUE(result);
  EXPECT_EQ(builder.getNumBodyNodes(), 3);

  result = builder.addBodyNode<BodyNode>("Link 4");
  EXPECT_TRUE(result);
  EXPECT_EQ(builder.getNumBodyNodes(), 4);

  // -- Joint

  result = builder.addJoint<FreeJoint>("Joint 1", "Link 1", "Link 2", true);
  EXPECT_TRUE(result);

  // Attempt to add a joint with repeated name -- succeed. The repeated joint
  // name will be renamed to be unique when it's created by and added to
  // Skeleton.
  result = builder.addJoint<FreeJoint>("Joint 2", "Link 2", "Link 3", true);
  EXPECT_TRUE(result);

  // Attempt to add a joint with repeated name -- succeed. The repeated joint
  // name will be renamed to be unique when it's created by and added to
  // Skeleton.
  result = builder.addJoint<FreeJoint>("Joint 2", "Link 3", "Link 4", true);
  EXPECT_TRUE(result);

  // Attempt to add a joint with missing parent name -- should return false
  result = builder.addJoint<FreeJoint>("Joint 3", "Link 3", "Link 4", true);
  EXPECT_FALSE(result);

  // Attempt to add a joint with missing parent name -- should return false
  result = builder.addJoint<FreeJoint>("Joint 3", "Link 0", "Link 2", true);
  EXPECT_FALSE(result);

  // -- Build
  SkeletonPtr skeleton = builder.build();
  EXPECT_TRUE(nullptr != skeleton);

  EXPECT_EQ(skeleton->getNumBodyNodes(), 4);
  EXPECT_EQ(skeleton->getNumJoints(), 4);
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

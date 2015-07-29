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

#include "dart/dart.h"

using namespace dart;
using namespace math;
using namespace collision;
using namespace dynamics;
using namespace simulation;
using namespace utils;

//==============================================================================
//TEST(CollisionDetector, Basic)
//{
//  BoxShape box1(Eigen::Vector3d(1.0, 1.0, 1.0));
//  BoxShape box2(Eigen::Vector3d(1.0, 1.0, 1.0));

//  SphereShape sphere1(0.5);
//  SphereShape sphere2(0.5);

//  Eigen::Isometry3d tf1(Eigen::Isometry3d::Identity());
//  tf1.translation() << 0, 0, 1;
//  Eigen::Isometry3d tf2(Eigen::Isometry3d::Identity());
//  tf2.translation() << 0, 0, 0;

//  CollisionOptions options;
//  CollisionResult result;

//  NarrowPhaseAlgorithm::collide(&box1, tf1, &box2, tf2, options, result);
//  EXPECT_EQ(result.getNumContacts(), 4);

//  NarrowPhaseAlgorithm::collide(&box1, tf1, &sphere2, tf2, options, result);
//  EXPECT_EQ(result.getNumContacts(), 0);
//}

//==============================================================================
TEST(CollisionDetector, BoxBox)
{
  CollisionOptions options;
  CollisionResult result;

  BoxShape box1(Eigen::Vector3d(1.0, 1.0, 1.0));
  BoxShape box2(Eigen::Vector3d(1.0, 1.0, 1.0));

  box1.setSize(Eigen::Vector3d(1.0, 1.0, 1.0));
  box2.setSize(Eigen::Vector3d(1.0, 1.0, 1.0));

  Eigen::Isometry3d tf1(Eigen::Isometry3d::Identity());
  Eigen::Isometry3d tf2(Eigen::Isometry3d::Identity());

  tf1.translation() << 0, 0, 2;
  tf2.translation() << 0, 0, 0;

  NarrowPhaseAlgorithm::collide(&box1, tf1, &box2, tf2, options, result);
  EXPECT_EQ(result.getNumContacts(), 0);

  tf1.translation() << 0, 0, 1;
  tf2.translation() << 0, 0, 0;

  //
  //    +---+             ^
  //    |   |   Box1      |  normal direction (into box1)
  //    +---+            ---
  //  +-------+
  //  |       | Box2
  //  |       |
  //  +-------+
  //
  NarrowPhaseAlgorithm::collide(&box1, tf1, &box2, tf2, options, result);
  EXPECT_EQ(result.getNumContacts(), 4);
  for (size_t i = 0; i < result.getNumContacts(); ++i)
  {
    Contact contact = result.getContact(i);

    EXPECT_EQ(contact.normal, Eigen::Vector3d::UnitZ());
    EXPECT_EQ(contact.point[2], 0.5);
    EXPECT_EQ(contact.penetrationDepth, 0.0);
  }
}

//==============================================================================
TEST(CollisionDetector, ConvexConvex)
{
//  CollisionOptions options;
//  CollisionResult result;

//  MeshShape convex1;
//  MeshShape convex2;

////  convex1.setSize(Eigen::Vector3d(1.0, 1.0, 1.0));
////  convex2.setSize(Eigen::Vector3d(1.0, 1.0, 1.0));

//  Eigen::Isometry3d tf1(Eigen::Isometry3d::Identity());
//  Eigen::Isometry3d tf2(Eigen::Isometry3d::Identity());

//  tf1.translation() << 0, 0, 2;
//  tf2.translation() << 0, 0, 0;

//  collide(&convex1, tf1, &convex2, tf2, options, result);
//  EXPECT_EQ(result.getNumContacts(), 0);

//  tf1.translation() << 0, 0, 1;
//  tf2.translation() << 0, 0, 0;

//  //
//  collide(&convex1, tf1, &convex2, tf2, options, result);
////  EXPECT_EQ(result.getNumContacts(), 4);
////  for (size_t i = 0; i < result.getNumContacts(); ++i)
////  {
////    Contact contact = result.getContact(i);

////    EXPECT_EQ(contact.normal, Eigen::Vector3d::UnitZ());
////    EXPECT_EQ(contact.point[2], 0.5);
////    EXPECT_EQ(contact.penetrationDepth, 0.0);
////  }
}

////==============================================================================
//TEST(CollisionDetector, SphereSphere)
//{
//  CollisionOptions options;
//  CollisionResult result;

//  SphereShape sphere1(0.5);
//  SphereShape sphere2(0.5);

//  Eigen::Isometry3d tf1(Eigen::Isometry3d::Identity());
//  Eigen::Isometry3d tf2(Eigen::Isometry3d::Identity());

//  tf1.translation() << 0, 0, 0.5;
//  tf2.translation() << 0, 0, 0;

//  NarrowPhaseAlgorithm::collide(&sphere1, tf1, &sphere2, tf2, options, result);
//  EXPECT_EQ(result.getNumContacts(), 0);

//  tf1.translation() << 0, 0, 0.99;
//  tf2.translation() << 0, 0, 0;

//  //
//  NarrowPhaseAlgorithm::collide(&sphere1, tf1, &sphere2, tf2, options, result);
////  EXPECT_EQ(result.getNumContacts(), 4);
////  for (size_t i = 0; i < result.getNumContacts(); ++i)
////  {
////    Contact contact = result.getContact(i);

////    EXPECT_EQ(contact.normal, Eigen::Vector3d::UnitZ());
////    EXPECT_EQ(contact.point[2], 0.5);
////    EXPECT_EQ(contact.penetrationDepth, 0.0);
////  }
//}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


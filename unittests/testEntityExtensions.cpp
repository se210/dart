/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
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

#include <gtest/gtest.h>

#include "dart/dynamics/SimpleFrame.h"
#include "dart/dynamics/SpatialMotion.h"
#include "dart/dynamics/SpatialForce.h"
#include "dart/dynamics/FreeVector.h"
#include "dart/dynamics/Point.h"

#include "TestHelpers.h"

using namespace dart;
using namespace dynamics;

std::vector<SimpleFrame*> getFrames()
{
  std::vector<SimpleFrame*> frames;
  static SimpleFrame A(Frame::World(), "A", randomTransform());
  frames.push_back(&A);

  static SimpleFrame B(&A, "B", randomTransform());
  frames.push_back(&B);

  static SimpleFrame C(&B, "C", randomTransform());
  frames.push_back(&C);

  static SimpleFrame D(Frame::World(), "D", randomTransform());
  frames.push_back(&D);

  static SimpleFrame E(&D, "E", randomTransform());
  frames.push_back(&E);

  return frames;
}

TEST(EntityExtensions, FreeVector)
{
#ifndef NDEBUG // Debug mode
  size_t iterations = 1;
#else
  size_t iterations = 100;
#endif

  std::vector<SimpleFrame*> frames = getFrames();

  for(size_t i=0; i<iterations; ++i)
  {
    // Test construction
    for(size_t j=0; j<frames.size(); ++j)
    {
      Frame* parent = frames[j];
      FreeVector v(randomVector<3>(), parent, "v");
      for(size_t k=0; k<frames.size(); ++k)
      {
        Frame* F = frames[k];
        Eigen::Vector3d check = parent->getTransform(F).linear()*v;
        EXPECT_TRUE( equals(v.wrt(F), check) );
      }
    }

    // Test raw assignment
    for(size_t j=0; j<frames.size(); ++j)
    {
      Frame* parent = frames[j];
      FreeVector v;
      v.setParentFrame(parent);
      Eigen::Vector3d vrand = randomVector<3>();
      v = vrand;
      for(size_t k=0; k<frames.size(); ++k)
      {
        Frame* F = frames[k];
        Eigen::Vector3d check = parent->getTransform(F).linear()*vrand;
        EXPECT_TRUE( equals( v.wrt(F), check) );
      }
    }

    // Test component-wise assignment
    for(size_t j=0; j<frames.size(); ++j)
    {
      Frame* parent = frames[j];
      FreeVector v;
      v.setParentFrame(parent);
      Eigen::Vector3d vrand = randomVector<3>();
      for(size_t c=0; c<3; ++c)
        v[c] = vrand[c];
      for(size_t k=0; k<frames.size(); ++k)
      {
        Frame* F = frames[k];
        Eigen::Vector3d check = parent->getTransform(F).linear()*vrand;
        EXPECT_TRUE( equals( v.wrt(F), check) );
      }
    }

    // Test assignment
    for(size_t j=0; j<frames.size(); ++j)
    {
      Frame* J = frames[j];
      FreeVector vj;
      vj.setParentFrame(J);
      for(size_t k=0; k<frames.size(); ++k)
      {
        Frame* K = frames[k];
        FreeVector vk(randomVector<3>(), K);
        vj = vk;
        EXPECT_TRUE( vj.getParentFrame() == J );
        EXPECT_TRUE( equals(vj.wrtWorld(), vk.wrtWorld()) );
        for(size_t n=0; n<frames.size(); ++n)
        {
          Frame* N = frames[n];
          EXPECT_TRUE( equals( vj.wrt(N), vk.wrt(N) ) );
        }
      }
    }
  }
}

int main(int argc, char* argv[])
{
  srand(11424); // Seed with an arbitrary fixed integer
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

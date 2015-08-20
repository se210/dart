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

#include <gtest/gtest.h>
#include "dart/dart.h"
#include "TestHelpers.h"

using namespace dart;

class Controller
{
public:
  Controller(dynamics::SkeletonPtr _skel, double _t)
  {
    mSkel = _skel;
    mLeftHeel = _skel->getBodyNode("h_heel_left");

    mLeftFoot[0] = _skel->getDof("j_heel_left_1")->getIndexInSkeleton();
    mLeftFoot[1] = _skel->getDof("j_toe_left")->getIndexInSkeleton();

    mRightFoot[0] = _skel->getDof("j_heel_right_1")->getIndexInSkeleton();
    mRightFoot[1] = _skel->getDof("j_toe_right")->getIndexInSkeleton();

    mTimestep = _t;
    mFrame = 0;
    int nDof = mSkel->getNumDofs();
    mKp = Eigen::MatrixXd::Identity(nDof, nDof);
    mKd = Eigen::MatrixXd::Identity(nDof, nDof);

    mTorques.resize(nDof);
    mTorques.setZero();

    mDesiredDofs = mSkel->getPositions();

    // using SPD results in simple Kp coefficients
    for (int i = 0; i < 6; i++)
    {
      mKp(i, i) = 0.0;
      mKd(i, i) = 0.0;
    }

    for (int i = 6; i < nDof; i++)
      mKp(i, i) = 400.0;
    for (int i = 6; i < nDof; i++)
      mKd(i, i) = 40.0;

    mPreOffset = 0.0;
  }

  ~Controller() = default;

  Eigen::VectorXd getTorques() { return mTorques; }
  double getTorque(int _index) { return mTorques[_index]; }
  void setDesiredDof(int _index, double _val) { mDesiredDofs[_index] = _val; }

  void computeTorques()
  {
    Eigen::VectorXd _dof = mSkel->getPositions();
    Eigen::VectorXd _dofVel = mSkel->getVelocities();
    Eigen::VectorXd constrForces = mSkel->getConstraintForces();

    // SPD tracking
    //size_t nDof = mSkel->getNumDofs();
    Eigen::MatrixXd invM = (mSkel->getMassMatrix() + mKd * mTimestep).inverse();
    Eigen::VectorXd p = -mKp * (_dof + _dofVel * mTimestep - mDesiredDofs);
    Eigen::VectorXd d = -mKd * _dofVel;
    Eigen::VectorXd qddot =
        invM * (-mSkel->getCoriolisAndGravityForces() + p + d + constrForces);

    mTorques = p + d - mKd * qddot * mTimestep;

    // ankle strategy for sagital plane
    Eigen::Vector3d com = mSkel->getCOM();
    Eigen::Vector3d cop = mLeftHeel->getTransform()
                          * Eigen::Vector3d(0.05, 0, 0);

    double offset = com[0] - cop[0];
    if (offset < 0.1 && offset > 0.0)
    {
      double k1 = 200.0;
      double k2 = 100.0;
      double kd = 10.0;
      mTorques[mLeftFoot[0]]  += -k1 * offset + kd * (mPreOffset - offset);
      mTorques[mLeftFoot[1]]  += -k2 * offset + kd * (mPreOffset - offset);
      mTorques[mRightFoot[0]] += -k1 * offset + kd * (mPreOffset - offset);
      mTorques[mRightFoot[1]] += -k2 * offset + kd * (mPreOffset - offset);
      mPreOffset = offset;
    }
    else if (offset > -0.2 && offset < -0.05)
    {
      double k1 = 2000.0;
      double k2 = 100.0;
      double kd = 100.0;
      mTorques[mLeftFoot[0]]  += -k1 * offset + kd * (mPreOffset - offset);
      mTorques[mLeftFoot[1]]  += -k2 * offset + kd * (mPreOffset - offset);
      mTorques[mRightFoot[0]] += -k1 * offset + kd * (mPreOffset - offset);
      mTorques[mRightFoot[1]] += -k2 * offset + kd * (mPreOffset - offset);
      mPreOffset = offset;
    }

    // Just to make sure no illegal torque is used
    for (int i = 0; i < 6; i++)
    {
      mTorques[i] = 0.0;
    }
    mFrame++;
  }

  dynamics::MetaSkeletonPtr getSkel() { return mSkel; }
  Eigen::VectorXd getDesiredDofs() { return mDesiredDofs; }
  Eigen::MatrixXd getKp() { return mKp; }
  Eigen::MatrixXd getKd() { return mKd; }

protected:
  dynamics::MetaSkeletonPtr mSkel;
  dynamics::BodyNodePtr mLeftHeel;
  Eigen::VectorXd mTorques;
  Eigen::VectorXd mDesiredDofs;
  Eigen::MatrixXd mKp;
  Eigen::MatrixXd mKd;
  size_t mLeftFoot[2];
  size_t mRightFoot[2];
  int mFrame;
  double mTimestep;
  double mPreOffset;

  /// \brief SPD utilizes the current info about contact forces
};

TEST(FiniteDifference, BipedRoot)
{
  simulation::WorldPtr world
      = utils::SkelParser::readWorld(DART_DATA_PATH"skel/fullbody1.skel");
  EXPECT_TRUE(world != nullptr);

  Eigen::Vector3d gravity(0.0, -9.81, 0.0);
  world->setGravity(gravity);

  dynamics::SkeletonPtr biped = world->getSkeleton("fullbody1");
  dynamics::BodyNodePtr root = biped->getRootBodyNode();

  biped->getDof("j_pelvis_rot_y")->setPosition( -0.20);
  biped->getDof("j_thigh_left_z")->setPosition(  0.15);
  biped->getDof("j_shin_left")->setPosition(    -0.40);
  biped->getDof("j_heel_left_1")->setPosition(   0.25);
  biped->getDof("j_thigh_right_z")->setPosition( 0.15);
  biped->getDof("j_shin_right")->setPosition(   -0.40);
  biped->getDof("j_heel_right_1")->setPosition(  0.25);
  biped->getDof("j_abdomen_2")->setPosition(     0.00);

  Eigen::VectorXd testPose1;
  Eigen::VectorXd testPose2;
  Eigen::VectorXd testVelocity;

  // create controller
  Controller* controller = new Controller(biped, world->getTimeStep());

  // Run 1000 steps
  const std::size_t numFrames = 1000;
  for (std::size_t i = 0; i < numFrames; ++i)
  {
    controller->computeTorques();
    controller->getSkel()->setForces(controller->getTorques());
    world->step();

    testPose1 = biped->getPositions();
  }

  // Run one more step
  controller->computeTorques();
  controller->getSkel()->setForces(controller->getTorques());
  world->step();

  testPose2 = biped->getPositions();
  testVelocity = biped->getVelocities();

  // Compare two spatial velocities of root's velocities: one is computed from
  // forward simulation and the other one is computed using finite difference.
  std::cout.precision(15);
  const double h = biped->getTimeStep();

  biped->setPositions(testPose1);
  Isometry3d T1 = root->getTransform();
  biped->setPositions(testPose2);
  Isometry3d T2 = root->getTransform();
  Vector3d omega = T2.linear() * dart::math::logMap(
    T1.linear().transpose() * T2.linear()) / h;
  Vector6d V;
  V.head(3) = omega;
  V.tail(3) = (T2.translation()- T1.translation()) / h;
  std::cout << "Spatial velocity from finite diff:" << std::endl << V
            << std::endl << std::endl;

  biped->setVelocities(testVelocity);
  Vector6d Vp;
  Vp.head(3) = root->getAngularVelocity(Frame::World(), Frame::World());
  Vp.tail(3) = root->getLinearVelocity();
  std::cout << "Spatial velocity from data:" << std::endl << Vp
            << std::endl << std::endl;

  std::cout << "Differences:" << std::endl << V - Vp << std::endl << std::endl;
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

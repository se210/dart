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

using namespace Eigen;

using namespace dart;
using namespace math;
using namespace constraint;
using namespace dynamics;
using namespace simulation;

//==============================================================================
class RigidBody
{
public:
  enum class ReferenceFrame
  {
    WORLD,
    BODY
  };

  enum class RotationMatrixType
  {
    ROTATION,
    LINEAR
  };

  void setRotationMatrixType(RotationMatrixType type)
  {
    mRotationMatrixType = type;
  }

  RotationMatrixType getRotationMatrixType() const
  {
    return mRotationMatrixType;
  }

  virtual void setMass(double mass) = 0;
  virtual double getMass() const = 0;

  virtual void setInertia(const Matrix3d& I) = 0;
  virtual const Matrix3d getInertia() const = 0;
  virtual const Matrix3d getInertia(
      const ReferenceFrame& frame) const = 0;

  void setRotation(const Matrix3d& R)
  {
    mT.linear() = R;
  }

  const Matrix3d getRotation() const
  {
    if (RotationMatrixType::ROTATION == mRotationMatrixType)
      return mT.rotation();
    else
      return mT.linear();
  }

  void setTranslation(const Vector3d& p)
  {
    mT.translation() = p;
  }

  const Vector3d getTranslation() const
  {
    return mT.translation();
  }

  virtual void setLinearVelocity(
      const Vector3d& v,
      const ReferenceFrame& frame = ReferenceFrame::WORLD) = 0;

  virtual const Vector3d getLinearVelocity(
      const ReferenceFrame& frame = ReferenceFrame::WORLD) const = 0;

  virtual void setAngularVelocity(
      const Vector3d& w,
      const ReferenceFrame& frame = ReferenceFrame::WORLD) = 0;

  virtual const Vector3d getAngularVelocity(
      const ReferenceFrame& frame = ReferenceFrame::WORLD) const = 0;

  virtual void setLinearAcceleration(
      const Vector3d& dv,
      const ReferenceFrame& frame = ReferenceFrame::WORLD) = 0;

  virtual const Vector3d getLinearAcceleration(
      const ReferenceFrame& frame = ReferenceFrame::WORLD) const = 0;

  virtual void setAngularAcceleration(
      const Vector3d& dw,
      const ReferenceFrame& frame = ReferenceFrame::WORLD) = 0;

  virtual const Vector3d getAngularAcceleration(
      const ReferenceFrame& frame = ReferenceFrame::WORLD) const = 0;

  virtual void setLinearForce(
      const Vector3d& linearForce,
      const ReferenceFrame& frame = ReferenceFrame::WORLD) = 0;
  virtual void setLinearForceToZero() = 0;
  virtual const Vector3d getLinearForce(
      const ReferenceFrame& frame = ReferenceFrame::WORLD) const = 0;

  virtual void setAngularForce(
      const Vector3d& angularForce,
      const ReferenceFrame& frame = ReferenceFrame::WORLD) = 0;
  virtual void setAngularForceToZero() = 0;
  virtual const Vector3d getAngularForce(
      const ReferenceFrame& frame = ReferenceFrame::WORLD) const = 0;

  void setTimeStep(double timeStep)
  {
    mTimeStep = timeStep;
  }

  double getTimeStep() const
  {
    return mTimeStep;
  }

  void step()
  {
    computeForwardDynamics();

    // Semi-implicit Euler method
    integrateVelocity();
    integrateTransform();

    setLinearForceToZero();
    setAngularForceToZero();
  }

  virtual void computeForwardDynamics() = 0;

  virtual void integrateVelocity() = 0;
  virtual void integrateTransform() = 0;

  virtual const Vector3d getLinearMomentum() const = 0;
  virtual const Vector3d getAngularMomentum() const = 0;

protected:
  explicit RigidBody(RotationMatrixType rotationMatrixType)
    : mRotationMatrixType(rotationMatrixType),
      mT(Isometry3d::Identity()),
      mTimeStep(0.001) {}
  virtual ~RigidBody() = default;

  RotationMatrixType mRotationMatrixType;
  Isometry3d mT;
  double mTimeStep;
};

//==============================================================================
class ClassicalRigidBody : public RigidBody
{
public:
  ClassicalRigidBody(RotationMatrixType type)
    : RigidBody(type),
      mMass(1.0),
      mInertia(Matrix3d::Identity()),
      mLinearVelocity(Vector3d::Zero()),
      mAngularVelocity(Vector3d::Zero()),
      mLinearAcceleration(Vector3d::Zero()),
      mAngularAcceleration(Vector3d::Zero())
  {
  }

  ~ClassicalRigidBody() = default;

  void setMass(double mass) override
  {
    if (mass <= 0.0)
    {
      std::cerr << "Negative or zero mass is not allowed. Ignoring '"
                << mass << "'.\n";
      return;
    }

    mMass = mass;
  }

  double getMass() const override
  {
    return mMass;
  }

  void setInertia(const Matrix3d& I) override
  {
    mInertia = I;
  }

  const Matrix3d getInertia() const override
  {
    return mInertia;
  }

  const Matrix3d getInertia(const ReferenceFrame& frame) const override
  {
    if (ReferenceFrame::BODY == frame)
      return mInertia;

    const Matrix3d R = getRotation();

    return R * mInertia * R.transpose();
  }

  void setLinearVelocity(
      const Vector3d& v,
      const ReferenceFrame& frame = ReferenceFrame::WORLD) override
  {
    if (ReferenceFrame::WORLD == frame)
      mLinearVelocity = v;
    else
      mLinearVelocity = getRotation() * v;
  }

  const Vector3d getLinearVelocity(
      const ReferenceFrame& frame = ReferenceFrame::WORLD) const override
  {
    if (ReferenceFrame::WORLD == frame)
      return mLinearVelocity;
    else
      return getRotation().transpose() * mLinearVelocity;
  }

  void setAngularVelocity(
      const Vector3d& w,
      const ReferenceFrame& frame = ReferenceFrame::WORLD) override
  {
    if (ReferenceFrame::WORLD == frame)
      mAngularVelocity = w;
    else
      mAngularVelocity = getRotation() * w;
  }

  const Vector3d getAngularVelocity(
      const ReferenceFrame& frame = ReferenceFrame::WORLD) const override
  {
    if (ReferenceFrame::WORLD == frame)
      return mAngularVelocity;
    else
      return getRotation().transpose() * mAngularVelocity;
  }

  void setLinearAcceleration(
      const Vector3d& dv,
      const ReferenceFrame& frame = ReferenceFrame::WORLD) override
  {
    if (ReferenceFrame::WORLD == frame)
      mLinearAcceleration = dv;
    else
      mLinearAcceleration = getRotation() * dv;
  }

  const Vector3d getLinearAcceleration(
      const ReferenceFrame& frame = ReferenceFrame::WORLD) const override
  {
    if (ReferenceFrame::WORLD == frame)
      return mLinearAcceleration;
    else
      return getRotation().transpose() * mLinearAcceleration;
  }

  void setAngularAcceleration(
      const Vector3d& dw,
      const ReferenceFrame& frame = ReferenceFrame::WORLD) override
  {
    if (ReferenceFrame::WORLD == frame)
      mAngularAcceleration = dw;
    else
      mAngularAcceleration = getRotation() * dw;
  }

  const Vector3d getAngularAcceleration(
      const ReferenceFrame& frame = ReferenceFrame::WORLD) const override
  {
    if (ReferenceFrame::WORLD == frame)
      return mAngularAcceleration;
    else
      return getRotation().transpose() * mAngularAcceleration;
  }

  void setLinearForce(
      const Vector3d& linearForce,
      const ReferenceFrame& frame = ReferenceFrame::WORLD) override
  {
    if (ReferenceFrame::WORLD == frame)
      mLinearForce = linearForce;
    else
      mLinearForce = getRotation() * linearForce;
  }

  void setLinearForceToZero() override
  {
    mLinearForce.setZero();
  }

  const Vector3d getLinearForce(
      const ReferenceFrame& frame = ReferenceFrame::WORLD) const override
  {
    if (ReferenceFrame::WORLD == frame)
      return mLinearForce;
    else
      return getRotation().transpose() * mLinearForce;
  }

  void setAngularForce(
      const Vector3d& angularForce,
      const ReferenceFrame& frame = ReferenceFrame::WORLD) override
  {
    if (ReferenceFrame::WORLD == frame)
      mAngularForce = angularForce;
    else
      mAngularForce = getRotation() * angularForce;
  }

  void setAngularForceToZero() override
  {
    mAngularForce.setZero();
  }

  const Vector3d getAngularForce(
      const ReferenceFrame& frame = ReferenceFrame::WORLD) const override
  {
    if (ReferenceFrame::WORLD == frame)
      return mAngularForce;
    else
      return getRotation().transpose() * mAngularForce;
  }

  void computeForwardDynamics() override
  {
    // Linear acceleration
    mLinearAcceleration = mLinearForce / mMass;

    // Angular acceleration
    const Matrix3d I    = getInertia(ReferenceFrame::WORLD);
    const Matrix3d invI = I.inverse();
    const Vector3d w    = mAngularVelocity;
    const Vector3d tau  = mAngularForce;
    mAngularAcceleration = invI * (tau - w.cross(I * w));
  }

  void integrateVelocity() override
  {
    mLinearVelocity  = mLinearVelocity  + mTimeStep * mLinearAcceleration;
    mAngularVelocity = mAngularVelocity + mTimeStep * mAngularAcceleration;
  }

  void integrateTransform() override
  {
    const double dt = mTimeStep;

    mT.linear()      = math::expMapRot(dt * mAngularVelocity) * getRotation();
    mT.translation() = mT.translation() + dt * mLinearVelocity;
  }

  const Vector3d getLinearMomentum() const override
  {
    return getMass() * mLinearVelocity;
  }

  const Vector3d getAngularMomentum() const override
  {
    return getInertia(ReferenceFrame::WORLD) * mAngularVelocity;
  }

private:

  double mMass;
  Matrix3d mInertia;

  Vector3d mLinearVelocity;
  Vector3d mAngularVelocity;

  Vector3d mLinearAcceleration;
  Vector3d mAngularAcceleration;

  Vector3d mLinearForce;
  Vector3d mAngularForce;
};

//==============================================================================
class SpatialRigidBody : public RigidBody
{
public:
  enum class CoordinateType
  {
    SE3,
    SO3_AND_R3
  };

  SpatialRigidBody(RotationMatrixType type,
                   CoordinateType coordType = CoordinateType::SE3)
    : RigidBody(type),
      mCoordinateType(coordType),
      mInertia(dynamics::Inertia(1.0)),
      mSpatialVelocity(Vector6d::Zero()),
      mSpatialAcceleration(Vector6d::Zero()),
      mSpatialForce(Vector6d::Zero())
  {
  }

  ~SpatialRigidBody() = default;

  void setMass(double mass) override
  {
    if (mass <= 0.0)
    {
      std::cerr << "Negative or zero mass is not allowed. Ignoring '"
                << mass << "'.\n";
      return;
    }

    mInertia.setMass(mass);
  }

  double getMass() const override
  {
    return mInertia.getMass();
  }

  void setInertia(const Matrix3d& I) override
  {
    mInertia.setMoment(I);
  }

  const Matrix3d getInertia() const override
  {
    return mInertia.getMoment();
  }

  const Matrix3d getInertia(const ReferenceFrame& frame) const override
  {
    if (ReferenceFrame::BODY == frame)
      return getInertia();

    const Matrix3d R = getRotation();

    return R * getInertia() * R.transpose();
  }

  void setLinearVelocity(
      const Vector3d& v,
      const ReferenceFrame& frame = ReferenceFrame::WORLD) override
  {
    if (ReferenceFrame::WORLD == frame)
      mSpatialVelocity.tail<3>() = getRotation().transpose() * v;
    else
      mSpatialVelocity.tail<3>() = v;
  }

  const Vector3d getLinearVelocity(
      const ReferenceFrame& frame = ReferenceFrame::WORLD) const override
  {
    if (ReferenceFrame::WORLD == frame)
      return getRotation() * mSpatialVelocity.tail<3>();
    else
      return mSpatialVelocity.tail<3>();
  }

  void setAngularVelocity(
      const Vector3d& w,
      const ReferenceFrame& frame = ReferenceFrame::WORLD) override
  {
    if (ReferenceFrame::WORLD == frame)
      mSpatialVelocity.head<3>() = getRotation().transpose() * w;
    else
      mSpatialVelocity.head<3>() = w;
  }

  const Vector3d getAngularVelocity(
      const ReferenceFrame& frame = ReferenceFrame::WORLD) const override
  {
    if (ReferenceFrame::WORLD == frame)
      return getRotation() * mSpatialVelocity.head<3>();
    else
      return mSpatialVelocity.head<3>();
  }

  void setLinearAcceleration(
      const Vector3d& a,
      const ReferenceFrame& frame = ReferenceFrame::WORLD) override
  {
    const Vector3d v  = mSpatialVelocity.tail<3>();
    const Vector3d w  = mSpatialVelocity.head<3>();

    if (ReferenceFrame::WORLD == frame)
      mSpatialAcceleration.tail<3>() = getRotation().transpose() * a - w.cross(v);
    else
      mSpatialAcceleration.tail<3>() = a - w.cross(v);
  }

  const Vector3d getLinearAcceleration(
      const ReferenceFrame& frame = ReferenceFrame::WORLD) const override
  {
    const Vector3d dv = mSpatialAcceleration.tail<3>();
    const Vector3d v  = mSpatialVelocity.tail<3>();
    const Vector3d w  = mSpatialVelocity.head<3>();
    const Vector3d a  = dv + w.cross(v);

    if (ReferenceFrame::WORLD == frame)
      return getRotation() * a;
    else
      return a;
  }

  void setAngularAcceleration(
      const Vector3d& dw,
      const ReferenceFrame& frame = ReferenceFrame::WORLD) override
  {
    if (ReferenceFrame::WORLD == frame)
      mSpatialAcceleration.head<3>() = getRotation().transpose() * dw;
    else
      mSpatialAcceleration.head<3>() = dw;
  }

  const Vector3d getAngularAcceleration(
      const ReferenceFrame& frame = ReferenceFrame::WORLD) const override
  {
    if (ReferenceFrame::WORLD == frame)
      return getRotation() * mSpatialAcceleration.head<3>();
    else
      return mSpatialAcceleration.head<3>();
  }

  void setLinearForce(
      const Vector3d& linearForce,
      const ReferenceFrame& frame = ReferenceFrame::WORLD) override
  {
    if (ReferenceFrame::WORLD == frame)
      mSpatialForce.tail<3>() = getRotation().transpose() * linearForce;
    else
      mSpatialForce.tail<3>() = linearForce;
  }

  void setLinearForceToZero() override
  {
    mSpatialForce.tail<3>().setZero();
  }

  const Vector3d getLinearForce(
      const ReferenceFrame& frame = ReferenceFrame::WORLD) const override
  {
    if (ReferenceFrame::WORLD == frame)
      return getRotation() * mSpatialForce.tail<3>();
    else
      return mSpatialForce.tail<3>();
  }

  void setAngularForce(
      const Vector3d& angularForce,
      const ReferenceFrame& frame = ReferenceFrame::WORLD) override
  {
    if (ReferenceFrame::WORLD == frame)
      mSpatialForce.head<3>() = getRotation().transpose() * angularForce;
    else
      mSpatialForce.head<3>() = angularForce;
  }

  void setAngularForceToZero() override
  {
    mSpatialForce.head<3>().setZero();
  }

  const Vector3d getAngularForce(
      const ReferenceFrame& frame = ReferenceFrame::WORLD) const override
  {
    if (ReferenceFrame::WORLD == frame)
      return getRotation() * mSpatialForce.head<3>();
    else
      return mSpatialForce.head<3>();
  }

  void computeForwardDynamics() override
  {
    Matrix6d G = mInertia.getSpatialTensor();
    Vector6d V = mSpatialVelocity;

    mSpatialAcceleration = G.inverse() * math::dad(V, G * V);
  }

  void integrateVelocity() override
  {
    mSpatialVelocity += mTimeStep * mSpatialAcceleration;
  }

  void integrateTransform() override
  {
    if (CoordinateType::SE3 == mCoordinateType)
      integrateTransformSE3();
    else
      integrateTransformSO3AndR3();
  }

  const Vector3d getLinearMomentum() const override
  {
    return getMass() * getLinearVelocity(ReferenceFrame::WORLD);
  }

  const Vector3d getAngularMomentum() const override
  {
    return getRotation()
        * getInertia(ReferenceFrame::BODY)
        * getAngularVelocity(ReferenceFrame::BODY);
  }

private:
  void integrateTransformSE3()
  {
    mT = mT * math::expMap(mTimeStep * mSpatialVelocity);
  }

  void integrateTransformSO3AndR3()
  {
    const double dt = mTimeStep;

    Isometry3d dT = Isometry3d::Identity();

    dT.linear()      = math::expMapRot(dt * mSpatialVelocity.head<3>());
    dT.translation() = dt * mSpatialVelocity.tail<3>();

    mT = mT * dT;
  }

  CoordinateType mCoordinateType;

  dynamics::Inertia mInertia;

  Vector6d mSpatialVelocity;
  Vector6d mSpatialAcceleration;
  Vector6d mSpatialForce;
};

//==============================================================================
void testMomentumConservation(
    RigidBody::RotationMatrixType rotationMatrixType,
    double timeStep,
    size_t numSteps,
    double mass,
    double Ixx,
    double Iyy,
    double Izz,
    const Vector3d& v0,
    const Vector3d& w0,
    const Vector4d& positionTol,
    bool checkRotation,
    const Vector4d& rotationTol,
    const Vector4d& vTol,
    bool checkAngularVelocity,
    const Vector4d& wTol,
    const Vector4d& LTol,
    const Vector4d& HTol)
{
  const double tf = timeStep * numSteps;

  const Vector3d expectedPos = tf * v0;
  const Matrix3d expectedRot = math::expMapRot(tf * w0);

  const double v0mag = v0.norm();
  const double w0mag = w0.norm();

  dynamics::Inertia I(mass, 0, 0, 0, Ixx, Iyy, Izz, 0, 0, 0);
  const Matrix3d I0 = I.getMoment();

  // Compute initial linear/angular momentum
  const Vector3d L0 = mass * v0;
  const Vector3d H0 = I0 * w0;
  const double L0mag = L0.norm();
  const double H0mag = H0.norm();

  // Build single body skeleton
  SkeletonPtr skel1 = Skeleton::create();
  auto pair = skel1->createJointAndBodyNodePair<FreeJoint>(nullptr);
  FreeJoint* joint_a = pair.first;
  BodyNode* body_a = pair.second;
  body_a->setInertia(I);

  // Custom single bodies
  std::unique_ptr<ClassicalRigidBody> body_b(
        new ClassicalRigidBody(rotationMatrixType));
  std::unique_ptr<SpatialRigidBody> body_c(
        new SpatialRigidBody(
          rotationMatrixType, SpatialRigidBody::CoordinateType::SE3));
  std::unique_ptr<SpatialRigidBody> body_d(
        new SpatialRigidBody(
          rotationMatrixType, SpatialRigidBody::CoordinateType::SO3_AND_R3));
  body_b->setMass(mass);
  body_b->setInertia(I0);
  body_c->setMass(mass);
  body_c->setInertia(I0);
  body_d->setMass(mass);
  body_d->setInertia(I0);

  Vector6d V;
  V.head<3>() = w0;
  V.tail<3>() = v0;
  joint_a->setVelocities(V);
  body_b->setLinearVelocity(v0);
  body_b->setAngularVelocity(w0);
  body_c->setLinearVelocity(v0);
  body_c->setAngularVelocity(w0);
  body_d->setLinearVelocity(v0);
  body_d->setAngularVelocity(w0);

  // World
  WorldPtr world(new World);
  world->addSkeleton(skel1);
  world->setGravity(Vector3d::Zero());
  world->setTimeStep(timeStep);
  body_b->setTimeStep(timeStep);
  body_c->setTimeStep(timeStep);
  body_d->setTimeStep(timeStep);

  // Check initial linear/angular velocities
  EXPECT_EQ(v0, body_a->getLinearVelocity());
  EXPECT_EQ(v0, body_b->getLinearVelocity());
  EXPECT_EQ(v0, body_c->getLinearVelocity());
  EXPECT_EQ(v0, body_d->getLinearVelocity());

  EXPECT_EQ(w0, body_a->getAngularVelocity());
  EXPECT_EQ(w0, body_b->getAngularVelocity());
  EXPECT_EQ(w0, body_c->getAngularVelocity());
  EXPECT_EQ(w0, body_d->getAngularVelocity());

  for (size_t i = 0; i < numSteps; ++i)
  {
    world->step();
    body_b->step();
    body_c->step();
    body_d->step();

    const Vector3d v_a = body_a->getLinearVelocity();
    const Vector3d v_b = body_b->getLinearVelocity();
    const Vector3d v_c = body_c->getLinearVelocity();
    const Vector3d v_d = body_d->getLinearVelocity();

    const Vector3d w_a = body_a->getAngularVelocity();
    const Vector3d w_b = body_b->getAngularVelocity();
    const Vector3d w_c = body_c->getAngularVelocity();
    const Vector3d w_d = body_d->getAngularVelocity();

    //------------------
    // Linear Velocity
    //------------------

    double v_error_a = (v_a - v0).norm();
    double v_error_b = (v_b - v0).norm();
    double v_error_c = (v_c - v0).norm();
    double v_error_d = (v_d - v0).norm();

    if (v0mag > 0.0)
    {
      v_error_a /= v0mag;
      v_error_b /= v0mag;
      v_error_c /= v0mag;
      v_error_d /= v0mag;
    }

    EXPECT_LE(v_error_a, vTol[0]);
    EXPECT_LE(v_error_b, vTol[1]);
    EXPECT_LE(v_error_c, vTol[2]);
    EXPECT_LE(v_error_d, vTol[3]);

    //------------------
    // Angular Velocity
    //------------------

    if (checkAngularVelocity)
    {
      double w_error_a = (w_a - w0).norm();
      double w_error_b = (w_b - w0).norm();
      double w_error_c = (w_c - w0).norm();
      double w_error_d = (w_d - w0).norm();

      if (w0mag > 0.0)
      {
        w_error_a /= w0mag;
        w_error_b /= w0mag;
        w_error_c /= w0mag;
        w_error_d /= w0mag;
      }

      EXPECT_LE(w_error_a, wTol[0]);
      EXPECT_LE(w_error_b, wTol[1]);
      EXPECT_LE(w_error_c, wTol[2]);
      EXPECT_LE(w_error_d, wTol[3]);
    }

    //-------------------
    // Linear Momentum
    //-------------------

    Vector3d L_a = mass * body_a->getLinearVelocity();
    Vector3d L_b = body_b->getLinearMomentum();
    Vector3d L_c = body_c->getLinearMomentum();
    Vector3d L_d = body_d->getLinearMomentum();

    double Lerror_a = (L_a - L0).norm();
    double Lerror_b = (L_b - L0).norm();
    double Lerror_c = (L_c - L0).norm();
    double Lerror_d = (L_d - L0).norm();

    if (L0mag > 0.0)
    {
      Lerror_a /= L0mag;
      Lerror_b /= L0mag;
      Lerror_c /= L0mag;
      Lerror_d /= L0mag;
    }

    EXPECT_LE(Lerror_a, LTol[0]);
    EXPECT_LE(Lerror_b, LTol[1]);
    EXPECT_LE(Lerror_c, LTol[2]);
    EXPECT_LE(Lerror_d, LTol[3]);

    //-------------------
    // Angular Momentum
    //-------------------

    Matrix3d R_a = body_a->getTransform().linear();
    Vector3d H_a = R_a * I0 * R_a.transpose() * w_a;
    Vector3d H_b = body_b->getAngularMomentum();
    Vector3d H_c = body_c->getAngularMomentum();
    Vector3d H_d = body_d->getAngularMomentum();

    double Herror_a = (H_a - H0).norm();
    double Herror_b = (H_b - H0).norm();
    double Herror_c = (H_c - H0).norm();
    double Herror_d = (H_d - H0).norm();

    if (H0mag > 0.0)
    {
      Herror_a /= H0mag;
      Herror_b /= H0mag;
      Herror_c /= H0mag;
      Herror_d /= H0mag;
    }

    EXPECT_LE(Herror_a, HTol[0]);
    EXPECT_LE(Herror_b, HTol[1]);
    EXPECT_LE(Herror_c, HTol[2]);
    EXPECT_LE(Herror_d, HTol[3]);
  }

  //--------------
  // Translation
  //--------------

  const Vector3d pos_a = body_a->getTransform().translation();
  const Vector3d pos_b = body_b->getTranslation();
  const Vector3d pos_c = body_c->getTranslation();
  const Vector3d pos_d = body_d->getTranslation();

  const double posError_a = (pos_a - expectedPos).norm();
  const double posError_b = (pos_b - expectedPos).norm();
  const double posError_c = (pos_c - expectedPos).norm();
  const double posError_d = (pos_d - expectedPos).norm();

  EXPECT_LE(posError_a, positionTol[0]);
  EXPECT_LE(posError_b, positionTol[1]);
  EXPECT_LE(posError_c, positionTol[2]);
  EXPECT_LE(posError_d, positionTol[3]);

  //-----------
  // Rotation
  //-----------

  if (checkRotation)
  {
    const Matrix3d R_a = body_a->getTransform().linear();
    const Matrix3d R_b = body_b->getRotation();
    const Matrix3d R_c = body_c->getRotation();
    const Matrix3d R_d = body_d->getRotation();

    const double RError_a = math::logMap(expectedRot.transpose() * R_a).norm();
    const double RError_b = math::logMap(expectedRot.transpose() * R_b).norm();
    const double RError_c = math::logMap(expectedRot.transpose() * R_c).norm();
    const double RError_d = math::logMap(expectedRot.transpose() * R_d).norm();

    EXPECT_LE(RError_a, rotationTol[0]);
    EXPECT_LE(RError_b, rotationTol[1]);
    EXPECT_LE(RError_c, rotationTol[2]);
    EXPECT_LE(RError_d, rotationTol[3]);
  }
}

//==============================================================================
TEST(SingleBodyNode, MomentumConservation)
{
  Vector3d v0 = Vector3d::Zero();
  Vector3d w0 = Vector3d::Zero();

  Vector4d pTol;
  Vector4d rTol;
  Vector4d vTol;
  Vector4d wTol;
  Vector4d LTol;
  Vector4d HTol;

  v0.setZero();
  w0 << 1e-3, 1.5e0, 1.5e-2;
  pTol.setZero();
  rTol.setZero();
  vTol.setZero();
  wTol << 2.576e-15, 0.021e-15, 2.230e-15, 2.23e-15;
  LTol.setZero();
  HTol << 2.8e-15, 4.0e-15, 3.6e-15, 2.3e-15;
  testMomentumConservation(
        RigidBody::RotationMatrixType::ROTATION,
        1e-3, 5000,
        10.0, 1.0, 1.0, 1.0,
        v0, w0, pTol, true, rTol, vTol, true, wTol, LTol, HTol);

  v0.setZero();
  w0 << 1e-3, 1.5e0, 1.5e-2;
  pTol.setZero();
  rTol.setZero();
  vTol.setZero();
  wTol << 0.0033e-13, 0.022e-13, 1.79117e-13, 1.79117e-13;
  LTol.setZero();
  HTol << 0.0054e-13, 3.40616e-13, 1.79117e-13, 1.79117e-13;
  testMomentumConservation(
        RigidBody::RotationMatrixType::LINEAR,
        1e-3, 5000,
        10.0, 1.0, 1.0, 1.0,
        v0, w0, pTol, true, rTol, vTol, true, wTol, LTol, HTol);

  v0 << 1, 2, 3;
  w0 << 1e-3, 1.5e0, 1.5e-2;
  pTol << 5.01678e-2, 4.56632e-13, 4.58239e-2, 5.01678e-2;
  rTol.setZero();
  vTol << 4.73842e-3, 0.0, 4.73842e-3, 4.73842e-3;
  wTol << 0.00585e-13, 0.00223e-13, 1.79117e-13, 1.79117e-13;
  LTol << 4.73842e-3, 0.0, 4.73842e-3, 4.73842e-3;
  HTol << 0.0072e-13, 3.4062e-13, 1.7912e-13, 1.7912e-13;
  testMomentumConservation(
        RigidBody::RotationMatrixType::ROTATION,
        1e-3, 5000,
        10.0, 1.0, 1.0, 1.0,
        v0, w0, pTol, true, rTol, vTol, true, wTol, LTol, HTol);

  v0 << 1, 2, 3;
  w0 << 1e-3, 1.5e0, 1.5e-2;
  pTol << 5.01678e-2, 4.56632e-13, 4.58239e-2, 5.01678e-2;
  rTol.setZero();
  vTol << 4.73841e-3, 0.0, 4.73841e-3, 4.73841e-3;
  wTol << 0.00585e-13, 0.00223e-13, 1.79117e-13, 1.79117e-13;
  LTol << 4.73842e-3, 0.0, 4.73842e-3, 4.73842e-3;
  HTol << 0.0072e-13, 3.4062e-13, 1.7912e-13, 1.7912e-13;
  testMomentumConservation(
        RigidBody::RotationMatrixType::LINEAR,
        1e-3, 5000,
        10.0, 1.0, 1.0, 1.0,
        v0, w0, pTol, true, rTol, vTol, true, wTol, LTol, HTol);

  // Since Ixx > Iyy > Izz,
  // angular velocity with large y component will cause gyroscopic tumbling
  v0.setZero();
  w0 << 1e-3, 1.5e0, 1.5e-2;
  pTol.setZero();
  vTol.setZero();
  LTol.setZero();
  HTol << 8.404769e-4, 8.404769e-4, 8.404769e-4, 8.404769e-4;
  testMomentumConservation(
        RigidBody::RotationMatrixType::ROTATION,
        1e-3, 5000,
        10.0, 0.80833333, 0.68333333, 0.14166667,
        v0, w0, pTol, false, rTol, vTol, false, wTol, LTol, HTol);

  v0.setZero();
  w0 << 1e-3, 1.5e0, 1.5e-2;
  pTol.setZero();
  vTol.setZero();
  LTol.setZero();
  HTol << 8.40478e-4, 8.40478e-4, 8.40478e-4, 8.40478e-4;
  testMomentumConservation(
        RigidBody::RotationMatrixType::LINEAR,
        1e-3, 5000,
        10.0, 0.80833333, 0.68333333, 0.14166667,
        v0, w0, pTol, false, rTol, vTol, false, wTol, LTol, HTol);

  v0 << 1, 2, 3;
  w0 << 1e-3, 1.5e0, 1.5e-2;
  pTol << 5.01133e-2, 4.56632e-13, 4.58682e-2, 5.01133e-2;
  vTol << 5.04139e-3, 0.0, 5.04139e-3, 5.04139e-3;
  LTol << 5.04139e-3, 0.0, 5.04139e-3, 5.04139e-3;
  HTol << 8.404769e-4, 8.404769e-4, 8.404769e-4, 8.404769e-4;
  testMomentumConservation(
        RigidBody::RotationMatrixType::ROTATION,
        1e-3, 5000,
        10.0, 0.80833333, 0.68333333, 0.14166667,
        v0, w0, pTol, false, rTol, vTol, false, wTol, LTol, HTol);
  // Note that world frame representation is much more accurate than body-fixed
  // frame representation in terms of linear integration and linear momentum
  // conservation!

  v0 << 1, 2, 3;
  w0 << 1e-3, 1.5e0, 1.5e-2;
  pTol << 5.01133e-2, 4.56632e-13, 4.58682e-2, 5.01133e-2;
  vTol << 5.04139e-3, 0.0, 5.04139e-3, 5.04139e-3;
  LTol << 5.04139e-3, 0.0, 5.04139e-3, 5.04139e-3;
  HTol << 8.404769e-4, 8.404769e-4, 8.404769e-4, 8.404769e-4;
  testMomentumConservation(
        RigidBody::RotationMatrixType::LINEAR,
        1e-3, 5000,
        10.0, 0.80833333, 0.68333333, 0.14166667,
        v0, w0, pTol, false, rTol, vTol, false, wTol, LTol, HTol);
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

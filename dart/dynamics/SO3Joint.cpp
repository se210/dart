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

#include "dart/dynamics/SO3Joint.h"

#include "dart/common/Console.h"
#include "dart/math/Helpers.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/DegreeOfFreedom.h"

namespace dart {
namespace dynamics {

//==============================================================================
SO3Joint::SO3Joint(const std::string& _name)
  : Joint(_name),
    mCommands(Eigen::Vector3d::Zero()),
    mRotation(Eigen::Matrix3d::Identity()),
    mPositionLowerLimits(Eigen::Vector3d::Constant(-DART_DBL_INF)),
    mPositionUpperLimits(Eigen::Vector3d::Constant(DART_DBL_INF)),
    mPositionDeriv(Eigen::Vector3d::Zero()),
    mVelocities(Eigen::Vector3d::Zero()),
    mVelocityLowerLimits(Eigen::Vector3d::Constant(-DART_DBL_INF)),
    mVelocityUpperLimits(Eigen::Vector3d::Constant(DART_DBL_INF)),
    mVelocityDeriv(Eigen::Vector3d::Zero()),
    mAccelerations(Eigen::Vector3d::Zero()),
    mAccelerationLowerLimits(Eigen::Vector3d::Constant(-DART_DBL_INF)),
    mAccelerationUpperLimits(Eigen::Vector3d::Constant(DART_DBL_INF)),
    mAccelerationDeriv(Eigen::Vector3d::Zero()),
    mForces(Eigen::Vector3d::Zero()),
    mForceLowerLimits(Eigen::Vector3d::Constant(-DART_DBL_INF)),
    mForceUpperLimits(Eigen::Vector3d::Constant(DART_DBL_INF)),
    mForceDeriv(Eigen::Vector3d::Zero()),
    mVelocityChanges(Eigen::Vector3d::Zero()),
    mImpulses(Eigen::Vector3d::Zero()),
    mConstraintImpulses(Eigen::Vector3d::Zero()),
    mSpringStiffness(Eigen::Vector3d::Zero()),
    mRestPositions(Eigen::Vector3d::Zero()),
    mDampingCoefficient(Eigen::Vector3d::Zero()),
    mFrictions(Eigen::Vector3d::Zero()),
    mJacobian(Eigen::Matrix<double, 6, 3>::Zero()),
    mJacobianDeriv(Eigen::Matrix<double, 6, 3>::Zero()),
    mInvProjArtInertia(Eigen::Matrix3d::Zero()),
    mInvProjArtInertiaImplicit(Eigen::Matrix3d::Zero()),
    mTotalForce(Eigen::Vector3d::Zero()),
    mTotalImpulse(Eigen::Vector3d::Zero()),
    mInvM_a(Eigen::Vector3d::Zero()),
    mInvMassMatrixSegment(Eigen::Vector3d::Zero())
{
  mGenCoordType = GeneralizedCoordinateType::SO3_LIE_ALGEBRA;

  for (size_t i = 0; i < mDofs.size(); ++i)
    mDofs[i] = createDofPointer(mName, i);

  updateDegreeOfFreedomNames();
}

//==============================================================================
SO3Joint::~SO3Joint()
{
  for (size_t i = 0; i < mDofs.size(); ++i)
    delete mDofs[i];
}

//==============================================================================
size_t SO3Joint::getDof() const
{
  return getNumDofs();
}

//==============================================================================
size_t SO3Joint::getNumDofs() const
{
  return 3;
}

//==============================================================================
void SO3Joint::setIndexInSkeleton(size_t _index, size_t _indexInSkeleton)
{
  if (_index >= getNumDofs())
  {
    dterr << "[SO3Joint::setIndexInSkeleton] index[" << _index
          << "] out of range" << std::endl;
    return;
  }

  mDofs[_index]->mIndexInSkeleton = _indexInSkeleton;
}

//==============================================================================
size_t SO3Joint::getIndexInSkeleton(size_t _index) const
{
  if (_index >= getNumDofs())
  {
    dterr << "getIndexInSkeleton index[" << _index << "] out of range"
          << std::endl;
    return 0;
  }

  return mDofs[_index]->mIndexInSkeleton;
}

//==============================================================================
Eigen::Isometry3d SO3Joint::convertToTransform(
    const Eigen::Vector3d& _positions, GeneralizedCoordinateType _type)
{
  return Eigen::Isometry3d(convertToRotation(_positions, _type));
}

//==============================================================================
Eigen::Isometry3d SO3Joint::convertToTransform(
    const Eigen::Vector3d& _positions) const
{
  return Eigen::Isometry3d(convertToRotation(_positions));
}

//==============================================================================
Eigen::Matrix3d SO3Joint::convertToRotation(const Eigen::Vector3d& _positions,
                                             GeneralizedCoordinateType _type)
{
  switch (_type)
  {
    case GeneralizedCoordinateType::SO3_LIE_ALGEBRA:
      return math::expMapRot(_positions);
    default:
      dterr << "..." << std::endl;
      return Eigen::Matrix3d::Zero();
      break;
  }
}

//==============================================================================
Eigen::Matrix3d SO3Joint::convertToRotation(const Eigen::Vector3d& _positions)
const
{
  return SO3Joint::convertToRotation(_positions, mGenCoordType);
}

//==============================================================================
Eigen::Vector3d SO3Joint::getFiniteDifferenceStatic(
    const Eigen::Vector3d& _pos1,
    const Eigen::Vector3d& _pos2,
    GeneralizedCoordinateType _type)
{
  using math::expMapRot;
  using math::logMap;

  switch (_type)
  {
    case GeneralizedCoordinateType::SO3_LIE_ALGEBRA:
      return logMap(expMapRot(_pos2).transpose() * expMapRot(_pos1));
    default:
      dterr << "..." << std::endl;
      return Eigen::Vector3d::Zero();
  }
}

//==============================================================================
Eigen::Vector3d SO3Joint::getFiniteDifferenceStatic(
    const Eigen::Vector3d& _pos1, const Eigen::Vector3d& _pos2) const
{
  return SO3Joint::getFiniteDifferenceStatic(_pos1, _pos2, mGenCoordType);
}

//==============================================================================
Eigen::VectorXd SO3Joint::getFiniteDifference(
    const Eigen::VectorXd& _pos1, const Eigen::VectorXd& _pos2) const
{
  return getFiniteDifferenceStatic(_pos1, _pos2);
}

//==============================================================================
DegreeOfFreedom* SO3Joint::getDof(size_t _index)
{
  if (_index < getNumDofs())
    return mDofs[_index];

  return nullptr;
}

//==============================================================================
const DegreeOfFreedom* SO3Joint::getDof(size_t _index) const
{
  if (_index < getNumDofs())
    return mDofs[_index];

  return nullptr;
}

//==============================================================================
void SO3Joint::setCommand(size_t _index, double _command)
{
  if (_index >= getNumDofs())
  {
    dterr << "[SO3Joint::setCommand]: index[" << _index << "] out of range"
          << std::endl;
    return;
  }

  mCommands[_index] = _command;
}

//==============================================================================
double SO3Joint::getCommand(size_t _index) const
{
  if (_index >= getNumDofs())
  {
    dterr << "[SO3Joint::getCommand]: index[" << _index << "] out of range"
          << std::endl;
    return 0.0;
  }

  return mCommands[_index];
}

//==============================================================================
void SO3Joint::setCommands(const Eigen::VectorXd& _commands)
{
  if (static_cast<size_t>(_commands.size()) != getNumDofs())
  {
    dterr << "[SO3Joint::setCommands]: commands's size["
          << _commands.size() << "] is different with the dof [" << getNumDofs()
          << "]" << std::endl;
    return;
  }

  mCommands = _commands;
}

//==============================================================================
Eigen::VectorXd SO3Joint::getCommands() const
{
  return mCommands;
}

//==============================================================================
void SO3Joint::resetCommands()
{
  mCommands.setZero();
}

//==============================================================================
void SO3Joint::setPosition(size_t _index, double _position)
{
  if (_index >= getNumDofs())
  {
    dterr << "setPosition index[" << _index << "] out of range" << std::endl;
    return;
  }

  // TODO: Need verify
  Eigen::Vector3d positions = getPositions();
  positions[_index] = _position;
  setPositionsStatic(positions);
}

//==============================================================================
double SO3Joint::getPosition(size_t _index) const
{
  if (_index >= getNumDofs())
  {
    dterr << "getPosition index[" << _index << "] out of range" << std::endl;
    return 0.0;
  }

  return getPositionsStatic()[_index];
}

//==============================================================================
void SO3Joint::setPositions(const Eigen::VectorXd& _positions)
{
  if (static_cast<size_t>(_positions.size()) != getNumDofs())
  {
    dterr << "setPositions positions's size[" << _positions.size()
          << "] is different with the dof [" << getNumDofs() << "]" << std::endl;
    return;
  }

  setPositionsStatic(_positions);
}

//==============================================================================
Eigen::VectorXd SO3Joint::getPositions() const
{
  return getPositionsStatic();
}

//==============================================================================
void SO3Joint::resetPositions()
{
  setPositionsStatic(Eigen::Vector3d::Zero());
}

//==============================================================================
void SO3Joint::setPositionLowerLimit(size_t _index, double _position)
{
  if (_index >= getNumDofs())
  {
    dterr << "setPositionLowerLimit index[" << _index << "] out of range"
          << std::endl;
    return;
  }

  mPositionLowerLimits[_index] = _position;
}

//==============================================================================
double SO3Joint::getPositionLowerLimit(size_t _index) const
{
  if (_index >= getNumDofs())
  {
    dterr << "getPositionLowerLimit index[" << _index << "] out of range"
          << std::endl;
    return 0.0;
  }

  return mPositionLowerLimits[_index];
}

//==============================================================================
void SO3Joint::setPositionUpperLimit(size_t _index, double _position)
{
  if (_index >= getNumDofs())
  {
    dterr << "setPositionUpperLimit index[" << _index << "] out of range"
          << std::endl;
    return;
  }

  mPositionUpperLimits[_index] = _position;
}

//==============================================================================
double SO3Joint::getPositionUpperLimit(size_t _index) const
{
  if (_index >= getNumDofs())
  {
    dterr << "getPositionUpperLimit index[" << _index << "] out of range"
          << std::endl;
    return 0.0;
  }

  return mPositionUpperLimits[_index];
}

//==============================================================================
void SO3Joint::setVelocity(size_t _index, double _velocity)
{
  if (_index >= getNumDofs())
  {
    dterr << "setVelocity index[" << _index << "] out of range" << std::endl;
    return;
  }

  // TODO: Need verify
  Eigen::Vector3d velocities = getVelocitiesStatic();
  velocities[_index] = _velocity;
  setVelocitiesStatic(velocities);
}

//==============================================================================
double SO3Joint::getVelocity(size_t _index) const
{
  if (_index >= getNumDofs())
  {
    dterr << "getVelocity index[" << _index << "] out of range" << std::endl;
    return 0.0;
  }

  return getVelocitiesStatic()[_index];
}

//==============================================================================
void SO3Joint::setVelocities(const Eigen::VectorXd& _velocities)
{
  if (static_cast<size_t>(_velocities.size()) != getNumDofs())
  {
    dterr << "setVelocities velocities's size[" << _velocities.size()
          << "] is different with the dof [" << getNumDofs() << "]" << std::endl;
    return;
  }

  setVelocitiesStatic(_velocities);
}

//==============================================================================
Eigen::VectorXd SO3Joint::getVelocities() const
{
  return getVelocitiesStatic();
}

//==============================================================================
void SO3Joint::resetVelocities()
{
  setVelocitiesStatic(Eigen::Vector3d::Zero());
}

//==============================================================================
void SO3Joint::setVelocityLowerLimit(size_t _index, double _velocity)
{
  if (_index >= getNumDofs())
  {
    dterr << "setVelocityLowerLimit index[" << _index << "] out of range"
          << std::endl;
    return;
  }

  mVelocityLowerLimits[_index] = _velocity;
}

//==============================================================================
double SO3Joint::getVelocityLowerLimit(size_t _index) const
{
  if (_index >= getNumDofs())
  {
    dterr << "getVelocityLowerLimit index[" << _index << "] out of range"
          << std::endl;
    return 0.0;
  }

  return mVelocityLowerLimits[_index];
}

//==============================================================================
void SO3Joint::setVelocityUpperLimit(size_t _index, double _velocity)
{
  if (_index >= getNumDofs())
  {
    dterr << "setVelocityUpperLimit index[" << _index << "] out of range"
          << std::endl;
    return;
  }

  mVelocityUpperLimits[_index] = _velocity;
}

//==============================================================================
double SO3Joint::getVelocityUpperLimit(size_t _index) const
{
  if (_index >= getNumDofs())
  {
    dterr << "getVelocityUpperLimit index[" << _index << "] out of range"
          << std::endl;
    return 0.0;
  }

  return mVelocityUpperLimits[_index];
}

//==============================================================================
void SO3Joint::setAcceleration(size_t _index, double _acceleration)
{
  if (_index >= getNumDofs())
  {
    dterr << "setAcceleration index[" << _index << "] out of range"
          << std::endl;
    return;
  }

  // TODO: Need verify
  Eigen::Vector3d accelerations = getAccelerationsStatic();
  accelerations[_index] = _acceleration;
  setAccelerationsStatic(accelerations);

#if DART_MAJOR_VERSION == 4
  if (mActuatorType == ACCELERATION)
    mCommands[_index] = getAccelerationsStatic()[_index];
#endif
}

//==============================================================================
double SO3Joint::getAcceleration(size_t _index) const
{
  if (_index >= getNumDofs())
  {
    dterr << "getAcceleration index[" << _index << "] out of range"
          << std::endl;
    return 0.0;
  }

  return getAccelerationsStatic()[_index];
}

//==============================================================================
void SO3Joint::setAccelerations(const Eigen::VectorXd& _accelerations)
{
  if (static_cast<size_t>(_accelerations.size()) != getNumDofs())
  {
    dterr << "setAccelerations accelerations's size[" << _accelerations.size()
          << "] is different with the dof [" << getNumDofs() << "]" << std::endl;
    return;
  }

  setAccelerationsStatic(_accelerations);

#if DART_MAJOR_VERSION == 4
  if (mActuatorType == ACCELERATION)
    mCommands = getAccelerationsStatic();
#endif
}

//==============================================================================
Eigen::VectorXd SO3Joint::getAccelerations() const
{
  return getAccelerationsStatic();
}

//==============================================================================
void SO3Joint::resetAccelerations()
{
  setAccelerationsStatic(Eigen::Vector3d::Zero());
}

//==============================================================================
void SO3Joint::setAccelerationLowerLimit(size_t _index, double _acceleration)
{
  if (_index >= getNumDofs())
  {
    dterr << "setAccelerationLowerLimit index[" << _index << "] out of range"
          << std::endl;
    return;
  }

  mAccelerationLowerLimits[_index] = _acceleration;
}

//==============================================================================
double SO3Joint::getAccelerationLowerLimit(size_t _index) const
{
  if (_index >= getNumDofs())
  {
    dterr << "getAccelerationLowerLimit index[" << _index << "] out of range"
          << std::endl;
    return 0.0;
  }

  return mAccelerationLowerLimits[_index];
}

//==============================================================================
void SO3Joint::setAccelerationUpperLimit(size_t _index, double _acceleration)
{
  if (_index >= getNumDofs())
  {
    dterr << "setAccelerationUpperLimit index[" << _index << "] out of range"
          << std::endl;
    return;
  }

  mAccelerationUpperLimits[_index] = _acceleration;
}

//==============================================================================
double SO3Joint::getAccelerationUpperLimit(size_t _index) const
{
  if (_index >= getNumDofs())
  {
    dterr << "getAccelerationUpperLimit index[" << _index << "] out of range"
          << std::endl;
    return 0.0;
  }

  return mAccelerationUpperLimits[_index];
}

//==============================================================================
void SO3Joint::setPositionsStatic(const Eigen::Vector3d& _positions)
{
  setRotationStatic(convertToRotation(_positions));
}

//==============================================================================
void SO3Joint::setVelocitiesStatic(const Eigen::Vector3d& _velocities)
{
  switch (mGenCoordType)
  {
    case GeneralizedCoordinateType::SO3_LIE_ALGEBRA:
    {
      const Eigen::Matrix3d J = math::expMapJac(getPositionsStatic());
      setInternalVelocitiesStatic(J * _velocities);
      break;
    }
    default:
    {
      dterr << "..." << std::endl;
      break;
    }
  }
}

//==============================================================================
Eigen::Vector3d SO3Joint::getVelocitiesStatic() const
{
  switch (mGenCoordType)
  {
    case GeneralizedCoordinateType::SO3_LIE_ALGEBRA:
    {
      const Eigen::Matrix3d J = math::expMapJac(getPositionsStatic());
      return J.inverse() * getInternalVelocitiesStatic();
      break;
    }
    default:
    {
      dterr << "..." << std::endl;
      return Eigen::Vector3d::Zero();
      break;
    }
  }
}

//==============================================================================
void SO3Joint::setAccelerationsStatic(const Eigen::Vector3d& _accelerations)
{
  switch (mGenCoordType)
  {
    case GeneralizedCoordinateType::SO3_LIE_ALGEBRA:
    {
      const Eigen::Vector3d  q = getPositionsStatic();
      const Eigen::Vector3d dq = getVelocitiesStatic();
      const Eigen::Matrix3d  J = math::expMapJac(q);
      const Eigen::Matrix3d dJ = math::expMapJacDot(q, dq);
      setInternalVelocitiesStatic(dJ * q + J * _accelerations);
      break;
    }
    default:
    {
      dterr << "..." << std::endl;
      break;
    }
  }
}

//==============================================================================
Eigen::Vector3d SO3Joint::getAccelerationsStatic() const
{
  switch (mGenCoordType)
  {
    case GeneralizedCoordinateType::SO3_LIE_ALGEBRA:
    {
      const Eigen::Vector3d  q = getPositionsStatic();
      const Eigen::Vector3d dq = getVelocitiesStatic();
      const Eigen::Vector3d  a = getInternalAccelerationsStatic();
      const Eigen::Matrix3d  J = math::expMapJac(q);
      const Eigen::Matrix3d dJ = math::expMapJacDot(q, dq);
      return J.inverse() * (a - dJ * dq);
      break;
    }
    default:
    {
      dterr << "..." << std::endl;
      return Eigen::Vector3d::Zero();
      break;
    }
  }
}

//==============================================================================
void SO3Joint::setForce(size_t _index, double _force)
{
  if (_index >= getNumDofs())
  {
    dterr << "setForce index[" << _index << "] out of range" << std::endl;
    return;
  }

  mForces[_index] = _force;

#if DART_MAJOR_VERSION == 4
  if (mActuatorType == FORCE)
    mCommands = mForces;
#endif
  // TODO: Remove at DART 5.0.
}

//==============================================================================
double SO3Joint::getForce(size_t _index)
{
  if (_index >= getNumDofs())
  {
    dterr << "getForce index[" << _index << "] out of range" << std::endl;
    return 0.0;
  }

  return mForces[_index];
}

//==============================================================================
void SO3Joint::setForces(const Eigen::VectorXd& _forces)
{
  if (static_cast<size_t>(_forces.size()) != getNumDofs())
  {
    dterr << "setForces forces's size[" << _forces.size()
          << "] is different with the dof [" << getNumDofs() << "]" << std::endl;
    return;
  }

  mForces = _forces;

#if DART_MAJOR_VERSION == 4
  if (mActuatorType == FORCE)
    mCommands = mForces;
#endif
  // TODO: Remove at DART 5.0.
}

//==============================================================================
Eigen::VectorXd SO3Joint::getForces() const
{
  return mForces;
}

//==============================================================================
void SO3Joint::resetForces()
{
  mForces.setZero();

#if DART_MAJOR_VERSION == 4
  if (mActuatorType == FORCE)
    mCommands = mForces;
#endif
}

//==============================================================================
void SO3Joint::setForceLowerLimit(size_t _index, double _force)
{
  if (_index >= getNumDofs())
  {
    dterr << "setForceLowerLimit index[" << _index << "] out of range"
          << std::endl;
    return;
  }

  mForceLowerLimits[_index] = _force;
}

//==============================================================================
double SO3Joint::getForceLowerLimit(size_t _index) const
{
  if (_index >= getNumDofs())
  {
    dterr << "getForceMin index[" << _index << "] out of range" << std::endl;
    return 0.0;
  }

  return mForceLowerLimits[_index];
}

//==============================================================================
void SO3Joint::setForceUpperLimit(size_t _index, double _force)
{
  if (_index >= getNumDofs())
  {
    dterr << "setForceUpperLimit index[" << _index << "] out of range"
          << std::endl;
    return;
  }

  mForceUpperLimits[_index] = _force;
}

//==============================================================================
double SO3Joint::getForceUpperLimit(size_t _index) const
{
  if (_index >= getNumDofs())
  {
    dterr << "getForceUpperLimit index[" << _index << "] out of range"
          << std::endl;
    return 0.0;
  }

  return mForceUpperLimits[_index];
}

//==============================================================================
void SO3Joint::setVelocityChange(size_t _index, double _velocityChange)
{
  if (_index >= getNumDofs())
  {
    dterr << "setVelocityChange index[" << _index << "] out of range"
          << std::endl;
    return;
  }

  mVelocityChanges[_index] = _velocityChange;
}

//==============================================================================
double SO3Joint::getVelocityChange(size_t _index) const
{
  if (_index >= getNumDofs())
  {
    dterr << "getVelocityChange index[" << _index << "] out of range"
          << std::endl;
    return 0.0;
  }

  return mVelocityChanges[_index];
}

//==============================================================================
void SO3Joint::resetVelocityChanges()
{
  mVelocityChanges.setZero();
}

//==============================================================================
void SO3Joint::setConstraintImpulse(size_t _index, double _impulse)
{
  if (_index >= getNumDofs())
  {
    dterr << "setConstraintImpulse index[" << _index << "] out of range"
          << std::endl;
    return;
  }

  mConstraintImpulses[_index] = _impulse;
}

//==============================================================================
double SO3Joint::getConstraintImpulse(size_t _index) const
{
  if (_index >= getNumDofs())
  {
    dterr << "getConstraintImpulse index[" << _index << "] out of range"
          << std::endl;
    return 0.0;
  }

  return mConstraintImpulses[_index];
}

//==============================================================================
void SO3Joint::resetConstraintImpulses()
{
  mConstraintImpulses.setZero();
}

//==============================================================================
void SO3Joint::integratePositions(double _dt)
{
  // TODO(JS):
  mRotation = mRotation * math::expMapRot(getInternalVelocitiesStatic() * _dt);
  setRotationStatic(mRotation);
//  setPositionsStatic(getPositionsStatic() + getVelocityStatic() * _dt);
}

//==============================================================================
void SO3Joint::integrateVelocities(double _dt)
{
  const Eigen::Vector3d& intQ  = getInternalVelocitiesStatic();
  const Eigen::Vector3d& intDQ = getInternalAccelerationsStatic();

  setInternalVelocitiesStatic(intQ + intDQ * _dt);
}

//==============================================================================
void SO3Joint::setSpringStiffness(size_t _index, double _k)
{
  if (_index >= getNumDofs())
  {
    dterr << "[SO3Joint::setSpringStiffness]: index[" << _index
          << "] out of range." << std::endl;
    return;
  }

  assert(_k >= 0.0);

  mSpringStiffness[_index] = _k;
}

//==============================================================================
double SO3Joint::getSpringStiffness(size_t _index) const
{
  if (_index >= getNumDofs())
  {
    dterr << "[SO3Joint::getSpringStiffness]: index[" << _index
          << "] out of range." << std::endl;
    return 0.0;
  }

  return mSpringStiffness[_index];
}

//==============================================================================
void SO3Joint::setRestPosition(size_t _index, double _q0)
{
  if (_index >= getNumDofs())
  {
    dterr << "[SO3Joint::setRestPosition]: index[" << _index
          << "] out of range." << std::endl;
    return;
  }

  if (mPositionLowerLimits[_index] > _q0 || mPositionUpperLimits[_index] < _q0)
  {
    dterr << "Rest position of joint[" << getName() << "], " << _q0
          << ", is out of the limit range["
          << mPositionLowerLimits << ", "
          << mPositionUpperLimits << "] in index[" << _index
          << "].\n";
    return;
  }

  mRestPositions[_index] = _q0;
}

//==============================================================================
double SO3Joint::getRestPosition(size_t _index) const
{
  if (_index >= getNumDofs())
  {
    dterr << "[SO3Joint::getRestPosition]: index[" << _index
          << "] out of range." << std::endl;
    return 0.0;
  }

  return mRestPositions[_index];
}

//==============================================================================
void SO3Joint::setDampingCoefficient(size_t _index, double _d)
{
  if (_index >= getNumDofs())
  {
    dterr << "[SO3Joint::setDampingCoefficient]: index[" << _index
          << "] out of range." << std::endl;
    return;
  }

  assert(_d >= 0.0);

  mDampingCoefficient[_index] = _d;
}

//==============================================================================
double SO3Joint::getDampingCoefficient(size_t _index) const
{
  if (_index >= getNumDofs())
  {
    dterr << "[SO3Joint::getDampingCoefficient]: index[" << _index
          << "] out of range." << std::endl;
    return 0.0;
  }

  return mDampingCoefficient[_index];
}

//==============================================================================
void SO3Joint::setCoulombFriction(size_t _index, double _friction)
{
  if (_index >= getNumDofs())
  {
    dterr << "[SO3Joint::setFriction]: index[" << _index
          << "] out of range." << std::endl;
    return;
  }

  assert(_friction >= 0.0);

  mFrictions[_index] = _friction;
}

//==============================================================================
double SO3Joint::getCoulombFriction(size_t _index) const
{
  if (_index >= getNumDofs())
  {
    dterr << "[SO3Joint::getFriction]: index[" << _index
          << "] out of range." << std::endl;
    return 0.0;
  }

  return mFrictions[_index];
}

//==============================================================================
Eigen::Vector3d SO3Joint::getPositionsStatic() const
{
  return convertToPositions(getRotationStatic());
}

//==============================================================================
double SO3Joint::getPotentialEnergy() const
{
  // Spring energy
  // TODO(JS):
  double pe = 0.0;
//  double pe = 0.5 * mSpringStiffness
//       * (getPositionsStatic() - mRestPosition)
//       * (getPositionsStatic() - mRestPosition);

  return pe;
}

//==============================================================================
void SO3Joint::setRotationStatic(const Eigen::Matrix3d& _rotation)
{
  assert(math::verifyRotation(_rotation));
  mRotation = _rotation;
  notifyPositionUpdate();
}

//==============================================================================
const Eigen::Matrix3d& SO3Joint::getRotationStatic() const
{
  return mRotation;
}

//==============================================================================
void SO3Joint::setInternalVelocitiesStatic(const Eigen::Vector3d& _velocities)
{
  mVelocities = _velocities;
  notifyVelocityUpdate();
}

//==============================================================================
const Eigen::Vector3d& SO3Joint::getInternalVelocitiesStatic() const
{
  return mVelocities;
}

//==============================================================================
void SO3Joint::setInternalAccelerationsStatic(
    const Eigen::Vector3d& _accelerations)
{
  mAccelerations = _accelerations;
  notifyAccelerationUpdate();
}

//==============================================================================
const Eigen::Vector3d& SO3Joint::getInternalAccelerationsStatic() const
{
  return mAccelerations;
}

//==============================================================================
const SO3Joint::JacobianStatic& SO3Joint::getInternalLocalJacobian() const
{
  if (mIsLocalJacobianDirty)
  {
    updateInternalLocalJacobian();
    mIsLocalJacobianDirty = false;
  }

  return mJacobian;
}

//==============================================================================
void SO3Joint::updateDegreeOfFreedomNames()
{
  // TODO(JS): This should be depend on the generalized coordinates type
  if(!mDofs[0]->isNamePreserved())
    mDofs[0]->setName(mName + "_x", false);
  if(!mDofs[1]->isNamePreserved())
    mDofs[1]->setName(mName + "_y", false);
  if(!mDofs[2]->isNamePreserved())
    mDofs[2]->setName(mName + "_z", false);
}

//==============================================================================
void SO3Joint::updateLocalTransform() const
{
  mT = mT_ParentBodyToJoint
       * Eigen::Isometry3d(getRotationStatic())
       * mT_ChildBodyToJoint.inverse();

  assert(math::verifyTransform(mT));
}

//==============================================================================
void SO3Joint::updateInternalLocalJacobian() const
{
  // Jacobian
  JacobianStatic J = JacobianStatic::Zero();
  J.topRows<3>() = Eigen::Matrix3d::Identity();
  mJacobian.col(0) = math::AdT(mT_ChildBodyToJoint, J.col(0));
  mJacobian.col(1) = math::AdT(mT_ChildBodyToJoint, J.col(1));
  mJacobian.col(2) = math::AdT(mT_ChildBodyToJoint, J.col(2));
  assert(!math::isNan(mJacobian));
}

//==============================================================================
void SO3Joint::updateLocalJacobian(bool) const
{
  // update internal local Jacobian
  updateInternalLocalJacobian();
}

//==============================================================================
void SO3Joint::updateLocalJacobianTimeDeriv() const
{
  mJacobianDeriv.setZero();
}

//==============================================================================
void SO3Joint::updateLocalSpatialVelocity() const
{
  mSpatialVelocity = getLocalJacobianStatic() * getVelocitiesStatic();
}

//==============================================================================
void SO3Joint::updateLocalSpatialAcceleration() const
{
  mSpatialAcceleration
      = getLocalPrimaryAcceleration()
        + getLocalJacobianTimeDerivStatic() * getVelocitiesStatic();
}

//==============================================================================
void SO3Joint::updateLocalPrimaryAcceleration() const
{
  mPrimaryAcceleration = getLocalJacobianStatic() * getAccelerationsStatic();
}

//==============================================================================
Eigen::Vector6d SO3Joint::getBodyConstraintWrench() const
{
  assert(mChildBodyNode);
  return mChildBodyNode->getBodyForce() - getLocalJacobianStatic() * mForces;
}

//==============================================================================
const math::Jacobian SO3Joint::getLocalJacobian() const
{
  if(mIsLocalJacobianDirty)
  {
    updateLocalJacobian(false);
    mIsLocalJacobianDirty = false;
  }
  return mJacobian;
}

//==============================================================================
const Eigen::Matrix<double, 6, 3>& SO3Joint::getLocalJacobianStatic() const
{
  if(mIsLocalJacobianDirty)
  {
    updateLocalJacobian(false);
    mIsLocalJacobianDirty = false;
  }
  return mJacobian;
}

//==============================================================================
const math::JacobianDynamic SO3Joint::getLocalJacobianTimeDeriv() const
{
  if(mIsLocalJacobianTimeDerivDirty)
  {
    updateLocalJacobianTimeDeriv();
    mIsLocalJacobianTimeDerivDirty = false;
  }
  return mJacobianDeriv;
}

//==============================================================================
const SO3Joint::JacobianStatic& SO3Joint::getLocalJacobianTimeDerivStatic()
const
{
  if(mIsLocalJacobianTimeDerivDirty)
  {
    updateLocalJacobianTimeDeriv();
    mIsLocalJacobianTimeDerivDirty = false;
  }
  return mJacobianDeriv;
}

//==============================================================================
const Eigen::Matrix3d& SO3Joint::getInvProjArtInertia() const
{
  Joint::updateArticulatedInertia();
  return mInvProjArtInertia;
}

//==============================================================================
const Eigen::Matrix3d& SO3Joint::getInvProjArtInertiaImplicit() const
{
  Joint::updateArticulatedInertia();
  return mInvProjArtInertiaImplicit;
}

//==============================================================================
void SO3Joint::addVelocityTo(Eigen::Vector6d& _vel)
{
  // Add joint velocity to _vel
  _vel.noalias() += getLocalJacobianStatic() * getVelocitiesStatic();

  // Verification
  assert(!math::isNan(_vel));
}

//==============================================================================
void SO3Joint::setPartialAccelerationTo(
    Eigen::Vector6d& _partialAcceleration,
    const Eigen::Vector6d& _childVelocity)
{
  // ad(V, S * dq) + dS * dq
  _partialAcceleration = math::ad(_childVelocity, getLocalJacobianStatic()
                                  * getVelocitiesStatic())
                         + mJacobianDeriv * getVelocitiesStatic();
  // Verification
  assert(!math::isNan(_partialAcceleration));
}


//==============================================================================
void SO3Joint::addAccelerationTo(Eigen::Vector6d& _acc)
{
  //
  _acc += getLocalJacobianStatic() * getAccelerationsStatic();
}

//==============================================================================
void SO3Joint::addVelocityChangeTo(Eigen::Vector6d& _velocityChange)
{
  //
  _velocityChange += getLocalJacobianStatic() * mVelocityChanges;
}

//==============================================================================
void SO3Joint::addChildArtInertiaTo(
    Eigen::Matrix6d& _parentArtInertia, const Eigen::Matrix6d& _childArtInertia)
{
  switch (mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      addChildArtInertiaToDynamic(_parentArtInertia,
                                       _childArtInertia);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      addChildArtInertiaToKinematic(_parentArtInertia,
                                             _childArtInertia);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
void SO3Joint::addChildArtInertiaToDynamic(
    Eigen::Matrix6d& _parentArtInertia, const Eigen::Matrix6d& _childArtInertia)
{
  // Child body's articulated inertia
  Eigen::Matrix<double, 6, 3> AIS = _childArtInertia * getLocalJacobianStatic();
  Eigen::Matrix6d PI = _childArtInertia;
  PI.noalias() -= AIS * mInvProjArtInertia * AIS.transpose();
  assert(!math::isNan(PI));

  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  _parentArtInertia += math::transformInertia(getLocalTransform().inverse(), PI);
}

//==============================================================================
void SO3Joint::addChildArtInertiaToKinematic(
    Eigen::Matrix6d& _parentArtInertia, const Eigen::Matrix6d& _childArtInertia)
{
  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  _parentArtInertia += math::transformInertia(getLocalTransform().inverse(),
                                              _childArtInertia);
}

//==============================================================================
void SO3Joint::addChildArtInertiaImplicitTo(
    Eigen::Matrix6d& _parentArtInertia, const Eigen::Matrix6d& _childArtInertia)
{
  switch (mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      addChildArtInertiaImplicitToDynamic(_parentArtInertia,
                                             _childArtInertia);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      addChildArtInertiaImplicitToKinematic(_parentArtInertia,
                                                   _childArtInertia);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
void SO3Joint::addChildArtInertiaImplicitToDynamic(
    Eigen::Matrix6d& _parentArtInertia, const Eigen::Matrix6d& _childArtInertia)
{
  // Child body's articulated inertia
  Eigen::Matrix<double, 6, 3> AIS = _childArtInertia * getLocalJacobianStatic();
  Eigen::Matrix6d PI = _childArtInertia;
  PI.noalias() -= AIS * mInvProjArtInertiaImplicit * AIS.transpose();
  assert(!math::isNan(PI));

  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  _parentArtInertia += math::transformInertia(getLocalTransform().inverse(), PI);
}

//==============================================================================
void SO3Joint::addChildArtInertiaImplicitToKinematic(
    Eigen::Matrix6d& _parentArtInertia, const Eigen::Matrix6d& _childArtInertia)
{
  // Add child body's articulated inertia to parent body's articulated inertia.
  // Note that mT should be updated.
  _parentArtInertia += math::transformInertia(getLocalTransform().inverse(),
                                              _childArtInertia);
}

//==============================================================================
void SO3Joint::updateInvProjArtInertia(
    const Eigen::Matrix6d& _artInertia)
{
  switch (mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      updateInvProjArtInertiaDynamic(_artInertia);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      updateInvProjArtInertiaKinematic(_artInertia);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
void SO3Joint::updateInvProjArtInertiaDynamic(
    const Eigen::Matrix6d& _artInertia)
{
  // Projected articulated inertia
  const Eigen::Matrix<double, 6, 3>& Jacobian = getLocalJacobianStatic();
  const Eigen::Matrix3d projAI = Jacobian.transpose() * _artInertia * Jacobian;

  // Inversion of projected articulated inertia
  //mInvProjArtInertia = projAI.inverse();
  mInvProjArtInertia = projAI.ldlt().solve(Eigen::Matrix3d::Identity());

  // Verification
  assert(!math::isNan(mInvProjArtInertia));
}

//==============================================================================
void SO3Joint::updateInvProjArtInertiaKinematic(
    const Eigen::Matrix6d& /*_artInertia*/)
{
  // Do nothing
}

//==============================================================================
void SO3Joint::updateInvProjArtInertiaImplicit(
    const Eigen::Matrix6d& _artInertia,
    double _timeStep)
{
  switch (mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      updateInvProjArtInertiaImplicitDynamic(_artInertia, _timeStep);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      updateInvProjArtInertiaImplicitKinematic(_artInertia, _timeStep);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
void SO3Joint::updateInvProjArtInertiaImplicitDynamic(
    const Eigen::Matrix6d& _artInertia, double _timeStep)
{
  // Projected articulated inertia
  const Eigen::Matrix<double, 6, 3>& Jacobian = getLocalJacobianStatic();
  Eigen::Matrix3d projAI = Jacobian.transpose() * _artInertia * Jacobian;

  // Add additional inertia for implicit damping and spring force
  for (size_t i = 0; i < 3; ++i)
  {
    projAI(i, i) += _timeStep * mDampingCoefficient[i]
        + _timeStep * _timeStep * mSpringStiffness[i];
  }

  // Inversion of projected articulated inertia
  // mInvProjArtInertiaImplicit = projAI.inverse();
  mInvProjArtInertiaImplicit = projAI.ldlt().solve(Eigen::Matrix3d::Identity());

  // Verification
  assert(!math::isNan(mInvProjArtInertiaImplicit));
}

//==============================================================================
void SO3Joint::updateInvProjArtInertiaImplicitKinematic(
    const Eigen::Matrix6d& /*_artInertia*/, double /*_timeStep*/)
{
  // Do nothing
}

//==============================================================================
void SO3Joint::addChildBiasForceTo(
    Eigen::Vector6d& _parentBiasForce,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasForce,
    const Eigen::Vector6d& _childPartialAcc)
{
  switch (mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      addChildBiasForceToDynamic(_parentBiasForce,
                                    _childArtInertia,
                                    _childBiasForce,
                                    _childPartialAcc);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      addChildBiasForceToKinematic(_parentBiasForce,
                                          _childArtInertia,
                                          _childBiasForce,
                                          _childPartialAcc);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
void SO3Joint::addChildBiasForceToDynamic(
    Eigen::Vector6d& _parentBiasForce,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasForce,
    const Eigen::Vector6d& _childPartialAcc)
{
  // Compute beta
  const Eigen::Vector6d beta
      = _childBiasForce
        + _childArtInertia
          * (_childPartialAcc
             + getLocalJacobianStatic() * getInvProjArtInertiaImplicit()
               *mTotalForce);

  //    Eigen::Vector6d beta
  //        = _childBiasForce;
  //    beta.noalias() += _childArtInertia * _childPartialAcc;
  //    beta.noalias() += _childArtInertia *  mJacobian * getInvProjArtInertiaImplicit() * mTotalForce;

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasForce += math::dAdInvT(getLocalTransform(), beta);
}

//==============================================================================
void SO3Joint::addChildBiasForceToKinematic(
    Eigen::Vector6d& _parentBiasForce,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasForce,
    const Eigen::Vector6d& _childPartialAcc)
{
  // Compute beta
  const Eigen::Vector6d beta
      = _childBiasForce
        + _childArtInertia
          * (_childPartialAcc
             + getLocalJacobianStatic() * getAccelerationsStatic());

  //    Eigen::Vector6d beta
  //        = _childBiasForce;
  //    beta.noalias() += _childArtInertia * _childPartialAcc;
  //    beta.noalias() += _childArtInertia *  mJacobian * getInvProjArtInertiaImplicit() * mTotalForce;

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasForce += math::dAdInvT(getLocalTransform(), beta);
}

//==============================================================================
void SO3Joint::addChildBiasImpulseTo(
    Eigen::Vector6d& _parentBiasImpulse,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasImpulse)
{
  switch (mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      addChildBiasImpulseToDynamic(_parentBiasImpulse,
                                      _childArtInertia,
                                      _childBiasImpulse);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      addChildBiasImpulseToKinematic(_parentBiasImpulse,
                                            _childArtInertia,
                                            _childBiasImpulse);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
void SO3Joint::addChildBiasImpulseToDynamic(
    Eigen::Vector6d& _parentBiasImpulse,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasImpulse)
{
  // Compute beta
  const Eigen::Vector6d beta
      = _childBiasImpulse
        + _childArtInertia * getLocalJacobianStatic()
          * getInvProjArtInertia() * mTotalImpulse;

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasImpulse += math::dAdInvT(getLocalTransform(), beta);
}

//==============================================================================
void SO3Joint::addChildBiasImpulseToKinematic(
    Eigen::Vector6d& _parentBiasImpulse,
    const Eigen::Matrix6d& /*_childArtInertia*/,
    const Eigen::Vector6d& _childBiasImpulse)
{
  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasImpulse += math::dAdInvT(getLocalTransform(), _childBiasImpulse);
}

//==============================================================================
void SO3Joint::updateTotalForce(const Eigen::Vector6d& _bodyForce,
                                      double _timeStep)
{
  assert(_timeStep > 0.0);

  switch (mActuatorType)
  {
    case FORCE:
      mForces = mCommands;
      updateTotalForceDynamic(_bodyForce, _timeStep);
      break;
    case PASSIVE:
    case SERVO:
      mForces.setZero();
      updateTotalForceDynamic(_bodyForce, _timeStep);
      break;
    case ACCELERATION:
      setAccelerationsStatic(mCommands);
      updateTotalForceKinematic(_bodyForce, _timeStep);
      break;
    case VELOCITY:
      setAccelerationsStatic( (mCommands - getVelocitiesStatic()) / _timeStep );
      updateTotalForceKinematic(_bodyForce, _timeStep);
      break;
    case LOCKED:
      setVelocitiesStatic(Eigen::Vector3d::Zero());
      setAccelerationsStatic(Eigen::Vector3d::Zero());
      updateTotalForceKinematic(_bodyForce, _timeStep);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
void SO3Joint::updateTotalForceDynamic(
    const Eigen::Vector6d& _bodyForce, double _timeStep)
{
  // Spring force
  const Eigen::Vector3d springForce
      = (-mSpringStiffness).asDiagonal()
        *(getPositionsStatic() - mRestPositions
          + getVelocitiesStatic()*_timeStep);

  // Damping force
  const Eigen::Vector3d dampingForce
      = (-mDampingCoefficient).asDiagonal()*getVelocitiesStatic();

  //
  mTotalForce = mForces + springForce + dampingForce;
  mTotalForce.noalias() -= getLocalJacobianStatic().transpose()*_bodyForce;
}

//==============================================================================
void SO3Joint::updateTotalForceKinematic(
    const Eigen::Vector6d& /*_bodyForce*/, double /*_timeStep*/)
{
  // Do nothing
}

//==============================================================================
void SO3Joint::updateTotalImpulse(const Eigen::Vector6d& _bodyImpulse)
{
  switch (mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      updateTotalImpulseDynamic(_bodyImpulse);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      updateTotalImpulseKinematic(_bodyImpulse);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
void SO3Joint::updateTotalImpulseDynamic(
    const Eigen::Vector6d& _bodyImpulse)
{
  //
  mTotalImpulse = mConstraintImpulses;
  mTotalImpulse.noalias() -= getLocalJacobianStatic().transpose()
                             * _bodyImpulse;
}

//==============================================================================
void SO3Joint::updateTotalImpulseKinematic(
    const Eigen::Vector6d& /*_bodyImpulse*/)
{
  // Do nothing
}

//==============================================================================
void SO3Joint::resetTotalImpulses()
{
  mTotalImpulse.setZero();
}

//==============================================================================
void SO3Joint::updateAcceleration(const Eigen::Matrix6d& _artInertia,
                                        const Eigen::Vector6d& _spatialAcc)
{
  switch (mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      updateAccelerationDynamic(_artInertia, _spatialAcc);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      updateAccelerationKinematic(_artInertia, _spatialAcc);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
void SO3Joint::updateAccelerationDynamic(
    const Eigen::Matrix6d& _artInertia, const Eigen::Vector6d& _spatialAcc)
{
  //
  setAccelerationsStatic( getInvProjArtInertiaImplicit()
        * (mTotalForce - getLocalJacobianStatic().transpose()
           *_artInertia*math::AdInvT(getLocalTransform(), _spatialAcc)) );

  // Verification
  assert(!math::isNan(getAccelerationsStatic()));
}

//==============================================================================
void SO3Joint::updateAccelerationKinematic(
    const Eigen::Matrix6d& /*_artInertia*/,
    const Eigen::Vector6d& /*_spatialAcc*/)
{
  // Do nothing
}

//==============================================================================
void SO3Joint::updateVelocityChange(
    const Eigen::Matrix6d& _artInertia,
    const Eigen::Vector6d& _velocityChange)
{
  switch (mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      updateVelocityChangeDynamic(_artInertia, _velocityChange);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      updateVelocityChangeKinematic(_artInertia, _velocityChange);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
void SO3Joint::updateVelocityChangeDynamic(
    const Eigen::Matrix6d& _artInertia, const Eigen::Vector6d& _velocityChange)
{
  //
  mVelocityChanges
      = getInvProjArtInertia()
      * (mTotalImpulse - getLocalJacobianStatic().transpose()
         *_artInertia*math::AdInvT(getLocalTransform(), _velocityChange));

  // Verification
  assert(!math::isNan(mVelocityChanges));
}

//==============================================================================
void SO3Joint::updateVelocityChangeKinematic(
    const Eigen::Matrix6d& /*_artInertia*/,
    const Eigen::Vector6d& /*_velocityChange*/)
{
  // Do nothing
}

//==============================================================================
void SO3Joint::updateForceID(const Eigen::Vector6d& _bodyForce,
                                   double _timeStep,
                                   bool _withDampingForces,
                                   bool _withSpringForces)
{
  mForces = getLocalJacobianStatic().transpose() * _bodyForce;

  // Damping force
  if (_withDampingForces)
  {
    const Eigen::Vector3d dampingForces
        = (-mDampingCoefficient).asDiagonal()*getVelocitiesStatic();
    mForces -= dampingForces;
  }

  // Spring force
  if (_withSpringForces)
  {
    const Eigen::Vector3d springForces
        = (-mSpringStiffness).asDiagonal()
          *(getPositionsStatic() - mRestPositions
            + getVelocitiesStatic()*_timeStep);
    mForces -= springForces;
  }
}

//==============================================================================
void SO3Joint::updateForceFD(const Eigen::Vector6d& _bodyForce,
                                   double _timeStep,
                                   bool _withDampingForces,
                                   bool _withSpringForces)
{
  switch (mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      updateForceID(_bodyForce, _timeStep, _withDampingForces,
                    _withSpringForces);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
void SO3Joint::updateImpulseID(const Eigen::Vector6d& _bodyImpulse)
{
  mImpulses = getLocalJacobianStatic().transpose() * _bodyImpulse;
}

//==============================================================================
void SO3Joint::updateImpulseFD(const Eigen::Vector6d& _bodyImpulse)
{
  switch (mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      updateImpulseID(_bodyImpulse);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
void SO3Joint::updateConstrainedTerms(double _timeStep)
{
  switch (mActuatorType)
  {
    case FORCE:
    case PASSIVE:
    case SERVO:
      updateConstrainedTermsDynamic(_timeStep);
      break;
    case ACCELERATION:
    case VELOCITY:
    case LOCKED:
      updateConstrainedTermsKinematic(_timeStep);
      break;
    default:
      dterr << "Unsupported actuator type." << std::endl;
      break;
  }
}

//==============================================================================
void SO3Joint::updateConstrainedTermsDynamic(double _timeStep)
{
  const double invTimeStep = 1.0 / _timeStep;

  setVelocitiesStatic(getVelocitiesStatic() + mVelocityChanges);
  setAccelerationsStatic(getAccelerationsStatic()
                         + mVelocityChanges * invTimeStep);
  mForces += mConstraintImpulses*invTimeStep;
}

//==============================================================================
void SO3Joint::updateConstrainedTermsKinematic(double _timeStep)
{
  mForces += mImpulses / _timeStep;
}

//==============================================================================
void SO3Joint::addChildBiasForceForInvMassMatrix(
    Eigen::Vector6d& _parentBiasForce,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasForce)
{
  // Compute beta
  Eigen::Vector6d beta = _childBiasForce;
  beta.noalias() += _childArtInertia * getLocalJacobianStatic()
                    * getInvProjArtInertia() * mInvM_a;

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasForce += math::dAdInvT(getLocalTransform(), beta);
}

//==============================================================================
void SO3Joint::addChildBiasForceForInvAugMassMatrix(
    Eigen::Vector6d& _parentBiasForce,
    const Eigen::Matrix6d& _childArtInertia,
    const Eigen::Vector6d& _childBiasForce)
{
  // Compute beta
  Eigen::Vector6d beta = _childBiasForce;
  beta.noalias() += _childArtInertia * getLocalJacobianStatic()
                    * getInvProjArtInertiaImplicit() * mInvM_a;

  // Verification
  assert(!math::isNan(beta));

  // Add child body's bias force to parent body's bias force. Note that mT
  // should be updated.
  _parentBiasForce += math::dAdInvT(getLocalTransform(), beta);
}

//==============================================================================
void SO3Joint::updateTotalForceForInvMassMatrix(
    const Eigen::Vector6d& _bodyForce)
{
  // Compute alpha
  mInvM_a = mForces;
  mInvM_a.noalias() -= getLocalJacobianStatic().transpose() * _bodyForce;
}

//==============================================================================
void SO3Joint::getInvMassMatrixSegment(Eigen::MatrixXd& _invMassMat,
                                       const size_t _col,
                                       const Eigen::Matrix6d& _artInertia,
                                       const Eigen::Vector6d& _spatialAcc)
{
  //
  mInvMassMatrixSegment
      = getInvProjArtInertia()
      * (mInvM_a - getLocalJacobianStatic().transpose()
         * _artInertia * math::AdInvT(getLocalTransform(), _spatialAcc));

  // Verification
  assert(!math::isNan(mInvMassMatrixSegment));

  // Index
  size_t iStart = mDofs[0]->mIndexInSkeleton;

  // Assign
  _invMassMat.block<3, 1>(iStart, _col) = mInvMassMatrixSegment;
}

//==============================================================================
void SO3Joint::getInvAugMassMatrixSegment(
    Eigen::MatrixXd& _invMassMat,
    const size_t _col,
    const Eigen::Matrix6d& _artInertia,
    const Eigen::Vector6d& _spatialAcc)
{
  mInvMassMatrixSegment
      = getInvProjArtInertiaImplicit()
        * (mInvM_a - getLocalJacobianStatic().transpose()
           * _artInertia * math::AdInvT(getLocalTransform(), _spatialAcc));

  // Verification
  assert(!math::isNan(mInvMassMatrixSegment));

  // Index
  size_t iStart = mDofs[0]->mIndexInSkeleton;

  // Assign
  _invMassMat.block<3, 1>(iStart, _col) = mInvMassMatrixSegment;
}

//==============================================================================
void SO3Joint::addInvMassMatrixSegmentTo(Eigen::Vector6d& _acc)
{
  //
  _acc += getLocalJacobianStatic() * mInvMassMatrixSegment;
}

//==============================================================================
Eigen::VectorXd SO3Joint::getSpatialToGeneralized(
    const Eigen::Vector6d& _spatial)
{
  return getLocalJacobianStatic().transpose() * _spatial;
}

}  // namespace dynamics
}  // namespace dart

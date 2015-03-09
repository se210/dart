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

#ifndef DART_DYNAMICS_SO3JOINT_H_
#define DART_DYNAMICS_SO3JOINT_H_

#include <array>
#include <string>

#include "dart/common/Console.h"
#include "dart/dynamics/Joint.h"

namespace dart {
namespace dynamics {

class BodyNode;
class Skeleton;
class DegreeOfFreedom;

/// class SO3Joint
class SO3Joint : public Joint
{
public:
  using JacobianStatic = Eigen::Matrix<double, 6, 3>;

  /// Constructor
  SO3Joint(const std::string& _name);

  /// Destructor
  virtual ~SO3Joint();

  // Documentation inherited
  DEPRECATED(4.1)
  virtual size_t getDof() const override;

  // Documentation inherited
  size_t getNumDofs() const override;

  // Documentation inherited
  void setIndexInSkeleton(size_t _index, size_t _indexInSkeleton) override;

  // Documentation inherited
  size_t getIndexInSkeleton(size_t _index) const override;

  /// Convert a rotation into a 3D vector that can be used to set the positions
  /// of a BallJoint. The positions returned by this function will result in a
  /// relative transform of
  /// getTransformFromParentBodyNode() * _rotation * getTransformFromChildBodyNode().inverse()
  /// between the parent BodyNode and the child BodyNode frames when applied to
  /// a BallJoint.
  template <typename RotationType>
  static Eigen::Vector3d convertToPositions(const RotationType& _rotation,
                                            GeneralizedCoordinateType _type)
  {
    switch (_type)
    {
      case GeneralizedCoordinateType::SO3_LIE_ALGEBRA:
        return math::logMap(_rotation);
      default:
        dterr << "..." << std::endl;
        return Eigen::Vector3d::Zero();
        break;
    }
  }

  /// TODO
  template <typename RotationType>
  Eigen::Vector3d convertToPositions(const RotationType& _rotation) const
  {
    return SO3Joint::convertToPositions(_rotation, mGenCoordType);
  }

  /// Convert a BallJoint-style position vector into a transform
  static Eigen::Isometry3d convertToTransform(
      const Eigen::Vector3d& _positions,
      GeneralizedCoordinateType _type);

  /// Convert a BallJoint-style position vector into a transform
  Eigen::Isometry3d convertToTransform(
      const Eigen::Vector3d& _positions) const;

  /// Convert a BallJoint-style position vector into a rotation matrix
  static Eigen::Matrix3d convertToRotation(const Eigen::Vector3d& _positions,
                                           GeneralizedCoordinateType _type);

  /// TODO(JS)
  Eigen::Matrix3d convertToRotation(const Eigen::Vector3d& _positions) const;

  /// TODO(JS)
  static Eigen::Vector3d getFiniteDifferenceStatic(
      const Eigen::Vector3d& _pos1,
      const Eigen::Vector3d& _pos2,
      GeneralizedCoordinateType _type);

  /// TODO(JS)
  Eigen::Vector3d getFiniteDifferenceStatic(
      const Eigen::Vector3d& _pos1,
      const Eigen::Vector3d& _pos2) const;

  //----------------------------------------------------------------------------
  // Command
  //----------------------------------------------------------------------------

  // Documentation inherited
  void setCommand(size_t _index, double _command) override;

  // Documentation inherited
  double getCommand(size_t _index) const override;

  // Documentation inherited
  void setCommands(const Eigen::VectorXd& _commands) override;

  // Documentation inherited
  Eigen::VectorXd getCommands() const override;

  // Documentation inherited
  void resetCommands() override;

  //----------------------------------------------------------------------------
  // Position
  //----------------------------------------------------------------------------

  // TODO(JS): Not to use Eigen::VectorXd

  // Documentation inherited
  DegreeOfFreedom* getDof(size_t _index) override;

  // Documentation inherited
  const DegreeOfFreedom* getDof(size_t _index) const override;

  // Documentation inherited
  void setPosition(size_t _index, double _position) override;

  // Documentation inherited
  double getPosition(size_t _index) const override;

  // Documentation inherited
  void setPositions(const Eigen::VectorXd& _positions) override;

  // Documentation inherited
  Eigen::VectorXd getPositions() const override;

  // Documentation inherited
  void resetPositions() override;

  // Documentation inherited
  void setPositionLowerLimit(size_t _index, double _position) override;

  // Documentation inherited
  double getPositionLowerLimit(size_t _index) const override;

  // Documentation inherited
  void setPositionUpperLimit(size_t _index, double _position) override;

  // Documentation inherited
  double getPositionUpperLimit(size_t _index) const override;

  // Documentation inherited
  virtual Eigen::VectorXd getFiniteDifference(
      const Eigen::VectorXd& _pos1,
      const Eigen::VectorXd& _pos2) const override;

  //----------------------------------------------------------------------------
  // Velocity
  //----------------------------------------------------------------------------

  // Documentation inherited
  void setVelocity(size_t _index, double _velocity) override;

  // Documentation inherited
  double getVelocity(size_t _index) const override;

  // Documentation inherited
  void setVelocities(const Eigen::VectorXd& _velocities) override;

  // Documentation inherited
  Eigen::VectorXd getVelocities() const override;

  // Documentation inherited
  void resetVelocities() override;

  // Documentation inherited
  void setVelocityLowerLimit(size_t _index, double _velocity) override;

  // Documentation inherited
  double getVelocityLowerLimit(size_t _index) const override;

  // Documentation inherited
  void setVelocityUpperLimit(size_t _index, double _velocity) override;

  // Documentation inherited
  double getVelocityUpperLimit(size_t _index) const override;

  //----------------------------------------------------------------------------
  // Acceleration
  //----------------------------------------------------------------------------

  // Documentation inherited
  void setAcceleration(size_t _index, double _acceleration) override;

  // Documentation inherited
  double getAcceleration(size_t _index) const override;

  // Documentation inherited
  void setAccelerations(const Eigen::VectorXd& _accelerations) override;

  // Documentation inherited
  Eigen::VectorXd getAccelerations() const override;

  // Documentation inherited
  void resetAccelerations() override;

  // Documentation inherited
  void setAccelerationLowerLimit(size_t _index, double _acceleration) override;

  // Documentation inherited
  double getAccelerationLowerLimit(size_t _index) const override;

  // Documentation inherited
  void setAccelerationUpperLimit(size_t _index, double _acceleration) override;

  // Documentation inherited
  double getAccelerationUpperLimit(size_t _index) const override;

  //----------------------------------------------------------------------------
  // Fixed-size mutators and accessors
  //----------------------------------------------------------------------------

  // Note: The fixed-size versions of these functions exist to make it easier
  // to comply with the auto-updating design. Use these functions to avoid
  // accessing mPosition directly, that way it is easier to ensure that the
  // auto-updating design assumptions are being satisfied when reviewing the
  // code.

  /// Fixed-size version of setPositions()
  void setPositionsStatic(const Eigen::Vector3d& _positions);

  /// Fixed-size version of getPositions()
  Eigen::Vector3d getPositionsStatic() const;

  /// Fixed-size version of setVelocities()
  void setVelocitiesStatic(const Eigen::Vector3d& _velocities);

  /// Fixed-size version of getVelocities()
  Eigen::Vector3d getVelocitiesStatic() const;

  /// Fixed-size version of setAccelerations()
  void setAccelerationsStatic(const Eigen::Vector3d& _accelerations);

  /// Fixed-size version of getAccelerations()
  Eigen::Vector3d getAccelerationsStatic() const;

  //----------------------------------------------------------------------------
  // Force
  //----------------------------------------------------------------------------

  // Documentation inherited
  void setForce(size_t _index, double _force) override;

  // Documentation inherited
  double getForce(size_t _index) override;

  // Documentation inherited
  void setForces(const Eigen::VectorXd& _forces) override;

  // Documentation inherited
  Eigen::VectorXd getForces() const override;

  // Documentation inherited
  void resetForces() override;

  // Documentation inherited
  void setForceLowerLimit(size_t _index, double _force) override;

  // Documentation inherited
  double getForceLowerLimit(size_t _index) const override;

  // Documentation inherited
  void setForceUpperLimit(size_t _index, double _force) override;

  // Documentation inherited
  double getForceUpperLimit(size_t _index) const override;

  //----------------------------------------------------------------------------
  // Velocity change
  //----------------------------------------------------------------------------

  // Documentation inherited
  virtual void setVelocityChange(size_t _index, double _velocityChange)
  override;

  // Documentation inherited
  virtual double getVelocityChange(size_t _index) const override;

  // Documentation inherited
  virtual void resetVelocityChanges();

  //----------------------------------------------------------------------------
  // Constraint impulse
  //----------------------------------------------------------------------------

  // Documentation inherited
  virtual void setConstraintImpulse(size_t _index, double _impulse) override;

  // Documentation inherited
  virtual double getConstraintImpulse(size_t _index) const override;

  // Documentation inherited
  virtual void resetConstraintImpulses() override;

  //----------------------------------------------------------------------------
  // Integration
  //----------------------------------------------------------------------------

  // Documentation inherited
  virtual void integratePositions(double _dt) override;

  // Documentation inherited
  virtual void integrateVelocities(double _dt) override;

  //----------------------------------------------------------------------------
  /// \{ \name Passive forces - spring, viscous friction, Coulomb friction
  //----------------------------------------------------------------------------

  // Documentation inherited
  virtual void setSpringStiffness(size_t _index, double _k) override;

  // Documentation inherited
  virtual double getSpringStiffness(size_t _index) const override;

  // Documentation inherited
  virtual void setRestPosition(size_t _index, double _q0) override;

  // Documentation inherited
  virtual double getRestPosition(size_t _index) const override;

  // Documentation inherited
  virtual void setDampingCoefficient(size_t _index, double _d) override;

  // Documentation inherited
  virtual double getDampingCoefficient(size_t _index) const override;

  // Documentation inherited
  virtual void setCoulombFriction(size_t _index, double _friction) override;

  // Documentation inherited
  virtual double getCoulombFriction(size_t _index) const override;

  /// \}

  //----------------------------------------------------------------------------

  /// Get potential energy
  virtual double getPotentialEnergy() const override;

  // Documentation inherited
  virtual Eigen::Vector6d getBodyConstraintWrench() const override;

protected:
  //----------------------------------------------------------------------------
  /// \{ \name Internal variables
  //----------------------------------------------------------------------------

  ///
  // TODO(JS): More simpler name like setRotation()?
  void setRotationStatic(const Eigen::Matrix3d& _rotation);

  ///
  // TODO(JS): More simpler name like getRotation()?
  const Eigen::Matrix3d& getRotationStatic() const;

  ///
  void setInternalVelocitiesStatic(const Eigen::Vector3d& _velocities);

  ///
  const Eigen::Vector3d& getInternalVelocitiesStatic() const;

  ///
  void setInternalAccelerationsStatic(const Eigen::Vector3d& _accelerations);

  ///
  const Eigen::Vector3d& getInternalAccelerationsStatic() const;

  /// \}

  const JacobianStatic& getInternalLocalJacobian() const;

  // Documentation inherited
  virtual void updateDegreeOfFreedomNames() override;

  // Documentation inherited
  virtual void updateLocalTransform() const;

  ///
  void updateInternalLocalJacobian() const;

  // Documentation inherited
  virtual void updateLocalJacobian(bool =true) const override;

  // Documentation inherited
  virtual void updateLocalJacobianTimeDeriv() const override;

  //----------------------------------------------------------------------------
  /// \{ \name Recursive dynamics routines
  //----------------------------------------------------------------------------

  // Documentation inherited
  void updateLocalSpatialVelocity() const override;

  // Documentation inherited
  void updateLocalSpatialAcceleration() const override;

  // Documentation inherited
  void updateLocalPrimaryAcceleration() const override;

  // Documentation inherited
  const math::Jacobian getLocalJacobian() const override;

  /// Fixed-size version of getLocalJacobian()
  const JacobianStatic& getLocalJacobianStatic() const;

  // Documentation inherited
  const math::JacobianDynamic getLocalJacobianTimeDeriv() const override;

  /// Fixed-size version of getLocalJacobianTimeDeriv()
  const JacobianStatic& getLocalJacobianTimeDerivStatic() const;

  /// Get the inverse of projected articulated inertia
  const Eigen::Matrix3d& getInvProjArtInertia() const;

  /// Get the inverse of projected articulated inertia for implicit joint
  /// damping and spring forces
  const Eigen::Matrix3d& getInvProjArtInertiaImplicit() const;

  // Documentation inherited
  void addVelocityTo(Eigen::Vector6d& _vel) override;

  // Documentation inherited
  void setPartialAccelerationTo(
      Eigen::Vector6d& _partialAcceleration,
      const Eigen::Vector6d& _childVelocity) override;

  // Documentation inherited
  void addAccelerationTo(Eigen::Vector6d& _acc) override;

  // Documentation inherited
  void addVelocityChangeTo(Eigen::Vector6d& _velocityChange) override;

  // Documentation inherited
  void addChildArtInertiaTo(
      Eigen::Matrix6d& _parentArtInertia,
      const Eigen::Matrix6d& _childArtInertia) override;

  // Documentation inherited
  void addChildArtInertiaImplicitTo(
      Eigen::Matrix6d& _parentArtInertia,
      const Eigen::Matrix6d& _childArtInertia) override;

  // Documentation inherited
  void updateInvProjArtInertia(
      const Eigen::Matrix6d& _artInertia) override;

  // Documentation inherited
  void updateInvProjArtInertiaImplicit(
      const Eigen::Matrix6d& _artInertia,
      double _timeStep) override;

  // Documentation inherited
  void addChildBiasForceTo(
      Eigen::Vector6d& _parentBiasForce,
      const Eigen::Matrix6d& _childArtInertia,
      const Eigen::Vector6d& _childBiasForce,
      const Eigen::Vector6d& _childPartialAcc) override;

  // Documentation inherited
  void addChildBiasImpulseTo(
      Eigen::Vector6d& _parentBiasImpulse,
      const Eigen::Matrix6d& _childArtInertia,
      const Eigen::Vector6d& _childBiasImpulse) override;

  // Documentation inherited
  void updateTotalForce(const Eigen::Vector6d& _bodyForce,
                                  double _timeStep) override;

  // Documentation inherited
  void updateTotalImpulse(
      const Eigen::Vector6d& _bodyImpulse) override;

  // Documentation inherited
  void resetTotalImpulses() override;

  // Documentation inherited
  void updateAcceleration(
      const Eigen::Matrix6d& _artInertia,
      const Eigen::Vector6d& _spatialAcc) override;

  // Documentation inherited
  void updateVelocityChange(
      const Eigen::Matrix6d& _artInertia,
      const Eigen::Vector6d& _velocityChange) override;

  // Documentation inherited
  void updateForceID(const Eigen::Vector6d& _bodyForce,
                             double _timeStep,
                             bool _withDampingForces,
                             bool _withSpringForces) override;

  // Documentation inherited
  void updateForceFD(const Eigen::Vector6d& _bodyForce,
                             double _timeStep,
                             bool _withDampingForces,
                             bool _withSpringForces) override;

  // Documentation inherited
  void updateImpulseID(const Eigen::Vector6d& _bodyImpulse) override;

  // Documentation inherited
  void updateImpulseFD(const Eigen::Vector6d& _bodyImpulse) override;

  // Documentation inherited
  void updateConstrainedTerms(double _timeStep) override;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Recursive algorithm routines for equations of motion
  //----------------------------------------------------------------------------

  // Documentation inherited
  void addChildBiasForceForInvMassMatrix(
      Eigen::Vector6d& _parentBiasForce,
      const Eigen::Matrix6d& _childArtInertia,
      const Eigen::Vector6d& _childBiasForce) override;

  // Documentation inherited
  void addChildBiasForceForInvAugMassMatrix(
      Eigen::Vector6d& _parentBiasForce,
      const Eigen::Matrix6d& _childArtInertia,
      const Eigen::Vector6d& _childBiasForce) override;

  // Documentation inherited
  void updateTotalForceForInvMassMatrix(
      const Eigen::Vector6d& _bodyForce) override;

  // Documentation inherited
  void getInvMassMatrixSegment(
      Eigen::MatrixXd& _invMassMat,
      const size_t _col,
      const Eigen::Matrix6d& _artInertia,
      const Eigen::Vector6d& _spatialAcc) override;

  // Documentation inherited
  void getInvAugMassMatrixSegment(
      Eigen::MatrixXd& _invMassMat,
      const size_t _col,
      const Eigen::Matrix6d& _artInertia,
      const Eigen::Vector6d& _spatialAcc) override;

  // Documentation inherited
  void addInvMassMatrixSegmentTo(Eigen::Vector6d& _acc) override;

  // Documentation inherited
  Eigen::VectorXd getSpatialToGeneralized(
      const Eigen::Vector6d& _spatial) override;

  /// \}

protected:
  /// \brief DegreeOfFreedom pointer
  std::array<DegreeOfFreedom*, 3> mDofs;

  /// Command
  Eigen::Vector3d mCommands;

  //----------------------------------------------------------------------------
  // Configuration
  //----------------------------------------------------------------------------

  /// Position
  Eigen::Matrix3d mRotation;
  // TODO: Use Isometry3d and the vary the rotation part

  /// Lower limit of position
  Eigen::Vector3d mPositionLowerLimits;

  /// Upper limit of position
  Eigen::Vector3d mPositionUpperLimits;

  /// Derivatives w.r.t. an arbitrary scalr variable
  Eigen::Vector3d mPositionDeriv;

  //----------------------------------------------------------------------------
  // Velocity
  //----------------------------------------------------------------------------

  /// Generalized velocity
  Eigen::Vector3d mVelocities;

  /// Min value allowed.
  Eigen::Vector3d mVelocityLowerLimits;

  /// Max value allowed.
  Eigen::Vector3d mVelocityUpperLimits;

  /// Derivatives w.r.t. an arbitrary scalr variable
  Eigen::Vector3d mVelocityDeriv;

  //----------------------------------------------------------------------------
  // Acceleration
  //----------------------------------------------------------------------------

  /// Generalized acceleration
  Eigen::Vector3d mAccelerations;

  /// Min value allowed.
  Eigen::Vector3d mAccelerationLowerLimits;

  /// upper limit of generalized acceleration
  Eigen::Vector3d mAccelerationUpperLimits;

  /// Derivatives w.r.t. an arbitrary scalr variable
  Eigen::Vector3d mAccelerationDeriv;

  //----------------------------------------------------------------------------
  // Force
  //----------------------------------------------------------------------------

  /// Generalized force
  Eigen::Vector3d mForces;

  /// Min value allowed.
  Eigen::Vector3d mForceLowerLimits;

  /// Max value allowed.
  Eigen::Vector3d mForceUpperLimits;

  /// Derivatives w.r.t. an arbitrary scalr variable
  Eigen::Vector3d mForceDeriv;

  //----------------------------------------------------------------------------
  // Impulse
  //----------------------------------------------------------------------------

  /// Change of generalized velocity
  Eigen::Vector3d mVelocityChanges;

  /// Generalized impulse
  Eigen::Vector3d mImpulses;

  /// Generalized constraint impulse
  Eigen::Vector3d mConstraintImpulses;

  //----------------------------------------------------------------------------
  // Spring and damper
  //----------------------------------------------------------------------------

  /// Joint spring stiffness
  Eigen::Vector3d mSpringStiffness;

  /// Rest joint position for joint spring
  Eigen::Vector3d mRestPositions;

  /// Joint damping coefficient
  Eigen::Vector3d mDampingCoefficient;

  /// Coulomb friction force
  Eigen::Vector3d mFrictions;

  //----------------------------------------------------------------------------
  // For recursive dynamics algorithms
  //----------------------------------------------------------------------------

  /// Spatial Jacobian expressed in the child body frame
  ///
  /// Do not use directly! Use getLocalJacobianStatic() to access this quantity
  mutable Eigen::Matrix<double, 6, 3> mJacobian;

  /// Time derivative of spatial Jacobian expressed in the child body frame
  ///
  /// Do not use directly! Use getLocalJacobianTimeDerivStatic() to access this
  /// quantity
  mutable Eigen::Matrix<double, 6, 3> mJacobianDeriv;

  /// Inverse of projected articulated inertia
  ///
  /// Do not use directly! Use getInvProjArtInertia() to access this quantity
  mutable Eigen::Matrix3d mInvProjArtInertia;

  /// Inverse of projected articulated inertia for implicit joint damping and
  /// spring forces
  ///
  /// Do not use directly! Use getInvProjArtInertiaImplicit() to access this
  /// quantity
  mutable Eigen::Matrix3d mInvProjArtInertiaImplicit;

  /// Total force projected on joint space
  Eigen::Vector3d mTotalForce;

  /// Total impluse projected on joint space
  Eigen::Vector3d mTotalImpulse;

  //----------------------------------------------------------------------------
  // For equations of motion
  //----------------------------------------------------------------------------

  ///
  Eigen::Vector3d mInvM_a;

  ///
  Eigen::Vector3d mInvMassMatrixSegment;

private:
  //----------------------------------------------------------------------------
  /// \{ \name Recursive dynamics routines
  //----------------------------------------------------------------------------

  void addChildArtInertiaToDynamic(
      Eigen::Matrix6d& _parentArtInertia,
      const Eigen::Matrix6d& _childArtInertia);

  void addChildArtInertiaToKinematic(
      Eigen::Matrix6d& _parentArtInertia,
      const Eigen::Matrix6d& _childArtInertia);

  void addChildArtInertiaImplicitToDynamic(
      Eigen::Matrix6d& _parentArtInertia,
      const Eigen::Matrix6d& _childArtInertia);

  void addChildArtInertiaImplicitToKinematic(
      Eigen::Matrix6d& _parentArtInertia,
      const Eigen::Matrix6d& _childArtInertia);

  void updateInvProjArtInertiaDynamic(
      const Eigen::Matrix6d& _artInertia);

  void updateInvProjArtInertiaKinematic(
      const Eigen::Matrix6d& _artInertia);

  void updateInvProjArtInertiaImplicitDynamic(
      const Eigen::Matrix6d& _artInertia,
      double _timeStep);

  void updateInvProjArtInertiaImplicitKinematic(
      const Eigen::Matrix6d& _artInertia,
      double _timeStep);

  void addChildBiasForceToDynamic(
      Eigen::Vector6d& _parentBiasForce,
      const Eigen::Matrix6d& _childArtInertia,
      const Eigen::Vector6d& _childBiasForce,
      const Eigen::Vector6d& _childPartialAcc);

  void addChildBiasForceToKinematic(
      Eigen::Vector6d& _parentBiasForce,
      const Eigen::Matrix6d& _childArtInertia,
      const Eigen::Vector6d& _childBiasForce,
      const Eigen::Vector6d& _childPartialAcc);

  void addChildBiasImpulseToDynamic(
      Eigen::Vector6d& _parentBiasImpulse,
      const Eigen::Matrix6d& _childArtInertia,
      const Eigen::Vector6d& _childBiasImpulse);

  void addChildBiasImpulseToKinematic(
      Eigen::Vector6d& _parentBiasImpulse,
      const Eigen::Matrix6d& _childArtInertia,
      const Eigen::Vector6d& _childBiasImpulse);

  void updateTotalForceDynamic(const Eigen::Vector6d& _bodyForce,
                                  double _timeStep);

  void updateTotalForceKinematic(const Eigen::Vector6d& _bodyForce,
                                        double _timeStep);

  void updateTotalImpulseDynamic(
      const Eigen::Vector6d& _bodyImpulse);

  void updateTotalImpulseKinematic(
      const Eigen::Vector6d& _bodyImpulse);

  void updateAccelerationDynamic(
      const Eigen::Matrix6d& _artInertia,
      const Eigen::Vector6d& _spatialAcc);

  void updateAccelerationKinematic(
      const Eigen::Matrix6d& _artInertia,
      const Eigen::Vector6d& _spatialAcc);

  void updateVelocityChangeDynamic(
      const Eigen::Matrix6d& _artInertia,
      const Eigen::Vector6d& _velocityChange);

  void updateVelocityChangeKinematic(
      const Eigen::Matrix6d& _artInertia,
      const Eigen::Vector6d& _velocityChange);

  void updateConstrainedTermsDynamic(double _timeStep);

  void updateConstrainedTermsKinematic(double _timeStep);

  /// \}
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_SO3JOINT_H_

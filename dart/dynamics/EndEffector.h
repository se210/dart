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

#ifndef DART_DYNAMICS_ENDEFFECTOR_H_
#define DART_DYNAMICS_ENDEFFECTOR_H_

#include "dart/dynamics/FixedFrame.h"
#include "dart/dynamics/TemplatedJacobianNode.h"

namespace dart {
namespace dynamics {

class BodyNode;
class Skeleton;

class EndEffector : public FixedFrame,
                    public AccessoryNode,
                    public TemplatedJacobianNode<EndEffector>
{
public:

  friend class Skeleton;
  friend class BodyNode;

  struct UniqueProperties
  {
    /// The relative transform will be set to this whenever
    /// resetRelativeTransform() is called
    Eigen::Isometry3d mDefaultTransform;

    UniqueProperties(
        const Eigen::Isometry3d& _defaultTransform =
                                                Eigen::Isometry3d::Identity());
  };

  struct Properties : Entity::Properties, UniqueProperties
  {
    Properties(
        const Entity::Properties& _entityProperties = Entity::Properties(),
        const UniqueProperties& _effectorProperties = UniqueProperties() );
  };

  /// Destructor
  virtual ~EndEffector();

  //----------------------------------------------------------------------------
  /// \{ \name Structural Properties
  //----------------------------------------------------------------------------

  /// Set the Properties of this EndEffector. If _useNow is true, the current
  /// Transform will be set to the new default transform
  void setProperties(const Properties& _properties, bool _useNow=true);

  /// Set the Properties of this EndEffector. If _useNow is true, the current
  /// Transform will be set to the new default transform
  void setProperties(const UniqueProperties& _properties, bool _useNow=true);

  Properties getEndEffectorProperties() const;

  /// Copy the Properties of another EndEffector
  void copy(const EndEffector& _otherEndEffector);

  /// Copy the Properties of another EndEffector
  void copy(const EndEffector* _otherEndEffector);

  /// Copy the Properties of another EndEffector
  EndEffector& operator=(const EndEffector& _otherEndEffector);

  /// Set name. If the name is already taken, this will return an altered
  /// version which will be used by the Skeleton
  const std::string& setName(const std::string& _name) override;

  /// Set the current relative transform of this EndEffector
  void setRelativeTransform(const Eigen::Isometry3d& _newRelativeTf);

  /// Set the default relative transform of this EndEffector. The relative
  /// transform of this EndEffector will be set to _newDefaultTf the next time
  /// resetRelativeTransform() is called. If _useNow is set to true, then
  /// resetRelativeTransform() will be called at the end of this function.
  void setDefaultRelativeTransform(const Eigen::Isometry3d& _newDefaultTf,
                                   bool _useNow);

  /// Set the current relative transform of this EndEffector to the default
  /// relative transform of this EndEffector. The default relative transform can
  /// be set with setDefaultRelativeTransform()
  void resetRelativeTransform();

  // Documentation inherited
  std::shared_ptr<Skeleton> getSkeleton() override;

  // Documentation inherited
  std::shared_ptr<const Skeleton> getSkeleton() const override;

  // Documentation inherited
  size_t getNumDependentGenCoords() const override;

  // Documentation inherited
  const std::vector<size_t>& getDependentGenCoordIndices() const override;

  // Documentation inherited
  size_t getNumDependentDofs() const override;

  // Documentation inherited
  const std::vector<const DegreeOfFreedom*> getChainDofs() const override;

  /// Get the BodyNode that this EndEffector is rigidly attached to
  BodyNode* getParentBodyNode();

  /// Get the BodyNode that this EndEffector is rigidly attached to
  const BodyNode* getParentBodyNode() const;

  /// Get the index of this EndEffector within the Skeleton
  size_t getIndexInSkeleton() const;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Jacobian Functions
  //----------------------------------------------------------------------------

  // Documentation inherited
  const math::Jacobian& getJacobian() const override;

  // Prevent the inherited getJacobian functions from being shadowed
  using TemplatedJacobianNode<EndEffector>::getJacobian;

  // Documentation inherited
  const math::Jacobian& getWorldJacobian() const override;

  // Prevent the inherited getWorldJacobian functions from being shadowed
  using TemplatedJacobianNode<EndEffector>::getWorldJacobian;

  // Documentation inherited
  const math::Jacobian& getJacobianSpatialDeriv() const override;

  // Prevent the inherited getJacobianSpatialDeriv functions from being shadowed
  using TemplatedJacobianNode<EndEffector>::getJacobianSpatialDeriv;

  // Documentation inherited
  const math::Jacobian& getJacobianClassicDeriv() const override;

  // Prevent the inherited getJacobianClassicDeriv functions from being shadowed
  using TemplatedJacobianNode<EndEffector>::getJacobianClassicDeriv;

  /// \}

  //----------------------------------------------------------------------------
  /// \{ \name Notifications
  //----------------------------------------------------------------------------

  // Documentation inherited
  virtual void notifyTransformUpdate() override;

  // Documentation inherited
  virtual void notifyVelocityUpdate() override;

protected:

  /// Constructor used by the Skeleton class
  explicit EndEffector(BodyNode* _parent, const Properties& _properties);

  /// Create a clone of this BodyNode. This may only be called by the Skeleton
  /// class.
  virtual EndEffector* clone(BodyNode* _parent) const;

  /// Update the Jacobian of this EndEffector. getJacobian() calls this function
  /// if mIsEffectorJacobianDirty is true.
  void updateEffectorJacobian() const;

  /// Update the World Jacobian cache.
  void updateWorldJacobian() const;

  /// Update the spatial time derivative of the end effector Jacobian.
  /// getJacobianSpatialDeriv() calls this function if
  /// mIsEffectorJacobianSpatialDerivDirty is true.
  void updateEffectorJacobianSpatialDeriv() const;

  /// Update the classic time derivative of the end effector Jacobian.
  /// getJacobianClassicDeriv() calls this function if
  /// mIsWorldJacobianClassicDerivDirty is true.
  void updateWorldJacobianClassicDeriv() const;

  /// Properties of this EndEffector
  UniqueProperties mEndEffectorP;

  /// The index of this EndEffector within its Skeleton
  size_t mIndexInSkeleton;

  /// The index of this EndEffector within its BodyNode
  size_t mIndexInBodyNode;

  /// Cached Jacobian of this EndEffector
  ///
  /// Do not use directly! Use getJacobian() to access this quantity
  mutable math::Jacobian mEffectorJacobian;

  /// Dirty flag for end effector Jacobian
  mutable bool mIsEffectorJacobianDirty;

  /// Cached World Jacobian of this EndEffector
  ///
  /// Do not use directly! Use getWorldJacobian() to access this quantity
  mutable math::Jacobian mWorldJacobian;

  /// Dirty flag for world Jacobian
  mutable bool mIsWorldJacobianDirty;

  /// Spatial time derivative of end effector Jacobian
  ///
  /// Do not use directly! Use getJacobianSpatialDeriv() to access this quantity
  mutable math::Jacobian mEffectorJacobianSpatialDeriv;

  /// Dirty flag for spatial time derivative of the end effector Jacobian
  mutable bool mIsEffectorJacobianSpatialDerivDirty;

  /// Classic time derivative of the end effector Jacobian
  ///
  /// Do not use directly! Use getJacobianClassicDeriv() to access this quantity
  mutable math::Jacobian mWorldJacobianClassicDeriv;

  /// Dirty flag for the classic time derivative of the Jacobian
  mutable bool mIsWorldJacobianClassicDerivDirty;
};

} // namespace dynamics
} // namespace dart


#endif // DART_DYNAMICS_ENDEFFECTOR_H_

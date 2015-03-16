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

#ifndef DART_DYNAMICS_POINT_H_
#define DART_DYNAMICS_POINT_H_

#include "dart/dynamics/EigenEntity.h"

namespace dart {
namespace dynamics {

/// The Point class represents a geometric point that is fixed in its reference
/// Frame.
class Point : public EigenEntity<Eigen::Vector3d>
{
public:

  EIGENENTITY_COPIERS( Point, Eigen::Vector3d )

  // Inherit the constructor
  using EigenEntity<Eigen::Vector3d>::EigenEntity;

  /// Destructor
  virtual ~Point();

  /// Get the location of this Point with respect to some Frame. Equivalent to
  /// wrt()
  Eigen::Vector3d getLocation(
      const Frame* _withRespectTo = Frame::World()) const;

  /// Get the location of this point relative to its parent Frame.
  ///
  /// Note that this function is superfluous, because the Point class inherits
  /// Vector3d, and its value is equivalent to this relative location vector.
  const Eigen::Vector3d& getRelativeLocation() const;

  /// Get the world location of this Point. Equivalent to wrtWorld()
  Eigen::Vector3d getWorldLocation() const;

  /// Get the linear velocity of this Point relative to some Frame
  Eigen::Vector3d getLinearVelocity(
      const Frame* _relativeTo = Frame::World(),
      const Frame* _inCoordinatesOf = Frame::World()) const;

  /// Get the linear velocity of this Point relative to some other Point
  Eigen::Vector3d getLinearVelocity(
      const Point* _relativeTo,
      const Frame* _inCoordinatesOf = Frame::World()) const;

  /// Get the linear acceleration of this Point with respect to some Frame
  Eigen::Vector3d getLinearAcceleration(
      const Frame* _relativeTo = Frame::World(),
      const Frame* _inCoordinatesOf = Frame::World()) const;

  /// Get the linear acceleration of this Point with respect to some other Point
  Eigen::Vector3d getLinearAcceleration(
      const Point* _relativeTo,
      const Frame* _inCoordinatesOf = Frame::World()) const;

  /// Get the spatial velocity of this Point with respect to some Frame
  Eigen::Vector6d getSpatialVelocity(
      const Frame* _relativeTo = Frame::World(),
      const Frame* _inCoordinatesOf = Frame::World()) const;

  /// Get the spatial velocity of this Point with respect to some other Point
  Eigen::Vector6d getSpatialVelocity(
      const Point* _relativeTo,
      const Frame* _inCoordinatesOf = Frame::World()) const;

  /// Get the spatial acceleration of this Point with respect to some Frame
  Eigen::Vector6d getSpatialAcceleration(
      const Frame* _relativeTo = Frame::World(),
      const Frame* _inCoordinatesOf = Frame::World()) const;

  /// Get the spatial acceleration of this Point with respect to some other
  /// Point
  Eigen::Vector6d getSpatialAcceleration(
      const Point* _relativeTo,
      const Frame* _inCoordinatesOf = Frame::World()) const;

protected:

  // Documentation inherited
  Eigen::Vector3d computeRelativeTo(const Frame *_referenceFrame) const override;

  // Documentation inherited
  Eigen::Vector3d computeRelativeToWorld() const override;
};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_POINT_H_

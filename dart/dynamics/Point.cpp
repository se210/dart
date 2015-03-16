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

#include "Point.h"

namespace dart {
namespace dynamics {

//==============================================================================
Eigen::Vector3d Point::getLocation(const Frame *_withRespectTo) const
{
  return wrt(_withRespectTo);
}

//==============================================================================
const Eigen::Vector3d& Point::getRelativeLocation() const
{
  return static_cast<const Eigen::Vector3d&>(*this);
}

//==============================================================================
Eigen::Vector3d Point::getWorldLocation() const
{
  return wrtWorld();
}

//==============================================================================
Eigen::Vector3d Point::getLinearVelocity(const Frame* _relativeTo,
                                         const Frame* _inCoordinatesOf) const
{
  return getParentFrame()->getLinearVelocity(
        static_cast<const Eigen::Vector3d&>(*this),
        _relativeTo, _inCoordinatesOf);
}

//==============================================================================
Eigen::Vector3d Point::getLinearVelocity(const Point* _relativeTo,
                                         const Frame* _inCoordinatesOf) const
{
  return getParentFrame()->getLinearVelocity(
        static_cast<const Eigen::Vector3d&>(*this),
        _relativeTo, _inCoordinatesOf);
}

//==============================================================================
Eigen::Vector3d Point::getLinearAcceleration(const Frame* _relativeTo,
                                            const Frame* _inCoordinatesOf) const
{
  return getParentFrame()->getLinearAcceleration(
        static_cast<const Eigen::Vector3d&>(*this),
        _relativeTo, _inCoordinatesOf);
}

//==============================================================================
Eigen::Vector3d Point::getLinearAcceleration(const Point* _relativeTo,
                                            const Frame* _inCoordinatesOf) const
{
  return getParentFrame()->getLinearAcceleration(
        static_cast<const Eigen::Vector3d&>(*this),
        _relativeTo, _inCoordinatesOf);
}

//==============================================================================
Eigen::Vector6d Point::getSpatialVelocity(const Frame* _relativeTo,
                                          const Frame* _inCoordinatesOf) const
{
  return getParentFrame()->getSpatialVelocity(
        static_cast<const Eigen::Vector3d&>(*this),
        _relativeTo, _inCoordinatesOf);
}

//==============================================================================
Eigen::Vector6d Point::getSpatialVelocity(const Point* _relativeTo,
                                          const Frame* _inCoordinatesOf) const
{
  return getParentFrame()->getSpatialVelocity(
        static_cast<const Eigen::Vector3d&>(*this),
        _relativeTo, _inCoordinatesOf);
}

//==============================================================================
Eigen::Vector6d Point::getSpatialAcceleration(const Frame* _relativeTo,
                                            const Frame* _inCoordinatesOf) const
{
  return getParentFrame()->getSpatialAcceleration(
        static_cast<const Eigen::Vector3d&>(*this),
        _relativeTo, _inCoordinatesOf);
}

//==============================================================================
Eigen::Vector6d Point::getSpatialAcceleration(const Point* _relativeTo,
                                            const Frame* _inCoordinatesOf) const
{
  return getParentFrame()->getSpatialAcceleration(
        static_cast<const Eigen::Vector3d&>(*this),
        _relativeTo, _inCoordinatesOf);
}

//==============================================================================
Eigen::Vector3d Point::computeRelativeTo(const Frame* _referenceFrame) const
{
  return _referenceFrame->getWorldTransform().inverse() * wrtWorld();
}

//==============================================================================
Eigen::Vector3d Point::computeRelativeToWorld() const
{
  return getParentFrame()->getWorldTransform()
         * static_cast<const Eigen::Vector3d&>(*this);
}

} // namespace dynamics
} // namespace dart

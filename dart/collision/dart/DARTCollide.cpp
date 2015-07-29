/*
 * Copyright (c) 2013-2015, Georgia Tech Research Corporation
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

#include "dart/collision/dart/DARTCollide.h"

#include <memory>

#include "dart/math/Helpers.h"
#include "dart/collision/dart/DARTCollision.h"
#include "dart/dynamics/BoxShape.h"
#include "dart/dynamics/EllipsoidShape.h"
#include "dart/dynamics/CylinderShape.h"
#include "dart/dynamics/BodyNode.h"

namespace dart {
namespace collision {

//==============================================================================
int collideBoxBox(const Eigen::Vector3d& size0, const Eigen::Isometry3d& T0,
                  const Eigen::Vector3d& size1, const Eigen::Isometry3d& T1,
                  std::vector<Contact>* result)
{
  return collideBoxBoxDART(size0, T0, size1, T1, result);
}

//==============================================================================
int	collideBoxSphere(const Eigen::Vector3d& size0, const Eigen::Isometry3d& T0,
                     const double& r1, const Eigen::Isometry3d& T1,
                     std::vector<Contact>* result)
{
  return collideBoxSphereDART(size0, T0, r1, T1, result);
}

//==============================================================================
int collideSphereBox(const double& r0, const Eigen::Isometry3d& T0,
                     const Eigen::Vector3d& size1, const Eigen::Isometry3d& T1,
                     std::vector<Contact>* result)
{
  return collideSphereBoxDART(r0, T0, size1, T1, result);
}

//==============================================================================
int collideSphereSphere(const double& _r0, const Eigen::Isometry3d& c0,
                        const double& _r1, const Eigen::Isometry3d& c1,
                        std::vector<Contact>* result)
{
  return collideSphereSphereDART(_r0, c0, _r1, c1, result);
}

//==============================================================================
int collideCylinderSphere(const double& cyl_rad, const double& half_height,
                          const Eigen::Isometry3d& T0,
                          const double& sphere_rad, const Eigen::Isometry3d& T1,
                          std::vector<Contact>* result)
{
  return collideCylinderSphereDART(cyl_rad, half_height, T0,
                                   sphere_rad, T1, result);
}

//==============================================================================
int collideCylinderPlane(const double& cyl_rad, const double& half_height,
                         const Eigen::Isometry3d& T0,
                         const Eigen::Vector3d& plane_normal,
                         const Eigen::Isometry3d& T1,
                         std::vector<Contact>* result)
{
  return collideCylinderPlaneDART(cyl_rad, half_height, T0,
                                  plane_normal, T1, result);
}

//==============================================================================
int collide(dynamics::ConstShapePtr _shape0, const Eigen::Isometry3d& _T0,
            dynamics::ConstShapePtr _shape1, const Eigen::Isometry3d& _T1,
            std::vector<Contact>* _result)
{
  dynamics::Shape::ShapeType LeftType = _shape0->getShapeType();
  dynamics::Shape::ShapeType RightType = _shape1->getShapeType();

  switch(LeftType)
  {
    case dynamics::Shape::BOX:
    {
      const dynamics::BoxShape* box0 = static_cast<const dynamics::BoxShape*>(_shape0.get());

      switch(RightType)
      {
        case dynamics::Shape::BOX:
        {
          const dynamics::BoxShape* box1 = static_cast<const dynamics::BoxShape*>(_shape1.get());
          return collideBoxBoxDART(box0->getSize(), _T0,
                               box1->getSize(), _T1, _result);
        }
        case dynamics::Shape::ELLIPSOID:
        {
          const dynamics::EllipsoidShape* ellipsoid1 = static_cast<const dynamics::EllipsoidShape*>(_shape1.get());
          return collideBoxSphereDART(box0->getSize(), _T0,
                                  ellipsoid1->getSize()[0] * 0.5, _T1,
              _result);
        }
        case dynamics::Shape::CYLINDER:
        {
          //----------------------------------------------------------
          // NOT SUPPORT CYLINDER
          //----------------------------------------------------------
          const dynamics::CylinderShape* cylinder1 = static_cast<const dynamics::CylinderShape*>(_shape1.get());

          Eigen::Vector3d dimTemp(cylinder1->getRadius() * sqrt(2.0),
                                  cylinder1->getRadius() * sqrt(2.0),
                                  cylinder1->getHeight());
          return collideBoxBoxDART(box0->getSize(), _T0, dimTemp, _T1, _result);
        }
        default:
          return false;

          break;
      }

      break;
    }
    case dynamics::Shape::ELLIPSOID:
    {
      const dynamics::EllipsoidShape* ellipsoid0 = static_cast<const dynamics::EllipsoidShape*>(_shape0.get());

      switch(RightType)
      {
        case dynamics::Shape::BOX:
        {
          const dynamics::BoxShape* box1 = static_cast<const dynamics::BoxShape*>(_shape1.get());
          return collideSphereBoxDART(ellipsoid0->getSize()[0] * 0.5, _T0,
                                  box1->getSize(), _T1,
                                  _result);
        }
        case dynamics::Shape::ELLIPSOID:
        {
          const dynamics::EllipsoidShape* ellipsoid1 = static_cast<const dynamics::EllipsoidShape*>(_shape1.get());
          return collideSphereSphereDART(ellipsoid0->getSize()[0] * 0.5, _T0,
                                     ellipsoid1->getSize()[0] * 0.5, _T1,
                                     _result);
        }
        case dynamics::Shape::CYLINDER:
        {
          //----------------------------------------------------------
          // NOT SUPPORT CYLINDER
          //----------------------------------------------------------
          const dynamics::CylinderShape* cylinder1 = static_cast<const dynamics::CylinderShape*>(_shape1.get());

          Eigen::Vector3d dimTemp1(cylinder1->getRadius() * sqrt(2.0),
                                   cylinder1->getRadius() * sqrt(2.0),
                                   cylinder1->getHeight());
          return collideSphereBoxDART(
                ellipsoid0->getSize()[0] * 0.5, _T0, dimTemp1, _T1, _result);
        }
        default:
          return false;

          break;
      }

      break;
    }
    case dynamics::Shape::CYLINDER:
    {
      //----------------------------------------------------------
      // NOT SUPPORT CYLINDER
      //----------------------------------------------------------
      const dynamics::CylinderShape* cylinder0 = static_cast<const dynamics::CylinderShape*>(_shape0.get());

      Eigen::Vector3d dimTemp0(cylinder0->getRadius() * sqrt(2.0),
                               cylinder0->getRadius() * sqrt(2.0),
                               cylinder0->getHeight());
      switch(RightType)
      {
        case dynamics::Shape::BOX:
        {
          const dynamics::BoxShape* box1 = static_cast<const dynamics::BoxShape*>(_shape1.get());
          return collideBoxBoxDART(dimTemp0, _T0, box1->getSize(), _T1, _result);
        }
        case dynamics::Shape::ELLIPSOID:
        {
          const dynamics::EllipsoidShape* ellipsoid1 = static_cast<const dynamics::EllipsoidShape*>(_shape1.get());
          return collideBoxSphereDART(dimTemp0, _T0, ellipsoid1->getSize()[0] * 0.5, _T1, _result);
        }
        case dynamics::Shape::CYLINDER:
        {
          //----------------------------------------------------------
          // NOT SUPPORT CYLINDER
          //----------------------------------------------------------
          const dynamics::CylinderShape* cylinder1 = static_cast<const dynamics::CylinderShape*>(_shape1.get());

          Eigen::Vector3d dimTemp1(cylinder1->getRadius() * sqrt(2.0),
                                   cylinder1->getRadius() * sqrt(2.0),
                                   cylinder1->getHeight());
          return collideBoxBoxDART(dimTemp0, _T0, dimTemp1, _T1, _result);
        }
        default:
        {
          return false;
        }
      }

      break;
    }
    default:
      return false;

      break;
  }
}

} // namespace collision
} // namespace dart

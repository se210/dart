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

#ifndef DART_COLLISION_DART_CCDCOLLISION_H_
#define DART_COLLISION_DART_CCDCOLLISION_H_

#include <vector>
#include <functional>

#include <Eigen/Dense>

#include <ccd/ccd.h>
#include <ccd/quat.h>

#include "dart/common/common.h"
#include "dart/math/math.h"
#include "dart/collision/dart/NarrowPhase.h"
#include "dart/collision/dart/DARTCollisionDetector.h"

namespace dart {
namespace collision {

//==============================================================================
template <typename ShapeT>
class CCDConvexShapeUtils
{
public:
  static void* createCCDObject(const ShapeT& _shape, const Eigen::Isometry3d&)
  {
    dterr << "Attempt to create unsupported CCD object ["
          << _shape.getShapeType() << "]." << std::endl;
    return nullptr;
  }

  static void destroyCCDObject(void* _obj) {}

  static ccd_center_fn getCenterFunction()
  {
    dterr << "Attempt to get unsupported CCD center function." << std::endl;
    return nullptr;
  }

  static ccd_support_fn getSupportFunction()
  {
    dterr << "Attempt to get unsupported CCD support function." << std::endl;
    return nullptr;
  }
};

//==============================================================================
template <>
class CCDConvexShapeUtils<dynamics::BoxShape>
{
public:
  // Documentation inherited.
  static void* createCCDObject(const dynamics::BoxShape&,
                               const Eigen::Isometry3d&);

  // Documentation inherited.
  static void destroyCCDObject(void* _obj);

  // Documentation inherited.
  static ccd_center_fn getCenterFunction();

  // Documentation inherited.
  static ccd_support_fn getSupportFunction();
};

//==============================================================================
//template <>
//class CCDConvexShapeUtils<dynamics::SphereShape>
//{
//public:
//  // Documentation inherited.
//  static void* createCCDObject(const dynamics::SphereShape&,
//                               const Eigen::Isometry3d&);

//  // Documentation inherited.
//  static void destroyCCDObject(void* _obj);

//  // Documentation inherited.
//  static ccd_center_fn getCenterFunction();

//  // Documentation inherited.
//  static ccd_support_fn getSupportFunction();
//};

//==============================================================================
template <>
class CCDConvexShapeUtils<dynamics::EllipsoidShape>
{
public:
  // Documentation inherited.
  static void* createCCDObject(const dynamics::EllipsoidShape&,
                               const Eigen::Isometry3d&);

  // Documentation inherited.
  static void destroyCCDObject(void* _obj);

  // Documentation inherited.
  static ccd_center_fn getCenterFunction();

  // Documentation inherited.
  static ccd_support_fn getSupportFunction();
};

//==============================================================================
///
//==============================================================================
//template <>
//class CCDConvexShapeUtils<dynamics::CapsuleShape>
//{
//public:
//  // Documentation inherited.
//  static void* createCCDObject(const dynamics::CapsuleShape&,
//                               const Eigen::Isometry3d&);

//  // Documentation inherited.
//  static void destroyCCDObject(void* _obj);

//  // Documentation inherited.
//  static ccd_center_fn getCenterFunction();

//  // Documentation inherited.
//  static ccd_support_fn getSupportFunction();
//};

//==============================================================================
///
//==============================================================================
//template <>
//class CCDConvexShapeUtils<dynamics::ConeShape>
//{
//public:
//  // Documentation inherited.
//  static void* createCCDObject(const dynamics::ConeShape&,
//                               const Eigen::Isometry3d&);

//  // Documentation inherited.
//  static void destroyCCDObject(void* _obj);

//  // Documentation inherited.
//  static ccd_center_fn getCenterFunction();

//  // Documentation inherited.
//  static ccd_support_fn getSupportFunction();
//};

//==============================================================================
///
//==============================================================================
template <>
class CCDConvexShapeUtils<dynamics::CylinderShape>
{
public:
  // Documentation inherited.
  static void* createCCDObject(const dynamics::CylinderShape&,
                               const Eigen::Isometry3d&);

  // Documentation inherited.
  static void destroyCCDObject(void* _obj);

  // Documentation inherited.
  static ccd_center_fn getCenterFunction();

  // Documentation inherited.
  static ccd_support_fn getSupportFunction();
};

//==============================================================================
///
//==============================================================================
template <>
class CCDConvexShapeUtils<dynamics::MeshShape>
{
public:
  // Documentation inherited.
  static void* createCCDObject(const dynamics::MeshShape&,
                               const Eigen::Isometry3d&);

  // Documentation inherited.
  static void destroyCCDObject(void* _obj);

  // Documentation inherited.
  static ccd_center_fn getCenterFunction();

  // Documentation inherited.
  static ccd_support_fn getSupportFunction();
};

//==============================================================================
///
//==============================================================================
class CCDAlgorithmSet
{
public:
  template <typename S1, typename S2>
  static size_t collide(
      const S1& _geom1,
      const Eigen::Isometry3d& _tf1,
      const S2& _geom2,
      const Eigen::Isometry3d& _tf2,
      const CollisionOptions& _options,
      CollisionResult& _result)
  {
    void* o1 = CCDConvexShapeUtils<S1>::createCCDObject(_geom1, _tf1);
    void* o2 = CCDConvexShapeUtils<S2>::createCCDObject(_geom2, _tf2);

    ccd_support_fn supp1 = CCDConvexShapeUtils<S1>::getSupportFunction();
    ccd_support_fn supp2 = CCDConvexShapeUtils<S2>::getSupportFunction();

    assert(supp1 != nullptr);
    assert(supp2 != nullptr);

    ccd_center_fn cen1 = CCDConvexShapeUtils<S1>::getCenterFunction();
    ccd_center_fn cen2 = CCDConvexShapeUtils<S2>::getCenterFunction();

    assert(cen1 != nullptr);
    assert(cen2 != nullptr);

    Eigen::Vector3d pos;
    Eigen::Vector3d normal;
    double penetration;

    bool res = collideCCD(o1, supp1, cen1, o2, supp2, cen2,
                          pos, normal, penetration);

    if (res)
    {
      Contact contact;

      contact.point = pos;
      contact.normal = normal;
      contact.penetrationDepth = penetration;

      _result.removeAllContacts();
      _result.addContact(contact);
    }

    CCDConvexShapeUtils<S1>::destroyCCDObject(o1);
    CCDConvexShapeUtils<S2>::destroyCCDObject(o2);

    return res;
  }

private:
  /// Return true if the two objects are colliding with contact infomation:
  /// point, normal, penetration.
  static bool collideCCD(
      void *obj1, ccd_support_fn supp1, ccd_center_fn cen1,
      void *obj2, ccd_support_fn supp2, ccd_center_fn cen2,
      Eigen::Vector3d& _point,
      Eigen::Vector3d& _normal,
      double& penetration);

  /// Return true if the two objects are colliding
  static bool collideCCD(void *obj1, ccd_support_fn supp1, ccd_center_fn cen1,
                         void *obj2, ccd_support_fn supp2, ccd_center_fn cen2);

  static double mTolerance;
  static size_t mMaxIteration;

};

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_DART_CCDCOLLISION_H_

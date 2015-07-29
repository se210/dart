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

#include "dart/collision/dart/NarrowPhase.h"

#include <array>
#include <vector>

#include "dart/common/Console.h"
#include "dart/dynamics/Shape.h"
#include "dart/dynamics/BoxShape.h"
#include "dart/dynamics/CylinderShape.h"
#include "dart/dynamics/EllipsoidShape.h"
#include "dart/dynamics/PlaneShape.h"
#include "dart/dynamics/MeshShape.h"
#include "dart/dynamics/SoftMeshShape.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/collision/dart/DARTCollision.h"
#include "dart/collision/dart/CCDCollision.h"

namespace dart {
namespace collision {

//==============================================================================
CollisionOptions::CollisionOptions()
{
}

//==============================================================================
CollisionOptions::~CollisionOptions()
{
}

//==============================================================================
CollisionResult::CollisionResult()
{
}

//==============================================================================
CollisionResult::~CollisionResult()
{
}

//==============================================================================
void CollisionResult::addContact(const Contact& _contact)
{
  mContacts.push_back(_contact);
}

//==============================================================================
const Contact& CollisionResult::getContact(size_t _index)
{
  return mContacts[_index];
}

//==============================================================================
void CollisionResult::removeAllContacts()
{
  mContacts.clear();
}

//==============================================================================
std::size_t CollisionResult::getNumContacts()
{
  return mContacts.size();
}

//==============================================================================
using CollisionFunction = std::function<
    size_t(const dynamics::Shape* _geom1,
           const Eigen::Isometry3d& _tf1,
           const dynamics::Shape* _geom2,
           const Eigen::Isometry3d& _tf2,
           const CollisionOptions& _options,
           CollisionResult& _result)>;

//==============================================================================
///
class CollisionFunctionMatrix
    : public common::Singleton<CollisionFunctionMatrix>
{
public:

  friend class Singleton;

  // TODO: setCollisionFunction();

  const CollisionFunction& getCollisionFunction(
      dynamics::Shape::ShapeType _geom1,
      dynamics::Shape::ShapeType _geom2) const;

protected:

  CollisionFunctionMatrix();
  ~CollisionFunctionMatrix();

private:
  std::array<
      std::array<CollisionFunction, dynamics::Shape::COUNT>,
          dynamics::Shape::COUNT> mCollisionMatrix;
};

//==============================================================================
const CollisionFunction& CollisionFunctionMatrix::getCollisionFunction(
    dynamics::Shape::ShapeType _geom1, dynamics::Shape::ShapeType _geom2) const
{
  return mCollisionMatrix[_geom1][_geom2];
}

//==============================================================================
template <typename S1, typename S2, typename NarrowPhaseAlgorithmSet>
std::size_t collideShapeShape(const dynamics::Shape* shape1,
                              const Eigen::Isometry3d& tf1,
                              const dynamics::Shape* shape2,
                              const Eigen::Isometry3d& tf2,
                              const CollisionOptions& options,
                              CollisionResult& result)
{
  assert(shape1 != nullptr);
  assert(shape2 != nullptr);

  return NarrowPhaseAlgorithmSet::collide(
        *static_cast<const S1*>(shape1), tf1,
        *static_cast<const S2*>(shape2), tf2,
        options, result);
}

//==============================================================================
CollisionFunctionMatrix::CollisionFunctionMatrix()
{
  // Fill all the element with nullptr
  for (auto& collFuncArray : mCollisionMatrix)
    collFuncArray.fill(nullptr);

  // Assign collision functions
  mCollisionMatrix[dynamics::Shape::BOX      ][dynamics::Shape::BOX      ] = collideShapeShape<dynamics::BoxShape      , dynamics::BoxShape      , DARTCollisionAlgorithmSet>;
//  mCollisionMatrix[dynamics::Shape::BOX      ][dynamics::Shape::SPHERE   ] = collideShapeShape<dynamics::BoxShape      , dynamics::SphereShape   , CCDAlgorithmSet>;
  mCollisionMatrix[dynamics::Shape::BOX      ][dynamics::Shape::ELLIPSOID] = collideShapeShape<dynamics::BoxShape      , dynamics::EllipsoidShape, CCDAlgorithmSet>;
  mCollisionMatrix[dynamics::Shape::BOX      ][dynamics::Shape::CYLINDER ] = collideShapeShape<dynamics::BoxShape      , dynamics::CylinderShape , CCDAlgorithmSet>;
  mCollisionMatrix[dynamics::Shape::BOX      ][dynamics::Shape::MESH     ] = collideShapeShape<dynamics::BoxShape      , dynamics::MeshShape     , CCDAlgorithmSet>;

//  mCollisionMatrix[dynamics::Shape::SPHERE   ][dynamics::Shape::BOX      ] = collideShapeShape<dynamics::SphereShape   , dynamics::BoxShape      , CCDAlgorithmSet>;
//  mCollisionMatrix[dynamics::Shape::SPHERE   ][dynamics::Shape::SPHERE   ] = collideShapeShape<dynamics::SphereShape   , dynamics::SphereShape   , CCDAlgorithmSet>;
//  mCollisionMatrix[dynamics::Shape::SPHERE   ][dynamics::Shape::ELLIPSOID] = collideShapeShape<dynamics::SphereShape   , dynamics::EllipsoidShape, CCDAlgorithmSet>;
//  mCollisionMatrix[dynamics::Shape::SPHERE   ][dynamics::Shape::CYLINDER ] = collideShapeShape<dynamics::SphereShape   , dynamics::CylinderShape , CCDAlgorithmSet>;
//  mCollisionMatrix[dynamics::Shape::SPHERE   ][dynamics::Shape::MESH     ] = collideShapeShape<dynamics::SphereShape   , dynamics::MeshShape     , CCDAlgorithmSet>;

  mCollisionMatrix[dynamics::Shape::ELLIPSOID][dynamics::Shape::BOX      ] = collideShapeShape<dynamics::EllipsoidShape, dynamics::BoxShape      , CCDAlgorithmSet>;
//  mCollisionMatrix[dynamics::Shape::ELLIPSOID][dynamics::Shape::SPHERE   ] = collideShapeShape<dynamics::EllipsoidShape, dynamics::SphereShape   , CCDAlgorithmSet>;
  mCollisionMatrix[dynamics::Shape::ELLIPSOID][dynamics::Shape::ELLIPSOID] = collideShapeShape<dynamics::EllipsoidShape, dynamics::EllipsoidShape, CCDAlgorithmSet>;
  mCollisionMatrix[dynamics::Shape::ELLIPSOID][dynamics::Shape::CYLINDER ] = collideShapeShape<dynamics::EllipsoidShape, dynamics::CylinderShape , CCDAlgorithmSet>;
  mCollisionMatrix[dynamics::Shape::ELLIPSOID][dynamics::Shape::MESH     ] = collideShapeShape<dynamics::EllipsoidShape, dynamics::MeshShape     , CCDAlgorithmSet>;

  mCollisionMatrix[dynamics::Shape::CYLINDER ][dynamics::Shape::BOX      ] = collideShapeShape<dynamics::CylinderShape , dynamics::BoxShape      , CCDAlgorithmSet>;
//  mCollisionMatrix[dynamics::Shape::CYLINDER ][dynamics::Shape::SPHERE   ] = collideShapeShape<dynamics::CylinderShape , dynamics::SphereShape   , CCDAlgorithmSet>;
  mCollisionMatrix[dynamics::Shape::CYLINDER ][dynamics::Shape::ELLIPSOID] = collideShapeShape<dynamics::CylinderShape , dynamics::EllipsoidShape, CCDAlgorithmSet>;
  mCollisionMatrix[dynamics::Shape::CYLINDER ][dynamics::Shape::CYLINDER ] = collideShapeShape<dynamics::CylinderShape , dynamics::CylinderShape , CCDAlgorithmSet>;
  mCollisionMatrix[dynamics::Shape::CYLINDER ][dynamics::Shape::MESH     ] = collideShapeShape<dynamics::CylinderShape , dynamics::MeshShape     , CCDAlgorithmSet>;

  mCollisionMatrix[dynamics::Shape::MESH     ][dynamics::Shape::BOX      ] = collideShapeShape<dynamics::MeshShape     , dynamics::BoxShape      , CCDAlgorithmSet>;
//  mCollisionMatrix[dynamics::Shape::MESH     ][dynamics::Shape::SPHERE   ] = collideShapeShape<dynamics::MeshShape     , dynamics::SphereShape   , CCDAlgorithmSet>;
  mCollisionMatrix[dynamics::Shape::MESH     ][dynamics::Shape::ELLIPSOID] = collideShapeShape<dynamics::MeshShape     , dynamics::EllipsoidShape, CCDAlgorithmSet>;
  mCollisionMatrix[dynamics::Shape::MESH     ][dynamics::Shape::CYLINDER ] = collideShapeShape<dynamics::MeshShape     , dynamics::CylinderShape , CCDAlgorithmSet>;
  mCollisionMatrix[dynamics::Shape::MESH     ][dynamics::Shape::MESH     ] = collideShapeShape<dynamics::MeshShape     , dynamics::MeshShape     , CCDAlgorithmSet>;
}

//==============================================================================
CollisionFunctionMatrix::~CollisionFunctionMatrix()
{

}

//==============================================================================
std::size_t NarrowPhaseAlgorithm::collide(const dynamics::Shape* shape1,
                                          const Eigen::Isometry3d& tf1,
                                          const dynamics::Shape* shape2,
                                          const Eigen::Isometry3d& tf2,
                                          const CollisionOptions& options,
                                          CollisionResult& result)
{
  result.removeAllContacts();

  const dynamics::Shape::ShapeType& type1 = shape1->getShapeType();
  const dynamics::Shape::ShapeType& type2 = shape2->getShapeType();

  const CollisionFunction& func
      = CollisionFunctionMatrix::getInstance().getCollisionFunction(
          type1, type2);

  if (func == nullptr)
  {
    dtwarn << "Unsupported collision shapes. "
           << "Collide function is null pointer." << std::endl;
    return 0;
  }

  func(shape1, tf1, shape2, tf2, options, result);

  return result.getNumContacts();
}

}  // namespace collision
}  // namespace dart

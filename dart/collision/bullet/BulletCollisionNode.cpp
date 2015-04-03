/*
 * Copyright (c) 2014-2015, Georgia Tech Research Corporation
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

#include "dart/collision/bullet/BulletCollisionNode.h"

#include <iostream>

#include <assimp/scene.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>

#include "dart/common/Console.h"
#include "dart/collision/bullet/BulletTypes.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Shape.h"
#include "dart/dynamics/BoxShape.h"
#include "dart/dynamics/EllipsoidShape.h"
#include "dart/dynamics/CylinderShape.h"
#include "dart/dynamics/PlaneShape.h"
#include "dart/dynamics/MeshShape.h"

namespace dart {
namespace collision {

//==============================================================================
void addBulletCollObj(dynamics::BodyNode* _bodyNode,
                      dynamics::Shape* _shape,
                      BulletCollisionNode* _collNode,
                      std::vector<btCollisionObject*>& _btCollObjs,
                      btCollisionShape* btCollShape)
{
  btCollisionObject* btCollObj = new btCollisionObject();
  btCollObj->setCollisionShape(btCollShape);

  BulletUserData* userData = new BulletUserData;
  userData->bodyNode   = _bodyNode;
  userData->shape      = _shape;
  userData->btCollNode = _collNode;
  btCollObj->setUserPointer(userData);

  _btCollObjs.push_back(btCollObj);
}

//==============================================================================
btGImpactMeshShape* createMesh(const Eigen::Vector3d& _scale,
                               const aiScene* _mesh)
{
  btTriangleMesh* btMesh = new btTriangleMesh();

  for (size_t i = 0; i < _mesh->mNumMeshes; i++)
  {
    const aiMesh* aMesh = _mesh->mMeshes[i];
    for (size_t j = 0; j < aMesh->mNumFaces; j++)
    {
      const aiFace aFace = aMesh->mFaces[j];
      btVector3 vertices[3];
      for (size_t k = 0; k < 3; k++)
      {
        const aiVector3D& vertex = aMesh->mVertices[aFace.mIndices[k]];
        vertices[k] = btVector3(vertex.x * _scale[0],
                                vertex.y * _scale[1],
                                vertex.z * _scale[2]);
      }
      btMesh->addTriangle(vertices[0], vertices[1], vertices[2]);
    }
  }

  btGImpactMeshShape* btMeshShape = new btGImpactMeshShape(btMesh);
  btMeshShape->updateBound();

  return btMeshShape;
}

//==============================================================================
BulletCollisionNode::BulletCollisionNode(dynamics::BodyNode* _bodyNode)
  : CollisionNode(_bodyNode)
{
  using dynamics::Shape;
  using dynamics::BoxShape;
  using dynamics::EllipsoidShape;
  using dynamics::CylinderShape;
  using dynamics::PlaneShape;
  using dynamics::MeshShape;

  for (size_t i = 0; i < _bodyNode->getNumCollisionShapes(); i++)
  {
    Shape* shape = _bodyNode->getCollisionShape(i);
    btCollisionShape* btCollShape;

    switch (shape->getShapeType())
    {
      case Shape::BOX:
      {
        BoxShape* box = static_cast<BoxShape*>(shape);

        btCollShape = new btBoxShape(convertVector3(0.5*box->getSize()));
        break;
      }
      case Shape::ELLIPSOID:
      {
        EllipsoidShape* ellipsoid = static_cast<EllipsoidShape*>(shape);

        if (ellipsoid->isSphere())
        {
          const double radius = 0.5*ellipsoid->getSize()[0];
          btCollShape = new btSphereShape(radius);
        }
        else
        {
          const btVector3 position(0, 0, 0);
          const btScalar  radius = 1.0;
          btMultiSphereShape* btEllisoid
              = new btMultiSphereShape(&position, &radius, 1);
          btEllisoid->setLocalScaling(convertVector3(0.5*ellipsoid->getSize()));
          btCollShape = btEllisoid;
        }

        break;
      }
      case Shape::CYLINDER:
      {
        CylinderShape* cylinder = static_cast<CylinderShape*>(shape);

        btCollShape = new btCylinderShapeZ(btVector3(cylinder->getRadius(),
                                                     cylinder->getRadius(),
                                                     cylinder->getHeight() * 0.5));
        break;
      }
      case Shape::PLANE:
      {
        PlaneShape* plane = static_cast<PlaneShape*>(shape);

        btScalar d = plane->getNormal().dot(plane->getPoint())
                   / plane->getNormal().squaredNorm();

        btCollShape = new btStaticPlaneShape(convertVector3(plane->getNormal()),
                                             d);

        break;
      }
      case Shape::MESH:
      {
        MeshShape* shapeMesh = static_cast<MeshShape*>(shape);
        btCollShape = createMesh(shapeMesh->getScale(), shapeMesh->getMesh());

        break;
      }
      default:
      {
        dterr << "[BulletCollisionNode::BulletCollisionNode] Attempting to "
              << "create unsupported shape type '" << shape->getShapeType()
              << "' of '" << _bodyNode->getName() << "' body node."
              <<  std::endl;
        btCollShape = NULL;
        break;
      }
    }

    if (NULL == btCollShape)
      continue;

    addBulletCollObj(_bodyNode, shape, this, mbtCollsionObjects, btCollShape);
  }
}

//==============================================================================
BulletCollisionNode::~BulletCollisionNode()
{
}

//==============================================================================
void BulletCollisionNode::updateBulletCollisionObjects()
{
  for (size_t i = 0; i < mbtCollsionObjects.size(); ++i)
  {
    BulletUserData* userData =
        static_cast<BulletUserData*>(mbtCollsionObjects[i]->getUserPointer());
    dynamics::Shape* shape = userData->shape;
    btTransform T = convertTransform(mBodyNode->getTransform() *
                                     shape->getLocalTransform());
    mbtCollsionObjects[i]->setWorldTransform(T);
  }
}

//==============================================================================
int BulletCollisionNode::getNumBulletCollisionObjects() const
{
  return mbtCollsionObjects.size();
}

//==============================================================================
btCollisionObject*BulletCollisionNode::getBulletCollisionObject(int _i)
{
  return mbtCollsionObjects[_i];
}

}  // namespace collision
}  // namespace dart

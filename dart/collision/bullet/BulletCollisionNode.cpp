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
#include "dart/dynamics/BoxShape.h"
#include "dart/dynamics/EllipsoidShape.h"
#include "dart/dynamics/CylinderShape.h"
#include "dart/dynamics/PlaneShape.h"
#include "dart/dynamics/MeshShape.h"

namespace dart {
namespace collision {

//==============================================================================
void addBulletCollObj(dynamics::BodyNode* _bodyNode,
                      const dynamics::ShapePtr& _shape,
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
btGImpactMeshShape* createEllipsoid(
    float _sizeX, float _sizeY, float _sizeZ,
    const Eigen::Isometry3d& _transform)
{
  float v[59][3] = {
    {0, 0, 0},
    {0.135299, -0.461940, -0.135299},
    {0.000000, -0.461940, -0.191342},
    {-0.135299, -0.461940, -0.135299},
    {-0.191342, -0.461940, 0.000000},
    {-0.135299, -0.461940, 0.135299},
    {0.000000, -0.461940, 0.191342},
    {0.135299, -0.461940, 0.135299},
    {0.191342, -0.461940, 0.000000},
    {0.250000, -0.353553, -0.250000},
    {0.000000, -0.353553, -0.353553},
    {-0.250000, -0.353553, -0.250000},
    {-0.353553, -0.353553, 0.000000},
    {-0.250000, -0.353553, 0.250000},
    {0.000000, -0.353553, 0.353553},
    {0.250000, -0.353553, 0.250000},
    {0.353553, -0.353553, 0.000000},
    {0.326641, -0.191342, -0.326641},
    {0.000000, -0.191342, -0.461940},
    {-0.326641, -0.191342, -0.326641},
    {-0.461940, -0.191342, 0.000000},
    {-0.326641, -0.191342, 0.326641},
    {0.000000, -0.191342, 0.461940},
    {0.326641, -0.191342, 0.326641},
    {0.461940, -0.191342, 0.000000},
    {0.353553, 0.000000, -0.353553},
    {0.000000, 0.000000, -0.500000},
    {-0.353553, 0.000000, -0.353553},
    {-0.500000, 0.000000, 0.000000},
    {-0.353553, 0.000000, 0.353553},
    {0.000000, 0.000000, 0.500000},
    {0.353553, 0.000000, 0.353553},
    {0.500000, 0.000000, 0.000000},
    {0.326641, 0.191342, -0.326641},
    {0.000000, 0.191342, -0.461940},
    {-0.326641, 0.191342, -0.326641},
    {-0.461940, 0.191342, 0.000000},
    {-0.326641, 0.191342, 0.326641},
    {0.000000, 0.191342, 0.461940},
    {0.326641, 0.191342, 0.326641},
    {0.461940, 0.191342, 0.000000},
    {0.250000, 0.353553, -0.250000},
    {0.000000, 0.353553, -0.353553},
    {-0.250000, 0.353553, -0.250000},
    {-0.353553, 0.353553, 0.000000},
    {-0.250000, 0.353553, 0.250000},
    {0.000000, 0.353553, 0.353553},
    {0.250000, 0.353553, 0.250000},
    {0.353553, 0.353553, 0.000000},
    {0.135299, 0.461940, -0.135299},
    {0.000000, 0.461940, -0.191342},
    {-0.135299, 0.461940, -0.135299},
    {-0.191342, 0.461940, 0.000000},
    {-0.135299, 0.461940, 0.135299},
    {0.000000, 0.461940, 0.191342},
    {0.135299, 0.461940, 0.135299},
    {0.191342, 0.461940, 0.000000},
    {0.000000, -0.500000, 0.000000},
    {0.000000, 0.500000, 0.000000}
  };
  int f[112][3] = {
    {1, 2, 9},
    {9, 2, 10},
    {2, 3, 10},
    {10, 3, 11},
    {3, 4, 11},
    {11, 4, 12},
    {4, 5, 12},
    {12, 5, 13},
    {5, 6, 13},
    {13, 6, 14},
    {6, 7, 14},
    {14, 7, 15},
    {7, 8, 15},
    {15, 8, 16},
    {8, 1, 16},
    {16, 1, 9},
    {9, 10, 17},
    {17, 10, 18},
    {10, 11, 18},
    {18, 11, 19},
    {11, 12, 19},
    {19, 12, 20},
    {12, 13, 20},
    {20, 13, 21},
    {13, 14, 21},
    {21, 14, 22},
    {14, 15, 22},
    {22, 15, 23},
    {15, 16, 23},
    {23, 16, 24},
    {16, 9, 24},
    {24, 9, 17},
    {17, 18, 25},
    {25, 18, 26},
    {18, 19, 26},
    {26, 19, 27},
    {19, 20, 27},
    {27, 20, 28},
    {20, 21, 28},
    {28, 21, 29},
    {21, 22, 29},
    {29, 22, 30},
    {22, 23, 30},
    {30, 23, 31},
    {23, 24, 31},
    {31, 24, 32},
    {24, 17, 32},
    {32, 17, 25},
    {25, 26, 33},
    {33, 26, 34},
    {26, 27, 34},
    {34, 27, 35},
    {27, 28, 35},
    {35, 28, 36},
    {28, 29, 36},
    {36, 29, 37},
    {29, 30, 37},
    {37, 30, 38},
    {30, 31, 38},
    {38, 31, 39},
    {31, 32, 39},
    {39, 32, 40},
    {32, 25, 40},
    {40, 25, 33},
    {33, 34, 41},
    {41, 34, 42},
    {34, 35, 42},
    {42, 35, 43},
    {35, 36, 43},
    {43, 36, 44},
    {36, 37, 44},
    {44, 37, 45},
    {37, 38, 45},
    {45, 38, 46},
    {38, 39, 46},
    {46, 39, 47},
    {39, 40, 47},
    {47, 40, 48},
    {40, 33, 48},
    {48, 33, 41},
    {41, 42, 49},
    {49, 42, 50},
    {42, 43, 50},
    {50, 43, 51},
    {43, 44, 51},
    {51, 44, 52},
    {44, 45, 52},
    {52, 45, 53},
    {45, 46, 53},
    {53, 46, 54},
    {46, 47, 54},
    {54, 47, 55},
    {47, 48, 55},
    {55, 48, 56},
    {48, 41, 56},
    {56, 41, 49},
    {2, 1, 57},
    {3, 2, 57},
    {4, 3, 57},
    {5, 4, 57},
    {6, 5, 57},
    {7, 6, 57},
    {8, 7, 57},
    {1, 8, 57},
    {49, 50, 58},
    {50, 51, 58},
    {51, 52, 58},
    {52, 53, 58},
    {53, 54, 58},
    {54, 55, 58},
    {55, 56, 58},
    {56, 49, 58}
  };

  btTriangleMesh* btMesh = new btTriangleMesh();

  for (int i = 0; i < 112; i++)
  {
    btVector3 p1 = btVector3(v[f[i][0]][0] * _sizeX,
                             v[f[i][0]][1] * _sizeY,
                             v[f[i][0]][2] * _sizeZ);
    btVector3 p2 = btVector3(v[f[i][1]][0] * _sizeX,
                             v[f[i][1]][1] * _sizeY,
                             v[f[i][1]][2] * _sizeZ);
    btVector3 p3 = btVector3(v[f[i][2]][0] * _sizeX,
                             v[f[i][2]][1] * _sizeY,
                             v[f[i][2]][2] * _sizeZ);

    btTransform T = convertTransform(_transform);

    p1 = T * p1;
    p2 = T * p2;
    p3 = T * p3;

    btMesh->addTriangle(p1, p2, p3);
  }

  btGImpactMeshShape* btMeshShape = new btGImpactMeshShape(btMesh);
  btMeshShape->updateBound();

  return btMeshShape;
//  return new btConvexTriangleMeshShape(btMesh);
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
    dynamics::ShapePtr shape = _bodyNode->getCollisionShape(i);
    btCollisionShape* btCollShape;

    switch (shape->getShapeType())
    {
      case Shape::BOX:
      {
        BoxShape* box = static_cast<BoxShape*>(shape.get());

        btCollShape = new btBoxShape(convertVector3(0.5*box->getSize()));
        break;
      }
      case Shape::ELLIPSOID:
      {
        EllipsoidShape* ellipsoid = static_cast<EllipsoidShape*>(shape.get());

        if (ellipsoid->isSphere())
        {
          const double radius = 0.5 * ellipsoid->getSize()[0];
          btCollShape = new btSphereShape(radius);
        }
        else
        {
          btCollShape = createEllipsoid(1.0 * ellipsoid->getSize()[0],
                                        1.0 * ellipsoid->getSize()[1],
                                        1.0 * ellipsoid->getSize()[2],
                                        Eigen::Isometry3d::Identity());
        }

        break;
      }
      case Shape::CYLINDER:
      {
        CylinderShape* cylinder = static_cast<CylinderShape*>(shape.get());

        btCollShape = new btCylinderShapeZ(btVector3(cylinder->getRadius(),
                                                     cylinder->getRadius(),
                                                     cylinder->getHeight() * 0.5));
        break;
      }
      case Shape::PLANE:
      {
        PlaneShape* plane = static_cast<PlaneShape*>(shape.get());

        btCollShape = new btStaticPlaneShape(convertVector3(plane->getNormal()),
                                             plane->getOffset());
        break;
      }
      case Shape::MESH:
      {
        MeshShape* shapeMesh = static_cast<MeshShape*>(shape.get());
        btCollShape = createMesh(shapeMesh->getScale(), shapeMesh->getMesh());

        break;
      }
      default:
      {
        dterr << "[BulletCollisionNode::BulletCollisionNode] Attempting to "
              << "create unsupported shape type '" << shape->getShapeType()
              << "' of '" << _bodyNode->getName() << "' body node."
              <<  std::endl;
        btCollShape = nullptr;
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
    dynamics::ConstShapePtr shape = userData->shape;
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
btCollisionObject* BulletCollisionNode::getBulletCollisionObject(int _i)
{
  return mbtCollsionObjects[_i];
}

}  // namespace collision
}  // namespace dart

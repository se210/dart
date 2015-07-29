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

#include "dart/collision/dart/CCDCollision.h"

#include "dart/dynamics/Shape.h"
#include "dart/dynamics/BoxShape.h"
#include "dart/dynamics/CylinderShape.h"
#include "dart/dynamics/EllipsoidShape.h"
#include "dart/dynamics/PlaneShape.h"
#include "dart/dynamics/MeshShape.h"
#include "dart/dynamics/SoftMeshShape.h"

#include <memory>

namespace dart {
namespace collision {

///
struct ccd_obj_t
{
  ccd_vec3_t pos;
  ccd_quat_t rot, rot_inv;
};

///
struct ccd_box_t : public ccd_obj_t
{
  ccd_real_t dim[3];
};

///
struct ccd_sphere_t : public ccd_obj_t
{
  ccd_real_t radius;
};

///
struct ccd_ellip_t : public ccd_obj_t
{
  ccd_real_t halfSize[3];
};

///
struct ccd_cap_t : public ccd_obj_t
{
  ccd_real_t radius, height;
};

///
struct ccd_cone_t : public ccd_obj_t
{
  ccd_real_t radius, height;
};

///
struct ccd_cyl_t : public ccd_obj_t
{
  ccd_real_t radius, height;
};

struct ccd_convex_t : public ccd_obj_t
{
  const dynamics::MeshShape* convex;
};

struct ccd_triangle_t : public ccd_obj_t
{
  ccd_vec3_t p[3];
  ccd_vec3_t c;
};

//==============================================================================
void convTransform(const Eigen::Isometry3d& tf, ccd_obj_t* o)
{
  const Eigen::Quaterniond dt_rot(tf.rotation());
  const Eigen::Vector3d& dt_pos = tf.translation();

  ccdVec3Set(&o->pos, dt_pos[0], dt_pos[1], dt_pos[2]);
  ccdQuatSet(&o->rot, dt_rot.x(), dt_rot.y(), dt_rot.z(), dt_rot.w());

  ccdQuatInvert2(&o->rot_inv, &o->rot);
}

//==============================================================================
void convBox(const dynamics::BoxShape& s,
             const Eigen::Isometry3d& tf,
             ccd_box_t* box)
{
  convTransform(tf, box);
  box->dim[0] = s.getSize()[0] * 0.5;
  box->dim[1] = s.getSize()[1] * 0.5;
  box->dim[2] = s.getSize()[2] * 0.5;
}

//==============================================================================
//void convSphere(const dynamics::SphereShape& s,
//                const Eigen::Isometry3d& tf,
//                ccd_sphere_t* sph)
//{
//  convTransform(tf, sph);
//  sph->radius = s.getRadius();
//}

//==============================================================================
void convEllipsoid(const dynamics::EllipsoidShape& s,
                   const Eigen::Isometry3d& tf,
                   ccd_ellip_t* _ellipsoid)
{
  convTransform(tf, _ellipsoid);
  _ellipsoid->halfSize[0] = s.getSize()[0] * 0.5;
  _ellipsoid->halfSize[1] = s.getSize()[1] * 0.5;
  _ellipsoid->halfSize[2] = s.getSize()[2] * 0.5;
}

//==============================================================================
//void convCapsule(const dynamics::CapsuleShape& s,
//                 const Eigen::Isometry3d& tf,
//                 ccd_cap_t* cap)
//{
//  convTransform(tf, cap);
//  cap->radius = s.getRadius();
//  cap->height = s.getHeight();
//}

//==============================================================================
//void convCone(const dynamics::ConeShape& s,
//              const Eigen::Isometry3d& tf,
//              ccd_cone_t* cone)
//{
//  convTransform(tf, cone);
//  cone->radius = s.getRadius();
//  cone->height = s.getHeight();
//}

//==============================================================================
void convCylinder(const dynamics::CylinderShape& s,
                  const Eigen::Isometry3d& tf,
                  ccd_cyl_t* cyl)
{
  convTransform(tf, cyl);
  cyl->radius = s.getRadius();
  cyl->height = s.getHeight() * 0.5;
}

//==============================================================================
void convConvex(const dynamics::MeshShape& s,
                const Eigen::Isometry3d& tf,
                ccd_convex_t* conv)
{
  convTransform(tf, conv);
  conv->convex = &s;
}

//==============================================================================
void supportBox(const void* obj, const ccd_vec3_t* dir_, ccd_vec3_t* v)
{
  const ccd_box_t* o = static_cast<const ccd_box_t*>(obj);
  ccd_vec3_t dir;
  ccdVec3Copy(&dir, dir_);
  ccdQuatRotVec(&dir, &o->rot_inv);
  ccdVec3Set(v, ccdSign(ccdVec3X(&dir)) * o->dim[0],
                ccdSign(ccdVec3Y(&dir)) * o->dim[1],
                ccdSign(ccdVec3Z(&dir)) * o->dim[2]);
  ccdQuatRotVec(v, &o->rot);
  ccdVec3Add(v, &o->pos);
}

//==============================================================================
void supportSphere(const void* obj, const ccd_vec3_t* dir_, ccd_vec3_t* v)
{
  const ccd_sphere_t* s = static_cast<const ccd_sphere_t*>(obj);
  ccd_vec3_t dir;

  ccdVec3Copy(&dir, dir_);
  ccdQuatRotVec(&dir, &s->rot_inv);

  ccdVec3Copy(v, &dir);                                    // v = dir
  ccdVec3Scale(v, s->radius);                              // v *= s->radius
  ccdVec3Scale(v, CCD_ONE / CCD_SQRT(ccdVec3Len2(&dir)));  // v /= norm(dir)

  // transform support vertex
  ccdQuatRotVec(v, &s->rot);  // v = rot * v
  ccdVec3Add(v, &s->pos);     // v += s->pos
}

//==============================================================================
void supportEllipsoid(const void* obj, const ccd_vec3_t* dir_, ccd_vec3_t* v)
{
  const ccd_ellip_t* s = static_cast<const ccd_ellip_t*>(obj);
  ccd_vec3_t dir;

  ccdVec3Copy(&dir, dir_);
  ccdQuatRotVec(&dir, &s->rot_inv);

  ccd_vec3_t abc2;
  abc2.v[0] = s->halfSize[0] * s->halfSize[0];
  abc2.v[1] = s->halfSize[1] * s->halfSize[1];
  abc2.v[2] = s->halfSize[2] * s->halfSize[2];

  v->v[0] = abc2.v[0] * dir.v[0];
  v->v[1] = abc2.v[1] * dir.v[1];
  v->v[2] = abc2.v[2] * dir.v[2];

  ccdVec3Scale(v, CCD_ONE / CCD_SQRT(ccdVec3Dot(v, &dir)));

  // transform support vertex
  ccdQuatRotVec(v, &s->rot);
  ccdVec3Add(v, &s->pos);
}

//==============================================================================
void supportCone(const void* obj, const ccd_vec3_t* dir_, ccd_vec3_t* v)
{
  const ccd_cone_t* cone = static_cast<const ccd_cone_t*>(obj);
  ccd_vec3_t dir;

  ccdVec3Copy(&dir, dir_);
  ccdQuatRotVec(&dir, &cone->rot_inv);

  double zdist, len, rad;
  zdist = dir.v[0] * dir.v[0] + dir.v[1] * dir.v[1];
  len = zdist + dir.v[2] * dir.v[2];
  zdist = sqrt(zdist);
  len = sqrt(len);

  double sin_a = cone->radius / sqrt(cone->radius * cone->radius + 4 * cone->height * cone->height);

  if(dir.v[2] > len * sin_a)
    ccdVec3Set(v, 0., 0., cone->height);
  else if(zdist > 0)
  {
    rad = cone->radius / zdist;
    ccdVec3Set(v, rad * ccdVec3X(&dir), rad * ccdVec3Y(&dir), -cone->height);
  }
  else
    ccdVec3Set(v, 0, 0, -cone->height);

  // transform support vertex
  ccdQuatRotVec(v, &cone->rot);
  ccdVec3Add(v, &cone->pos);
}

//==============================================================================
void supportCapsule(const void* obj, const ccd_vec3_t* dir_, ccd_vec3_t* v)
{
  const ccd_cap_t* o = static_cast<const ccd_cap_t*>(obj);
  ccd_vec3_t dir, pos1, pos2;

  ccdVec3Copy(&dir, dir_);
  ccdQuatRotVec(&dir, &o->rot_inv);

  ccdVec3Set(&pos1, CCD_ZERO, CCD_ZERO, o->height);
  ccdVec3Set(&pos2, CCD_ZERO, CCD_ZERO, -o->height);

  ccdVec3Copy(v, &dir);
  ccdVec3Scale(v, o->radius);
  ccdVec3Add(&pos1, v);
  ccdVec3Add(&pos2, v);

  if(ccdVec3Dot(&dir, &pos1) > ccdVec3Dot(&dir, &pos2))
    ccdVec3Copy(v, &pos1);
  else
    ccdVec3Copy(v, &pos2);

  // transform support vertex
  ccdQuatRotVec(v, &o->rot);
  ccdVec3Add(v, &o->pos);
}

//==============================================================================
void supportCylinder(const void* obj, const ccd_vec3_t* dir_, ccd_vec3_t* v)
{
  const ccd_cyl_t* cyl = static_cast<const ccd_cyl_t*>(obj);
  ccd_vec3_t dir;
  double zdist, rad;

  ccdVec3Copy(&dir, dir_);
  ccdQuatRotVec(&dir, &cyl->rot_inv);

  zdist = dir.v[0] * dir.v[0] + dir.v[1] * dir.v[1];
  zdist = sqrt(zdist);
  if(ccdIsZero(zdist))
    ccdVec3Set(v, 0., 0., ccdSign(ccdVec3Z(&dir)) * cyl->height);
  else
  {
    rad = cyl->radius / zdist;

    ccdVec3Set(v, rad * ccdVec3X(&dir),
               rad * ccdVec3Y(&dir),
               ccdSign(ccdVec3Z(&dir)) * cyl->height);
  }

  // transform support vertex
  ccdQuatRotVec(v, &cyl->rot);
  ccdVec3Add(v, &cyl->pos);
}

//==============================================================================
void supportConvex(const void* obj, const ccd_vec3_t* dir_, ccd_vec3_t* v)
{
  const ccd_convex_t* c = static_cast<const ccd_convex_t*>(obj);

  const dynamics::MeshShape* convex = c->convex;

  const aiScene*         mesh   = convex->getMesh();
  const Eigen::Vector3d& center = convex->getCenter();
  const Eigen::Vector3d& scale  = convex->getScale();

  ccd_vec3_t dir, p;
  ccd_real_t maxdot = -CCD_REAL_MAX;
  ccd_real_t dot;

  ccdVec3Copy(&dir, dir_);
  ccdQuatRotVec(&dir, &c->rot_inv);

  for (unsigned int i = 0; i < mesh->mNumMeshes; i++)
  {
    for (unsigned int j = 0; j < mesh->mMeshes[i]->mNumVertices; j++)
    {
      const aiVector3D vertex(
          mesh->mMeshes[i]->mVertices[j].x * scale[0],
          mesh->mMeshes[i]->mVertices[j].y * scale[1],
          mesh->mMeshes[i]->mVertices[j].z * scale[2]);

      ccdVec3Set(&p, vertex.x - center[0], vertex.y - center[1],
          vertex.z - center[2]);

      dot = ccdVec3Dot(&dir, &p);

      if (dot > maxdot)
      {
        ccdVec3Set(v, vertex.x, vertex.y, vertex.z);
        maxdot = dot;
      }
    }
  }

  // transform support vertex
  ccdQuatRotVec(v, &c->rot);
  ccdVec3Add(v, &c->pos);
}

//==============================================================================
void centerShape(const void* obj, ccd_vec3_t* c)
{
  const ccd_obj_t *o = static_cast<const ccd_obj_t*>(obj);
  ccdVec3Copy(c, &o->pos);
}

//==============================================================================
void centerConvex(const void* obj, ccd_vec3_t* c)
{
  const ccd_convex_t *o = static_cast<const ccd_convex_t*>(obj);
  const Eigen::Vector3d& center = o->convex->getCenter();
  ccdVec3Set(c, center[0], center[1], center[2]);
  ccdQuatRotVec(c, &o->rot);
  ccdVec3Add(c, &o->pos);
}

//==============================================================================
void centerTriangle(const void* obj, ccd_vec3_t* c)
{
  const ccd_triangle_t *o = static_cast<const ccd_triangle_t*>(obj);
  ccdVec3Copy(c, &o->c);
  ccdQuatRotVec(c, &o->rot);
  ccdVec3Add(c, &o->pos);
}

//==============================================================================
void* CCDConvexShapeUtils<dynamics::BoxShape>::createCCDObject(
    const dynamics::BoxShape& s, const Eigen::Isometry3d& tf)
{
  ccd_box_t* o = new ccd_box_t;
  convBox(s, tf, o);
  return o;
}

//==============================================================================
//void* CCDConvexShapeUtils<dynamics::SphereShape>::createCCDObject(
//    const dynamics::SphereShape& s, const Eigen::Isometry3d& tf)
//{
//  ccd_sphere_t* o = new ccd_sphere_t;
//  convSphere(s, tf, o);
//  return o;
//}

//==============================================================================
void* CCDConvexShapeUtils<dynamics::EllipsoidShape>::createCCDObject(
    const dynamics::EllipsoidShape& s, const Eigen::Isometry3d& tf)
{
  ccd_ellip_t* o = new ccd_ellip_t;
  convEllipsoid(s, tf, o);
  return o;
}

//==============================================================================
void* CCDConvexShapeUtils<dynamics::CylinderShape>::createCCDObject(
    const dynamics::CylinderShape& s, const Eigen::Isometry3d& tf)
{
  ccd_cyl_t* o = new ccd_cyl_t;
  convCylinder(s, tf, o);
  return o;
}

//==============================================================================
//void* CCDConvexShapeUtils<dynamics::CapsuleShape>::createCCDObject(
//    const dynamics::CapsuleShape& s, const Eigen::Isometry3d& tf)
//{
//  ccd_cap_t* o = new ccd_cap_t;
//  convCapsule(s, tf, o);
//  return o;
//}

//==============================================================================
//void* CCDConvexShapeUtils<dynamics::ConeShape>::createCCDObject(
//    const dynamics::ConeShape& s, const Eigen::Isometry3d& tf)
//{
//  ccd_cone_t* o = new ccd_cone_t;
//  convCone(s, tf, o);
//  return o;
//}

//==============================================================================
void* CCDConvexShapeUtils<dynamics::MeshShape>::createCCDObject(
    const dynamics::MeshShape& s, const Eigen::Isometry3d& tf)
{
  ccd_convex_t* o = new ccd_convex_t;
  convConvex(s, tf, o);
  return o;
}

//==============================================================================
void CCDConvexShapeUtils<dynamics::BoxShape>::destroyCCDObject(void* _obj)
{
  ccd_box_t* obj = static_cast<ccd_box_t*>(_obj);
  delete obj;
}

//==============================================================================
//void CCDConvexShapeUtils<dynamics::SphereShape>::destroyCCDObject(void* _obj)
//{
//  ccd_sphere_t* obj = static_cast<ccd_sphere_t*>(_obj);
//  delete obj;
//}

//==============================================================================
void CCDConvexShapeUtils<dynamics::EllipsoidShape>::destroyCCDObject(void* _obj)
{
  ccd_ellip_t* obj = static_cast<ccd_ellip_t*>(_obj);
  delete obj;
}

//==============================================================================
//void CCDConvexShapeUtils<dynamics::CapsuleShape>::destroyCCDObject(void* _obj)
//{
//  ccd_cap_t* obj = static_cast<ccd_cap_t*>(_obj);
//  delete obj;
//}

//==============================================================================
//void CCDConvexShapeUtils<dynamics::ConeShape>::destroyCCDObject(void* _obj)
//{
//  ccd_cone_t* obj = static_cast<ccd_cone_t*>(_obj);
//  delete obj;
//}

//==============================================================================
void CCDConvexShapeUtils<dynamics::CylinderShape>::destroyCCDObject(void* _obj)
{
  ccd_cyl_t* obj = static_cast<ccd_cyl_t*>(_obj);
  delete obj;
}

//==============================================================================
void CCDConvexShapeUtils<dynamics::MeshShape>::destroyCCDObject(void* _obj)
{
  ccd_convex_t* obj = static_cast<ccd_convex_t*>(_obj);
  delete obj;
}

//==============================================================================
ccd_center_fn CCDConvexShapeUtils<dynamics::BoxShape>::getCenterFunction()
{
  return &centerShape;
}

//==============================================================================
//ccd_center_fn CCDConvexShapeUtils<dynamics::SphereShape>::getCenterFunction()
//{
//  return &centerShape;
//}

//==============================================================================
ccd_center_fn CCDConvexShapeUtils<dynamics::EllipsoidShape>::getCenterFunction()
{
  return &centerShape;
}

//==============================================================================
//ccd_center_fn CCDConvexShapeUtils<dynamics::CapsuleShape>::getCenterFunction()
//{
//  return &centerShape;
//}

//==============================================================================
//ccd_center_fn CCDConvexShapeUtils<dynamics::ConeShape>::getCenterFunction()
//{
//  return &centerShape;
//}

//==============================================================================
ccd_center_fn CCDConvexShapeUtils<dynamics::CylinderShape>::getCenterFunction()
{
  return &centerShape;
}

//==============================================================================
ccd_center_fn CCDConvexShapeUtils<dynamics::MeshShape>::getCenterFunction()
{
  return &centerConvex;
}

//==============================================================================
ccd_support_fn CCDConvexShapeUtils<dynamics::BoxShape>::getSupportFunction()
{
  return &supportBox;
}

//==============================================================================
//ccd_support_fn CCDConvexShapeUtils<dynamics::SphereShape>::getSupportFunction()
//{
//  return &supportSphere;
//}

//==============================================================================
ccd_support_fn CCDConvexShapeUtils<dynamics::EllipsoidShape>::getSupportFunction()
{
  return &supportEllipsoid;
}

//==============================================================================
//ccd_support_fn CCDConvexShapeUtils<dynamics::CapsuleShape>::getSupportFunction()
//{
//  return &supportCapsule;
//}

//==============================================================================
//ccd_support_fn CCDConvexShapeUtils<dynamics::ConeShape>::getSupportFunction()
//{
//  return &supportCone;
//}

//==============================================================================
ccd_support_fn CCDConvexShapeUtils<dynamics::CylinderShape>::getSupportFunction()
{
  return &supportCylinder;
}

//==============================================================================
ccd_support_fn CCDConvexShapeUtils<dynamics::MeshShape>::getSupportFunction()
{
  return &supportConvex;
}

//==============================================================================
//template <>
//GJKSupportFunction triGetSupportFunction<Tri>()
//{
//  return &supportTriangle;
//}

//==============================================================================
//template <>
//GJKCenterFunction getCenterFunction<Tri>()
//{
//  return &centerTriangle;
//}

//==============================================================================
size_t CCDAlgorithmSet::mMaxIteration = 500;
double CCDAlgorithmSet::mTolerance = 1e-6;

//==============================================================================
bool CCDAlgorithmSet::collideCCD(void* obj1, ccd_support_fn supp1, ccd_center_fn cen1,
                                 void* obj2, ccd_support_fn supp2, ccd_center_fn cen2,
                                 Eigen::Vector3d& _point,
                                 Eigen::Vector3d& _normal,
                                 double& penetration)
{
  ccd_t ccd;
  ccd_real_t depth;
  ccd_vec3_t dir, pos;

  CCD_INIT(&ccd);
  ccd.support1 = supp1;
  ccd.support2 = supp2;
  ccd.center1  = cen1;
  ccd.center2  = cen2;
  ccd.max_iterations = mMaxIteration;
  ccd.mpr_tolerance  = mTolerance;

  // Get contact point, normal, and penetration if there is collision.
  const bool isCollided
      = ccdMPRPenetration(obj1, obj2, &ccd, &depth, &dir, &pos) == 0;

  // If there is collision, extract contact information
  if (isCollided)
  {
    // Control point
    _point << ccdVec3X(&pos), ccdVec3Y(&pos), ccdVec3Z(&pos);

    // Normal -- pointing from object2 to object1
    ccdVec3Scale(&dir, -1.);
    _normal << ccdVec3X(&dir), ccdVec3Y(&dir), ccdVec3Z(&dir);

    // Penetration depth
    penetration = depth;

    return true;
  }

  return false;
}

//==============================================================================
bool CCDAlgorithmSet::collideCCD(void* obj1, ccd_support_fn supp1, ccd_center_fn cen1,
                                 void* obj2, ccd_support_fn supp2, ccd_center_fn cen2)
{
  ccd_t ccd;

  CCD_INIT(&ccd);
  ccd.support1 = supp1;
  ccd.support2 = supp2;
  ccd.center1  = cen1;
  ccd.center2  = cen2;
  ccd.max_iterations = mMaxIteration;
  ccd.mpr_tolerance  = mTolerance;

  // Check just if the two objects are colliding, and return with the result.
  return ccdMPRIntersect(obj1, obj2, &ccd) != 0;
}

} // namespace collision
} // namespace dart

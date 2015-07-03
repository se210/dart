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

#include <algorithm>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "dart/config.h"
#include "dart/common/Console.h"
#ifdef HAVE_BULLET_COLLISION
  #include "dart/collision/bullet/BulletCollisionDetector.h"
#endif
#include "dart/collision/dart/DARTCollisionDetector.h"
#include "dart/collision/fcl/FCLCollisionDetector.h"
#include "dart/collision/fcl_mesh/FCLMeshCollisionDetector.h"
#include "dart/constraint/ConstraintSolver.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/SoftBodyNode.h"
#include "dart/dynamics/BoxShape.h"
#include "dart/dynamics/CylinderShape.h"
#include "dart/dynamics/EllipsoidShape.h"
#include "dart/dynamics/PlaneShape.h"
#include "dart/dynamics/MeshShape.h"
#include "dart/dynamics/SoftMeshShape.h"
#include "dart/dynamics/WeldJoint.h"
#include "dart/dynamics/PrismaticJoint.h"
#include "dart/dynamics/RevoluteJoint.h"
#include "dart/dynamics/ScrewJoint.h"
#include "dart/dynamics/TranslationalJoint.h"
#include "dart/dynamics/BallJoint.h"
#include "dart/dynamics/FreeJoint.h"
#include "dart/dynamics/EulerJoint.h"
#include "dart/dynamics/UniversalJoint.h"
#include "dart/dynamics/PlanarJoint.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/Marker.h"
#include "dart/simulation/World.h"
#include "dart/utils/SkelParser.h"
#include "dart/utils/SkeletonBuilder.h"

namespace dart {
namespace utils {

namespace {

//==============================================================================
// TODO: Should be renamed once deprecated SkelParser::readShape() is remove
dynamics::ShapePtr readShapeDART510(
    tinyxml2::XMLElement* vizEle, const std::string& bodyName)
{
  dynamics::ShapePtr newShape;

  // Geometry
  assert(hasElement(vizEle, "geometry"));
  tinyxml2::XMLElement* geometryEle = getElement(vizEle, "geometry");

  if (hasElement(geometryEle, "box"))
  {
    tinyxml2::XMLElement* boxEle       = getElement(geometryEle, "box");
    Eigen::Vector3d       size         = getValueVector3d(boxEle, "size");
    newShape = dynamics::ShapePtr(new dynamics::BoxShape(size));
  }
  else if (hasElement(geometryEle, "ellipsoid"))
  {
    tinyxml2::XMLElement* ellipsoidEle = getElement(geometryEle, "ellipsoid");
    Eigen::Vector3d       size         = getValueVector3d(ellipsoidEle, "size");
    newShape = dynamics::ShapePtr(new dynamics::EllipsoidShape(size));
  }
  else if (hasElement(geometryEle, "cylinder"))
  {
    tinyxml2::XMLElement* cylinderEle  = getElement(geometryEle, "cylinder");
    double                radius       = getValueDouble(cylinderEle, "radius");
    double                height       = getValueDouble(cylinderEle, "height");
    newShape = dynamics::ShapePtr(new dynamics::CylinderShape(radius, height));
  }
  else if (hasElement(geometryEle, "plane"))
  {
    tinyxml2::XMLElement* planeEle = getElement(geometryEle, "plane");
    Eigen::Vector3d       normal   = getValueVector3d(planeEle, "normal");
    if (hasElement(planeEle, "offset")) {
      double offset = getValueDouble(planeEle, "offset");
      newShape = dynamics::ShapePtr(new dynamics::PlaneShape(normal, offset));
    } else if (hasElement(planeEle, "point")) {
      dtwarn << "[SkelParser::readShape] <point> element of <plane> is "
             << "deprecated as of DART 4.3. Please use <offset> element "
             << "instead." << std::endl;
      Eigen::Vector3d point = getValueVector3d(planeEle, "point");
      newShape = dynamics::ShapePtr(new dynamics::PlaneShape(normal, point));
    } else {
      dtwarn << "[SkelParser::readShape] <offset> element is not specified for "
             << "plane shape. DART will use 0.0." << std::endl;
      newShape = dynamics::ShapePtr(new dynamics::PlaneShape(normal, 0.0));
    }
  }
  else if (hasElement(geometryEle, "mesh"))
  {
    tinyxml2::XMLElement* meshEle      = getElement(geometryEle, "mesh");
    std::string           filename     = getValueString(meshEle, "file_name");
    Eigen::Vector3d       scale        = getValueVector3d(meshEle, "scale");
    // TODO(JS): Do we assume that all mesh files place at DART_DATA_PATH?
    const aiScene* model = dynamics::MeshShape::loadMesh(DART_DATA_PATH +
                                                         filename);
    if (model)
    {
      newShape = dynamics::ShapePtr(new dynamics::MeshShape(scale, model));
    }
    else
    {
      dterr << "Fail to load model[" << filename << "]." << std::endl;
    }
  }
  else
  {
    dterr << "[SkelParser::readShape] Unknown visualization shape in BodyNode "
          << "named [" << bodyName << "]\n";
    assert(0);
    return nullptr;
  }

  // transformation
  if (hasElement(vizEle, "transformation")) {
    Eigen::Isometry3d W = getValueIsometry3d(vizEle, "transformation");
    newShape->setLocalTransform(W);
  }

  // color
  if (hasElement(vizEle, "color")) {
    Eigen::Vector3d color = getValueVector3d(vizEle, "color");
    newShape->setColor(color);
  }

  return newShape;
}

//==============================================================================
// TODO: Should be renamed once deprecated SkelParser::readMarker() is remove
dynamics::Marker::Properties readMarkerDART510(
    tinyxml2::XMLElement* markerElement)
{
  // Name attribute
  std::string name = getAttribute(markerElement, "name");

  // offset
  Eigen::Vector3d offset = Eigen::Vector3d::Zero();
  if (hasElement(markerElement, "offset"))
    offset = getValueVector3d(markerElement, "offset");

  dynamics::Marker::Properties newMarker(name, offset);

  return newMarker;
}

//==============================================================================
SkeletonBuilder::BodyNodeBuildData readBodyNode(
    tinyxml2::XMLElement* bodyNodeElement,
    const Eigen::Isometry3d& skeletonFrame)
{
  assert(bodyNodeElement != nullptr);

  SkeletonBuilder::BodyNodePropPtr
      newBodyNode(new dynamics::BodyNode::Properties);
  Eigen::Isometry3d initTransform = Eigen::Isometry3d::Identity();

  // Name attribute
  newBodyNode->mName = getAttribute(bodyNodeElement, "name");

  //--------------------------------------------------------------------------
  // gravity
  if (hasElement(bodyNodeElement, "gravity"))
    newBodyNode->mGravityMode = getValueBool(bodyNodeElement, "gravity");

  //--------------------------------------------------------------------------
  // self_collide
  //    if (hasElement(_bodyElement, "self_collide"))
  //    {
  //        bool gravityMode = getValueBool(_bodyElement, "self_collide");
  //    }

  //--------------------------------------------------------------------------
  // transformation
  if (hasElement(bodyNodeElement, "transformation"))
  {
    Eigen::Isometry3d W =
        getValueIsometry3d(bodyNodeElement, "transformation");
    initTransform = skeletonFrame * W;
  }
  else
  {
    initTransform = skeletonFrame;
  }

  //--------------------------------------------------------------------------
  // visualization_shape
  ElementEnumerator vizShapes(bodyNodeElement, "visualization_shape");
  while (vizShapes.next())
  {
    dynamics::ShapePtr newShape = readShapeDART510(vizShapes.get(),
                                            newBodyNode->mName);

    if(newShape)
      newBodyNode->mVizShapes.push_back(newShape);
  }

  //--------------------------------------------------------------------------
  // visualization_shape
  ElementEnumerator collShapes(bodyNodeElement, "collision_shape");
  while (collShapes.next())
  {
    dynamics::ShapePtr newShape = readShapeDART510(collShapes.get(),
                                            newBodyNode->mName);

    if(newShape)
      newBodyNode->mColShapes.push_back(newShape);
  }

  //--------------------------------------------------------------------------
  // inertia
  if (hasElement(bodyNodeElement, "inertia"))
  {
    tinyxml2::XMLElement* inertiaElement =
        getElement(bodyNodeElement, "inertia");

    // mass
    double mass = getValueDouble(inertiaElement, "mass");
    newBodyNode->mInertia.setMass(mass);

    // moment of inertia
    if (hasElement(inertiaElement, "moment_of_inertia"))
    {
      tinyxml2::XMLElement* moiElement
          = getElement(inertiaElement, "moment_of_inertia");

      double ixx = getValueDouble(moiElement, "ixx");
      double iyy = getValueDouble(moiElement, "iyy");
      double izz = getValueDouble(moiElement, "izz");

      double ixy = getValueDouble(moiElement, "ixy");
      double ixz = getValueDouble(moiElement, "ixz");
      double iyz = getValueDouble(moiElement, "iyz");

      newBodyNode->mInertia.setMoment(ixx, iyy, izz, ixy, ixz, iyz);
    }
    else if (newBodyNode->mVizShapes.size() > 0)
    {
      Eigen::Matrix3d Ic =
          newBodyNode->mVizShapes[0]->computeInertia(mass);

      newBodyNode->mInertia.setMoment(Ic);
    }

    // offset
    if (hasElement(inertiaElement, "offset"))
    {
      Eigen::Vector3d offset = getValueVector3d(inertiaElement, "offset");
      newBodyNode->mInertia.setLocalCOM(offset);
    }
  }

  //--------------------------------------------------------------------------
  // marker
  ElementEnumerator markers(bodyNodeElement, "marker");
  while (markers.next())
    newBodyNode->mMarkerProperties.push_back(readMarkerDART510(markers.get()));

  SkeletonBuilder::BodyNodeBuildData buildData;
  buildData.properties    = newBodyNode;
  buildData.initTransform = initTransform;

  return buildData;
}

//==============================================================================
bool readSoftBodyNode(
    tinyxml2::XMLElement* softBodyNodeElement,
    SkeletonBuilder& builder,
    const Eigen::Isometry3d& skeletonFrame)
{
  //---------------------------------- Note ------------------------------------
  // SoftBodyNode is created if _softBodyNodeElement has <soft_shape>.
  // Otherwise, BodyNode is created.

  //----------------------------------------------------------------------------
  assert(softBodyNodeElement != nullptr);

  SkeletonBuilder::BodyNodeBuildData standardBodyNode
      = readBodyNode(softBodyNodeElement, skeletonFrame);

  // If _softBodyNodeElement has no <soft_shape>, return rigid body node
  if (!hasElement(softBodyNodeElement, "soft_shape"))
    return builder.addBodyNode<dynamics::BodyNode>(standardBodyNode);

  //----------------------------------------------------------------------------
  // Soft properties
  dynamics::SoftBodyNode::UniqueProperties newSoftBodyNode;

  if (hasElement(softBodyNodeElement, "soft_shape"))
  {
    tinyxml2::XMLElement* softShapeEle
        = getElement(softBodyNodeElement, "soft_shape");

    // mass
    double totalMass = getValueDouble(softShapeEle, "total_mass");

    // transformation
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    if (hasElement(softShapeEle, "transformation"))
      T = getValueIsometry3d(softShapeEle, "transformation");

    // geometry
    tinyxml2::XMLElement* geometryEle = getElement(softShapeEle, "geometry");
    if (hasElement(geometryEle, "box"))
    {
      tinyxml2::XMLElement* boxEle = getElement(geometryEle, "box");
      Eigen::Vector3d size  = getValueVector3d(boxEle, "size");
      Eigen::Vector3i frags = getValueVector3i(boxEle, "frags");
      newSoftBodyNode = dynamics::SoftBodyNodeHelper::makeBoxProperties(
            size, T, frags, totalMass);
    }
    else if (hasElement(geometryEle, "ellipsoid"))
    {
      tinyxml2::XMLElement* ellipsoidEle = getElement(geometryEle, "ellipsoid");
      Eigen::Vector3d size = getValueVector3d(ellipsoidEle, "size");
      double nSlices       = getValueDouble(ellipsoidEle, "num_slices");
      double nStacks       = getValueDouble(ellipsoidEle, "num_stacks");
      newSoftBodyNode = dynamics::SoftBodyNodeHelper::makeEllipsoidProperties(
            size,
            nSlices,
            nStacks,
            totalMass);
    }
    else if (hasElement(geometryEle, "cylinder"))
    {
      tinyxml2::XMLElement* ellipsoidEle = getElement(geometryEle, "cylinder");
      double radius  = getValueDouble(ellipsoidEle, "radius");
      double height  = getValueDouble(ellipsoidEle, "height");
      double nSlices = getValueDouble(ellipsoidEle, "num_slices");
      double nStacks = getValueDouble(ellipsoidEle, "num_stacks");
      double nRings = getValueDouble(ellipsoidEle, "num_rings");
      newSoftBodyNode = dynamics::SoftBodyNodeHelper::makeCylinderProperties(
            radius,
            height,
            nSlices,
            nStacks,
            nRings,
            totalMass);
    }
    else
    {
      dterr << "[SkelParser::readSoftBodyNode] Unknown soft shape in "
            << "SoftBodyNode named [" << standardBodyNode.properties->mName
            << "]\n";
    }

    // kv
    if (hasElement(softShapeEle, "kv"))
      newSoftBodyNode.mKv = getValueDouble(softShapeEle, "kv");

    // ke
    if (hasElement(softShapeEle, "ke"))
      newSoftBodyNode.mKe = getValueDouble(softShapeEle, "ke");

    // damp
    if (hasElement(softShapeEle, "damp"))
      newSoftBodyNode.mDampCoeff = getValueDouble(softShapeEle, "damp");
  }

  SkeletonBuilder::BodyNodeBuildData softBodyNode;
  softBodyNode.properties =
      Eigen::make_aligned_shared<dynamics::SoftBodyNode::Properties>(
          *standardBodyNode.properties, newSoftBodyNode);
  softBodyNode.initTransform = standardBodyNode.initTransform;

  return builder.addBodyNode<dynamics::SoftBodyNode>(softBodyNode);
}

//==============================================================================
// This structure exists to allow a common interface for setting values in both
// SingleDofJoint::Properties and MultiDofJoint::Properties
struct DofProxy
{
  size_t index;
  bool valid;

  double* lowerPosition;
  double* upperPosition;
  double* initalPosition;

  double* lowerVelocity;
  double* upperVelocity;
  double* initialVelocity;

  double* lowerAcceleration;
  double* upperAcceleration;
  double* initialAcceleration;

  double* lowerForce;
  double* upperForce;
  double* initialForce;

  double* springStiffness;
  double* restPosition;
  double* dampingCoefficient;
  double* friction;

  bool* preserveName;
  std::string* name;

  DofProxy(dynamics::SingleDofJoint::Properties& properties,
           SkeletonBuilder::JointBuildData& buildData,
           size_t index,
           const std::string& jointName)
    : index(index),
      valid(true),

      lowerPosition(&properties.mPositionLowerLimit),
      upperPosition(&properties.mPositionUpperLimit),
      initalPosition(&buildData.position.data()[0]),

      lowerVelocity(&properties.mVelocityLowerLimit),
      upperVelocity(&properties.mVelocityUpperLimit),
      initialVelocity(&buildData.velocity.data()[0]),

      lowerAcceleration(&properties.mAccelerationLowerLimit),
      upperAcceleration(&properties.mAccelerationUpperLimit),
      initialAcceleration(&buildData.acceleration.data()[0]),

      lowerForce(&properties.mForceLowerLimit),
      upperForce(&properties.mForceUpperLimit),
      initialForce(&buildData.force.data()[0]),

      springStiffness(&properties.mSpringStiffness),
      restPosition(&properties.mRestPosition),
      dampingCoefficient(&properties.mDampingCoefficient),
      friction(&properties.mFriction),

      preserveName(&properties.mPreserveDofName),
      name(&properties.mDofName)
  {
    if(index > 0)
    {
      dterr << "[SkelParser] Joint named [" << jointName << "] has a dof "
            << "element (" << index << ") which is out of bounds (max 0)\n";
      valid = false;
    }
  }

  template <typename PropertyType>
  DofProxy(PropertyType& properties,
           SkeletonBuilder::JointBuildData& buildData,
           size_t index,
           const std::string& jointName)
    : index(index),
      valid(true),

      lowerPosition(&properties.mPositionLowerLimits.data()[index]),
      upperPosition(&properties.mPositionUpperLimits.data()[index]),
      initalPosition(&buildData.position.data()[index]),

      lowerVelocity(&properties.mVelocityLowerLimits.data()[index]),
      upperVelocity(&properties.mVelocityUpperLimits.data()[index]),
      initialVelocity(&buildData.velocity.data()[index]),

      lowerAcceleration(&properties.mAccelerationLowerLimits.data()[index]),
      upperAcceleration(&properties.mAccelerationUpperLimits.data()[index]),
      initialAcceleration(&buildData.acceleration.data()[index]),

      lowerForce(&properties.mForceLowerLimits.data()[index]),
      upperForce(&properties.mForceUpperLimits.data()[index]),
      initialForce(&buildData.force.data()[index]),

      springStiffness(&properties.mSpringStiffnesses.data()[index]),
      restPosition(&properties.mRestPositions.data()[index]),
      dampingCoefficient(&properties.mDampingCoefficients.data()[index]),
      friction(&properties.mFrictions.data()[index]),

      preserveName(&properties.mPreserveDofNames[index]),
      name(&properties.mDofNames[index])
  {
    if((int)index >= properties.mPositionLowerLimits.size())
    {
      dterr << "[SkelParser] Joint named [" << jointName << "] has a dof "
            << "element (" << index << ") which is out of bounds (max "
            << properties.mPositionLowerLimits.size()-1 << ")\n";
      valid = false;
    }
  }
};

//==============================================================================
template <typename PropertyType>
void readJointDynamicsAndLimit(
    tinyxml2::XMLElement* jointElement,
    PropertyType& properties,
    SkeletonBuilder::JointBuildData& buildData,
    const std::string& name,
    size_t numAxis)
{
  // TODO(MXG): Consider printing warnings for these tags that recommends using
  // the dof tag instead, because all functionality of these tags have been
  // moved to the dof tag

  assert(jointElement != nullptr);
  assert(numAxis <= 6);

  std::string axisName = "axis";

  // axis
  for (size_t i = 0; i < numAxis; ++i)
  {
    if (i != 0)
      axisName = "axis" + std::to_string(i + 1);

    if (hasElement(jointElement, axisName))
    {
      DofProxy proxy(properties, buildData, i, name);

      tinyxml2::XMLElement* axisElement = getElement(jointElement, axisName);

      // damping
      if (hasElement(axisElement, "damping"))
      {
        dtwarn << "[SkelParser] <damping> tag is now an element under the "
               << "<dynamics> tag. Please see "
               << "(https://github.com/dartsim/dart/wiki/) for more details.\n";
        double damping = getValueDouble(axisElement, "damping");
        *proxy.dampingCoefficient = damping;
      }

      // dynamics
      if (hasElement(axisElement, "dynamics"))
      {
        tinyxml2::XMLElement* dynamicsElement
            = getElement(axisElement, "dynamics");

        // damping
        if (hasElement(dynamicsElement, "damping"))
        {
          double val = getValueDouble(dynamicsElement, "damping");
          *proxy.dampingCoefficient = val;
        }

        // friction
        if (hasElement(dynamicsElement, "friction"))
        {
          double val = getValueDouble(dynamicsElement, "friction");
          *proxy.friction = val;
        }

        // spring_rest_position
        if (hasElement(dynamicsElement, "spring_rest_position"))
        {
          double val = getValueDouble(dynamicsElement, "spring_rest_position");
          *proxy.restPosition = val;
        }

        // friction
        if (hasElement(dynamicsElement, "spring_stiffness"))
        {
          double val = getValueDouble(dynamicsElement, "spring_stiffness");
          *proxy.springStiffness = val;
        }
      }

      // limit
      if (hasElement(axisElement, "limit"))
      {
        tinyxml2::XMLElement* limitElement
            = getElement(axisElement, "limit");

        // lower
        if (hasElement(limitElement, "lower"))
        {
          double lower = getValueDouble(limitElement, "lower");
          *proxy.lowerPosition = lower;
        }

        // upper
        if (hasElement(limitElement, "upper"))
        {
          double upper = getValueDouble(limitElement, "upper");
          *proxy.upperPosition = upper;
        }
      }
    }
  }
}

//==============================================================================
void getDofAttributeIfItExists(
    const std::string& attribute,
    double* value,
    const std::string& elementType,
    const tinyxml2::XMLElement* xmlElement,
    const std::string& jointName,
    size_t index)
{
  if (xmlElement->QueryDoubleAttribute(attribute.c_str(), value)
      == tinyxml2::XML_WRONG_ATTRIBUTE_TYPE)
  {
    dterr << "[SkelParser::getDofAttributeIfItExists] Invalid type for ["
          << attribute << "] attribute of [" << elementType
          << "] element in the [" << index << "] dof of Joint ["
          << jointName << "].\n";
  }
}

//==============================================================================
void setDofLimitAttributes(tinyxml2::XMLElement* dofElement,
                           const std::string& elementType,
                           const std::string& jointName,
                           size_t index,
                           double* lower, double* upper, double* initial)
{
  const tinyxml2::XMLElement* xmlElement
      = getElement(dofElement, elementType);

  getDofAttributeIfItExists("lower", lower, elementType, xmlElement, jointName, index);
  getDofAttributeIfItExists("upper", upper, elementType, xmlElement, jointName, index);
  getDofAttributeIfItExists("initial", initial, elementType, xmlElement, jointName, index);
}

//==============================================================================
template <typename PropertyType>
void readDegreeOfFreedom(tinyxml2::XMLElement* dofElement,
                         PropertyType& properties,
                         SkeletonBuilder::JointBuildData& buildData,
                         const std::string& jointName,
                         size_t numDofs)
{
  int localIndex = -1;
  int xml_err = dofElement->QueryIntAttribute("local_index", &localIndex);

  // If the localIndex is out of bounds, quit
  if (localIndex >= (int)numDofs)
  {
    dterr << "[SkelParser::readDegreeOfFreedom] Joint named '"
          << jointName << "' contains dof element with invalid "
          << "number attribute [" << localIndex << "]. It must be less than "
          << numDofs << ".\n";
    return;
  }

  // If no localIndex was found, report an error and quit
  if (localIndex == -1 && numDofs > 1)
  {
    if (tinyxml2::XML_NO_ATTRIBUTE == xml_err)
    {
      dterr << "[SkelParser::readDegreeOfFreedom] Joint named ["
            << jointName << "] has [" << numDofs
            << "] DOFs, but the xml contains a dof element without its "
            << "local_index specified. For Joints with multiple DOFs, all dof "
            << "elements must specify their local_index attribute.\n";
    }
    else if (tinyxml2::XML_WRONG_ATTRIBUTE_TYPE == xml_err)
    {
      dterr << "[SkelParser::readDegreeOfFreedom] Joint named ["
            << jointName << "] has a dof element with a wrongly "
            << "formatted local_index attribute.\n";
    }

    return;
  }
  // Unless the joint is a single-dof joint
  else if (localIndex == -1 && numDofs == 1)
    localIndex = 0;

  DofProxy proxy(properties, buildData, localIndex, jointName);

  const char* name = dofElement->Attribute("name");
  if (name)
  {
    *proxy.name = std::string(name);
    *proxy.preserveName = true;
  }

  if (hasElement(dofElement, "position"))
  {
    setDofLimitAttributes(dofElement, "position", jointName, localIndex,
                          proxy.lowerPosition,
                          proxy.upperPosition,
                          proxy.initalPosition);
  }

  if (hasElement(dofElement, "velocity"))
  {
    setDofLimitAttributes(dofElement, "velocity", jointName, localIndex,
                          proxy.lowerVelocity,
                          proxy.upperVelocity,
                          proxy.initialVelocity);
  }

  if (hasElement(dofElement, "acceleration"))
  {
    setDofLimitAttributes(dofElement, "acceleration", jointName, localIndex,
                          proxy.lowerAcceleration,
                          proxy.upperAcceleration,
                          proxy.initialAcceleration);
  }

  if (hasElement(dofElement, "force"))
  {
    setDofLimitAttributes(dofElement, "force", jointName, localIndex,
                          proxy.lowerForce,
                          proxy.upperForce,
                          proxy.initialForce);
  }

  if (hasElement(dofElement, "damping"))
    *proxy.dampingCoefficient = getValueDouble(dofElement, "damping");

  if (hasElement(dofElement, "friction"))
    *proxy.friction = getValueDouble(dofElement, "friction");

  if (hasElement(dofElement, "spring_rest_position"))
    *proxy.restPosition = getValueDouble(dofElement, "spring_rest_position");

  if (hasElement(dofElement, "spring_stiffness"))
    *proxy.springStiffness = getValueDouble(dofElement, "spring_stiffness");
}

//==============================================================================
template <typename PropertyType>
void readAllDegreesOfFreedom(
    tinyxml2::XMLElement* jointElement,
    PropertyType& properties,
    SkeletonBuilder::JointBuildData& joint,
    const std::string& jointName,
    size_t numDofs)
{
  if(joint.position.size() < (int)numDofs)
  {
    joint.position.resize(numDofs);
    joint.position.setZero();
  }

  if(joint.velocity.size() < (int)numDofs)
  {
    joint.velocity.resize(numDofs);
    joint.velocity.setZero();
  }

  if(joint.acceleration.size() < (int)numDofs)
  {
    joint.acceleration.resize((int)numDofs);
    joint.acceleration.setZero();
  }

  if(joint.force.size() < (int)numDofs)
  {
    joint.force.resize(numDofs);
    joint.force.setZero();
  }

  ElementEnumerator DofElements(jointElement, "dof");
  while(DofElements.next())
    readDegreeOfFreedom(DofElements.get(), properties,
                         joint, jointName, numDofs);
}

//==============================================================================
void readRevoluteJoint(
    tinyxml2::XMLElement* jointElement,
    SkeletonBuilder::JointBuildData& buildData,
    const std::string& name)
{
  assert(jointElement != nullptr);

  dynamics::RevoluteJoint::Properties properties;

  //--------------------------------------------------------------------------
  // axis
  if (hasElement(jointElement, "axis"))
  {
    tinyxml2::XMLElement* axisElement = getElement(jointElement, "axis");

    // xyz
    Eigen::Vector3d xyz = getValueVector3d(axisElement, "xyz");
    properties.mAxis = xyz;
  }
  else
  {
    dterr << "[SkelParser::readRevoluteJoint] Revolute Joint named ["
          << name << "] is missing axis information!\n";
    assert(false);
  }

  readJointDynamicsAndLimit<dynamics::SingleDofJoint::Properties>(
        jointElement, properties, buildData, name, 1);

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(jointElement, "init_pos"))
  {
    double init_pos = getValueDouble(jointElement, "init_pos");
    Eigen::VectorXd ipos = Eigen::VectorXd(1);
    ipos << init_pos;
    buildData.position = ipos;
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(jointElement, "init_vel"))
  {
    double init_vel = getValueDouble(jointElement, "init_vel");
    Eigen::VectorXd ivel = Eigen::VectorXd(1);
    ivel << init_vel;
    buildData.velocity = ivel;
  }

  readAllDegreesOfFreedom<dynamics::SingleDofJoint::Properties>(
        jointElement, properties, buildData, name, 1);

  buildData.properties
      = Eigen::make_aligned_shared<dynamics::RevoluteJoint::Properties>(
          properties);
}

//==============================================================================
void readPrismaticJoint(
    tinyxml2::XMLElement* jointElement,
    SkeletonBuilder::JointBuildData& buildData,
    const std::string& name)
{
  assert(jointElement != nullptr);

  dynamics::PrismaticJoint::Properties properties;

  //--------------------------------------------------------------------------
  // axis
  if (hasElement(jointElement, "axis"))
  {
    tinyxml2::XMLElement* axisElement = getElement(jointElement, "axis");

    // xyz
    Eigen::Vector3d xyz = getValueVector3d(axisElement, "xyz");
    properties.mAxis = xyz;
  }
  else
  {
    dterr << "[SkelParser::readPrismaticJoint] Prismatic Joint named ["
          << name << "] is missing axis information!\n";
    assert(0);
  }

  readJointDynamicsAndLimit<dynamics::SingleDofJoint::Properties>(
        jointElement, properties, buildData, name, 1);

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(jointElement, "init_pos"))
  {
    double init_pos = getValueDouble(jointElement, "init_pos");
    Eigen::VectorXd ipos = Eigen::VectorXd(1);
    ipos << init_pos;
    buildData.position = ipos;
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(jointElement, "init_vel"))
  {
    double init_vel = getValueDouble(jointElement, "init_vel");
    Eigen::VectorXd ivel = Eigen::VectorXd(1);
    ivel << init_vel;
    buildData.velocity = ivel;
  }

  readAllDegreesOfFreedom<dynamics::SingleDofJoint::Properties>(
        jointElement, properties, buildData, name, 1);

  buildData.properties
      = Eigen::make_aligned_shared<dynamics::PrismaticJoint::Properties>(
          properties);
}

//==============================================================================
void readScrewJoint(
    tinyxml2::XMLElement* jointElement,
    SkeletonBuilder::JointBuildData& buildData,
    const std::string& name)
{
  assert(jointElement != nullptr);

  dynamics::ScrewJoint::Properties properties;

  //--------------------------------------------------------------------------
  // axis
  if (hasElement(jointElement, "axis"))
  {
    tinyxml2::XMLElement* axisElement = getElement(jointElement, "axis");

    // xyz
    Eigen::Vector3d xyz = getValueVector3d(axisElement, "xyz");
    properties.mAxis = xyz;

    // pitch
    if (hasElement(axisElement, "pitch"))
    {
      double pitch = getValueDouble(axisElement, "pitch");
      properties.mPitch = pitch;
    }
  }
  else
  {
    dterr << "[SkelParser::readScrewJoint] Screw Joint named [" << name
          << "] is missing axis information!\n";
    assert(0);
  }

  readJointDynamicsAndLimit<dynamics::SingleDofJoint::Properties>(
        jointElement, properties, buildData, name, 1);

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(jointElement, "init_pos"))
  {
    double init_pos = getValueDouble(jointElement, "init_pos");
    Eigen::VectorXd ipos = Eigen::VectorXd(1);
    ipos << init_pos;
    buildData.position = ipos;
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(jointElement, "init_vel"))
  {
    double init_vel = getValueDouble(jointElement, "init_vel");
    Eigen::VectorXd ivel = Eigen::VectorXd(1);
    ivel << init_vel;
    buildData.velocity = ivel;
  }

  readAllDegreesOfFreedom<dynamics::SingleDofJoint::Properties>(
        jointElement, properties, buildData, name, 1);

  buildData.properties
      = Eigen::make_aligned_shared<dynamics::ScrewJoint::Properties>(
          properties);
}

//==============================================================================
void readUniversalJoint(
    tinyxml2::XMLElement* jointElement,
    SkeletonBuilder::JointBuildData& buildData,
    const std::string& name)
{
  assert(jointElement != nullptr);

  dynamics::UniversalJoint::Properties properties;

  //--------------------------------------------------------------------------
  // axis
  if (hasElement(jointElement, "axis"))
  {
    tinyxml2::XMLElement* axisElement = getElement(jointElement, "axis");

    // xyz
    Eigen::Vector3d xyz = getValueVector3d(axisElement, "xyz");
    properties.mAxis[0] = xyz;
  }
  else
  {
    dterr << "[SkelParser::readUniversalJoint] Universal Joint named [" << name
          << "] is missing axis information!\n";
    assert(0);
  }

  //--------------------------------------------------------------------------
  // axis2
  if (hasElement(jointElement, "axis2"))
  {
    tinyxml2::XMLElement* axis2Element = getElement(jointElement, "axis2");

    // xyz
    Eigen::Vector3d xyz = getValueVector3d(axis2Element, "xyz");
    properties.mAxis[1] = xyz;
  }
  else
  {
    dterr << "[SkelParser::readUniversalJoint] Universal Joint named [" << name
          << "] is missing axis2 information!\n";
    assert(0);
  }

  readJointDynamicsAndLimit(jointElement, properties, buildData, name, 2);

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(jointElement, "init_pos"))
  {
    Eigen::Vector2d init_pos = getValueVector2d(jointElement, "init_pos");
    buildData.position = init_pos;
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(jointElement, "init_vel"))
  {
    Eigen::Vector2d init_vel = getValueVector2d(jointElement, "init_vel");
    buildData.velocity = init_vel;
  }

  readAllDegreesOfFreedom(jointElement, properties, buildData, name, 2);

  buildData.properties
      = Eigen::make_aligned_shared<dynamics::UniversalJoint::Properties>(
          properties);
}

//==============================================================================
void readBallJoint(
    tinyxml2::XMLElement* jointElement,
    SkeletonBuilder::JointBuildData& buildData,
    const std::string& name)
{
  assert(jointElement != nullptr);

  dynamics::BallJoint::Properties properties;

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(jointElement, "init_pos"))
  {
    Eigen::Vector3d init_pos = getValueVector3d(jointElement, "init_pos");
    buildData.position = init_pos;
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(jointElement, "init_vel"))
  {
    Eigen::Vector3d init_vel = getValueVector3d(jointElement, "init_vel");
    buildData.velocity = init_vel;
  }

  readAllDegreesOfFreedom(jointElement, properties, buildData, name, 3);

  buildData.properties
      = Eigen::make_aligned_shared<dynamics::BallJoint::Properties>(properties);
}

//==============================================================================
void readEulerJoint(
    tinyxml2::XMLElement* jointElement,
    SkeletonBuilder::JointBuildData& buildData,
    const std::string& name)
{
  assert(jointElement != nullptr);

  dynamics::EulerJoint::Properties properties;

  //--------------------------------------------------------------------------
  // axis order
  std::string order = getValueString(jointElement, "axis_order");
  if (order == "xyz")
  {
    properties.mAxisOrder = dynamics::EulerJoint::AO_XYZ;
  }
  else if (order == "zyx")
  {
    properties.mAxisOrder = dynamics::EulerJoint::AO_ZYX;
  }
  else
  {
    dterr << "[SkelParser::readEulerJoint] Undefined Euler axis order for "
          << "Euler Joint named [" << name << "]\n";
    assert(0);
  }

  //--------------------------------------------------------------------------
  // axis
  readJointDynamicsAndLimit(jointElement, properties, buildData, name, 3);

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(jointElement, "init_pos"))
  {
    Eigen::Vector3d init_pos = getValueVector3d(jointElement, "init_pos");
    buildData.position = init_pos;
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(jointElement, "init_vel"))
  {
    Eigen::Vector3d init_vel = getValueVector3d(jointElement, "init_vel");
    buildData.velocity = init_vel;
  }

  readAllDegreesOfFreedom(jointElement, properties, buildData, name, 3);

  buildData.properties
      = Eigen::make_aligned_shared<dynamics::EulerJoint::Properties>(
          properties);
}

//==============================================================================
void readTranslationalJoint(
    tinyxml2::XMLElement* jointElement,
    SkeletonBuilder::JointBuildData& buildData,
    const std::string& name)
{
  assert(jointElement != nullptr);

  dynamics::TranslationalJoint::Properties properties;

  //--------------------------------------------------------------------------
  // axis
  readJointDynamicsAndLimit(jointElement, properties, buildData, name, 3);

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(jointElement, "init_pos"))
  {
    Eigen::Vector3d init_pos = getValueVector3d(jointElement, "init_pos");
    buildData.position = init_pos;
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(jointElement, "init_vel"))
  {
    Eigen::Vector3d init_vel = getValueVector3d(jointElement, "init_vel");
    buildData.velocity = init_vel;
  }

  readAllDegreesOfFreedom(jointElement, properties, buildData, name, 3);

  buildData.properties
      = Eigen::make_aligned_shared<dynamics::TranslationalJoint::Properties>(
          properties);
}

//==============================================================================
void readPlanarJoint(
    tinyxml2::XMLElement* jointElement,
    SkeletonBuilder::JointBuildData& buildData,
    const std::string& name)
{
  assert(jointElement != nullptr);

  dynamics::PlanarJoint::Properties properties;

  //--------------------------------------------------------------------------
  // Plane
  if (hasElement(jointElement, "plane"))
  {
    tinyxml2::XMLElement* planeElement = getElement(jointElement, "plane");

    // Type attribute
    std::string type = getAttribute(planeElement, "type");

    if (type == "xy")
    {
      properties.mPlaneType = dynamics::PlanarJoint::PT_XY;
    }
    else if (type == "yz")
    {
      properties.mPlaneType = dynamics::PlanarJoint::PT_YZ;
    }
    else if (type == "zx")
    {
      properties.mPlaneType = dynamics::PlanarJoint::PT_ZX;
    }
    else if (type == "arbitrary")
    {
      properties.mPlaneType = dynamics::PlanarJoint::PT_ARBITRARY;

      tinyxml2::XMLElement* transAxis1Element
          = getElement(planeElement, "translation_axis1");

      properties.mTransAxis1 = getValueVector3d(transAxis1Element, "xyz");

      tinyxml2::XMLElement* transAxis2Element
          = getElement(planeElement, "translation_axis2");

      properties.mTransAxis2 = getValueVector3d(transAxis2Element, "xyz");
    }
    else
    {
      dterr << "[SkelParser::readPlanarJoint] Planar Joint named [" << name
            << "] is missing plane type information. Defaulting to XY-Plane.\n";
    }
  }
  else
  {
    dterr << "[SkelParser::readPlanarJoint] Planar Joint named [" << name
          << "] is missing plane type information. Defaulting to XY-Plane.\n";
  }

  //--------------------------------------------------------------------------
  // axis
  readJointDynamicsAndLimit(jointElement, properties, buildData, name, 3);

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(jointElement, "init_pos"))
  {
    Eigen::Vector3d init_pos = getValueVector3d(jointElement, "init_pos");
    buildData.position = init_pos;
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(jointElement, "init_vel"))
  {
    Eigen::Vector3d init_vel = getValueVector3d(jointElement, "init_vel");
    buildData.velocity = init_vel;
  }

  readAllDegreesOfFreedom(jointElement, properties, buildData, name, 3);

  buildData.properties
      = Eigen::make_aligned_shared<dynamics::PlanarJoint::Properties>(
          properties);
}

//==============================================================================
void readFreeJoint(
    tinyxml2::XMLElement* jointElement,
    SkeletonBuilder::JointBuildData& buildData,
    const std::string& name)
{
  assert(jointElement != nullptr);

  dynamics::FreeJoint::Properties properties;

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(jointElement, "init_pos"))
  {
    Eigen::Vector6d init_pos = getValueVector6d(jointElement, "init_pos");
    buildData.position = init_pos;
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(jointElement, "init_vel"))
  {
    Eigen::Vector6d init_vel = getValueVector6d(jointElement, "init_vel");
    buildData.velocity = init_vel;
  }

  readAllDegreesOfFreedom(jointElement, properties, buildData, name, 6);

  buildData.properties
      = Eigen::make_aligned_shared<dynamics::FreeJoint::Properties>(properties);
}

//==============================================================================
void readWeldJoint(
    tinyxml2::XMLElement* jointElement,
    SkeletonBuilder::JointBuildData& buildData,
    const std::string&)
{
  assert(jointElement != nullptr);

  buildData.properties
      = Eigen::make_aligned_shared<dynamics::WeldJoint::Properties>();
}

//==============================================================================
void readJoint(tinyxml2::XMLElement* jointElement, SkeletonBuilder& builder)
{
  assert(jointElement != nullptr);

  SkeletonBuilder::JointBuildData buildData;

  //--------------------------------------------------------------------------
  // Name attribute
  std::string name = getAttribute(jointElement, "name");

  //--------------------------------------------------------------------------
  // Type attribute
  std::string type = getAttribute(jointElement, "type");
  assert(!type.empty());

  if (type == std::string("weld"))
  {
    readWeldJoint(jointElement, buildData, name);
  }
  else if (type == std::string("prismatic"))
  {
    readPrismaticJoint(jointElement, buildData, name);
  }
  else if (type == std::string("revolute"))
  {
    readRevoluteJoint(jointElement, buildData, name);
  }
  else if (type == std::string("screw"))
  {
    readScrewJoint(jointElement, buildData, name);
  }
  else if (type == std::string("universal"))
  {
    readUniversalJoint(jointElement, buildData, name);
  }
  else if (type == std::string("ball"))
  {
    readBallJoint(jointElement, buildData, name);
  }
  else if (type == std::string("euler"))
  {
    readEulerJoint(jointElement, buildData, name);
  }
  else if (type == std::string("translational"))
  {
    readTranslationalJoint(jointElement, buildData, name);
  }
  else if (type == std::string("planar"))
  {
    readPlanarJoint(jointElement, buildData, name);
  }
  else if (type == std::string("free"))
  {
    readFreeJoint(jointElement, buildData, name);
  }
  else
  {
    dterr << "[SkelParser::readJoint] Unsupported joint type [" << type
          << "] requested by Joint named [" << name << "]. This Joint will be "
          << "discarded.\n";
    return;
  }
  assert(buildData.properties != nullptr);

  buildData.properties->mName = name;

  //--------------------------------------------------------------------------
  // Actuator attribute
  if (hasAttribute(jointElement, "actuator"))
  {
    const std::string actuator = getAttribute(jointElement, "actuator");

    if (actuator == "force")
      buildData.properties->mActuatorType = dynamics::Joint::FORCE;
    else if (actuator == "passive")
      buildData.properties->mActuatorType = dynamics::Joint::PASSIVE;
    else if (actuator == "servo")
      buildData.properties->mActuatorType = dynamics::Joint::SERVO;
    else if (actuator == "acceleration")
      buildData.properties->mActuatorType = dynamics::Joint::ACCELERATION;
    else if (actuator == "velocity")
      buildData.properties->mActuatorType = dynamics::Joint::VELOCITY;
    else if (actuator == "locked")
      buildData.properties->mActuatorType = dynamics::Joint::LOCKED;
    else
      dterr << "Joint named [" << name
            << "] contains invalid actuator attribute ["
            << actuator << "].\n";
  }
  else
  {
    buildData.properties->mActuatorType = dynamics::Joint::DefaultActuatorType;
  }

  //--------------------------------------------------------------------------
  // parent
  if (hasElement(jointElement, "parent"))
  {
    buildData.parentName = getValueString(jointElement, "parent");
  }
  else
  {
    dterr << "[SkelParser::readJoint] Joint named [" << name << "] is missing "
          << "a parent BodyNode!\n";
    assert(false);
  }

  //--------------------------------------------------------------------------
  // child
  if (hasElement(jointElement, "child"))
  {
    buildData.childName = getValueString(jointElement, "child");
  }
  else
  {
    dterr << "[SkelParser::readJoint] Joint named [" << name << "] is missing "
          << "a child BodyNode!\n";
    assert(false);
  }

  //--------------------------------------------------------------------------
  // transformation
  Eigen::Isometry3d childToJoint = Eigen::Isometry3d::Identity();

  if (hasElement(jointElement, "transformation"))
    childToJoint = getValueIsometry3d(jointElement, "transformation");

  buildData.properties->mT_ChildBodyToJoint = childToJoint;
  if (!math::verifyTransform(buildData.properties->mT_ChildBodyToJoint))
  {
    dterr << "[SkelParser::readJoint] Invalid child to Joint transform for "
          << "Joint named [" << name << "]:\n"
          << buildData.properties->mT_ChildBodyToJoint.matrix() << "\n";
  }

  //--------------------------------------------------------------------------
  // Type attribute
  if (type == std::string("weld"))
    builder.addJoint<dynamics::WeldJoint>(buildData);
  else if (type == std::string("prismatic"))
    builder.addJoint<dynamics::PrismaticJoint>(buildData);
  else if (type == std::string("revolute"))
    builder.addJoint<dynamics::RevoluteJoint>(buildData);
  else if (type == std::string("universal"))
    builder.addJoint<dynamics::UniversalJoint>(buildData);
  else if (type == std::string("ball"))
    builder.addJoint<dynamics::BallJoint>(buildData);
  else if (type == std::string("euler"))
    builder.addJoint<dynamics::EulerJoint>(buildData);
  else if (type == std::string("translational"))
    builder.addJoint<dynamics::TranslationalJoint>(buildData);
  else if (type == std::string("planar"))
    builder.addJoint<dynamics::PlanarJoint>(buildData);
  else if (type == std::string("free"))
    builder.addJoint<dynamics::FreeJoint>(buildData);
  else
  {
    dterr << "[SkelParser::readJoint] Unsupported joint type [" << type
          << "] requested by Joint named [" << name << "]. This Joint will be "
          << "discarded.\n";
    return;
  }
}

//==============================================================================
// TODO: Should be renamed once deprecated SkelParser::readSkeleton() is remove
dynamics::SkeletonPtr readSkeletonDART510(tinyxml2::XMLElement* skeletonElement)
{
  assert(skeletonElement != nullptr);

  //--------------------------------------------------------------------------
  // Name attribute
  std::string name = getAttribute(skeletonElement, "name");

  //--------------------------------------------------------------------------
  // Create SkeletonBuilder with given Skeleton name
  SkeletonBuilder builder(name, SkeletonBuilder::BODYNODE_AND_CHILDTOJOINT);
  SkeletonBuilder::SkeletonPropPtr skelProp = builder.getSkeletonProperties();

  //--------------------------------------------------------------------------
  // transformation
  Eigen::Isometry3d skeletonFrame = Eigen::Isometry3d::Identity();
  if (hasElement(skeletonElement, "transformation"))
    skeletonFrame = getValueIsometry3d(skeletonElement, "transformation");

  //--------------------------------------------------------------------------
  // mobile
  if (hasElement(skeletonElement, "mobile"))
    skelProp->mIsMobile = getValueBool(skeletonElement, "mobile");

  //--------------------------------------------------------------------------
  // Bodies
  ElementEnumerator xmlBodies(skeletonElement, "body");
  while (xmlBodies.next())
    readSoftBodyNode(xmlBodies.get(), builder, skeletonFrame);

  //--------------------------------------------------------------------------
  // Joints
  ElementEnumerator xmlJoints(skeletonElement, "joint");
  while (xmlJoints.next())
    readJoint(xmlJoints.get(), builder);

  //--------------------------------------------------------------------------
  // Assemble and return skeleton
  return builder.build();
}

//==============================================================================
// TODO: Should be renamed once deprecated SkelParser::readWorld() is remove
simulation::WorldPtr readWorldDART510(tinyxml2::XMLElement* worldElement)
{
  assert(worldElement != nullptr);

  // Create a world
  simulation::WorldPtr newWorld(new simulation::World);

  //--------------------------------------------------------------------------
  // Load physics
  tinyxml2::XMLElement* physicsElement
      = worldElement->FirstChildElement("physics");
  if (physicsElement != nullptr)
  {
    // Time step
    tinyxml2::XMLElement* timeStepElement = nullptr;
    timeStepElement = physicsElement->FirstChildElement("time_step");
    if (timeStepElement != nullptr)
    {
      std::string strTimeStep = timeStepElement->GetText();
      double timeStep = toDouble(strTimeStep);
      newWorld->setTimeStep(timeStep);
    }

    // Gravity
    tinyxml2::XMLElement* gravityElement = nullptr;
    gravityElement = physicsElement->FirstChildElement("gravity");
    if (gravityElement != nullptr)
    {
      std::string strGravity = gravityElement->GetText();
      Eigen::Vector3d gravity = toVector3d(strGravity);
      newWorld->setGravity(gravity);
    }

    // Collision detector
    if (hasElement(physicsElement, "collision_detector"))
    {
      std::string strCD = getValueString(physicsElement, "collision_detector");
      if (strCD == "fcl_mesh")
      {
        newWorld->getConstraintSolver()->setCollisionDetector(
              new collision::FCLMeshCollisionDetector());
      }
      else if (strCD == "fcl")
      {
        newWorld->getConstraintSolver()->setCollisionDetector(
              new collision::FCLCollisionDetector());
      }
      else if (strCD == "dart")
      {
        newWorld->getConstraintSolver()->setCollisionDetector(
              new collision::DARTCollisionDetector());
      }
#ifdef HAVE_BULLET_COLLISION
      else if (strCD == "bullet")
      {
        newWorld->getConstraintSolver()->setCollisionDetector(
              new collision::BulletCollisionDetector());
      }
#endif
      else
      {
        dtwarn << "Unknown collision detector[" << strCD << "]. "
               << "Default collision detector[fcl] will be loaded."
               << std::endl;
      }
    }
    else
    {
      newWorld->getConstraintSolver()->setCollisionDetector(
            new collision::FCLMeshCollisionDetector());
    }
  }

  //--------------------------------------------------------------------------
  // Load skeletons
  ElementEnumerator SkeletonElements(worldElement, "skeleton");
  while (SkeletonElements.next())
  {
    dynamics::SkeletonPtr newSkeleton = readSkeletonDART510(SkeletonElements.get());

    newWorld->addSkeleton(newSkeleton);
  }

  return newWorld;
}

//==============================================================================
tinyxml2::XMLElement* checkFormatAndGetWorldElement(
    tinyxml2::XMLDocument& document)
{
  //--------------------------------------------------------------------------
  // Check xml tag
  tinyxml2::XMLElement* skelElement = nullptr;
  skelElement = document.FirstChildElement("skel");
  if (skelElement == nullptr)
  {
    dterr << "XML Document does not contain <skel> as the root element.\n";
    return nullptr;
  }

  //--------------------------------------------------------------------------
  // Load World
  tinyxml2::XMLElement* worldElement = nullptr;
  worldElement = skelElement->FirstChildElement("world");
  if (worldElement == nullptr)
  {
    dterr << "XML Document does not contain a <world> element under the <skel> "
          << "element.\n";
    return nullptr;
  }

  return worldElement;
}

}  // (unnamed) namespace

//==============================================================================
simulation::WorldPtr SkelParser::readWorld(const std::string& filename)
{
  //--------------------------------------------------------------------------
  // Load xml and create Document
  tinyxml2::XMLDocument _dartFile;
  try
  {
    openXMLFile(_dartFile, filename.c_str());
  }
  catch(std::exception const& e)
  {
    dterr << "[SkelParser::readWorld] LoadFile [" << filename
          << "] Failed: " << e.what() << "\n";
    return nullptr;
  }

  tinyxml2::XMLElement* worldElement = checkFormatAndGetWorldElement(_dartFile);
  if(!worldElement)
  {
    dterr << "[SkelParser::readWorld] File named [" << filename << "] could "
          << "not be parsed!\n";
    return nullptr;
  }

  return readWorldDART510(worldElement);
}

//==============================================================================
simulation::WorldPtr SkelParser::readWorldXML(const std::string& xmlString)
{
  tinyxml2::XMLDocument _dartXML;
  if(_dartXML.Parse(xmlString.c_str()) != tinyxml2::XML_SUCCESS)
  {
    _dartXML.PrintError();
    return nullptr;
  }

  tinyxml2::XMLElement* worldElement = checkFormatAndGetWorldElement(_dartXML);
  if(!worldElement)
  {
    dterr << "[SkelParser::readWorldXML] XML String could not be parsed!\n";
    return nullptr;
  }

  return readWorldDART510(worldElement);
}

//==============================================================================
dynamics::SkeletonPtr SkelParser::readSkeleton(const std::string& filename)
{
  //--------------------------------------------------------------------------
  // Load xml and create Document
  tinyxml2::XMLDocument _dartFile;
  try
  {
    openXMLFile(_dartFile, filename.c_str());
  }
  catch(std::exception const& e)
  {
    std::cout << "LoadFile [" << filename << "] Fails: "
              << e.what() << std::endl;
    return nullptr;
  }

  //--------------------------------------------------------------------------
  // Load DART
  tinyxml2::XMLElement* skelElement = nullptr;
  skelElement = _dartFile.FirstChildElement("skel");
  if (skelElement == nullptr)
  {
    dterr << "Skel file[" << filename << "] does not contain <skel> as the "
          << "element.\n";
    return nullptr;
  }

  //--------------------------------------------------------------------------
  // Load World
  tinyxml2::XMLElement* skeletonElement = nullptr;
  skeletonElement = skelElement->FirstChildElement("skeleton");
  if (skeletonElement == nullptr)
  {
    dterr << "Skel file[" << filename
          << "] does not contain <skeleton> element "
          <<"under <skel> element.\n";
    return nullptr;
  }

  return readSkeletonDART510(skeletonElement);
}

//==============================================================================
void reportDeprecatedFunctionCalled(const std::string& functionName)
{
  dterr << "[SkelParser::" << functionName << "] Deprecated function called.\n";
  assert(false);
}

//==============================================================================
simulation::WorldPtr SkelParser::readWorld(tinyxml2::XMLElement*)
{
  reportDeprecatedFunctionCalled("readWorld");
  return simulation::WorldPtr();
}

//==============================================================================
dynamics::SkeletonPtr SkelParser::readSkeleton(tinyxml2::XMLElement*)
{
  reportDeprecatedFunctionCalled("readSkeleton");
  return dynamics::SkeletonPtr();
}

//==============================================================================
SkelParser::SkelBodyNode SkelParser::readBodyNode(
    tinyxml2::XMLElement*, const Eigen::Isometry3d&)
{
  reportDeprecatedFunctionCalled("readSkeleton");
  return SkelBodyNode();
}

//==============================================================================
SkelParser::SkelBodyNode SkelParser::readSoftBodyNode(
    tinyxml2::XMLElement*, const Eigen::Isometry3d&)
{
  reportDeprecatedFunctionCalled("readSkeleton");
  return SkelBodyNode();
}

//==============================================================================
dynamics::ShapePtr SkelParser::readShape(tinyxml2::XMLElement* vizEle,
                                         const std::string& bodyName)
{
  reportDeprecatedFunctionCalled("readSkeleton");
  return dynamics::ShapePtr();
}

//==============================================================================
dynamics::Marker::Properties SkelParser::readMarker(tinyxml2::XMLElement*)
{
  reportDeprecatedFunctionCalled("readSkeleton");
  return dynamics::Marker::Properties();
}

//==============================================================================
void SkelParser::readJoint(
    tinyxml2::XMLElement*,
    const BodyMap&,
    JointMap&,
    IndexToJoint&,
    JointToIndex&)
{
  reportDeprecatedFunctionCalled("readSkeleton");
}

//==============================================================================
SkelParser::JointPropPtr SkelParser::readRevoluteJoint(
    tinyxml2::XMLElement* /*_jointElement*/,
    SkelJoint& /*_joint*/,
    const std::string& /*_name*/)
{
  reportDeprecatedFunctionCalled("readRevoluteJoint");
  return nullptr;
}

//==============================================================================
SkelParser::JointPropPtr SkelParser::readPrismaticJoint(
    tinyxml2::XMLElement* /*_jointElement*/,
    SkelJoint& /*_joint*/,
    const std::string& /*_name*/)
{
  reportDeprecatedFunctionCalled("readPrismaticJoint");
  return nullptr;
}

//==============================================================================
SkelParser::JointPropPtr SkelParser::readScrewJoint(
    tinyxml2::XMLElement* /*_jointElement*/,
    SkelJoint& /*_joint*/,
    const std::string& /*_name*/)
{
  reportDeprecatedFunctionCalled("readScrewJoint");
  return nullptr;
}

//==============================================================================
SkelParser::JointPropPtr SkelParser::readUniversalJoint(
    tinyxml2::XMLElement* /*_jointElement*/,
    SkelJoint& /*_joint*/,
    const std::string& /*_name*/)
{
  reportDeprecatedFunctionCalled("readUniversalJoint");
  return nullptr;
}

//==============================================================================
SkelParser::JointPropPtr SkelParser::readBallJoint(
    tinyxml2::XMLElement* /*_jointElement*/,
    SkelJoint& /*_joint*/,
    const std::string& /*_name*/)
{
  reportDeprecatedFunctionCalled("readBallJoint");
  return nullptr;
}

//==============================================================================
SkelParser::JointPropPtr SkelParser::readEulerJoint(
    tinyxml2::XMLElement* /*_jointElement*/,
    SkelJoint& /*_joint*/,
    const std::string& /*_name*/)
{
  reportDeprecatedFunctionCalled("readEulerJoint");
  return nullptr;
}

//==============================================================================
SkelParser::JointPropPtr SkelParser::readTranslationalJoint(
    tinyxml2::XMLElement* /*_jointElement*/,
    SkelJoint& /*_joint*/,
    const std::string& /*_name*/)
{
  reportDeprecatedFunctionCalled("readTranslationalJoint");
  return nullptr;
}

//==============================================================================
SkelParser::JointPropPtr SkelParser::readPlanarJoint(
    tinyxml2::XMLElement* /*_jointElement*/,
    SkelJoint& /*_joint*/,
    const std::string& /*_name*/)
{
  reportDeprecatedFunctionCalled("readPlanarJoint");
  return nullptr;
}

//==============================================================================
SkelParser::JointPropPtr SkelParser::readFreeJoint(
    tinyxml2::XMLElement* /*_jointElement*/,
    SkelJoint& /*_joint*/,
    const std::string& /*_name*/)
{
  reportDeprecatedFunctionCalled("readFreeJoint");
  return nullptr;
}

//==============================================================================
SkelParser::JointPropPtr SkelParser::readWeldJoint(
    tinyxml2::XMLElement* /*_jointElement*/,
    SkelJoint& /*_joint*/,
    const std::string&)
{
  reportDeprecatedFunctionCalled("readWeldJoint");
  return nullptr;
}

}  // namespace utils
}  // namespace dart

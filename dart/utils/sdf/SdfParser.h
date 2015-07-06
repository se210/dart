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

#ifndef DART_UTILS_SDFPARSER_H_
#define DART_UTILS_SDFPARSER_H_

#include <map>
#include <string>
#include <functional>
#include <Eigen/Dense>
#include <tinyxml2.h>

#include "dart/common/Deprecated.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/WeldJoint.h"
#include "dart/dynamics/RevoluteJoint.h"
#include "dart/dynamics/PrismaticJoint.h"
#include "dart/dynamics/ScrewJoint.h"
#include "dart/dynamics/UniversalJoint.h"
#include "dart/dynamics/BallJoint.h"
#include "dart/dynamics/TranslationalJoint.h"
#include "dart/dynamics/FreeJoint.h"
#include "dart/simulation/World.h"
#include "dart/utils/Parser.h"

namespace dart {

namespace dynamics {
class Skeleton;
class BodyNode;
class Shape;
class Joint;
class WeldJoint;
class PrismaticJoint;
class RevoluteJoint;
class ScrewJoint;
class UniversalJoint;
class BallJoint;
class EulerXYZJoint;
class EulerJoint;
class TranslationalJoint;
class FreeJoint;
}
namespace simulation {
class World;
}

namespace utils {

class SdfParser
{
public:
  static simulation::WorldPtr readSdfFile(const std::string& _filename);

  static dynamics::SkeletonPtr readSkeleton(const std::string& _fileName);

  DEPRECATED(5.1)
  static simulation::WorldPtr readSdfFile(
      const std::string& _filename,
      std::function<simulation::WorldPtr(tinyxml2::XMLElement*, const std::string&)> xmlReader);

  DEPRECATED(5.1)
  static dynamics::SkeletonPtr readSkeleton(
      const std::string& _fileName,
      std::function<dynamics::SkeletonPtr(
        tinyxml2::XMLElement*, const std::string&)> xmlReader);

  typedef std::shared_ptr<dynamics::BodyNode::Properties> BodyPropPtr;

  struct SDFBodyNode
  {
    BodyPropPtr properties;
    Eigen::Isometry3d initTransform;
    std::string type;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  typedef std::shared_ptr<dynamics::Joint::Properties> JointPropPtr;

  struct SDFJoint
  {
    JointPropPtr properties;
    std::string parentName;
    std::string childName;
    std::string type;
  };

  typedef Eigen::aligned_map<std::string, SDFBodyNode> BodyMap;
  typedef std::map<std::string, SDFJoint> JointMap;

  static simulation::WorldPtr readWorld(
      tinyxml2::XMLElement* _worldElement,
      const std::string& _skelPath);

  DEPRECATED(5.1)
  static simulation::WorldPtr readWorld(
      tinyxml2::XMLElement* _worldElement,
      const std::string& _skelPath,
      std::function<dynamics::SkeletonPtr(tinyxml2::XMLElement*,
                                          const std::string&)> skeletonReader);

  static void readPhysics(tinyxml2::XMLElement* _physicsElement,
                          simulation::WorldPtr _world);

  static dynamics::SkeletonPtr readSkeleton(
      tinyxml2::XMLElement* _skeletonElement,
      const std::string& _skelPath);

  DEPRECATED(5.1)
  static dynamics::SkeletonPtr readSkeleton(
      tinyxml2::XMLElement* _skeletonElement,
      const std::string& _skelPath,

      std::function<SDFBodyNode (tinyxml2::XMLElement*,
                                 const Eigen::Isometry3d&,
                                 const std::string&)> bodyReader,

      std::function<bool(dynamics::SkeletonPtr,
                         dynamics::BodyNode*,
                         const SDFJoint&,
                         const SDFBodyNode&)> pairCreator);

  static bool createPair(dynamics::SkeletonPtr skeleton,
                         dynamics::BodyNode* parent,
                         const SDFJoint& newJoint,
                         const SDFBodyNode& newBody);

  static bool createSoftPair(
      dynamics::SkeletonPtr skeleton,
      dynamics::BodyNode* parent,
      const SDFJoint& newJoint,
      const SDFBodyNode& newBody);

  enum NextResult
  {
    VALID,
    CONTINUE,
    BREAK,
    CREATE_FREEJOINT_ROOT
  };

  static NextResult getNextJointAndNodePair(
      JointMap::iterator& it,
      BodyMap::const_iterator& child,
      dynamics::BodyNode*& parent,
      const dynamics::SkeletonPtr skeleton,
      JointMap& sdfJoints,
      const BodyMap& sdfBodyNodes);

  static dynamics::SkeletonPtr makeSkeleton(
      tinyxml2::XMLElement* _skeletonElement,
      Eigen::Isometry3d& skeletonFrame);

  template <class NodeType>
  static std::pair<dynamics::Joint*,dynamics::BodyNode*> createJointAndNodePair(
      dynamics::SkeletonPtr skeleton,
      dynamics::BodyNode* parent,
      const SDFJoint& joint,
      const SDFBodyNode& node)
  {
    const std::string& type = joint.type;

    if (std::string("prismatic") == type)
      return skeleton->createJointAndBodyNodePair<dynamics::PrismaticJoint>(parent,
                                                                            static_cast<const dynamics::PrismaticJoint::Properties&>(*joint.properties),
                                                                            static_cast<const typename NodeType::Properties&>(*node.properties));
    else if (std::string("revolute") == type)
      return skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(parent,
                                                                           static_cast<const dynamics::RevoluteJoint::Properties&>(*joint.properties),
                                                                           static_cast<const typename NodeType::Properties&>(*node.properties));
    else if (std::string("screw") == type)
      return skeleton->createJointAndBodyNodePair<dynamics::ScrewJoint>(parent,
                                                                        static_cast<const dynamics::ScrewJoint::Properties&>(*joint.properties),
                                                                        static_cast<const typename NodeType::Properties&>(*node.properties));
    else if (std::string("revolute2") == type)
      return skeleton->createJointAndBodyNodePair<dynamics::UniversalJoint>(parent,
                                                                            static_cast<const dynamics::UniversalJoint::Properties&>(*joint.properties),
                                                                            static_cast<const typename NodeType::Properties&>(*node.properties));
    else if (std::string("ball") == type)
      return skeleton->createJointAndBodyNodePair<dynamics::BallJoint>(parent,
                                                                       static_cast<const dynamics::BallJoint::Properties&>(*joint.properties),
                                                                       static_cast<const typename NodeType::Properties&>(*node.properties));
    else if (std::string("free") == type)
      return skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(parent,
                                                                       static_cast<const dynamics::FreeJoint::Properties&>(*joint.properties),
                                                                       static_cast<const typename NodeType::Properties&>(*node.properties));

    dterr << "[SdfParser::createJointAndNodePair] Unsupported Joint type encountered: "
          << type << ". Please report this as a bug! We will now quit parsing.\n";
    return std::pair<dynamics::Joint*, dynamics::BodyNode*>(nullptr, nullptr);
  }

  static BodyMap readAllBodyNodes(
      tinyxml2::XMLElement* _skeletonElement,
      const std::string& _skelPath,
      const Eigen::Isometry3d& skeletonFrame);

  static BodyMap readAllBodyNodes(
      tinyxml2::XMLElement* _skeletonElement,
      const std::string& _skelPath,
      const Eigen::Isometry3d& skeletonFrame,

      std::function<SDFBodyNode(tinyxml2::XMLElement*,
                                const Eigen::Isometry3d&,
                                const std::string&)> bodyReader);

  static SDFBodyNode readBodyNode(
      tinyxml2::XMLElement* _bodyNodeElement,
      const Eigen::Isometry3d& _skeletonFrame,
      const std::string& _skelPath);

  static SDFBodyNode readSoftBodyNode(
      tinyxml2::XMLElement* _softBodyNodeElement,
      const Eigen::Isometry3d& _skeletonFrame,
      const std::string& _skelPath);

  static dynamics::ShapePtr readShape(
      tinyxml2::XMLElement* _shapelement,
      const std::string& _skelPath);

  static JointMap readAllJoints(
      tinyxml2::XMLElement* _skeletonElement,
      const Eigen::Isometry3d& skeletonFrame,
      const BodyMap& sdfBodyNodes);

  static SDFJoint readJoint(tinyxml2::XMLElement* _jointElement,
                            const BodyMap& _bodies,
                            const Eigen::Isometry3d& _skeletonFrame);

  static dynamics::WeldJoint::Properties readWeldJoint(
      tinyxml2::XMLElement* _jointElement,
      const Eigen::Isometry3d& _parentModelFrame,
      const std::string& _name);

  static dynamics::RevoluteJoint::Properties readRevoluteJoint(
      tinyxml2::XMLElement* _revoluteJointElement,
      const Eigen::Isometry3d& _parentModelFrame,
      const std::string& _name);

  static dynamics::PrismaticJoint::Properties readPrismaticJoint(
      tinyxml2::XMLElement* _jointElement,
      const Eigen::Isometry3d& _parentModelFrame,
      const std::string& _name);

  static dynamics::ScrewJoint::Properties readScrewJoint(
      tinyxml2::XMLElement* _jointElement,
      const Eigen::Isometry3d& _parentModelFrame,
      const std::string& _name);

  static dynamics::UniversalJoint::Properties readUniversalJoint(
      tinyxml2::XMLElement* _jointElement,
      const Eigen::Isometry3d& _parentModelFrame,
      const std::string& _name);

  static dynamics::BallJoint::Properties readBallJoint(
      tinyxml2::XMLElement* _jointElement,
      const Eigen::Isometry3d& _parentModelFrame,
      const std::string& _name);

  static dynamics::EulerJoint* readEulerJoint(
      tinyxml2::XMLElement* _jointElement,
      const Eigen::Isometry3d& _parentModelFrame,
      const std::string& _name);

  static dynamics::TranslationalJoint::Properties readTranslationalJoint(
      tinyxml2::XMLElement* _jointElement,
      const Eigen::Isometry3d& _parentModelFrame,
      const std::string& _name);

  static dynamics::FreeJoint::Properties readFreeJoint(
      tinyxml2::XMLElement* _jointElement,
      const Eigen::Isometry3d& _parentModelFrame,
      const std::string& _name);
};

using SoftSdfParser DEPRECATED(5.1) = SdfParser;

} // namespace utils
} // namespace dart

#endif // #ifndef DART_UTILS_SDFPARSER_H_

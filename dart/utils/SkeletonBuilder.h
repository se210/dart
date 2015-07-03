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

#ifndef DART_UTILS_SKELETONBUILDER_H_
#define DART_UTILS_SKELETONBUILDER_H_

#include <algorithm>
#include <string>

#include "dart/config.h"
#include "dart/common/common.h"
#include "dart/math/math.h"
#include "dart/collision/collision.h"
#include "dart/constraint/constraint.h"
#include "dart/dynamics/dynamics.h"
#include "dart/simulation/simulation.h"

namespace dart {
namespace utils {

class SkeletonBuilder
{
public:
  using SkeletonPropPtr
      = std::shared_ptr<dynamics::Skeleton::Properties>;

  using SkeletonPropConstPtr
      = std::shared_ptr<const dynamics::Skeleton::Properties>;

  using BodyNodePropPtr = std::shared_ptr<dynamics::BodyNode::Properties>;

  template <class JointT = dynamics::FreeJoint>
  using JointPropPtr = std::shared_ptr<typename JointT::Properties>;

  enum TransformationDataType
  {
    BODYNODE_AND_CHILDTOJOINT,
    PARENTTOJOINT_AND_CHILDTOJOINT
  };

  struct BodyNodeBuildData
  {
    BodyNodePropPtr properties;
    Eigen::Isometry3d initTransform;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  struct JointBuildData
  {
    JointPropPtr<dynamics::Joint> properties;
    std::string parentName;
    std::string childName;
    Eigen::VectorXd position;
    Eigen::VectorXd velocity;
    Eigen::VectorXd acceleration;
    Eigen::VectorXd force;
  };

  SkeletonBuilder(
      const std::string& name = "Skeleton Builder",
      TransformationDataType transformDataType = BODYNODE_AND_CHILDTOJOINT);

  ~SkeletonBuilder();

  SkeletonPropPtr getSkeletonProperties();

  SkeletonPropConstPtr getSkeletonProperties() const;

  template <class BodyNodeT = dynamics::BodyNode>
  bool addBodyNode(const BodyNodeBuildData& buildData);

  template <class BodyNodeT = dynamics::BodyNode>
  bool addBodyNode(
      const BodyNodePropPtr& bodyNodeProp,
      const Eigen::Isometry3d& initTransform = Eigen::Isometry3d::Identity());

  /// Create BodyNode property set with given name
  template <class BodyNodeT = dynamics::BodyNode>
  bool addBodyNode(
      const std::string& name,
      const Eigen::Isometry3d& initTransform = Eigen::Isometry3d::Identity());

  // TODO(JS): addBodyNodes()

  std::size_t getNumBodyNodes() const;

  bool getBodyNodeBuildData(
      const std::string& name, BodyNodeBuildData& buildData);

  bool removeBodyNode(const std::string& name);

  void removeAllBodyNodes();

  template <class JointT = dynamics::FreeJoint>
  bool addJoint(const JointBuildData& buildData, bool checkParentChild = false);

  template <class JointT = dynamics::FreeJoint>
  bool addJoint(
      const JointPropPtr<JointT>& jointProp,
      const std::string& parentName,
      const std::string& childName,
      bool checkParentChild = false);

  template <class JointT = dynamics::FreeJoint>
  bool addJoint(
      const std::string& name,
      const std::string& parentName,
      const std::string& childName,
      bool checkParentChild = false);

  // TODO(JS): addBodyNodes()

  std::size_t getNumJoints() const;

  bool removeJoint(const std::string& name);

  void removeAllJoints();

  dynamics::SkeletonPtr build();

protected:
  struct BodyNodeBuildDataInternal : BodyNodeBuildData
  {
    std::string type;
  };

  struct JointBuildDataInternal : JointBuildData
  {
    std::string type;
  };

  enum NextResult
  {
    VALID,
    CONTINUE,
    BREAK,
    CREATE_FREEJOINT_ROOT
  };

  // first: BodyNode name | second: BodyNode information
  using BodyNodeMap = Eigen::aligned_map<std::string, BodyNodeBuildDataInternal>;

  // first: Child BodyNode name | second: Joint information
  using JointMap = std::map<std::string, JointBuildDataInternal>;

  // first: Order that Joint appears in file | second: Child BodyNode name
  using IndexToChildBodyNodeName = std::map<std::size_t, std::string>;

  // first: Child BodyNode name | second: Order that Joint appears in file
  using ChildBodyNodeNameToIndex = std::map<std::string, std::size_t>;

  // ---- Member functions

  bool addBodyNode(const BodyNodeBuildDataInternal& buildData);

  bool addJoint(const JointBuildDataInternal& buildData,
                bool checkParentChild = true);

  NextResult getNextJointAndNodePair(
      SkeletonBuilder::JointMap::const_iterator& it,
      SkeletonBuilder::BodyNodeMap::const_iterator& child,
      dynamics::BodyNode*& parent,
      const dynamics::SkeletonPtr skeleton,
      const JointMap& jointMap,
      const BodyNodeMap& bodyNodeMap);

  template <typename BodyType>
  std::pair<dynamics::Joint*, dynamics::BodyNode*> createJointAndNodePair(
      dynamics::SkeletonPtr skeleton,
      dynamics::BodyNode* parent,
      const JointBuildDataInternal& joint,
      const typename BodyType::Properties& body);

  bool createJointAndNodePair(
      dynamics::SkeletonPtr skeleton,
      dynamics::BodyNode* parent,
      const JointBuildDataInternal& joint,
      const BodyNodeBuildDataInternal& body);

  SkeletonPropPtr mSkeletonProp;

  TransformationDataType mTransformDataType;

  BodyNodeMap mBodyNodeMap;

  JointMap mJointMap;

  IndexToChildBodyNodeName mIndexToChildBodyNodeName;

  ChildBodyNodeNameToIndex mChildBodyNodeNameToIndex;
};

#include "dart/utils/detail/SkeletonBuilder.h"

} // namespace utils
} // namespace dart

#endif  // #ifndef DART_UTILS_SKELETONBUILDER_H_

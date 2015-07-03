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

#include "dart/utils/SkeletonBuilder.h"

namespace dart {
namespace utils {

//==============================================================================
SkeletonBuilder::SkeletonBuilder(
    const std::string& name,
    SkeletonBuilder::TransformationDataType transformDataType)
  : mSkeletonProp(new dynamics::Skeleton::Properties(name)),
    mTransformDataType(transformDataType)
{
}

//==============================================================================
SkeletonBuilder::~SkeletonBuilder()
{
  // Do nothing
}

//==============================================================================
SkeletonBuilder::SkeletonPropPtr SkeletonBuilder::getSkeletonProperties()
{
  return mSkeletonProp;
}

//==============================================================================
SkeletonBuilder::SkeletonPropConstPtr
SkeletonBuilder::getSkeletonProperties() const
{
  return mSkeletonProp;
}

//==============================================================================
std::size_t SkeletonBuilder::getNumBodyNodes() const
{
  return mBodyNodeMap.size();
}

//==============================================================================
bool SkeletonBuilder::getBodyNodeBuildData(
    const std::string& name, BodyNodeBuildData& buildData)
{
  BodyNodeMap::const_iterator internalDataIt = mBodyNodeMap.find(name);

  if (internalDataIt == mBodyNodeMap.end())
    return false;

  BodyNodeBuildDataInternal internalData = internalDataIt->second;

  buildData.properties    = internalData.properties;
  buildData.initTransform = internalData.initTransform;

  return true;
}

//==============================================================================
bool SkeletonBuilder::removeBodyNode(const std::string& name)
{
  const std::size_t numRemoved = mBodyNodeMap.erase(name);

  assert(numRemoved <= 1);

  if (numRemoved == 0)
  {
    dtwarn << "[SkeletonBuilder::removeBodyNode] Attemping to remove "
           << "unregistered BodyNode [" << name << "]." << std::endl;
    return false;
  }

  return true;
}

//==============================================================================
void SkeletonBuilder::removeAllBodyNodes()
{
  mBodyNodeMap.clear();
}

//==============================================================================
std::size_t SkeletonBuilder::getNumJoints() const
{
  return mJointMap.size();
}

//==============================================================================
bool SkeletonBuilder::removeJoint(const std::string& name)
{
  const std::size_t numRemoved = mJointMap.erase(name);

  assert(numRemoved <= 1);

  if (numRemoved == 0)
  {
    dtwarn << "[SkeletonBuilder::removeJoint] Attemping to remove "
           << "unregistered Joint [" << name << "]." << std::endl;
    return false;
  }

  return true;
}

//==============================================================================
void SkeletonBuilder::removeAllJoints()
{
  mJointMap.clear();
}

//==============================================================================
dynamics::SkeletonPtr SkeletonBuilder::build()
{
  // TODO: build graph and check validity

  // Use an empty string (rather than "world") to indicate that the joint has no
  // parent.
  const bool hasBodyNodeNamedAsWorld
      = (mBodyNodeMap.find("world") != mBodyNodeMap.end());
  if (!hasBodyNodeNamedAsWorld)
  {
    for (auto& jointItr : mJointMap)
    {
      if (jointItr.second.parentName.compare("world") == 0)
        jointItr.second.parentName.clear();
    }
  }

  dynamics::SkeletonPtr newSkel = dynamics::Skeleton::create(*mSkeletonProp);

  JointMap::const_iterator currJointMapItr
      = mJointMap.find(mIndexToChildBodyNodeName.begin()->second);
  BodyNodeMap::const_iterator child;
  dynamics::BodyNode* parent;

  while (currJointMapItr != mJointMap.end())
  {
    NextResult result = getNextJointAndNodePair(
          currJointMapItr, child, parent, newSkel, mJointMap, mBodyNodeMap);

    if (BREAK == result)
    {
      break;
    }
    else if (CONTINUE == result)
    {
      // Create the parent before creating the current Joint
      continue;
    }
    else if (CREATE_FREEJOINT_ROOT == result)
    {
      // If a root FreeJoint is needed for the parent of the current joint, then
      // create it
      BodyNodeMap::const_iterator rootNode
          = mBodyNodeMap.find(currJointMapItr->second.parentName);
      JointBuildDataInternal rootJoint;
      rootJoint.properties =
          Eigen::make_aligned_shared<dynamics::FreeJoint::Properties>(
            dynamics::Joint::Properties("root"/*, rootNode->second.initTransform*/));
      rootJoint.type = dynamics::FreeJoint::getStaticType();

      if (!createJointAndNodePair(newSkel, nullptr, rootJoint, rootNode->second))
        break;

      continue;
    }

    if (!createJointAndNodePair(
          newSkel, parent, currJointMapItr->second, child->second))
    {
      break;
    }

    ChildBodyNodeNameToIndex::iterator index
        = mChildBodyNodeNameToIndex.find(currJointMapItr->first);
    mIndexToChildBodyNodeName.erase(index->second);
    mChildBodyNodeNameToIndex.erase(index);
    mJointMap.erase(currJointMapItr);

    IndexToChildBodyNodeName::iterator nextJoint
        = mIndexToChildBodyNodeName.begin();
    if (nextJoint == mIndexToChildBodyNodeName.end())
      break;

    currJointMapItr = mJointMap.find(nextJoint->second);
  }

  return newSkel;
}

//==============================================================================
bool SkeletonBuilder::addBodyNode(const BodyNodeBuildDataInternal& buildData)
{
  if (nullptr == buildData.properties)
    return false;

  // BodyNode name uniqueness check
  if (mBodyNodeMap.find(buildData.properties->mName) != mBodyNodeMap.end())
  {
    dtwarn << "[SkeletonBuilder::addBodyNode] Skeleton named ["
           << mSkeletonProp->mName << "] has "
           << "multiple BodyNodes with the name ["
           << buildData.properties->mName
           << "], but BodyNode names must be "
           << "unique! We will discard all BodyNodes with a repeated name.\n";
    return false;
  }

  mBodyNodeMap[buildData.properties->mName] = buildData;

  return true;
}

//==============================================================================
bool SkeletonBuilder::addJoint(const JointBuildDataInternal& buildData,
                               bool checkParentChild)
{
  if (nullptr == buildData.properties)
    return false;

  // Empty parent BodyNode name and is regarded the world.

  // However, child BodyNode name shouldn't be empty.
  if (buildData.childName.empty())
  {
    dtwarn << "[SkeletonBuilder::addJoint] Joint named ["
           << buildData.properties->mName << "] has empty child BodyNode name. "
           << "We will discard all Joints with a empty child BodyNode name."
           << std::endl;
    return false;
  }

  // Check if parent/child BodyNode is already added
  if (checkParentChild)
  {
    // -- Parent

    std::string nonConstParentName = buildData.parentName;

    // Use an empty string (rather than "world") to indicate that the joint has
    // no parent
    if (nonConstParentName == std::string("world")
        && mBodyNodeMap.find("world") == mBodyNodeMap.end())
    {
      nonConstParentName.clear();
    }

    if (mBodyNodeMap.find(nonConstParentName) == mBodyNodeMap.end()
        && !nonConstParentName.empty())
    {
      dtwarn << "[SkeletonBuilder::addJoint] Joint named ["
             << buildData.properties->mName
             << "] is missing a parent BodyNode [" << nonConstParentName << "]!"
             << std::endl;
      return false;
    }

    // -- Child

    if (mBodyNodeMap.find(buildData.childName) == mBodyNodeMap.end())
    {
      dtwarn << "[SkeletonBuilder::addJoint] Joint named ["
             << buildData.properties->mName
             << "] is missing a child BodyNode [" << buildData.childName
             << "]!" << std::endl;
      return false;
    }
  }

  JointMap::iterator it = mJointMap.find(buildData.childName);
  if (it != mJointMap.end())
  {
    dtwarn << "[SkeletonBuilder::addJoint] BodyNode named ["
          << buildData.childName
          << "] has been assigned two parent Joints: ["
          << it->second.properties->mName << "] and ["
          << buildData.properties->mName << "]. A "
          << "BodyNode must have exactly one parent Joint. ["
          << buildData.properties->mName << "] "
          << "will be discarded!\n";
    return false;
  }

  mJointMap[buildData.childName] = buildData;

  // Keep track of when each joint added
  std::size_t nextIndex;
  IndexToChildBodyNodeName::reverse_iterator lastIndex
      = mIndexToChildBodyNodeName.rbegin();
  if (lastIndex == mIndexToChildBodyNodeName.rend())
    nextIndex = 0;
  else
    nextIndex = lastIndex->first + 1;

  mIndexToChildBodyNodeName[nextIndex]           = buildData.childName;
  mChildBodyNodeNameToIndex[buildData.childName] = nextIndex;

  return true;
}

//==============================================================================
SkeletonBuilder::NextResult SkeletonBuilder::getNextJointAndNodePair(
          SkeletonBuilder::JointMap::const_iterator&    it,
          SkeletonBuilder::BodyNodeMap::const_iterator& child,
          dynamics::BodyNode*&                          parent,
    const dynamics::SkeletonPtr                         skeleton,
    const SkeletonBuilder::JointMap&                    jointMap,
    const SkeletonBuilder::BodyNodeMap&                 bodyNodeMap)
{
  NextResult result = VALID;

  const SkeletonBuilder::JointBuildData& jointBuildData = it->second;
  parent = skeleton->getBodyNode(jointBuildData.parentName);

  // If the parent BodyNode hasn't been added to the Skeleton yet, add the
  // parent Joint-BodyNode pair first.
  if (nullptr == parent && !jointBuildData.parentName.empty())
  {
    // Find the properties of the parent Joint of the current Joint, because it
    // does not seem to be created yet.
    SkeletonBuilder::JointMap::const_iterator check_parent_joint
        = jointMap.find(jointBuildData.parentName);
    if (check_parent_joint == jointMap.end())
    {
      SkeletonBuilder::BodyNodeMap::const_iterator check_parent_node
          = bodyNodeMap.find(jointBuildData.parentName);
      if (check_parent_node == bodyNodeMap.end())
      {
        dterr << "[SkeletonBuilder::getNextJointAndNodePair] Could not find BodyNode "
              << "named [" << jointBuildData.parentName << "] requested as parent of "
              << "the Joint named [" << jointBuildData.properties->mName << "]. We will "
              << "now quit parsing.\n";
        return BREAK;
      }

      // If the current Joint has a parent BodyNode but does not have a parent
      // Joint, then we need to create a FreeJoint for the parent BodyNode.
      result = CREATE_FREEJOINT_ROOT;
    }
    else
    {
      it = check_parent_joint;
      return CONTINUE; // Create the parent before creating the current Joint
    }
  }

  // Find the child node of this Joint, so we can create them together
  child = bodyNodeMap.find(jointBuildData.childName);
  if (child == bodyNodeMap.end())
  {
    dterr << "[SkeletonBuilder::getNextJointAndNodePair] Could not find BodyNode "
          << "named [" << jointBuildData.childName << "] requested as child of Joint ["
          << jointBuildData.properties->mName << "]. This should not be possible! "
          << "We will now quit parsing. Please report this bug!\n";
    return BREAK;
  }

  return result;
}

//==============================================================================
template <typename BodyType>
std::pair<dynamics::Joint*, dynamics::BodyNode*>
SkeletonBuilder::createJointAndNodePair(
    dynamics::SkeletonPtr skeleton,
    dynamics::BodyNode* parent,
    const JointBuildDataInternal& joint,
    const typename BodyType::Properties& body)
{
  if (dynamics::WeldJoint::getStaticType() == joint.type)
  {
    return skeleton->createJointAndBodyNodePair<dynamics::WeldJoint, BodyType>(parent,
      static_cast<const dynamics::WeldJoint::Properties&>(*joint.properties), body);
  }
  else if (dynamics::PrismaticJoint::getStaticType() == joint.type)
  {
    return skeleton->createJointAndBodyNodePair<dynamics::PrismaticJoint, BodyType>(parent,
      static_cast<const dynamics::PrismaticJoint::Properties&>(*joint.properties), body);
  }
  else if (dynamics::RevoluteJoint::getStaticType() == joint.type)
  {
    return skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint, BodyType>(parent,
      static_cast<const dynamics::RevoluteJoint::Properties&>(*joint.properties), body);
  }
  else if (dynamics::UniversalJoint::getStaticType() == joint.type)
  {
    return skeleton->createJointAndBodyNodePair<dynamics::UniversalJoint, BodyType>(parent,
      static_cast<const dynamics::UniversalJoint::Properties&>(*joint.properties), body);
  }
  else if (dynamics::BallJoint::getStaticType() == joint.type)
  {
    return skeleton->createJointAndBodyNodePair<dynamics::BallJoint, BodyType>(parent,
      static_cast<const dynamics::BallJoint::Properties&>(*joint.properties), body);
  }
  else if (dynamics::EulerJoint::getStaticType() == joint.type)
  {
    return skeleton->createJointAndBodyNodePair<dynamics::EulerJoint, BodyType>(parent,
      static_cast<const dynamics::EulerJoint::Properties&>(*joint.properties), body);
  }
  else if (dynamics::TranslationalJoint::getStaticType() == joint.type)
  {
    return skeleton->createJointAndBodyNodePair<dynamics::TranslationalJoint, BodyType>(parent,
      static_cast<const dynamics::TranslationalJoint::Properties&>(*joint.properties), body);
  }
  else if (dynamics::PlanarJoint::getStaticType() == joint.type)
  {
    return skeleton->createJointAndBodyNodePair<dynamics::PlanarJoint, BodyType>(parent,
      static_cast<const dynamics::PlanarJoint::Properties&>(*joint.properties), body);
  }
  else if (dynamics::FreeJoint::getStaticType() == joint.type)
  {
    return skeleton->createJointAndBodyNodePair<dynamics::FreeJoint, BodyType>(parent,
      static_cast<const dynamics::FreeJoint::Properties&>(*joint.properties), body);
  }
  else
  {
    dterr << "[SkeletonBuilder::createJointAndNodePair] Unsupported Joint type ("
          << joint.type << ") for Joint named [" << joint.properties->mName
          << "]! It will be discarded.\n";
    return std::pair<dynamics::Joint*, dynamics::BodyNode*>(nullptr, nullptr);
  }
}

//==============================================================================
bool SkeletonBuilder::createJointAndNodePair(
    dynamics::SkeletonPtr skeleton,
    dynamics::BodyNode* parent,
    const JointBuildDataInternal& joint,
    const BodyNodeBuildDataInternal& body)
{
  std::pair<dynamics::Joint*, dynamics::BodyNode*> pair;

  // Compute parent-to-joint transform for each joint
  if (mTransformDataType == BODYNODE_AND_CHILDTOJOINT)
  {
    Eigen::Isometry3d parentWorld = Eigen::Isometry3d::Identity();

    if (parent)
      parentWorld = parent->getTransform();

    const Eigen::Isometry3d parentToJoint
        = parentWorld.inverse()
          * body.initTransform
          * joint.properties->mT_ChildBodyToJoint;

    joint.properties->mT_ParentBodyToJoint = parentToJoint;
  }

  if (dynamics::BodyNode::getStaticType() == body.type)
  {
    pair = createJointAndNodePair<dynamics::BodyNode>(skeleton, parent, joint,
      static_cast<const dynamics::BodyNode::Properties&>(*body.properties));
  }
  else if (dynamics::SoftBodyNode::getStaticType() == body.type)
  {
    pair = createJointAndNodePair<dynamics::SoftBodyNode>(skeleton, parent, joint,
      static_cast<const dynamics::SoftBodyNode::Properties&>(*body.properties));
  }
  else
  {
    dterr << "[SkeletonBuilder::createJointAndNodePair] Invalid type ("
          << body.type << ") for BodyNode named [" << body.properties->mName
          << "]" << std::endl;
    return false;
  }

  if (pair.first == nullptr || pair.second == nullptr)
    return false;

  dynamics::Joint* newJoint = pair.first;
  const int numDofs = static_cast<int>(newJoint->getNumDofs());

  if (joint.position.size() != numDofs)
    newJoint->setPositions(Eigen::VectorXd::Zero(numDofs));
  else
    newJoint->setPositions(joint.position);

  if (joint.velocity.size() != numDofs)
    newJoint->setVelocities(Eigen::VectorXd::Zero(numDofs));
  else
    newJoint->setVelocities(joint.velocity);

  if (joint.acceleration.size() != numDofs)
    newJoint->setAccelerations(Eigen::VectorXd::Zero(numDofs));
  else
    newJoint->setAccelerations(joint.acceleration);

  if (joint.force.size() != numDofs)
    newJoint->setForces(Eigen::VectorXd::Zero(numDofs));
  else
    newJoint->setForces(joint.force);

  return true;
}

} // namespace utils
} // namespace dart


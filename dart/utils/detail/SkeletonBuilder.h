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

#ifndef DART_UTILS_DETAIL_SKELETONBUILDER_H_
#define DART_UTILS_DETAIL_SKELETONBUILDER_H_

//==============================================================================
template <class BodyNodeT>
bool SkeletonBuilder::addBodyNode(const BodyNodeBuildData& buildData)
{
  BodyNodeBuildDataInternal buildDataInternal;
  buildDataInternal.properties    = buildData.properties;
  buildDataInternal.initTransform = buildData.initTransform;
  buildDataInternal.type          = BodyNodeT::getStaticType();

  return addBodyNode(buildDataInternal);
}

//==============================================================================
template <class BodyNodeT>
bool SkeletonBuilder::addBodyNode(
    const BodyNodePropPtr& bodyNodeProp,
    const Eigen::Isometry3d& initTransform)
{
  BodyNodeBuildDataInternal bodyNodeBuildData;
  bodyNodeBuildData.properties    = bodyNodeProp;
  bodyNodeBuildData.initTransform = initTransform;
  bodyNodeBuildData.type          = BodyNodeT::getStaticType();

  return addBodyNode(bodyNodeBuildData);
}

//==============================================================================
// Create BodyNode property set with given name
template <class BodyNodeT>
bool SkeletonBuilder::addBodyNode(
    const std::string& name,
    const Eigen::Isometry3d& initTransform)
{
  std::shared_ptr<typename BodyNodeT::Properties>
      bodyNodeProp(new typename BodyNodeT::Properties());
  bodyNodeProp->mName = name;

  return addBodyNode<BodyNodeT>(bodyNodeProp, initTransform);
}

//==============================================================================
template <class JointT>
bool SkeletonBuilder::addJoint(
    const JointBuildData& buildData,
    bool checkParentChild)
{
  JointBuildDataInternal buildDataInternal;
  buildDataInternal.properties   = buildData.properties;
  buildDataInternal.parentName   = buildData.parentName;
  buildDataInternal.childName    = buildData.childName;
  buildDataInternal.position     = buildData.position;
  buildDataInternal.velocity     = buildData.velocity;
  buildDataInternal.acceleration = buildData.acceleration;
  buildDataInternal.force        = buildData.force;
  buildDataInternal.type         = JointT::getStaticType();

  return addJoint(buildDataInternal, checkParentChild);
}

//==============================================================================
template <class JointT>
bool SkeletonBuilder::addJoint(
    const JointPropPtr<JointT>& jointProp,
    const std::string& parentName,
    const std::string& childName,
    bool checkParentChild)
{
  JointBuildDataInternal buildData;
  buildData.properties   = jointProp;
  buildData.parentName   = parentName;
  buildData.childName    = childName;
  buildData.position     = Eigen::VectorXd::Zero(JointT::getStaticNumDofs());
  buildData.velocity     = Eigen::VectorXd::Zero(JointT::getStaticNumDofs());
  buildData.acceleration = Eigen::VectorXd::Zero(JointT::getStaticNumDofs());
  buildData.force        = Eigen::VectorXd::Zero(JointT::getStaticNumDofs());
  buildData.type         = JointT::getStaticType();

  return addJoint(buildData, checkParentChild);
}

//==============================================================================
template <class JointT>
bool SkeletonBuilder::addJoint(
    const std::string& name,
    const std::string& parentName,
    const std::string& childName,
    bool checkParentChild)
{
  JointPropPtr<JointT> jointProp(new typename JointT::Properties());
  jointProp->mName = name;

  return addJoint<JointT>(jointProp, parentName, childName, checkParentChild);
}


#endif  // #ifndef DART_UTILS_DETAIL_SKELETONBUILDER_H_

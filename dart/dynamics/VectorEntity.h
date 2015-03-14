/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
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

#ifndef DART_DYNAMICS_VECTORENTITY_H_
#define DART_DYNAMICS_VECTORENTITY_H_

#include "Frame.h"

namespace dart {
namespace dynamics {

/// VECTORENTITY_COPIERS is a macro for easily creating copy constructors and
/// assignment operators for classes that use VectorEntity.
///
/// Note that because of const correctness, it is not possible for an Entity to
/// set its parent frame to the parent frame of a  const Entity*, because a
/// non-const Frame* needs to passed to the  setParentFrame() member function.
/// So when copying another VectorEntity using the assignment operator, the
/// parent Frame of the VectorEntity will remain the same and only the relative
/// vector will be altered. It will be altered in such a way that the World
/// vector of this VectorEntity will match the World vector of the VectorEntity
/// that is being copied. For example if you have:
///
/// Point p1, p2;
/// // ... set the values and the parent Frames of p1 and p2 ...
/// p1 = p2;
///
/// then after the assignment operation, p1.wrtWorld() will exactly match
/// p2.wrtWorld() even though their Frames and their relative vectors might be
/// different.
///
/// The copy constructor works similarly. When the copy constructor is used, the
/// newly constructed VectorEntity will have a World vector that matches the
/// VectorEntity that was passed in, but its parent Frame will be based on the
/// second argument, which defaults to Frame::World().
///
/// For more flexibility in copying the values of a VectorEntity, the copy()
/// member function is offered. The copy() function allows you to specify which
/// parameters you want to copy from one VectorEntity to another using bitwise
/// logic. The default is to copy everything. For example, to only copy the
/// relative vector and parent frame:
///
/// Point p1, p2;
/// // ... set the values of and the parent Frames of p1 and p2 ...
/// p1.copy(p2, Point::RELATIVE_VECTOR | Point::PARENT_FRAME);
///
/// Note that the VectorEntity that is passed in cannot be const, even though
/// no changes will be made to it in the process of copying from it.
#define VECTORENTITY_COPIERS(T)                                                 \
  inline T ( const T & copy ## T , Frame* _refFrame = Frame::World())           \
    : Entity(_refFrame, copy ## T .getName(), false),                           \
      VectorEntity(copy ## T .wrt(mParentFrame), _refFrame,                     \
                   copy ## T .getName()+"_copy") { }                            \
                                                                                \
  inline T & operator=( const T & copy ## T ) {                                 \
    mRelativeVector = copy ## T .wrt(mParentFrame);                             \
    return *this;                                                               \
  }                                                                             \
                                                                                \
  inline T & operator=( const Vector& copyVector ) {                            \
    mRelativeVector = copyVector;                                               \
    return *this;                                                               \
  }                                                                             \
                                                                                \
  inline T & copy( T & copy ## T , int copyOptions = EVERYTHING) {              \
    if(RELATIVE_VECTOR & copyOptions) *this = copy ## T .mRelativeVector;       \
    if(PARENT_FRAME & copyOptions) setParentFrame(copy ## T .getParentFrame()); \
    if(NAME & copyOptions) mName = copy ## T .getName();                        \
    if(VISUALIZATIONS & copyOptions) mVizShapes = copy ## T .mVizShapes;        \
                                                                                \
    return *this;                                                               \
  }                                                                             \


/// This class is designed to merge Eigen::Vector types and the Entity concept.
/// It offers convenience functions for implicitly converting the Entity into a
/// raw Eigen::Vector, and for interacting with vector components while still
/// supporting auto-update features.
template <typename Scalar, int Rows>
class VectorEntity : public Detachable
{
public:

  typedef Eigen::Matrix<Scalar, Rows, 1> Vector;

  friend class RefScalarType;

  /// Used with the copy() function to indicate which values should be copied
  /// over.
  enum CopyOptions
  {
    NOTHING         = 0,
    RELATIVE_VECTOR = 1,
    PARENT_FRAME    = 1 << 1,
    NAME            = 1 << 2,
    VISUALIZATIONS  = 1 << 3,

    EVERYTHING      = 0xFF
  };

  /// Class for intercepting user interaction with the components of a
  /// VectorEntity's local vector. Conceptually similar to Eigen's Block
  /// class except this notices when changes are made to vector components,
  /// so it can support auto-updating.
  class RefScalarType
  {
  public:

    /// Constructor
    RefScalarType(VectorEntity<Scalar,Rows>* _vector,
                  size_t _index)
      : mVector(_vector),
        mIndex(_index) { }

    /// Implicit conversion to Scalar type
    operator Scalar () const { return mVector->mRelativeVector[mIndex]; }

    /// Constructor
    RefScalarType& operator=(const Scalar& _value)
    {
      mVector->mRelativeVector[mIndex] = _value;
      mVector->mNeedUpdate = true;
    }

  private:

    /// Pointer to the VectorEntity that this Reference is associated with
    VectorEntity<Scalar,Rows>* mVector;

    /// Index of the component that this Reference is associated with
    size_t mIndex;
  };

  /// Constructor
  VectorEntity(const Vector& _relativeVector = Vector::Zero(),
                  Frame* _refFrame = Frame::World(),
                  const std::string& _name = "vector")
    : Entity(_refFrame, _name, false),
      Detachable(_refFrame, _name, false),
      mRelativeVector(_relativeVector),
      mNeedUpdate(true) { }

  virtual ~VectorEntity() { }

  /// Access a component of the relative vector
  RefScalarType operator[](size_t _index)
  {
    return RefScalarType(this, _index);
  }

  /// Access a component of the relative vector
  const RefScalarType operator[](size_t _index) const
  {
    return RefScalarType(this, _index);
  }

  /// Implicit conversion to Vector type
  operator const Vector& () const { return mRelativeVector; }

  /// Get the value of this vector with respect to a reference frame
  Vector wrt(const Frame* _referenceFrame) const
  {
    if(_referenceFrame == mParentFrame)
      return mRelativeVector;
    else if(_referenceFrame->isWorld())
      return wrtWorld();

    return computeRelativeTo(_referenceFrame);
  }

  /// Get the value of this vector with respect to the World frame
  const Vector& wrtWorld() const
  {
    if(mNeedUpdate)
    {
      computeWorldVector();
      mNeedUpdate = false;
    }

    return mWorldVector;
  }

  /// Returns true iff an update is needed for the World vector
  bool needsUpdate() const
  {
    return mNeedUpdate;
  }

protected:

  /// Returns the value of this vector with respect to an arbitrary reference
  /// Frame
  virtual Vector computeRelativeTo(const Frame* _referenceFrame) const = 0;

  /// Computes the value of the world vector
  virtual void computeWorldVector() const = 0;

  /// Storage for the relative vector
  Vector mRelativeVector;

  /// Cache for the World vector
  mutable Vector mWorldVector;

  /// Flag for updates
  mutable bool mNeedUpdate;
};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_VECTORENTITY_H_

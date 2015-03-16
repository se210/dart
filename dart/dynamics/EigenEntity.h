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

/// EIGENENTITY_SETUP is a macro for easily creating copy constructors and
/// assignment operators for classes that use EigenEntity.
///
/// Note that because of const correctness, it is not possible for an Entity to
/// set its parent frame to the parent frame of a  const Entity*, because a
/// non-const Frame* needs to passed to the  setParentFrame() member function.
/// So when copying another EigenEntity using the assignment operator, the
/// parent Frame of the EigenEntity will remain the same and only the relative
/// vector will be altered. It will be altered in such a way that the World
/// vector of this EigenEntity will match the World vector of the EigenEntity
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
/// newly constructed EigenEntity will have a World vector that matches the
/// EigenEntity that was passed in, but its parent Frame will be based on the
/// second argument, which defaults to Frame::World().
///
/// For more flexibility in copying the values of a EigenEntity, the copy()
/// member function is offered. The copy() function allows you to specify which
/// parameters you want to copy from one EigenEntity to another using bitwise
/// logic. The default is to copy everything. For example, to only copy the
/// relative vector and parent frame:
///
/// Point p1, p2;
/// // ... set the values of and the parent Frames of p1 and p2 ...
/// p1.copy(p2, Point::RELATIVE_VECTOR | Point::PARENT_FRAME);
///
/// Note that the EigenEntity that is passed in cannot be const, even though
/// no changes will be made to it in the process of copying from it.
#define EIGENENTITY_SETUP(T, BaseT)                                             \
  typedef BaseT Base;                                                           \
                                                                                \
  inline T (const Base& _relative ## T = Base::Zero(),                          \
            Frame* _refFrame = Frame::World(),                                  \
            const std::string& _name = #T )                                     \
    : Entity(_refFrame, _name, false) {                                         \
      static_cast<Base&>(*this) = _relative ## T ;                              \
  }                                                                             \
  inline T ( const T & copy ## T , Frame* _refFrame = Frame::World())           \
    : Entity(_refFrame, copy ## T .getName()+"_copy", false) {                  \
      static_cast<Base&>(*this) = copy ## T .wrt(mParentFrame);                 \
  }                                                                             \
                                                                                \
  inline T & operator=( const T & copy ## T ) {                                 \
    static_cast<Base&>(*this) = copy ## T .wrt(mParentFrame);                   \
    return *this;                                                               \
  }                                                                             \
                                                                                \
  template <typename OtherDerived>                                              \
  T & operator= (const Eigen::MatrixBase <OtherDerived>& other) {               \
    this->Base::operator=(other);                                               \
    return *this;                                                               \
  }                                                                             \
                                                                                \
  inline T & copy( T & copy ## T , int copyOptions = EVERYTHING) {              \
    if(RELATIVE_VECTOR & copyOptions)                                           \
      static_cast<Base&>(*this) = static_cast<const Base&>( copy ## T );        \
    if(PARENT_FRAME & copyOptions) setParentFrame(copy ## T .getParentFrame()); \
    if(NAME & copyOptions) mName = copy ## T .getName();                        \
    if(VISUALIZATIONS & copyOptions) mVizShapes = copy ## T .mVizShapes;        \
                                                                                \
    return *this;                                                               \
  }                                                                             \
  inline virtual ~ T () { }                                                     \


/// This class is designed to merge raw Eigen types with the Entity concept.
template <typename Base>
class EigenEntity : public Base, public Detachable
{
public:

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

  // This constructor allows you to construct EigenEntity from Eigen expressions
  template<typename OtherDerived>
  EigenEntity(const Eigen::MatrixBase<OtherDerived>& other)
    : Base(other),
      Entity(Frame::World(), "", false) { }

  // This method allows you to assign Eigen expressions to EigenEntity
  template<typename OtherDerived>
  EigenEntity & operator= (const Eigen::MatrixBase <OtherDerived>& other)
  {
      this->Base::operator=(other);
      return *this;
  }

  /// Constructor
  EigenEntity(const Base& _relativeVector = Base::Zero(),
                  Frame* _refFrame = Frame::World(),
                  const std::string& _name = "")
    : Entity(_refFrame, _name, false)
  {
    (*this) = _relativeVector;
  }

  virtual ~EigenEntity() { }

  /// Get the value of this vector with respect to a reference frame
  Base wrt(const Frame* _referenceFrame) const
  {
    if(_referenceFrame == mParentFrame)
      return static_cast<Base>(*this);
    else if(_referenceFrame->isWorld())
      return wrtWorld();

    return computeRelativeTo(_referenceFrame);
  }

  /// Get the value of this vector with respect to the World frame
  Base wrtWorld() const
  {
    return computeRelativeToWorld();
  }

protected:

  /// Returns the value of this vector with respect to an arbitrary reference
  /// Frame
  virtual Base computeRelativeTo(const Frame* _referenceFrame) const = 0;

  /// Computes the value of this vector with respect to the World
  virtual Base computeRelativeToWorld() const = 0;
};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_VECTORENTITY_H_

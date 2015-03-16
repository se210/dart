/*
 * Copyright (c) 2011-2015, Georgia Tech Research Corporation
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

#ifndef DART_MATH_MATHTYPES_H_
#define DART_MATH_MATHTYPES_H_

#include <limits>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/StdVector>

//------------------------------------------------------------------------------
// Defines
//------------------------------------------------------------------------------
#define DART_EPSILON (1.0E-6)
#define DART_PI      (3.14159265358979323846)      // = pi
#define DART_2PI     (6.28318530717958647693)      // = 2 * pi
#define DART_PI_HALF (1.57079632679489661923)      // = pi / 2
#define DART_PI_SQR  (9.86960440108935861883)      // = pi^2
#define DART_RADIAN  (0.0174532925199432957692)    // = pi / 180
#define DART_DEGREE  (57.2957795130823208768)      // = 180 / pi

#define DART_1_3     (0.333333333333333333333)     // = 1 / 3
#define DART_1_6     (0.166666666666666666667)     // = 1 / 6
#define DART_1_12    (0.0833333333333333333333)    // = 1 / 12
#define DART_1_24    (0.0416666666666666666667)    // = 1 / 24
#define DART_1_30    (0.0333333333333333333333)    // = 1 / 30
#define DART_1_60    (0.0166666666666666666667)    // = 1 / 60
#define DART_1_120   (0.00833333333333333333333)   // = 1 / 120
#define DART_1_180   (0.00555555555555555555556)   // = 1 / 180
#define DART_1_720   (0.00138888888888888888889)   // = 1 / 720
#define DART_1_1260  (0.000793650793650793650794)  // = 1 / 1260
#define DART_4_3     (1.33333333333333333333)      // = 4 / 3

#define DART_DBL_INF (std::numeric_limits<double>::infinity())

//------------------------------------------------------------------------------
// Macros
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Types
//------------------------------------------------------------------------------
namespace Eigen {

typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;

typedef std::vector<Eigen::Vector3d> EIGEN_V_VEC3D;
typedef std::vector<std::vector<Eigen::Vector3d > > EIGEN_VV_VEC3D;

class SpatialVector : public Vector6d
{
public:
  typedef Vector6d Base;

  inline SpatialVector() { }

  // This constructor allows you to construct SpatialVector from Eigen expressions
  template<typename OtherDerived>
  SpatialVector(const Eigen::MatrixBase<OtherDerived>& other)
    : Base(other) { }

  // This method allows you to assign Eigen expressions to SpatialVector
  template<typename OtherDerived>
  SpatialVector& operator= (const Eigen::MatrixBase <OtherDerived>& other)
  {
      this->Base::operator=(other);
      return *this;
  }

  /// Construct a spatial vector from its angular and linear parts
  template <typename Derived1, typename Derived2>
  SpatialVector(const MatrixBase<Derived1>& _angular,
                const MatrixBase<Derived2>& _linear)
  {
    angular() = _angular;
    linear() = _linear;
  }

  /// Retrieve the top 3 components of the spatial vector
  inline Eigen::Block<Vector6d, 3, 1> upper() { return head<3>(); }

  /// Retrieve the top 3 components of the spatial vector
  inline const Eigen::Block<const Vector6d, 3, 1> upper() const { return head<3>(); }

  /// Retrieve the angular components of the spatial vector
  inline Eigen::Block<Vector6d, 3, 1> angular() { return upper(); }

  /// Retrieve the angular components of the spatial vector
  inline const Eigen::Block<const Vector6d, 3, 1> angular() const { return upper(); }

  /// Retrieve the bottom 3 components of the spatial vector
  inline Eigen::Block<Vector6d, 3, 1> lower() { return tail<3>(); }

  /// Retrieve the bottom 3 components of the spatial vector
  inline const Eigen::Block<const Vector6d, 3, 1> lower() const { return tail<3>(); }

  /// Retrieve the linear components of the spatial vector
  inline Eigen::Block<Vector6d, 3, 1> linear() { return lower(); }

  inline const Eigen::Block<const Vector6d, 3, 1> linear() const { return lower(); }
};

class SpatialForce : public SpatialVector
{
public:
  typedef SpatialVector Base;

  // Inherit constructor
  using SpatialVector::SpatialVector;

  // This method allows you to assign Eigen expressions to SpatialForce
  template<typename OtherDerived>
  SpatialForce& operator= (const Eigen::MatrixBase <OtherDerived>& other)
  {
      this->Base::operator=(other);
      return *this;
  }
};

class SpatialMotion : public SpatialVector
{
public:
  typedef SpatialVector Base;

  // Inherit constructor
  using SpatialVector::SpatialVector;

  // This method allows you to assign Eigen expressions to SpatialMotion
  template<typename OtherDerived>
  SpatialMotion& operator= (const Eigen::MatrixBase <OtherDerived>& other)
  {
      this->Base::operator=(other);
      return *this;
  }

  inline SpatialMotion cross(const SpatialMotion& other)
  {
    //------------------------------------------------------------------------
    // | m1  | x | m2  | = |       m1 x m2       |
    // | m1o |   | m2o |   | m1 x m2o + m1o x m2 |
    //------------------------------------------------------------------------
    return SpatialMotion(
                            upper().cross(other.upper()),
           upper().cross(other.lower()) + lower().cross(other.upper()));
  }

  inline SpatialForce cross(const SpatialForce& other)
  {
    //------------------------------------------------------------------------
    // | m  | x | fo | = | m x fo + mo x f |
    // | mo |   | f  |   |      m x f      |
    //------------------------------------------------------------------------
    return SpatialForce(
          upper().cross(other.upper()) + lower().cross(other.lower()),
                           upper().cross(other.lower()));
  }
};

typedef SpatialMotion SpatialVelocity;
typedef SpatialMotion SpatialAcceleration;

}

namespace dart {
namespace math {

typedef Eigen::Matrix6d Inertia;
typedef Eigen::Matrix<double, 3, Eigen::Dynamic> LinearJacobian;
typedef Eigen::Matrix<double, 3, Eigen::Dynamic> AngularJacobian;
typedef Eigen::Matrix<double, 6, Eigen::Dynamic> Jacobian;

}  // namespace math
}  // namespace dart

#endif  // DART_MATH_MATHTYPES_H_

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

#ifndef DART_DYNAMICS_DYNAMICSTYPES_H_
#define DART_DYNAMICS_DYNAMICSTYPES_H_

// TODO(JS): This should be in Joint?
/// Generalized coordinate type for rotation matrix
enum class GeneralizedCoordinateType
{
  /// Zero dimensional space
  NONE,

  /// One dimensional Euclidean space
  EUCLIDEAN_SPACE_1D,

  /// Two dimensional Euclidean space
  EUCLIDEAN_SPACE_2D,

  /// Three dimensional Euclidean space
  EUCLIDEAN_SPACE_3D,

  /// Space: \f$ se(3) \f$,
  /// Map: \f$ R = exp(r) \f$
  SO3_LIE_ALGEBRA,

  /// Space: \f$ \mathbf{R}^3 \f$,
  /// Map: \f$ R = RotX(r_1) RotY(r_2) RotZ(r_3) \f$
  EULER_INTRINSIC_XYZ,

  /// Space: \f$ \mathbf{R}^3 \f$,
  /// Map: \f$ R = RotZ(r_1) RotY(r_2) RotX(r_3) \f$
  EULER_INTRINSIC_ZYX,

  /// Space: \f$ \mathbf{R}^3 \f$,
  /// Map: \f$ R = RotZ(r_3) RotY(r_2) RotX(r_1) \f$
  EULER_EXTRINSIC_XYZ,

  /// Space: \f$ \mathbf{R}^3 \f$,
  /// Map: \f$ R = RotX(r_3) RotY(r_2) RotZ(r_1) \f$
  EULER_EXTRINSIC_ZYX,

  /// Space: \f$ se(3) \f$
  /// Map: \f$ t = log(T) \f$
  SE3_LIE_ALGEBRA,

  /// Space: \f$ so(3) \times \mathbf{R}^3 \f$
  /// Map: \f$ (t1, t2, t3) = log(R), ~ (t4, t5, t6) = p \f$
  SO3_LIE_ALGEBRA_AND_POSITION,

  /// Space: \f$ \mathbf{R}^3 \times so(3) \f$
  /// Map: \f$ (t1, t2, t3) = p, ~ (t4, t5, t6) = log(R) \f$
  POSITION_AND_SO3_LIE_ALGEBRA,

  /// Space: \f$ \mathbf{R}^3 \times \mathbf{R}^3 \f$
  /// Map: \f$ (t1, t2, t3) = p, ~ (t4, t5, t6) = EulerXYZ^{-1}(R) \f$
  POSITION_AND_EULER_INTRINSIC_XYZ,

  /// Space: \f$ \mathbf{R}^3 \times \mathbf{R}^3 \f$
  /// Map: \f$ (t1, t2, t3) = p, ~ (t4, t5, t6) = EulerZYX^{-1}(R) \f$
  POSITION_AND_EULER_INTRINSIC_ZYX,

  /// Space: \f$ \mathbf{R}^3 \times \mathbf{R}^3 \f$
  /// Map: \f$ (t1, t2, t3) = p, ~ (t6, t5, t4) = EulerZYX^{-1}(R) \f$
  POSITION_AND_EULER_EXTRINSIC_XYZ,

  /// Space: \f$ \mathbf{R}^3 \times \mathbf{R}^3 \f$
  /// Map: \f$ (t1, t2, t3) = p, ~ (t6, t5, t4) = EulerXYZ^{-1}(R) \f$
  POSITION_AND_EULER_EXTRINSIC_ZYX
};

/// Generalized coordinate type for rotation
enum class RotationGeneralizedCoordinatesType
{
  /// Space: \f$ se(3) \f$,
  /// Map: \f$ R = exp(r) \f$
  SO3_LIE_ALGEBRA,

  /// Space: \f$ \mathbf{R}^3 \f$,
  /// Map: \f$ R = RotX(r_1) RotY(r_2) RotZ(r_3) \f$
  EULER_INTRINSIC_XYZ,

  /// Space: \f$ \mathbf{R}^3 \f$,
  /// Map: \f$ R = RotZ(r_1) RotY(r_2) RotX(r_3) \f$
  EULER_INTRINSIC_ZYX,

  /// Space: \f$ \mathbf{R}^3 \f$,
  /// Map: \f$ R = RotZ(r_3) RotY(r_2) RotX(r_1) \f$
  EULER_EXTRINSIC_XYZ,

  /// Space: \f$ \mathbf{R}^3 \f$,
  /// Map: \f$ R = RotX(r_3) RotY(r_2) RotZ(r_1) \f$
  EULER_EXTRINSIC_ZYX,
};

/// Generalized coordinate type for transformation
enum class TransformGeneralizedCoordinatesType
{
  /// Space: \f$ se(3) \f$
  /// Map: \f$ t = log(T) \f$
  SE3_LIE_ALGEBRA,

  /// Space: \f$ so(3) \times \mathbf{R}^3 \f$
  /// Map: \f$ (t1, t2, t3) = log(R), ~ (t4, t5, t6) = p \f$
  SO3_LIE_ALGEBRA_AND_POSITION,

  /// Space: \f$ \mathbf{R}^3 \times so(3) \f$
  /// Map: \f$ (t1, t2, t3) = p, ~ (t4, t5, t6) = log(R) \f$
  POSITION_AND_SO3_LIE_ALGEBRA,

  /// Space: \f$ \mathbf{R}^3 \times \mathbf{R}^3 \f$
  /// Map: \f$ (t1, t2, t3) = p, ~ (t4, t5, t6) = EulerXYZ^{-1}(R) \f$
  POSITION_AND_EULER_INTRINSIC_XYZ,

  /// Space: \f$ \mathbf{R}^3 \times \mathbf{R}^3 \f$
  /// Map: \f$ (t1, t2, t3) = p, ~ (t4, t5, t6) = EulerZYX^{-1}(R) \f$
  POSITION_AND_EULER_INTRINSIC_ZYX,

  /// Space: \f$ \mathbf{R}^3 \times \mathbf{R}^3 \f$
  /// Map: \f$ (t1, t2, t3) = p, ~ (t6, t5, t4) = EulerZYX^{-1}(R) \f$
  POSITION_AND_EULER_EXTRINSIC_XYZ,

  /// Space: \f$ \mathbf{R}^3 \times \mathbf{R}^3 \f$
  /// Map: \f$ (t1, t2, t3) = p, ~ (t6, t5, t4) = EulerXYZ^{-1}(R) \f$
  POSITION_AND_EULER_EXTRINSIC_ZYX
};

#endif  // DART_DYNAMICS_DYNAMICSTYPES_H_


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

#ifndef DART_COMMON_SINGLETON_H_
#define DART_COMMON_SINGLETON_H_

namespace dart {
namespace common {

/// Singleton template class
///
/// \note This singleton is not thread safe. For use of thread safe singleton,
/// use static initialization as:
/// \code{.cpp}
/// // Singletone class Engine
/// class Engine : public Singleton<Engine> {};
///
/// // Call before main() and use theT only instead of calling getInstance()
/// static T& theT = T::getInstance();
/// \endcode
template <class T>
class Singleton
{
public:
  /// Get reference of the singleton
  static T& getInstance();

  /// Get pointer of the singleton
  static T* getInstancePtr();

protected:
  /// Constructor
  Singleton() = default;

  /// Destructor
  virtual ~Singleton() = default;

private:
  /// Deleted copy constructor
  Singleton(const T&) = delete;

  /// Deleted assignment operator
  const T& operator=(const T&) = delete;

private:
  static T* mInstance;
};

}  // namespace common
}  // namespace dart

#include "dart/common/detail/Singleton.h"

#endif  // DART_COMMON_SINGLETON_H_

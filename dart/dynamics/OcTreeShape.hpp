/*
 * Copyright (c) 2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2017, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2017, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#ifndef DART_DYNAMICS_OCTREESHAPE_HPP_
#define DART_DYNAMICS_OCTREESHAPE_HPP_

#ifdef DART_DYNAMICS_HAVE_OCTOMAP_

#include "dart/dynamics/Shape.hpp"

#include <octomap/octomap.h>

namespace dart {
namespace dynamics {

class OcTreeShape : public Shape, public octomap::OcTree
{
public:

  /// Uses the octomap::OcTree(double) constructor and then adds the provided
  /// list of points as "occupied" nodes to the OcTree
  explicit OcTreeShape(
      const double resolution,
      const std::vector<Eigen::Vector3d>& occupiedPoints = {});

  /// Uses the octomap::OcTree(double) constructor and then adds the provided
  /// list of points as nodes to the OcTree with the corresponding probabilities.
  ///
  /// For each entry in the probablePoints vector, the first component is the
  /// center of the node while the second component is the log_odds_value to be
  /// assigned to that node.
  explicit OcTreeShape(
      const double resolution,
      const std::vector<std::pair<Eigen::Vector3d, float>>& probablePoints);

  /// Uses the octomap::OcTree(std::string) constructor
  explicit OcTreeShape(const std::string& filename);

  // Documentation inherited
  Eigen::Matrix3d computeInertia(double mass) const override;
};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_HAVE_OCTOMAP_

#endif // DART_DYNAMICS_OCTREESHAPE_HPP_

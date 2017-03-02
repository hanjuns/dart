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

#include "dart/dynamics/OcTreeShape.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
OcTreeShape::OcTreeShape(
    const double resolution,
    const double occupancyThreshold,
    const std::vector<Eigen::Vector3d>& occupiedPoints)
  : Shape(UNSUPPORTED),
    octomap::OcTree(resolution)
{
  for(const Eigen::Vector3d& p : occupiedPoints)
    updateNode(p[0], p[1], p[2], true, true);

  setOccupancyThres(occupancyThreshold);
  updateInnerOccupancy();
}

//==============================================================================
OcTreeShape::OcTreeShape(
    const double resolution,
    const double occupancyThreshold,
    const std::vector<std::pair<Eigen::Vector3d, float> >& probablePoints)
  : Shape(UNSUPPORTED),
    octomap::OcTree(resolution)
{
  for(const std::pair<Eigen::Vector3d, float>& entry : probablePoints)
  {
    const Eigen::Vector3d& p = entry.first;
    const float value = entry.second;
    updateNode(p[0], p[1], p[2], value, true);
  }

  setOccupancyThres(occupancyThreshold);
  updateInnerOccupancy();
}

//==============================================================================
OcTreeShape::OcTreeShape(const std::string& filename)
  : Shape(UNSUPPORTED),
    octomap::OcTree(filename)
{
  // Do nothing
}

//==============================================================================
const std::string& OcTreeShape::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& OcTreeShape::getStaticType()
{
  static const std::string type("OcTreeShape");
  return type;
}

//==============================================================================
Eigen::Matrix3d OcTreeShape::computeInertia(double) const
{
  // TODO(MXG) Do this later
  return Eigen::Matrix3d::Zero();
}

//==============================================================================
void OcTreeShape::updateVolume()
{
  // TODO(MXG) Do this later
}

} // namespace dynamics
} // namespace dart

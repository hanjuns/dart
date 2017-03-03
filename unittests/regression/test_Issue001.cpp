/*
 * Copyright (c) 2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016-2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016-2017, Personal Robotics Lab, Carnegie Mellon University
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

#include <gtest/gtest.h>
#include <TestHelpers.hpp>
#include <dart/dart.hpp>

using namespace dart;

//==============================================================================
TEST(Issue001, Basic)
{
  dart::dynamics::SimpleFramePtr test_octree =
      std::make_shared<dart::dynamics::SimpleFrame>(dart::dynamics::Frame::World());
  std::shared_ptr<dart::dynamics::OcTreeShape> octree =
      std::make_shared<dart::dynamics::OcTreeShape>(0.05, 0.001);
  test_octree->setShape(octree);

  octree->updateNode(octomap::point3d(0.0, 0.0, 0.0), true);
  octree->updateNode(octomap::point3d(0.05, 0.0, 0.0), true);
  octree->updateNode(octomap::point3d(0.19, 0.0, 0.0), true);
  octree->updateNode(octomap::point3d(0.0, 0.1, 0.0), true);
  octree->updateNode(octomap::point3d(0.0, 0.0, 0.2), true);
  octree->updateNode(octomap::point3d(0.1, 0.1, 0.0), true);

  test_octree->createVisualAspect();
  test_octree->createCollisionAspect();
  test_octree->getVisualAspect()->setColor(dart::Color::Red(0.4));


  dart::dynamics::SimpleFramePtr test_box =
      std::make_shared<dart::dynamics::SimpleFrame>(dart::dynamics::Frame::World());
  std::shared_ptr<dart::dynamics::BoxShape> box =
      std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(1.0, 1.0, 0.04));
  test_box->setShape(box);
  test_box->setTranslation(Eigen::Vector3d(0.0, 0.0, -0.01));

  test_box->createVisualAspect();
  test_box->getVisualAspect()->setColor(dart::Color::Green(0.8));
  test_box->createCollisionAspect();

  test_octree->setShape(box);

  dart::collision::CollisionOption option;
  option.enableContact = false;
  dart::collision::CollisionResult result;

  dart::collision::CollisionDetectorPtr detector =
      dart::collision::FCLCollisionDetector::create();

  dart::collision::CollisionGroupPtr group_octree =
      detector->createCollisionGroupAsSharedPtr();

  group_octree->addShapeFrame(test_octree.get());


  dart::collision::CollisionGroupPtr group_box =
      detector->createCollisionGroupAsSharedPtr();

  group_box->addShapeFrame(test_box.get());


  if(detector->collide(group_octree.get(), group_box.get(),
                       option, &result))
  {
    std::cout << "Collision detected! #" << result.getNumContacts()
              << std::endl;
  }
  else
  {
    std::cout << " --- No collision detected" << std::endl;
  }
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

/*
 * Copyright (c) 2015-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
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

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/utils.hpp>

#include <memory>

using namespace dart::dynamics;
using namespace dart::simulation;

const double DefaultFloorThickness = 0.5;
const double DefaultWallHeight = 1.4;
const double DefaultCeilHeight = 0.8;
const double DefaultWallWidth = 0.05;
const double DefaultBarRadius = 0.05/2.0;
const double DefaultTableHeight = 0.55;
const double DefaultIBeamHeight = 0.20;
const double DefaultIBeamWidth = 0.15;
const double DefaultIBeamThickness = 0.02;
const Eigen::Vector4d DefaultWallColor = Eigen::Vector4d(238.0, 238.0, 224.0, 255.0)/255.0;
const Eigen::Vector4d DefaultCeilColor = Eigen::Vector4d(238.0, 238.0, 224.0, 150.0)/255.0;
const Eigen::Vector4d DefaultBarColor = Eigen::Vector4d(220.0, 220.0, 220.0, 255.0)/255.0;
const Eigen::Vector4d DefaultFloorColor = Eigen::Vector4d(112.0, 128.0, 134.0, 255.0)/255.0;
const Eigen::Vector4d DefaultBrickColor = Eigen::Vector4d(178.0, 34.0, 34.0, 255.0)/255.0;
//const Eigen::Vector4d DefaultIBeamColor = Eigen::Vector4d(0x91, 0x2C, 0x00, 255.0)/255.0;
const Eigen::Vector4d DefaultIBeamColor = Eigen::Vector4d(41, 232, 128, 255.0)/255.0;


const double DefaultAlpha = 0.3;
const Eigen::Vector4d LeftColor = dart::Color::Blue(DefaultAlpha);
const Eigen::Vector4d RightColor = dart::Color::Fuschia(DefaultAlpha);
//const Eigen::Vector4d CopyRightColor = Eigen::Vector4d(1.0, 0.0, 0.15, DefaultAlpha);
//const Eigen::Vector4d CopyRightColor = dart::Color::Orange(DefaultAlpha);
const Eigen::Vector4d CopyRightColor = dart::Color::Green(DefaultAlpha);
const Eigen::Vector4d SelectedColor = Eigen::Vector4d(1.0, 0.0, 0.0, DefaultAlpha);
const Eigen::Vector4d MinimalColor = Eigen::Vector4d(1.0, 1.0, 0.0, 2*DefaultAlpha);
//const Eigen::Vector4d MinimalColor = dart::Color::Orange(2*DefaultAlpha);

const double JumpHeight = 0.1;

inline Eigen::AngleAxisd findRotation(
    const Eigen::Vector3d& x1, const Eigen::Vector3d& x0)
{
  Eigen::Vector3d N = x1.cross(x0);
  double s = N.norm();
  double theta = asin(s);
  // Deal with numerical imprecision issues
  if(s < -1.1 || 1.1 < s)
  {
    std::cout << "[findRotation] Strange value for s: " << s
              << " | Results in theta: " << theta << std::endl;
    return Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
  }
  else if(1.0 <= s)
  {
    theta = M_PI/2.0;
  }
  else if(s <= -1.0)
  {
    theta = -M_PI/2.0;
  }
  else if(std::abs(s) < 1e-10)
  {
    return Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
  }

  N.normalize();

  const double wrap = x1.dot(x0);
  if(wrap < 0)
    theta = M_PI - theta;

  if(std::abs(theta-M_PI) < 1e-4
     || std::abs(theta+M_PI) < 1e-4)
    return Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());

  return Eigen::AngleAxisd(theta, N);
}

const double AngleWeight = 1.0/(360*M_PI/180.0);
double diff(const Eigen::Isometry3d& tf1, const Eigen::Isometry3d& tf2,
            const double angleWeight = AngleWeight)
{
//  Eigen::Vector6d dx;
//  dx.block<3,1>(3,0) = tf1.translation() - tf2.translation();

//  Eigen::AngleAxisd aa(tf2.linear().transpose()*tf1.linear());
//  dx.block<3,1>(0,0) = AngleWeight*aa.angle()*aa.axis();

//  return dx.norm();


  const double wR = angleWeight/(1+angleWeight);
  const double wX = 1-wR;

  const double X = (tf1.translation() - tf2.translation()).norm();

  Eigen::AngleAxisd aa(tf2.linear().transpose()*tf1.linear());
  const double R = std::abs(aa.angle());

  return wX*X + wR*R;
}

Eigen::Isometry3d stepTowards(const Eigen::Isometry3d& target,
                              const Eigen::Isometry3d& from,
                              const double byNoMoreThan,
                              const double angleWeight = AngleWeight)
{
  const double wR = angleWeight/(1+angleWeight);
  const double wX = 1-wR;

  const Eigen::Vector3d x = target.translation() - from.translation();
  const double X = x.norm();
  const Eigen::AngleAxisd aa(from.linear().transpose()*target.linear());
  const double R = std::abs(aa.angle());

  double cost = wX*X + wR*R;

  if(cost > byNoMoreThan)
  {
    const double s = byNoMoreThan/cost;

    const Eigen::Vector3d dx = s*x;
    const double r = s*aa.angle();

    Eigen::Isometry3d result = from;

    result.rotate(Eigen::AngleAxisd(r, aa.axis()));
    result.pretranslate(dx);

    return result;
  }

  return target;
}

SkeletonPtr makeBoxObject(const std::string& name,
                          const Eigen::Vector3d& x,
                          double yaw,
                          const Eigen::Vector3d& dim,
                          const Eigen::Vector4d& color)
{
  SkeletonPtr box = Skeleton::create(name);

  box->createJointAndBodyNodePair<FreeJoint>();
  BoxShapePtr shape = std::make_shared<BoxShape>(dim);
  ShapeNode* node = box->getBodyNode(0)->createShapeNodeWith<
      VisualAspect, CollisionAspect, DynamicsAspect>(shape);
  node->getVisualAspect()->setColor(color);

  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = x;
  tf.rotate(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  box->setPositions(dart::dynamics::FreeJoint::convertToPositions(tf));

  return box;
}

SkeletonPtr makeWall(const std::string& name,
                     double x, double y, double theta,
                     double L, double w, double height,
                     const Eigen::Vector4d& color,
                     double bottom = 0.0)
{
  const double h = height;
  SkeletonPtr wall = Skeleton::create(name);

  BodyNode* bn = wall->createJointAndBodyNodePair<WeldJoint>().second;
  BoxShapePtr shape = std::make_shared<BoxShape>(
        Eigen::Vector3d(L, w, h));
  ShapeNode* node = bn->createShapeNodeWith<
      VisualAspect, CollisionAspect, DynamicsAspect>(shape);
  node->getVisualAspect()->setColor(color);

  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translate(Eigen::Vector3d(x, y, h/2 + bottom));
  tf.rotate(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()));
  bn->getParentJoint()->setTransformFromParentBodyNode(tf);

  return wall;
}

SkeletonPtr makeBar(const std::string& name,
                    double x, double y, double z,
                    const Eigen::Vector3d& dir,
                    double R, double L, const Eigen::Vector4d& color)
{
  SkeletonPtr bar = Skeleton::create(name);

  BodyNode* bn = bar->createJointAndBodyNodePair<WeldJoint>().second;
  CylinderShapePtr shape = std::make_shared<CylinderShape>(R, L);
  ShapeNode* node = bn->createShapeNodeWith<
      VisualAspect, CollisionAspect, DynamicsAspect>(shape);
  node->getVisualAspect()->setColor(color);


  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translate(Eigen::Vector3d(x, y, z));
  tf.rotate(findRotation(Eigen::Vector3d::UnitZ(), dir));

  bn->getParentJoint()->setTransformFromParentBodyNode(tf);

  return bar;
}

SkeletonPtr addBar(const std::vector<Eigen::Vector3d>& points,
            const double R = DefaultBarRadius,
            const Eigen::Vector4d& color = DefaultBarColor)
{
  assert(points.size() == 2);
  const Eigen::Vector3d& p0 = points[0];
  const Eigen::Vector3d& p1 = points[1];

  const double L = (p1-p0).norm();
  const Eigen::Vector3d c = (p1+p0)/2.0;
  Eigen::Vector3d dir = p1-p0;
  dir.normalize();

  return makeBar("bar", c[0], c[1], c[2], dir, R, L, color);
}

SkeletonPtr addWall(
             const std::vector<Eigen::Vector2d>& points,
             const double height = DefaultWallHeight,
             const Eigen::Vector4d& color = DefaultWallColor)
{
  assert(points.size() == 2);
  const Eigen::Vector2d& p0 = points[0];
  const Eigen::Vector2d& p1 = points[1];

  const double L = (p1-p0).norm();
  const Eigen::Vector2d c = (p1+p0)/2.0;
  const double x = p1[0] - p0[0];
  const double y = p1[1] - p0[1];
  const double theta = atan2(y, x);

  return makeWall("wall", c[0], c[1], theta, L, DefaultWallWidth, height, color);
}

SkeletonPtr addYPost(
              const Eigen::Vector3d& b,
              const double nudge,
              const double thickness = 2*DefaultBarRadius,
              const Eigen::Vector4d& color = DefaultWallColor)
{
  Eigen::Vector2d w0(b[0], b[1]-thickness/2.0+nudge*DefaultWallWidth/2.0);
  Eigen::Vector2d w1(b[0], b[1]+thickness/2.0+nudge*DefaultWallWidth/2.0);

  return addWall({w0, w1}, b[2]+thickness, color);
}

void limbo(WorldPtr world)
{
//  const double w_x = 3.0;
  const double w_x = 2.0;
  const double w_y = 0.8;
//  const double w_y = 1.2;
  const double t = DefaultWallWidth/2.0;

//  const double h_box = 0.1;
  const double h_box = 0.02;


//  model->addObstacle(makeWall("north", 0,  w_y+t, 0.0*M_PI/180.0, 2.0*w_x));
//  model->addObstacle(makeWall("south", 0, -w_y-t, 0.0*M_PI/180.0, 2.0*w_x));

//  Eigen::Vector3d b0(0.8, -w_y, 0.7);
//  Eigen::Vector3d b1(0.8,  w_y, 1.4);
//  Eigen::Vector3d b0(0.4, -w_y, 0.7);
//  Eigen::Vector3d b1(0.4,  w_y, 1.4);
  Eigen::Vector3d b0(0.0, -w_y, 1.2);
  Eigen::Vector3d b1(0.0,  w_y, 1.2);
  world->addSkeleton(addBar({b0, b1}));
  world->addSkeleton(addYPost(b0,  1.0));
  world->addSkeleton(addYPost(b1, -1.0));
}

static void setAlpha(const SkeletonPtr skel, const double alpha)
{
  for(size_t i=0; i < skel->getNumBodyNodes(); ++i)
  {
    BodyNode* bn = skel->getBodyNode(i);
    for(size_t j=0; j < bn->getNumShapeNodes(); ++j)
    {
      ShapeNode* sn = bn->getShapeNode(j);
      VisualAspect* visual = sn->getVisualAspect();
      if(visual)
      {
        visual->setRGBA(Eigen::Vector4d(0.9,0.9,0.9,alpha));
      }

      ShapePtr shape = sn->getShape();
//      shape->addDataVariance(Shape::DYNAMIC_COLOR);
      MeshShapePtr ms = std::dynamic_pointer_cast<MeshShape>(shape);
      if(ms)
      {
        ms->setColorMode(MeshShape::SHAPE_COLOR);
      }
    }
  }
}

static void addBox(const SkeletonPtr& skel,
                   Eigen::Vector3d lower,
                   Eigen::Vector3d upper,
                   const Eigen::Matrix3d& R = Eigen::Matrix3d::Identity(),
                   const Eigen::Vector4d color = Eigen::Vector4d(0.0, 208.0, 255.0, 0.4*255.0)/255.0)
{
  const Eigen::Matrix3d Rinv = R.transpose();
  lower = Rinv*lower;
  upper = Rinv*upper;
  const Eigen::Vector3d dim = upper - lower;

  BodyNode* bn = skel->getBodyNode(0)->
      createChildJointAndBodyNodePair<WeldJoint>().second;
  bn->setName("component_"+std::to_string(skel->getNumBodyNodes()));
  bn->getParentJoint()->setName(bn->getName());

  BoxShapePtr shape = std::make_shared<BoxShape>(dim);
  ShapeNode* node = bn->createShapeNodeWith<
      VisualAspect, CollisionAspect, DynamicsAspect>(shape);
  node->getVisualAspect()->setColor(color);
  node->setRelativeTranslation(R*(dim/2.0 + lower));
  node->setRelativeRotation(R);
}

SkeletonPtr getMinimalRobot(const SkeletonPtr& robot)
{
  SkeletonPtr minimal = robot->getBodyNode(0)->copyAs("minimal", false);

  addBox(minimal,
         Eigen::Vector3d(-0.1,  -0.086, 0.03),
         Eigen::Vector3d( 0.06,  0.086, 0.25),
         Eigen::Matrix3d::Identity(),
         Eigen::Vector4d(1.0, 0.0, 0.0, 1.0));

  for(size_t j=0; j < minimal->getNumBodyNodes(); ++j)
  {
    BodyNode* bn = minimal->getBodyNode(j);
    for(size_t k=0; k < bn->getNumShapeNodes(); ++k)
    {
      ShapeNode* sn = bn->getShapeNode(k);
      VisualAspect* visual = sn->getVisualAspect();
      if(visual)
        visual->setRGBA(MinimalColor);

      MeshShapePtr shape = std::dynamic_pointer_cast<MeshShape>(sn->getShape());
      if(shape)
      {
        shape->setColorMode(MeshShape::SHAPE_COLOR);
      }
    }
  }

  return minimal;
}

SkeletonPtr getWalkSweeper(const SkeletonPtr& hubo)
{
  SkeletonPtr sweeper = dart::dynamics::Skeleton::create("walk_sweeper");
  sweeper->createJointAndBodyNodePair<FreeJoint>();
  BodyNode* walkSweeper = sweeper->getBodyNode(0);
  walkSweeper->setName(sweeper->getName());
  walkSweeper->getParentJoint()->setName(walkSweeper->getName()+"_joint");

  Eigen::Isometry3d base_tf(Eigen::Isometry3d::Identity());
  const double height = hubo->getEndEffector("l_foot")->getTransform(
        hubo->getBodyNode(0)).translation()[2] - 0.01;
  base_tf.translation()[2] = height;
  sweeper->getJoint(0)->setTransformFromParentBodyNode(base_tf);


  Eigen::Vector4d color(dart::Color::Green(DefaultAlpha));

  // Bottom
  {
    const double sweepHeight = 0.4;
    const double sweepWidth = 0.6;
    const double sweepDepth = 0.5;
    Eigen::Vector3d sweepOffset(Eigen::Vector3d::Zero());
    sweepOffset[0] = sweepDepth/2.0 - 0.15;
    sweepOffset[2] = sweepHeight/2.0;

    dart::dynamics::ShapePtr bottomShape =
        std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(sweepDepth, sweepWidth, sweepHeight));

    dart::dynamics::ShapeNode* node = walkSweeper->createShapeNodeWith<
        dart::dynamics::VisualAspect, dart::dynamics::CollisionAspect,
        dart::dynamics::DynamicsAspect>(bottomShape);

    node->getVisualAspect()->setColor(color);
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.translate(sweepOffset);
    node->setRelativeTransform(tf);
  }

  // Thighs
  {
    const double sweepHeight = 0.14;
    const double sweepWidth = 0.6;
    const double sweepDepth = 0.48;
    const double sweepAngle = 40.0*M_PI/180.0;
    Eigen::Vector3d sweepOffset(Eigen::Vector3d::Zero());
    sweepOffset[0] = 0.13;
    sweepOffset[2] = 0.55;

    dart::dynamics::ShapePtr midShape =
        std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(sweepDepth, sweepWidth, sweepHeight));

    dart::dynamics::ShapeNode* node = walkSweeper->createShapeNodeWith<
        dart::dynamics::VisualAspect, dart::dynamics::CollisionAspect,
        dart::dynamics::DynamicsAspect>(midShape);

    node->getVisualAspect()->setColor(color);
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.translate(sweepOffset);
    tf.rotate(Eigen::AngleAxisd(sweepAngle, Eigen::Vector3d::UnitY()));
    node->setRelativeTransform(tf);
  }

  // Arms
  {
    const double sweepHeight = 0.15;
//    const double sweepHeight = 0.65;
    const double sweepWidth = 0.9;
    const double sweepDepth = 0.50;
    Eigen::Vector3d sweepOffset(Eigen::Vector3d::Zero());
    sweepOffset[0] = sweepDepth/2.0 - 0.165;
    sweepOffset[2] = sweepHeight/2.0 + 0.61;

    dart::dynamics::ShapePtr topShape =
        std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(sweepDepth, sweepWidth, sweepHeight));

    dart::dynamics::ShapeNode* node = walkSweeper->createShapeNodeWith<
        dart::dynamics::VisualAspect, dart::dynamics::CollisionAspect,
        dart::dynamics::DynamicsAspect>(topShape);

    node->getVisualAspect()->setColor(color);
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.translate(sweepOffset);
    node->setRelativeTransform(tf);
  }

  // Chest
  {
//    const double sweepHeight = 0.50;
    const double sweepHeight = 0.28;
    const double sweepWidth = 0.76;
    const double sweepDepth = 0.31;
    Eigen::Vector3d sweepOffset(Eigen::Vector3d::Zero());
    sweepOffset[0] = sweepDepth/2.0 - 0.16;
    sweepOffset[2] = sweepHeight/2.0 + 0.76;

    dart::dynamics::ShapePtr topShape =
        std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(sweepDepth, sweepWidth, sweepHeight));

    dart::dynamics::ShapeNode* node = walkSweeper->createShapeNodeWith<
        dart::dynamics::VisualAspect, dart::dynamics::CollisionAspect,
        dart::dynamics::DynamicsAspect>(topShape);

    node->getVisualAspect()->setColor(color);
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.translate(sweepOffset);
    node->setRelativeTransform(tf);
  }

  // Head
  {
    const double sweepHeight = 0.22;
    const double sweepWidth = 0.57;
    const double sweepDepth = 0.13;
    Eigen::Vector3d sweepOffset(Eigen::Vector3d::Zero());
    sweepOffset[0] = sweepDepth/2.0 - 0.06;
    sweepOffset[2] = sweepHeight/2.0 + 1.04;

    dart::dynamics::ShapePtr topShape =
        std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(sweepDepth, sweepWidth, sweepHeight));

    dart::dynamics::ShapeNode* node = walkSweeper->createShapeNodeWith<
        dart::dynamics::VisualAspect, dart::dynamics::CollisionAspect,
        dart::dynamics::DynamicsAspect>(topShape);

    node->getVisualAspect()->setColor(color);
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.translate(sweepOffset);
    node->setRelativeTransform(tf);
  }

  return sweeper;
}

std::vector<dart::dynamics::SkeletonPtr> getFootprints()
{
  dart::dynamics::SkeletonPtr l_foot =
      dart::dynamics::Skeleton::create("l_foot_print");
  BodyNode* bn = l_foot->createJointAndBodyNodePair<FreeJoint>().second;
  bn->setName("base"); bn->getParentJoint()->setName("base");

  addBox(l_foot,
         Eigen::Vector3d(-0.222, -0.072, -0.01),
         Eigen::Vector3d(0.002, 0.079, 0.01),
         Eigen::Matrix3d::Identity(),
         dart::Color::Blue(0.7));

  dart::dynamics::SkeletonPtr r_foot =
      dart::dynamics::Skeleton::create("r_foot_print");
  bn = r_foot->createJointAndBodyNodePair<FreeJoint>().second;
  bn->setName("base"); bn->getParentJoint()->setName("base");

  addBox(r_foot,
         Eigen::Vector3d(-0.222, -0.079, -0.01),
         Eigen::Vector3d(0.002, 0.072, 0.01),
         Eigen::Matrix3d::Identity(),
         dart::Color::Fuschia(0.7));

  return {l_foot, r_foot};
}

struct Box
{
  bool left;

  dart::dynamics::SimpleFramePtr node;

  Box(const Eigen::Isometry3d& T = Eigen::Isometry3d::Identity())
    : left(true),
      node(nullptr)
  {
    // Do nothing
  }
};

void printBox(const std::shared_ptr<Box>& box)
{
  Eigen::Vector3d scale = std::dynamic_pointer_cast<BoxShape>(
        box->node->getShape())->getSize();
  Eigen::Vector3d v = box->node->getRelativeTransform().translation();

  std::cout << "ADD_BOX(";
  for(size_t i=0; i < 3; ++i)
    std::cout << v[i] << ", ";

  for(size_t i=0; i < 3; ++i)
    std::cout << scale[i] << ", ";

  if(box->left)
    std::cout << "1 ";
  else
    std::cout << "0 ";

  std::cout << ");" << std::endl;
}

void printBoxes(const std::vector<std::shared_ptr<Box>>& boxes)
{
  for(size_t i=0; i < boxes.size(); ++i)
    printBox(boxes[i]);

  std::cout << " ---- " << std::endl;
}

void printConfigs(const std::vector<Eigen::VectorXd>& Q)
{
  for(size_t i=0; i < Q.size(); ++i)
  {
    std::cout << "ADD_CONFIG(";
    Eigen::VectorXd q = Q[i];
    for(size_t j=0; j < q.size(); ++j)
    {
      std::cout << q[j];
      if(j < q.size()-1)
        std::cout << ", ";
      else
        std::cout << ");";
    }
    std::cout << std::endl;
  }

  std::cout << "-----------\n" << std::endl;
}

class Viewer : public dart::gui::osg::Viewer
{
public:

  void setupCustomLights(size_t version = 1)
  {
    switchHeadlights(false);

    mLight1->setAmbient( ::osg::Vec4(0.4,0.4,0.4,1.0));

    const osg::Vec3 p1 = mUpwards+osg::Vec3(0, 1, 0);
    const osg::Vec3 p2 = mUpwards+osg::Vec3(1, 0, 0);

    if(1 == version)
    {
      mLight1->setPosition(osg::Vec4( p1[0], p1[1], p1[2], 0.0));
      mLight2->setPosition(osg::Vec4(-p2[0], p2[1], p2[2], 0.0));
    }
    else if(2 == version)
    {
      mLight1->setPosition(osg::Vec4(p1[0], -p1[1], p1[2], 0.0));
      mLight2->setPosition(osg::Vec4(p2[0],  p2[1], p2[2], 0.0));
    }
    else if(3 == version)
    {
      mLight1->setPosition(osg::Vec4(p1[0], p1[1], p1[2], 0.0));
      mLight2->setPosition(osg::Vec4(p2[0], p2[1], p2[2], 0.0));
    }
    else if(4 == version)
    {
      mLight1->setPosition(osg::Vec4( p1[0], -p1[1], p1[2], 0.0));
      mLight2->setPosition(osg::Vec4(-p2[0],  p2[1], p2[2], 0.0));
    }
    else if(5 == version)
    {
      mLight1->setPosition(osg::Vec4(-p2[0], p2[1], p2[2], 0.0));
      mLight2->setPosition(osg::Vec4( p1[0], p1[1], p1[2], 0.0));
    }
    else if(6 == version)
    {
      mLight1->setPosition(osg::Vec4(p2[0],  p2[1], p2[2], 0.0));
      mLight2->setPosition(osg::Vec4(p1[0], -p1[1], p1[2], 0.0));
    }
    else if(7 == version)
    {
      mLight1->setPosition(osg::Vec4(p2[0], p2[1], p2[2], 0.0));
      mLight2->setPosition(osg::Vec4(p1[0], p1[1], p1[2], 0.0));
    }
    else if(8 == version)
    {
      mLight1->setPosition(osg::Vec4(-p2[0],  p2[1], p2[2], 0.0));
      mLight2->setPosition(osg::Vec4( p1[0], -p1[1], p1[2], 0.0));
    }
  }
};

class RelaxedPosture : public dart::optimizer::Function
{
public:

  RelaxedPosture(const Eigen::VectorXd& idealPosture,
                 const Eigen::VectorXd& lower, const Eigen::VectorXd& upper,
                 const Eigen::VectorXd& weights, bool enforceIdeal = false)
    : enforceIdealPosture(enforceIdeal),
      mIdeal(idealPosture),
      mLower(lower),
      mUpper(upper),
      mWeights(weights)
  {
    int dofs = mIdeal.size();
    if(mLower.size() != dofs || mWeights.size() != dofs || mUpper.size() != dofs)
    {
      dterr << "[RelaxedPose::RelaxedPose] Dimension mismatch:\n"
            << "  ideal:   " << mIdeal.size()   << "\n"
            << "  lower:   " << mLower.size()   << "\n"
            << "  upper:   " << mUpper.size()   << "\n"
            << "  weights: " << mWeights.size() << "\n";
    }
    mResultVector.setZero(dofs);
  }

  double eval(const Eigen::VectorXd& _x) override
  {
    computeResultVector(_x);
    return 0.5 * mResultVector.dot(mResultVector);
  }

  void evalGradient(const Eigen::VectorXd& _x,
                    Eigen::Map<Eigen::VectorXd> _grad) override
  {
    computeResultVector(_x);

    _grad.setZero();
    int smaller = std::min(mResultVector.size(), _grad.size());
    for(int i=0; i < smaller; ++i)
      _grad[i] = mResultVector[i];
  }

  void computeResultVector(const Eigen::VectorXd& _x)
  {
    mResultVector.setZero();

    if(enforceIdealPosture)
    {
      for(int i=0; i < _x.size(); ++i)
      {
        if(mIdeal.size() <= i)
          break;

        mResultVector[i] = mWeights[i]*(_x[i] - mIdeal[i]);
      }
    }
    else
    {
      for(int i=0; i < _x.size(); ++i)
      {
        if(mIdeal.size() <= i)
          break;

        if(_x[i] < mLower[i])
          mResultVector[i] = mWeights[i]*(_x[i] - mLower[i]);
        else if(mUpper[i] < _x[i])
          mResultVector[i] = mWeights[i]*(_x[i] - mUpper[i]);
      }
    }
  }

  bool enforceIdealPosture;

protected:

  Eigen::VectorXd mResultVector;

  Eigen::VectorXd mIdeal;

  Eigen::VectorXd mLower;

  Eigen::VectorXd mUpper;

  Eigen::VectorXd mWeights;
};

static inline bool checkDist(Eigen::Vector3d& p, double a, double b)
{
  double d = p.norm();
  double dmax = a+b;
  double dmin = fabs(a-b);

  if (d > dmax)
  {
    p *= dmax/d;
    return false;
  }
  else if (d < dmin)
  {
    p *= dmin/d;
    return false;
  }
  else
  {
    return true;
  }
}

static inline void clamp_sincos(double& sincos, bool& valid)
{
  if (sincos < -1)
  {
    valid = false;
    sincos = -1;
  }
  else if (sincos > 1)
  {
    valid = false;
    sincos = 1;
  }
}

static inline Eigen::Vector3d flipEuler3Axis(const Eigen::Vector3d& u)
{
  Eigen::Vector3d v;
  v[0] = u[0] - M_PI;
  v[1] = M_PI - u[1];
  v[2] = u[2] - M_PI;
  return v;
}

/// The HuboArmIK is based on the derivation of Hubo's arm IK by Matt Zucker.
class HuboArmIK : public InverseKinematics::Analytical
{
public:

  HuboArmIK(InverseKinematics* _ik, const std::string& baseLinkName,
            const Analytical::Properties& properties = Analytical::Properties())
    : Analytical(_ik, "HuboArmIK_"+baseLinkName, properties),
      configured(false),
      mBaseLinkName(baseLinkName)
  {
    // Do nothing
  }

  std::unique_ptr<GradientMethod> clone(InverseKinematics* _newIK) const override
  {
    return dart::common::make_unique<HuboArmIK>(
        _newIK, mBaseLinkName, getAnalyticalProperties());
  }

  const std::vector<Solution>& computeSolutions(
      const Eigen::Isometry3d& _desiredBodyTf) override
  {
    mSolutions.clear();
    mSolutions.reserve(8);

    if(!configured)
    {
      configure();

      if(!configured)
      {
        dtwarn << "[HuboArmIK::computeSolutions] This analytical IK was not able "
               << "to configure properly, so it will not be able to compute "
               << "solutions\n";
        return mSolutions;
      }
    }

    const BodyNodePtr& base = mBaseLink.lock();
    if(nullptr == base)
    {
      dterr << "[HuboArmIK::computeSolutions] Attempting to perform an IK on a "
            << "limb that no longer exists [" << getMethodName() << "]!\n";
      assert(false);
      return mSolutions;
    }

    if(nullptr == mWristEnd)
    {
      dterr << "[HuboArmIK::computeSolutions] Attempting to perform IK without "
            << "a wrist!\n";
      assert(false);
      return mSolutions;
    }

    const std::size_t SP = 0;
    const std::size_t SR = 1;
    const std::size_t SY = 2;
    const std::size_t EP = 3;
    const std::size_t WY = 4;
    const std::size_t WP = 5;

    const SkeletonPtr& skel = base->getSkeleton();

    Eigen::Isometry3d B =
        base->getParentBodyNode()->getWorldTransform().inverse()
        * _desiredBodyTf * mWristEnd->getTransform(mIK->getNode());

    Eigen::Isometry3d shoulder_from_wrist = shoulderTf.inverse() * B;
    Eigen::Vector3d p = shoulder_from_wrist.inverse().translation();

    const double a2 = L5*L5 + L4*L4;
    const double b2 = L3*L3 + L4*L4;
    const double a = sqrt(a2);
    const double b = sqrt(b2);

    const double alpha = atan2(L5, L4);
    const double beta = atan2(L3, L4);

    bool startValid = checkDist(p, a, b);

    double c2 = p.dot(p);
    double x = p.x();
    double y = p.y();
    double z = p.z();

    for(std::size_t i = 0; i < 8; ++i)
    {
      const int flipEP = alterantives(i,0);
      const int incWY = alterantives(i,1);
      const int flipShoulder = alterantives(i,2);

      Eigen::Vector6d testQ;
      bool isValid = startValid;

      double cosGamma = (a2 + b2 - c2) / (2*a*b);
      clamp_sincos(cosGamma, isValid);

      double gamma = flipEP * acos( cosGamma );
      double theta3 = alpha + beta + gamma - 2*M_PI;

      testQ(EP) = theta3;

      double c3 = cos(theta3);
      double s3 = sin(theta3);

      double numer = -y;
      double denom = (-L4*c3 - L3*s3 + L4);

      double s2, theta2;

      if(std::abs(denom) < zeroSize)
      {
        isValid = false;
        const double& prevWY = skel->getPosition(mDofs[WY]);
        theta2 = incWY ? prevWY : M_PI - prevWY;
        s2 = sin(theta2);
      }
      else
      {
        s2 = numer / denom;
        clamp_sincos(s2, isValid);
        theta2 = incWY ? M_PI - asin(s2) : asin(s2);
      }

      testQ(WY) = theta2;

      double c2 = cos(theta2);

      double r =  L4*c2 - L4*c2*c3 - L3*s3*c2;
      double q = -L4*s3 + L3*c3 + L5;

      double det = -(q*q + r*r);

      if(std::abs(det) < zeroSize)
        isValid = false;

      double k = det < 0 ? -1 : 1;

      double ks1 = k*( q*x - r*z );
      double kc1 = k*(-r*x - q*z );

      double theta1 = atan2(ks1, kc1);
      testQ(WP) = theta1;

      Eigen::Quaterniond Rlower =
        Eigen::Quaterniond(Eigen::AngleAxisd(testQ(EP), Eigen::Vector3d::UnitY())) *
        Eigen::Quaterniond(Eigen::AngleAxisd(testQ(WY), Eigen::Vector3d::UnitZ())) *
        Eigen::Quaterniond(Eigen::AngleAxisd(testQ(WP), Eigen::Vector3d::UnitY()));

      Eigen::Matrix3d Rupper = B.rotation() * Rlower.inverse().matrix();

      Eigen::Vector3d euler = Rupper.eulerAngles(1, 0, 2);

      if(flipShoulder)
        euler = flipEuler3Axis(euler);

      testQ(SP) = euler[0];
      testQ(SR) = euler[1];
      testQ(SY) = euler[2];

      for(std::size_t j=0; j < 6; ++j)
      {
        testQ[j] = dart::math::wrapToPi(testQ[j]);
        if(std::abs(testQ[j]) < zeroSize)
          testQ[j] = 0.0;
      }

      int validity = isValid? VALID : OUT_OF_REACH;
      mSolutions.push_back(Solution(testQ, validity));
    }

    checkSolutionJointLimits();

    return mSolutions;
  }

  const std::vector<std::size_t>& getDofs() const override
  {
    if(!configured)
      configure();

    return mDofs;
  }

  const double zeroSize = 1e-8;

protected:

  void configure() const
  {
    configured = false;

    mBaseLink = mIK->getNode()->getSkeleton()->getBodyNode(mBaseLinkName);

    BodyNode* base = mBaseLink.lock();
    if(nullptr == base)
    {
      dterr << "[HuboArmIK::configure] base link is a nullptr\n";
      assert(false);
      return;
    }

    const SkeletonPtr& skel = base->getSkeleton();
    const BodyNodePtr& pelvis = skel->getBodyNode("Body_TSY");
    if(nullptr == pelvis)
    {
      dterr << "[HuboArmIK::configure] Could not find Hubo's pelvis "
            << "(Body_TSY)\n";
      assert(false);
      return;
    }

    Eigen::Vector6d saved_q;

    DegreeOfFreedom* dofs[6];
    BodyNode* bn = base;
    for(std::size_t i=0; i < 6; ++i)
    {
      Joint* joint = bn->getParentJoint();
      if(joint->getNumDofs() != 1)
      {
        dterr << "[HuboArmIK::configure] Invalid number of DOFs ("
              << joint->getNumDofs() << ") in the Joint [" << joint->getName()
              << "]\n";
        assert(false);
        return;
      }

      dofs[i] = joint->getDof(0);
      saved_q[i] = dofs[i]->getPosition();
      dofs[i]->setPosition(0.0);
      bn = bn->getChildBodyNode(0);
    }

    BodyNode* elbow = dofs[3]->getChildBodyNode();
    L3 = std::abs(elbow->getTransform(dofs[2]->getParentBodyNode()).translation()[2]);
    L4 = std::abs(elbow->getTransform(dofs[3]->getParentBodyNode()).translation()[0]);

    BodyNode* wrist = dofs[5]->getChildBodyNode();
    Eigen::Isometry3d wrist_tf = wrist->getTransform(elbow);
    L5 = std::abs(wrist_tf.translation()[2]);

    shoulderTf = Eigen::Isometry3d::Identity();
    shoulderTf.translate(dofs[3]->getParentBodyNode()->getTransform(pelvis)
        .translation()[0] * Eigen::Vector3d::UnitX());
    shoulderTf.translate(dofs[2]->getParentBodyNode()->getTransform(pelvis)
        .translation()[1] * Eigen::Vector3d::UnitY());
    shoulderTf.translate(dofs[2]->getParentBodyNode()->getTransform(pelvis)
        .translation()[2] * Eigen::Vector3d::UnitZ());

    mWristEnd = dofs[5]->getChildBodyNode();

    alterantives <<
         1,  1,  1,
         1,  1,  0,
         1,  0,  1,
         1,  0,  0,
        -1,  1,  1,
        -1,  1,  0,
        -1,  0,  1,
        -1,  0,  0;

    for(std::size_t i=0; i < 6; ++i)
    {
      dofs[i]->setPosition(saved_q[i]);
      mDofs.push_back(dofs[i]->getIndexInSkeleton());
    }

    configured = true;
  }

  mutable bool configured;

  mutable Eigen::Isometry3d shoulderTf;
  mutable Eigen::Isometry3d wristTfInv;
  mutable Eigen::Isometry3d mNodeOffsetTfInv;
  mutable double L3, L4, L5;

  mutable Eigen::Matrix<int, 8, 3> alterantives;

  mutable std::vector<std::size_t> mDofs;

  std::string mBaseLinkName;
  mutable WeakBodyNodePtr mBaseLink;

  mutable JacobianNode* mWristEnd;
};

class HuboLegIK : public InverseKinematics::Analytical
{
public:

  /// baseLink should be Body_LHY or Body_RHY
  HuboLegIK(InverseKinematics* _ik, const std::string& baseLinkName,
            const Analytical::Properties& properties = Analytical::Properties())
    : Analytical(_ik, "HuboLegIK_"+baseLinkName, properties),
      configured(false),
      mBaseLinkName(baseLinkName)
  {
    // Do nothing
  }

  std::unique_ptr<GradientMethod> clone(InverseKinematics* _newIK) const override
  {
    return dart::common::make_unique<HuboLegIK>(
        _newIK, mBaseLinkName, getAnalyticalProperties());
  }

  const std::vector<Solution>& computeSolutions(
      const Eigen::Isometry3d& _desiredBodyTf) override
  {
    mSolutions.clear();
    mSolutions.reserve(8);

    if(!configured)
    {
      configure();

      if(!configured)
      {
        dtwarn << "[HuboLegIK::computeSolutions] This analytical IK was not able "
              << "to configure properly, so it will not be able to compute "
              << "solutions\n";
        return mSolutions;
      }
    }

    const BodyNodePtr& base = mBaseLink.lock();
    if(nullptr == base)
    {
      dterr << "[HuboLegIK::computeSolutions] Attempting to perform IK on a "
            << "limb that no longer exists!\n";
      assert(false);
      return mSolutions;
    }

    double nx, ny, sx, sy, ax, ay, az, px, py, pz;
    double q1, q2, q3, q4, q5, q6;
    double S2, S4, S6;
    double C2, C4, C5, C6;
    double C45, psi, q345;
    std::complex<double> radical;
    std::complex<double> sqrt_radical;
    Eigen::Isometry3d B, Binv;

    Eigen::Vector6d testQ;

    B = (base->getParentBodyNode()->getWorldTransform() * waist).inverse()
        * _desiredBodyTf * footTfInv;
    Binv = B.inverse();

    nx = Binv(0,0); sx = Binv(0,1); ax = Binv(0,2); px = Binv(0,3);
    ny = Binv(1,0); sy = Binv(1,1); ay = Binv(1,2); py = Binv(1,3);
                                    az = Binv(2,2); pz = Binv(2,3);

    for(std::size_t i=0; i < 8; ++i)
    {
      bool isValid = true;

      C4 = ((px+L6)*(px+L6) - L4*L4 - L5*L5 + py*py + pz*pz)/(2*L4*L5);
      radical = 1-C4*C4;
      sqrt_radical = std::sqrt(radical);
      if(sqrt_radical.imag() != 0)
          isValid = false;
      q4 = atan2(alternatives(i,0)*sqrt_radical.real(), C4);

      S4 = sin(q4);
      psi = atan2(S4*L4, C4*L4+L5);
      radical = (px+L6)*(px+L6) + py*py;
      sqrt_radical = std::sqrt(radical);
      if(sqrt_radical.imag() != 0)
          isValid = false;

      q5 = dart::math::wrapToPi(atan2(-pz, alternatives(i,1)*sqrt_radical.real())-psi);

      q6 = atan2(py, -(px+L6));
      C45 = cos(q4+q5);
      C5 = cos(q5);
      if( C45*L4 + C5*L5 < 0 )
          q6 = dart::math::wrapToPi(q6+M_PI);

      S6 = sin(q6);
      C6 = cos(q6);

      S2 = C6*ay + S6*ax;
      radical = 1-S2*S2;
      sqrt_radical = std::sqrt(radical);
      if(sqrt_radical.imag() != 0)
          isValid = false;
      q2 = atan2(S2, alternatives(i,2)*sqrt_radical.real());

      q1 = atan2(C6*sy + S6*sx, C6*ny + S6*nx);
      C2 = cos(q2);
      if( C2 < 0 )
          q1 = dart::math::wrapToPi(q1+M_PI);

      q345 = atan2(-az/C2, -(C6*ax - S6*ay)/C2);
      q3 = dart::math::wrapToPi(q345 - q4 - q5);

      testQ[0]=q1; testQ[1]=q2; testQ[2]=q3; testQ[3]=q4; testQ[4]=q5; testQ[5]=q6;

      for(int k=0; k<testQ.size(); ++k)
          if( fabs(testQ[k]) < zeroSize )
              testQ[k] = 0;

      int validity = isValid? VALID : OUT_OF_REACH;
      mSolutions.push_back(Solution(testQ, validity));
    }

    checkSolutionJointLimits();

    return mSolutions;
  }

  const std::vector<std::size_t>& getDofs() const override
  {
    if(!configured)
      configure();

    return mDofs;
  }

  const double zeroSize = 1e-8;

protected:

  void configure() const
  {
    configured = false;

    mBaseLink = mIK->getNode()->getSkeleton()->getBodyNode(mBaseLinkName);

    BodyNode* base = mBaseLink.lock();
    if(nullptr == base)
    {
      dterr << "[HuboLegIK::configure] base link is a nullptr\n";
      assert(false);
      return;
    }

    const SkeletonPtr& skel = mIK->getNode()->getSkeleton();
    BodyNode* pelvis = skel->getBodyNode("Body_TSY");
    if(nullptr == pelvis)
    {
      dterr << "[HuboLegIK::configure] Could not find Hubo's pelvis "
            << "(Body_TSY)\n";
      assert(false);
      return;
    }

    Eigen::Vector6d saved_q;

    DegreeOfFreedom* dofs[6];
    BodyNode* bn = base;
    for(std::size_t i=0; i < 6; ++i)
    {
      Joint* joint = bn->getParentJoint();
      if(joint->getNumDofs() != 1)
      {
        dterr << "[HuboLegIK::configure] Invalid number of DOFs ("
              << joint->getNumDofs() << ") in the Joint [" << joint->getName()
              << "]\n";
        assert(false);
        return;
      }

      dofs[i] = joint->getDof(0);
      saved_q[i] = dofs[i]->getPosition();
      dofs[i]->setPosition(0.0);

      if(bn->getNumChildBodyNodes() > 0)
        bn = bn->getChildBodyNode(0);
    }

    L4 = std::abs(dofs[3]->getChildBodyNode()->
        getRelativeTransform().translation()[2]);

    L5 = std::abs(dofs[4]->getChildBodyNode()->
        getRelativeTransform().translation()[2]);

    // This offset will be taken care of with footTfInv
    L6 = 0.0;

    hipRotation = Eigen::Isometry3d::Identity();
    hipRotation.rotate(Eigen::AngleAxisd(90*M_PI/180.0,
                                         Eigen::Vector3d::UnitZ()));

    waist = dofs[2]->getChildBodyNode()->getTransform(
              dofs[0]->getParentBodyNode()) * hipRotation;

    footTfInv = Eigen::Isometry3d::Identity();
    footTfInv.rotate(Eigen::AngleAxisd(-90*M_PI/180.0,
                                       Eigen::Vector3d::UnitY()));
    footTfInv = footTfInv * mIK->getNode()->getTransform(dofs[5]->getChildBodyNode());
    footTfInv = footTfInv.inverse();

    alternatives <<
         1,  1,  1,
         1,  1, -1,
         1, -1,  1,
         1, -1, -1,
        -1,  1,  1,
        -1,  1, -1,
        -1, -1,  1,
        -1, -1, -1;

    for(std::size_t i=0; i < 6; ++i)
    {
      dofs[i]->setPosition(saved_q[i]);
      mDofs.push_back(dofs[i]->getIndexInSkeleton());
    }

    configured = true;
  }

  mutable double L4, L5, L6;
  mutable Eigen::Isometry3d waist;
  mutable Eigen::Isometry3d hipRotation;
  mutable Eigen::Isometry3d footTfInv;
  mutable Eigen::Matrix<int, 8, 3> alternatives;

  mutable std::vector<std::size_t> mDofs;

  mutable bool configured;

  std::string mBaseLinkName;

  mutable WeakBodyNodePtr mBaseLink;

};

class InputHandler;

class TeleoperationWorld : public dart::gui::osg::WorldNode
{
public:

  enum MoveEnum_t
  {
    MOVE_Q = 0,
    MOVE_W,
    MOVE_E,
    MOVE_A,
    MOVE_S,
    MOVE_D,
    MOVE_F,
    MOVE_Z,

    NUM_MOVE
  };

  TeleoperationWorld(WorldPtr _world, SkeletonPtr _robot)
    : dart::gui::osg::WorldNode(_world),
      mHubo(_robot),
      iter(0),
      l_foot(_robot->getEndEffector("l_foot")),
      r_foot(_robot->getEndEffector("r_foot")),
      l_hand(_robot->getEndEffector("l_hand")),
      r_hand(_robot->getEndEffector("r_hand"))
  {
    mMoveComponents.resize(NUM_MOVE, false);
    mAnyMovement = false;
    mAmplifyMovement = false;
  }

  void setMovement(const std::vector<bool>& moveComponents)
  {
    mMoveComponents = moveComponents;

    mAnyMovement = false;

    for(bool move : mMoveComponents)
    {
      if(move)
      {
        mAnyMovement = true;
        break;
      }
    }
  }

  void customPreRefresh() override;

  bool mAmplifyMovement;

  InputHandler* mInput;

protected:

  SkeletonPtr mHubo;
  std::size_t iter;

  EndEffectorPtr l_foot;
  EndEffectorPtr r_foot;

  EndEffectorPtr l_hand;
  EndEffectorPtr r_hand;

  std::vector<IK::Analytical::Solution> mSolutions;

  Eigen::VectorXd grad;

  // Order: q, w, e, a, s, d
  std::vector<bool> mMoveComponents;

  bool mAnyMovement;
};

class InputHandler : public ::osgGA::GUIEventHandler
{
public:

  InputHandler(Viewer* viewer, TeleoperationWorld* teleop,
               const SkeletonPtr& hubo, const WorldPtr& world)
    : mViewer(viewer),
      mTeleop(teleop),
      mHubo(hubo),
      mWorld(world)
  {
    mTeleop->mInput = this;
    initialize();
  }

  std::shared_ptr<Box> createBox()
  {
    std::shared_ptr<Box> box = std::make_shared<Box>();
    Eigen::Vector3d s(0.5*Eigen::Vector3d::Ones());
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.translation()[2] += s[2]/2.0;
    box->node = std::make_shared<SimpleFrame>(
          mHubo->getEndEffector("l_foot"),
          "box_"+std::to_string(numBoxes), tf);
    ++numBoxes;

    BoxShapePtr shape = std::make_shared<BoxShape>(s);
    shape->addDataVariance(Shape::DYNAMIC_COLOR);
    shape->addDataVariance(Shape::DYNAMIC_PRIMITIVE);
    box->node->setShape(shape);
    box->node->createVisualAspect();

    boxes.push_back(box);

    mWorld->addSimpleFrame(box->node);

    mSelectedBox = boxes.size()-1;
    selectBox();

    return box;
  }

  void flipBox(std::shared_ptr<Box> box)
  {
    const Eigen::Vector3d v = box->node->getRelativeTransform().translation();
    const Eigen::Vector3d s = std::dynamic_pointer_cast<BoxShape>(
          box->node->getShape())->getSize();

    ADD_BOX(v[0], v[1], v[2], s[0], s[1], s[2], box->left? 1 : 0, true);
  }

  void ADD_BOX(double v0, double v1, double v2,
               double s0, double s1, double s2,
               int l, bool flip = false)
  {
    std::shared_ptr<Box> box = createBox();
    Eigen::Vector3d v(v0, v1, v2);
    Eigen::Vector3d s(s0, s1, s2);
    bool left = l!=0;

    if(flip)
    {
      v[1] = -v[1];
      left = !left;
    }

    std::dynamic_pointer_cast<BoxShape>(box->node->getShape())->setSize(s);

    std::cout << v.transpose() << " : " << s.transpose() << std::endl;

    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.translation() = v;
    box->node->setRelativeTransform(tf);

    box->left = left;
    if(left)
      box->node->setParentFrame(mHubo->getEndEffector("l_foot"));
    else
      box->node->setParentFrame(mHubo->getEndEffector("r_foot"));

    selectBox();
  }

  void ADD_CONFIG(std::vector<double>& X)
  {
    Eigen::VectorXd q(X.size());
    for(size_t i=0; i < X.size(); ++i)
      q[i] = X[i];

    configs.push_back(q);
  }

  template<typename... R>
  void ADD_CONFIG(std::vector<double>& X, double x, R... remainder)
  {
    X.push_back(x);
    ADD_CONFIG(X, remainder...);
  }

  template<typename... R>
  void ADD_CONFIG(double x, R... remainder)
  {
    std::vector<double> X;
    ADD_CONFIG(X, x, remainder...);
  }

  void createInitialBoxes()
  {
//    ADD_BOX(-0.11, 0.005, 0.076, 0.225, 0.15, 0.175, 1 );
//    ADD_BOX(-0.109144, -0.0590679, 0.337675, 0.35, 0.55, 0.35, 1 );


//    ADD_BOX(-0.11, 0.005, 0.076, 0.225, 0.15, 0.175, 1 );
//    ADD_BOX(-0.109144, 0.0103071, 0.337675, 0.5, 0.4, 0.35, 1 );
//    ADD_BOX(-0.15, 0.11, 0.813, 0.8, 0.6, 0.6, 1 );
//    ADD_BOX(-0.15, 0.01, 1.21087, 0.35, 0.4, 0.2, 1 );
//    ADD_BOX(-0.11, -0.005, 0.076, 0.225, 0.15, 0.175, 0 );
//    ADD_BOX(-0.109144, -0.0103071, 0.337675, 0.5, 0.4, 0.35, 0 );
//    ADD_BOX(-0.15, -0.11, 0.813, 0.8, 0.6, 0.6, 0 );
//    ADD_BOX(-0.15, -0.01, 1.21087, 0.35, 0.4, 0.2, 0 );


    ADD_BOX(-0.110391, 0.00156427, 0.0110119, 0.25, 0.15, 0.05, 1 );
    ADD_BOX(-0.140391, 0.00656427, 0.0590119, 0.55, 0.5, 0.05, 1 );
    ADD_BOX(-0.125391, 0.0415643, 0.108012, 0.8, 0.65, 0.05, 1 );
    ADD_BOX(-0.105739, 0.0294226, 0.15635, 0.8375, 0.675, 0.05, 1 );
    ADD_BOX(-0.101268, -0.00938678, 0.206045, 0.95, 0.65, 0.05, 1 );
    ADD_BOX(-0.112681, -0.00614026, 0.25549, 1.1, 0.575, 0.05, 1 );
    ADD_BOX(-0.0985427, -0.00835481, 0.305927, 1.3, 0.525, 0.05, 1 );
    ADD_BOX(-0.0935731, 0.0031974, 0.355399, 1.225, 0.45, 0.05, 1 );
    ADD_BOX(-0.100817, 0.00413067, 0.406698, 1.1, 0.375, 0.05, 1 );


//    ADD_CONFIG(-0.0064084, 0.0542555, 1.56454, 0.592375, -0.99157, 1.28162, 0.00726405, -0.0933735, -0.362946, 1.02419, -0.699833, 0.06266, 0.00723432, -0.09414, -0.393854, 1.06434, -0.709078, 0.0634271, -6.38935e-10, 0.674194, -0.0433137, -0.00769002, -2.29854, -0.0163607, 0.0576931, -0.0123386, 0, 0, 0, 0.778503, -0.0283702, 0.115122, -2.30866, 0.0251078, 0.00902984, 0.0115897);
//    ADD_CONFIG(-0.0064084, 0.0542555, 1.56454, 0.592375, -0.99157, 1.28162, 0.00726405, -0.0933735, -0.773723, 0.976543, -0.241411, 0.06266, 0.00723432, -0.09414, -0.393854, 1.06434, -0.709078, 0.0634271, -6.38935e-10, 0.674194, -0.0433137, -0.00769002, -2.29854, -0.0163607, 0.0576931, -0.0123386, 0, 0, 0, 0.778503, -0.0283702, 0.115122, -2.30866, 0.0251078, 0.00902984, 0.0115897);
//    ADD_CONFIG(-0.0064084, 0.0542555, 1.56454, 0.593812, -0.721574, 1.11362, 0.00649108, -0.113267, -0.614157, 1.77212, -1.19663, 0.0825683, 0.0075281, -0.0865599, -0.674375, 0.835819, -0.20001, 0.0558413, -6.38935e-10, 0.674194, -0.0433137, -0.00769002, -2.29854, -0.0163607, 0.0576931, -0.0123386, 0, 0, 0, 0.778503, -0.0283702, 0.115122, -2.30866, 0.0251078, 0.00902984, 0.0115897);
//    ADD_CONFIG(-0.0044694, 0.0539438, 1.49475, 0.629892, -0.508705, 0.97962, 0.0797254, -0.0266514, -0.511336, 0.772974, -0.297831, -0.0067189, 0.079756, -0.0258067, -0.667968, 1.59744, -0.965661, -0.00756413, -6.38935e-10, 0.674194, -0.0433137, -0.00769002, -2.29854, -0.0163607, 0.0576931, -0.0123386, 0, 0, 0, 0.778503, -0.0283702, 0.115122, -2.30866, 0.0251078, 0.00902984, 0.0115897);
//    ADD_CONFIG(-0.0044694, 0.0539438, 1.49475, 0.633645, -0.458847, 0.81562, 0.080138, -0.0152569, -0.876787, 1.66182, -0.821217, -0.0181209, 0.0799367, -0.0208173, -0.672125, 0.821822, -0.185886, -0.0125568, -6.38935e-10, 0.674194, -0.0433137, -0.00769002, -2.29854, -0.0163607, 0.0576931, -0.0123386, 0, 0, 0, 0.778503, -0.0283702, 0.115122, -2.30866, 0.0251078, 0.00902984, 0.0115897);
//    ADD_CONFIG(0.412139, -0.411002, 1.56306, 0.626723, -0.258966, 0.62662, -0.0315267, 0.00263372, 0.0215828, 1.03206, -0.530868, -0.0106397, -0.0332993, 0.00570971, 0.0779193, 1.61739, -1.17253, -0.0141899, -6.38935e-10, 0.557723, 0, 0, -1.61551, 5.57197e-08, -0.118365, 5.54044e-08, 0, 0, 0, 0.639658, 0, 0, -1.69424, 0, 0.0223095, -3.71196e-10);
//    ADD_CONFIG(0.412139, -0.411002, 1.56306, 0.618416, -0.0191102, 0.38312, -0.0244701, -0.00961172, 0.273572, 1.78769, -1.53846, 0.00349342, -0.0249461, -0.00878562, 0.0923191, 1.42418, -0.993705, 0.00253996, -6.38935e-10, 0.557723, 0, 0, -1.61551, 5.57197e-08, -0.118365, 5.54044e-08, 0, 0, 0, 0.639658, 0, 0, -1.69424, 0, 0.0223095, -3.71196e-10);
//    ADD_CONFIG(0.412139, -0.411002, 1.56306, 0.617204, 0.0158689, 0.30512, -0.024206, -0.0100699, -0.0307712, 1.16854, -0.614962, 0.00402225, -0.0233843, -0.0114956, 0.0857103, 1.69058, -1.25347, 0.00566774, -6.38935e-10, 0.557723, 0, 0, -1.61551, 5.57197e-08, -0.118365, 5.54044e-08, 0, 0, 0, 0.639658, 0, 0, -1.69424, 0, 0.0223095, -3.71196e-10);
//    ADD_CONFIG(0.412139, -0.411002, 1.56306, 0.60959, 0.235737, 0.16112, -0.0153117, -0.0254986, 0.273068, 1.55512, -1.30523, 0.0218302, -0.0181967, -0.0204951, 0.17082, 1.07825, -0.72618, 0.016055, -6.38935e-10, 0.557723, 0, 0, -1.61551, 5.57197e-08, -0.118365, 5.54044e-08, 0, 0, 0, 0.639658, 0, 0, -1.69424, 0, 0.0223095, -3.71196e-10);
//    ADD_CONFIG(-0.0427557, 0.0310215, 1.54569, 0.611088, 0.295718, 0.0821198, 0.0289223, -0.0014629, -0.555767, 0.822249, -0.313728, 0.00817425, 0.0287901, -0.00426044, -0.690165, 1.59398, -0.951061, 0.0109749, -6.38935e-10, 0.557723, 0, 0, -1.61551, 5.57197e-08, -0.118365, 5.54044e-08, 0, 0, 0, 0.639658, 0, 0, -1.69424, 0, 0.0223095, -3.71196e-10);
//    ADD_CONFIG(-0.0427557, 0.0310215, 1.54569, 0.615209, 0.460667, -0.0778802, 0.0292997, 0.00651962, -0.622584, 1.6217, -1.04636, 0.000182819, 0.0292956, 0.00643169, -0.55191, 0.897004, -0.39234, 0.000270843, -6.38935e-10, 0.557723, 0, 0, -1.61551, 5.57197e-08, -0.118365, 5.54044e-08, 0, 0, 0, 0.639658, 0, 0, -1.69424, 0, 0.0223095, -3.71196e-10);
//    ADD_CONFIG(-0.0427557, 0.0310215, 1.54569, 0.619205, 0.620617, -0.16388, 0.0296934, 0.014844, -0.72101, 1.03975, -0.365995, -0.00815082, 0.0297301, 0.0156205, -0.458176, 1.31034, -0.899413, -0.00892819, -6.38935e-10, 0.557723, 0, 0, -1.61551, 5.57197e-08, -0.118365, 5.54044e-08, 0, 0, 0, 0.639658, 0, 0, -1.69424, 0, 0.0223095, -3.71196e-10);
//    ADD_CONFIG(-0.0427557, 0.0310215, 1.54569, 0.624574, 0.83555, -0.18888, 0.030262, 0.0268645, -0.385879, 1.13876, -0.800144, -0.0201848, 0.0302556, 0.0267303, -0.766229, 1.20074, -0.481777, -0.0200504, -6.38935e-10, 0.557723, 0, 0, -1.61551, 5.57197e-08, -0.118365, 5.54044e-08, 0, 0, 0, 0.639658, 0, 0, -1.69424, 0, 0.0223095, -3.71196e-10);
//    ADD_CONFIG(-0.0427557, 0.0310215, 1.54569, 0.626322, 0.905528, -0.18888, 0.0304413, 0.0306547, -0.653296, 1.2189, -0.612867, -0.0239792, 0.0304351, 0.030523, -0.647355, 1.22068, -0.620596, -0.0238474, -6.38935e-10, 0.557723, 0, 0, -1.61551, 5.57197e-08, -0.118365, 5.54044e-08, 0, 0, 0, 0.639658, 0, 0, -1.69424, 0, 0.0223095, -3.71196e-10);


//    ADD_CONFIG(0, 0, 0, 0, 0, 0.76662, 0, 0, -0.785398, 1.5708, -0.785398, 0, 0, 0, -0.785398, 1.5708, -0.785398, 0, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0, 0, 0, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0);
//    ADD_CONFIG(0, 0, 0, -1.7925, 0, 0.83037, 0, 0, -0.405784, 1.26095, -0.85517, 0, 0, 0, -0.93727, 1.12304, -0.18577, 0, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0, 0, 0, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0);
//    ADD_CONFIG(0.010739, 0.472984, -0.0166738, -1.7875, 0, 0.83037, 0.0116087, -0.0180401, -1.00013, 1.22387, -0.69667, 0.004299, 0.0115756, -0.0181049, -1.36142, 0.811991, 0.0764968, 0.00437179, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0, 0, 0, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0);
//    ADD_CONFIG(-0.0200414, -0.434693, 0.0219532, -0.837609, -0.0146939, 0.73937, -0.0276086, 0.0191315, 0.213719, 1.09865, -0.877419, -0.00486763, -0.0275317, 0.0189661, -0.391096, 1.14419, -0.318148, -0.00468524, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0, 0, 0, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0);
//    ADD_CONFIG(0.010739, 0.472984, -0.0166738, -0.467653, -0.0201153, 0.74137, 0.00722262, -0.0266093, -1.35091, 0.755238, 0.122637, 0.0139249, 0.0162932, -0.00888492, -0.92129, 1.20582, -0.757402, -0.00598482, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0, 0, 0, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0);
//    ADD_CONFIG(-0.0200414, -0.434693, 0.0219532, 0.301939, 0.0174184, 0.73937, -0.0375264, 0.0404603, -0.472735, 0.89778, 0.0101975, -0.0283876, -0.0226695, 0.00850125, -0.0205193, 1.20845, -0.753047, 0.0068538, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0, 0, 0, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0);
//    ADD_CONFIG(0.010739, 0.472984, -0.0166738, 0.817198, -0.0396972, 0.74137, 0.0127696, -0.0157716, -0.797336, 1.14283, -0.818414, 0.00175075, 0.0116402, -0.0179786, -1.36284, 0.82949, 0.060416, 0.00422992, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0, 0, 0, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0);
//    ADD_CONFIG(-0.0200414, -0.434693, 0.0219532, 1.63641, 0.0550236, 0.73937, -0.0280297, 0.0200378, 0.0631171, 1.52229, -1.15045, -0.00586693, -0.0279275, 0.0198177, -0.648348, 1.56244, -0.479141, -0.00562427, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0, 0, 0, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0);
//    ADD_CONFIG(-0.0884185, 0.467971, 0.394323, -1.76911, 0.00785536, 0.83037, -0.340548, 0.219028, -0.997312, 1.22497, -0.674541, -0.0508541, -0.311425, 0.21401, -1.65174, 1.07575, -0.320413, -0.0653585, -0.447719, -0.937051, 0.107684, -0.0416174, -1.54725, 0.361615, 0.0149998, -0.207424, 0, 0, 0, 0.711052, 0.00939222, -0.376081, -1.00428, -0.135502, -0.223307, -0.460984);
//    ADD_CONFIG(-0.0200414, -0.434693, 0.0219532, -0.837609, -0.0146939, 0.73937, -0.0312993, 0.0160104, 0.1628, 1.46851, -0.281912, -0.00588489, -0.0275317, 0.0189661, -0.391096, 1.14419, -0.318148, -0.00468524, 0, -0.191409, 0.0471485, -0.0162657, -1.13354, 0.0226592, -0.251753, 0.0284777, 0, 0, 0, 0.81341, -0.0948062, -0.114358, -1.33267, -0.139596, -0.441466, -0.160936);
//    ADD_CONFIG(0.010739, 0.472984, -0.0166738, -0.467653, -0.0201153, 0.74137, 0.00400309, -0.0234543, -1.57893, 0.879265, -0.102893, 0.0132982, 0.0162932, -0.00888492, -0.92129, 1.20582, -0.757402, -0.00598482, 0, 0.706595, 0.0974204, -0.242543, -0.640141, 0.336393, -0.14842, -0.0597675, 0, 0, 0, -0.423601, -0.147833, 0.0570435, -1.7182, -0.117669, -0.37894, 0.028999);
//    ADD_CONFIG(-0.0200414, -0.434693, 0.0219532, 0.302958, -0.042573, 0.73937, -0.0308769, 0.0261635, -0.478209, 0.9838, -0.0705699, -0.0126216, -0.0173139, 0.0123333, 0.0841802, 1.5461, -0.341139, 0.00858068, 0, 1.32366, 0.102771, 0.278691, -1.12307, 0.0945863, 0.0673144, 0.112537, 0, 0, 0, -0.0338947, 0.0507568, -0.00493848, -1.39261, 0.0228408, -0.144186, 0.028243);
////    ADD_CONFIG(-0.0200414, -0.434693, 0.0219532, 0.301939, 0.0174184, 0.73937, -0.0375264, 0.0404603, -0.472735, 0.89778, 0.0101975, -0.0283876, -0.0173139, 0.0123333, 0.0841802, 1.5461, -0.341139, 0.00858068, 0, 1.32366, 0.102771, 0.278691, -1.12307, 0.0945863, 0.0673144, 0.112537, 0, 0, 0, -0.0338947, 0.0507568, -0.00493848, -1.39261, 0.0228408, -0.144186, 0.028243);
//    ADD_CONFIG(0.010739, 0.472984, -0.0166738, 0.817198, -0.0396972, 0.74137, 0.0127696, -0.0157716, -0.797336, 1.14283, -0.818414, 0.00175075, 0.0101865, -0.0163325, -1.6562, 1.03036, -0.384914, 0.00398978, 0, -0.801251, 0.246015, -0.00490418, -1.65162, -0.0548297, 0.0090178, 0.234999, 0, 0, 0, 0.761586, -0.125419, -0.00162068, -0.619121, -0.0270319, 0.136909, -0.138378);
//    ADD_CONFIG(-0.0200414, -0.434693, 0.0219532, 1.63641, 0.0550236, 0.73937, -0.0306251, 0.0163461, 0.261575, 1.36128, -0.362954, -0.00532234, -0.0279275, 0.0198177, -0.648348, 1.56244, -0.479141, -0.00562427, 0, -0.279874, 0.222149, -0.0873567, -1.09222, 0.113565, -0.206902, 0.126148, 0, 0, 0, 1.36927, -0.00902004, -0.0218707, -1.22307, 0.177575, 0.161155, -0.250746);


//    ADD_CONFIG(0, 0, -1.5708, -0.252783, 1.07924, -4.78138, 0, 0, -0.785398, 1.5708, -0.785398, 0, 0, 0, -0.785398, 1.5708, -0.785398, 0, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0, 0, 0, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0);
//    ADD_CONFIG(0, 0, -1.5708, -0.252783, 0.394236, -4.78138, 0, 0, -0.785398, 1.5708, -0.785398, 0, 0, 0, -0.785398, 1.5708, -0.785398, 0, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0, 0, 0, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0);
//    ADD_CONFIG(0, 0, -1.02974, -0.204075, 0.168254, -4.78138, 0, 0, -0.785398, 1.5708, -0.785398, 0, 0, 0, -0.785398, 1.5708, -0.785398, 0, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0, 0, 0, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0);
//    ADD_CONFIG(0, 0, -0.855211, -0.0654131, -0.020373, -4.78138, 0, 0, -0.785398, 1.5708, -0.785398, 0, 0, 0, -0.785398, 1.5708, -0.785398, 0, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0, 0, 0, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0);
//    ADD_CONFIG(0, 0, -0.366519, 0.0833809, -0.130205, -4.78138, 0, 0, -0.785398, 1.5708, -0.785398, 0, 0, 0, -0.785398, 1.5708, -0.785398, 0, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0, 0, 0, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0);
//    ADD_CONFIG(0, 0, 0.0523599, 0.455177, -0.18563, -4.78138, 0, 0, -0.785398, 1.5708, -0.785398, 0, 0, 0, -0.785398, 1.5708, -0.785398, 0, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0, 0, 0, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0);
//    ADD_CONFIG(0, 0, 0.261799, 1.45381, -0.133294, -4.78138, 0, 0, -0.785398, 1.5708, -0.785398, 0, 0, 0, -0.785398, 1.5708, -0.785398, 0, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0, 0, 0, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0);
//    ADD_CONFIG(0, 0, 0.593412, 3.66934, 0.955254, -4.78138, 0, 0, -0.785398, 1.5708, -0.785398, 0, 0, 0, -0.785398, 1.5708, -0.785398, 0, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0, 0, 0, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0);
//    ADD_CONFIG(0, 0, -1.5708, -0.252783, 0.314236, -4.78138, 0, -6.29756e-07, -0.600877, 1.54135, -0.940468, 6.29756e-07, 0, 7.87196e-07, -0.973573, 1.52492, -0.551345, -7.87196e-07, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0, 0, 0, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0);
//    ADD_CONFIG(0, 0, 0.261799, 0.803358, -0.255818, -4.78138, -5.14339e-07, 0.127883, -1.06782, 1.40482, -0.336989, -0.127883, -5.14338e-07, 0.127883, -0.618143, 1.53171, -0.913559, -0.127883, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0, 0, 0, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0);
//    ADD_CONFIG(0, 0, 0.261799, 2.13754, 0.174143, -4.78138, 1.02868e-06, -0.251714, -0.346029, 1.37048, -1.02445, 0.251714, 1.02868e-06, -0.251715, -0.948886, 1.44886, -0.499972, 0.251715, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0, 0, 0, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0);


    ADD_CONFIG(0, 0, 0, -0.81, 0, 0.87462, 0, 0, -0.513997, 1.02807, -0.514072, 0, 0, 0, -0.513997, 1.02807, -0.514072, 0, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0, 0, 0, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0);
    ADD_CONFIG(0, 0, 0, -0.5, 0, 0.73462, 0, 0, -0.851831, 1.70364, -0.85181, 0, 0, 0, -0.851831, 1.70364, -0.85181, 0, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0, 0, 0, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0);
    ADD_CONFIG(0, 0, 0, -0.05, 0, 0.73462, 0, 0, -0.851831, 1.70364, -0.85181, 0, 0, 0, -0.851831, 1.70364, -0.85181, 0, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0, 0, 0, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0);
    ADD_CONFIG(0, 0, 0, 0.55, 0, 0.89062, 0, 0, -0.462289, 0.924667, -0.462378, 0, 0, 0, -0.462289, 0.924667, -0.462378, 0, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0, 0, 0, 0, 0.523599, 0, 0, -2.0944, 0, 0, 0);


    mSelectedBox = -1;
    selectBox();
  }

  void initialize()
  {
    ds = 0.05;
    dx = 0.01;
    mSelectedBox = -1;
    currentBox = nullptr;
    currentDnD = nullptr;
    numBoxes = 0;
    mLighting = 1;

    mRestConfig = mHubo->getPositions();

    for(std::size_t i=0; i < mHubo->getNumEndEffectors(); ++i)
    {
      const InverseKinematicsPtr ik = mHubo->getEndEffector(i)->getIK();
      if(ik)
      {
        mDefaultBounds.push_back(ik->getErrorMethod().getBounds());
        mDefaultTargetTf.push_back(ik->getTarget()->getRelativeTransform());
        mConstraintActive.push_back(false);
        mEndEffectorIndex.push_back(i);
      }
    }

//    mPosture = std::dynamic_pointer_cast<RelaxedPosture>(
//          mHubo->getIK(true)->getObjective());

//    mBalance = std::dynamic_pointer_cast<dart::constraint::BalanceConstraint>(
//          mHubo->getIK(true)->getProblem()->getEqConstraint(1));

    mOptimizationKey = 'r';

    mMoveComponents.resize(TeleoperationWorld::NUM_MOVE, false);

//    createInitialBoxes();
    mViewer->getCamera()->getViewMatrixAsLookAt(eye, center, up);

    c_index = -1;

    minimals = getFootprints();
    minimals.push_back(getMinimalRobot(mHubo));
  }

  void makeMinimalCopyWithFeet()
  {
    std::vector<SkeletonPtr> copies;

    SkeletonPtr lf = minimals[0]->clone("lf_#"+std::to_string(minimalCopies.size()));
    lf->setPositions(FreeJoint::convertToPositions(mHubo->getEndEffector("l_foot")->getWorldTransform()));
    copies.push_back(lf);

    SkeletonPtr rf = minimals[1]->clone("rf_#"+std::to_string(minimalCopies.size()));
    rf->setPositions(FreeJoint::convertToPositions(mHubo->getEndEffector("r_foot")->getWorldTransform()));
    copies.push_back(rf);

//    SkeletonPtr mc = minimals[2]->clone("mc_#"+std::to_string(minimalCopies.size()));
    SkeletonPtr mc = getMinimalRobot(mHubo);
    mc->setName("mc_#"+std::to_string(minimalCopies.size()));
    mc->setPositions(mHubo->getJoint(0)->getPositions());
    copies.push_back(mc);

    for(size_t i=0; i < copies.size(); ++i)
      mWorld->addSkeleton(copies[i]);

    minimalCopies.push_back(copies);
  }

  void makeMinimalCopy(const Eigen::Isometry3d tf)
  {
    std::vector<SkeletonPtr> copies;
//    SkeletonPtr mc = minimals[2]->clone("mc_#"+std::to_string(minimalCopies.size()));
    SkeletonPtr mc = getMinimalRobot(mHubo);
//    SkeletonPtr mc = getWalkSweeper(mHubo);
    mc->setPositions(FreeJoint::convertToPositions(tf));
    copies.push_back(mc);
    minimalCopies.push_back(copies);

    mc->setName("mc_#"+std::to_string(minimalCopies.size()));
    mWorld->addSkeleton(mc);
  }

  virtual bool handle(const ::osgGA::GUIEventAdapter& ea,
                      ::osgGA::GUIActionAdapter&) override
  {
    if(nullptr == mHubo)
    {
      return false;
    }

    if( ::osgGA::GUIEventAdapter::KEYDOWN == ea.getEventType() )
    {
      if( ea.getKey() == osgGA::GUIEventAdapter::KEY_Right)
      {
        ++mLighting;
        if(mLighting > 8)
          mLighting = 8;

        mViewer->setupCustomLights(mLighting);
        return true;
      }

      if( ea.getKey() == osgGA::GUIEventAdapter::KEY_Left)
      {
        --mLighting;
        if(mLighting == 0)
          mLighting = 1;

        mViewer->setupCustomLights(mLighting);
        return true;
      }

      if( ea.getKey() == osgGA::GUIEventAdapter::KEY_Up )
      {
        selectNextBox();
        return true;
      }

      if( ea.getKey() == osgGA::GUIEventAdapter::KEY_Down )
      {
        selectPrevBox();
        return true;
      }

      if( ea.getKey() == osgGA::GUIEventAdapter::KEY_BackSpace )
      {
        mSelectedBox = -1;
        selectBox();
        return true;
      }

      if( ea.getKey() == osgGA::GUIEventAdapter::KEY_Delete )
      {
        if(mSelectedBox < 0)
          return true;

        if(boxes.empty())
        {
          mSelectedBox = -1;
          selectBox();
          return true;
        }

        mTeleop->getWorld()->removeSimpleFrame(currentBox->node);
        boxes.erase(boxes.begin()+mSelectedBox);
        --mSelectedBox;

        selectBox();
      }

      if( ea.getKey() == 'o' )
      {
        mInitialLeftTf = mHubo->getEndEffector("l_foot")->getTransform();

        return true;
      }

      if( ea.getKey() == 'p' )
      {
//        for(std::size_t i=0; i < mHubo->getNumDofs(); ++i)
//          std::cout << mHubo->getDof(i)->getName() << ": "
//                    << mHubo->getDof(i)->getPosition() << std::endl;
//        return true;

//        std::cout << mHubo->getEndEffector("l_foot")->getTransform(
//              mHubo->getBodyNode(0)).matrix() << std::endl;

        const Eigen::Isometry3d tf = mInitialLeftTf.inverse()*
            mHubo->getEndEffector("l_foot")->getTransform();

        Eigen::AngleAxisd aa(tf.linear());
        const double angle = dart::math::wrapToPi(aa.angle())*
            (Eigen::Vector3d::UnitZ().dot(aa.axis()))*180.0/M_PI;


        std::cout << tf.translation()[0] << ", "
                  << tf.translation()[1] << ", "
                  << angle << std::endl;


//        printBoxes(boxes);

//        printConfigs(configs);

        return true;
      }

      if( ea.getKey() == 't' )
      {
        // Reset all the positions except for x, y, and yaw
//        for(std::size_t i=0; i < mHubo->getNumDofs(); ++i)
//        {
//          if( i < 2 || 4 < i )
//            mHubo->getDof(i)->setPosition(mRestConfig[i]);
//        }

        if(!currentBox)
          return true;

        Eigen::Isometry3d tf = currentBox->node->getRelativeTransform();
        tf.linear() = Eigen::Matrix3d::Identity();
        currentBox->node->setRelativeTransform(tf);

        return true;
      }

      if( ea.getKey() == 'y' )
      {
        mViewer->getCamera()->getViewMatrixAsLookAt(eye, center, up);
        return true;
      }

      if( ea.getKey() == 'Y' )
      {
        mViewer->getCameraManipulator()->setHomePosition(eye, center, up);
        mViewer->setCameraManipulator(mViewer->getCameraManipulator());
        return true;
      }

      if( ea.getKey() == 'u' )
      {
        configs.push_back(mHubo->getPositions());
        printConfigs(configs);

        std::cout << "Existing configs: " << configs.size() << std::endl;

        return true;
      }

      if( ea.getKey() == 'U' )
      {
        if(!configs.empty())
          configs.pop_back();

        std::cout << "Remaining configs: " << configs.size() << std::endl;

        return true;
      }

      if( ea.getKey() == 'h' )
      {
        ++c_index;
        if(c_index >= configs.size())
        {
          if(configs.empty())
          {
            c_index = -1;
            return true;
          }

          c_index = 0;
        }

        mHubo->setPositions(configs[c_index]);
        return true;
      }

      if( ea.getKey() == 'H' )
      {
        --c_index;
        if(c_index < 0)
        {
          if(configs.empty())
          {
            c_index = -1;
            return true;
          }

          c_index = configs.size()-1;
        }

        mHubo->setPositions(configs[c_index]);
        return true;
      }

      if( ea.getKey() == '-' )
      {
        ds /= 2.0;
        return true;
      }

      if( ea.getKey() == '=' )
      {
        ds *= 2.0;
        return true;
      }

      if( ea.getKey() == '_' )
      {
        dx /= 2.0;
        return true;
      }

      if( ea.getKey() == '+' )
      {
        dx *= 2.0;
        return true;
      }

      if( ea.getKey() == '[' )
      {
        setScale(0, -ds);
        return true;
      }

      if( ea.getKey() == ']' )
      {
        setScale(0, ds);
        return true;
      }

      if( ea.getKey() == ';' )
      {
        setScale(1, -ds);
        return true;
      }

      if( ea.getKey() == '\'' )
      {
        setScale(1, ds);
        return true;
      }

      if( ea.getKey() == '.' )
      {
        setScale(2, -ds);
        return true;
      }

      if( ea.getKey() == '/' )
      {
        setScale(2, ds);
        return true;
      }

      if( ea.getKey() == 'm' )
      {
        createBox();
        printBoxes(boxes);
        return true;
      }

//      if( ea.getKey() == 'm' )
//      {
//        jumpConfigs.push_back(mHubo->getPositions());
//        return true;
//      }

//      if( ea.getKey() == 'M' )
//      {
//        jumpConfigs.clear();
//        return true;
//      }

      if( ea.getKey() == 'n' )
      {
        const size_t b = boxes.size();
        for(size_t i=0; i < b; ++i)
          flipBox(boxes[i]);
        std::cout << " ---- " << std::endl;

        return true;
      }

      if( ea.getKey() == 'l' )
      {
        if(!currentBox)
          return true;

        currentBox->left = !currentBox->left;
        Eigen::Isometry3d tf = currentBox->node->getWorldTransform();

        if(currentBox->left)
        {
          currentBox->node->setParentFrame(mHubo->getEndEffector("l_foot"));
          currentBox->node->setTransform(tf);
        }
        else
        {
          currentBox->node->setParentFrame(mHubo->getEndEffector("r_foot"));
          currentBox->node->setTransform(tf);
        }


        return true;
      }

      if( ea.getKey() == 'j' )
      {
        std::vector<SimpleFramePtr> copies;
        for(size_t i=0; i < boxes.size(); ++i)
        {
          if(!boxes[i]->left)
            continue;

          SimpleFramePtr copy = boxes[i]->node->clone();
//          copy->getVisualAspect()->setAlpha(DefaultAlpha + 0.4*(leftCopies.size()+1));
          copy->getVisualAspect()->setRGBA(CopyRightColor);
          copies.push_back(copy);
          mWorld->addSimpleFrame(copy);
        }

        leftCopies.push_back(copies);

        return true;
      }

      if( ea.getKey() == 'J' )
      {
        if(leftCopies.empty())
          return true;

        const std::vector<SimpleFramePtr>& last = leftCopies.back();
        for(size_t i=0; i < last.size(); ++i)
          mWorld->removeSimpleFrame(last[i]);

        leftCopies.pop_back();
        return true;
      }

      if( ea.getKey() == 'k' )
      {
        std::vector<SimpleFramePtr> copies;
        for(size_t i=0; i < boxes.size(); ++i)
        {
          if(boxes[i]->left)
            continue;

          SimpleFramePtr copy = boxes[i]->node->clone();
//          copy->getVisualAspect()->setAlpha(DefaultAlpha + 0.4*rightCopies.size());
          copy->getVisualAspect()->setRGBA(CopyRightColor);
          copies.push_back(copy);
          mWorld->addSimpleFrame(copy);
        }

        rightCopies.push_back(copies);
      }

      if( ea.getKey() == 'K' )
      {
        if(rightCopies.empty())
          return true;

        const std::vector<SimpleFramePtr>& last = rightCopies.back();
        for(size_t i=0; i < last.size(); ++i)
          mWorld->removeSimpleFrame(last[i]);

        rightCopies.pop_back();
        return true;
      }

      if( ea.getKey() == '`' )
      {
        for(size_t i=0; i < boxes.size(); ++i)
        {
          mWorld->removeSimpleFrame(boxes[i]->node);
          boxes[i]->node->setParentFrame(Frame::World());
        }

        return true;
      }

      if( ea.getKey() == '~' )
      {
        for(size_t i=0; i < boxes.size(); ++i)
        {
          mWorld->addSimpleFrame(boxes[i]->node);
          if(boxes[i]->left)
            boxes[i]->node->setParentFrame(mHubo->getEndEffector("l_foot"));
          else
            boxes[i]->node->setParentFrame(mHubo->getEndEffector("r_foot"));
        }

        return true;
      }

      if( ea.getKey() == 'v' )
      {
        for(size_t i=0; i < rightCopies.size(); ++i)
        {
          const std::vector<SimpleFramePtr>& copies = rightCopies[i];
          for(size_t j=0; j < copies.size(); ++j)
            mWorld->addSimpleFrame(copies[j]);
        }

        for(size_t i=0; i < leftCopies.size(); ++i)
        {
          const std::vector<SimpleFramePtr>& copies = leftCopies[i];
          for(size_t j=0; j < copies.size(); ++j)
            mWorld->addSimpleFrame(copies[j]);
        }

        return true;
      }

      if( ea.getKey() == 'V' )
      {
        for(size_t i=0; i < rightCopies.size(); ++i)
        {
          const std::vector<SimpleFramePtr>& copies = rightCopies[i];
          for(size_t j=0; j < copies.size(); ++j)
            mWorld->removeSimpleFrame(copies[j]);
        }

        for(size_t i=0; i < leftCopies.size(); ++i)
        {
          const std::vector<SimpleFramePtr>& copies = leftCopies[i];
          for(size_t j=0; j < copies.size(); ++j)
            mWorld->removeSimpleFrame(copies[j]);
        }

        return true;
      }

      if( ea.getKey() == 'g' )
      {
        if(jumpConfigs.size() < 2)
        {
          std::cout << "Not enough jump configs!" << std::endl;
          return true;
        }

        mHubo->setPositions(jumpConfigs[0]);
        const Eigen::Isometry3d tf0 = mHubo->getBodyNode(0)->getWorldTransform();
        mHubo->setPositions(jumpConfigs[1]);
        const Eigen::Isometry3d tf1 = mHubo->getBodyNode(0)->getWorldTransform();

//        const double rho = 1e-2;
        const double rho = 1e-3;
        const Eigen::Vector2d x0 = tf0.translation().block<2,1>(0,0);
        const double z0 = tf0.translation()[2];
        const Eigen::Vector2d x1 = tf1.translation().block<2,1>(0,0);
        const double z1 = tf1.translation()[2];
        const double D = (x1-x0).norm();

        if(z1-z0 >= JumpHeight)
        {
          std::cout << "TOO HIGH! " << z1-z0 << std::endl;
          return true;
        }

        if(D < 1e-8)
        {
          std::cout << "Too short! " << D << std::endl;
          return true;
        }

        Eigen::AngleAxisd aa(tf0.linear().transpose()*tf1.linear());

        const size_t samples = D/rho;

        const double da = aa.angle()/(double)(samples);

        for(size_t i=0; i < samples; ++i)
        {
          const double d = i*rho;
//          std::cout << "d: " << d << std::endl;
          const double angle = i*da;

          const double h = z0 + JumpHeight;
          const double c = z0;
          const double alpha = 1.0/(4*(z0-h));
          const double beta = 1.0/D;
          const double gamma = z0 - z1;

          const double Det = pow(beta,2)-4*alpha*gamma;
          if(Det < 0)
          {
            std::cout << "z0: " << z0 << " | z1: " << z1 << " | h: " << h
                      << " | alpha: " << alpha << " | beta: " << beta
                      << " | gamma: " << gamma << std::endl;

            std::cout << "Invalid Det: " << Det << std::endl;
            return true;
          }

          const double b = (-beta - sqrt(Det))/(2*alpha);
//          const double b = (-beta + sqrt(pow(beta,2)-4*alpha*gamma))/(2*alpha);
          const double a = (z1-z0-b*D)/pow(D,2);

          const double z = a*pow(d,2) + b*d + c;

          const Eigen::Vector2d x = (x1-x0)*(double)(i)/(double)(samples) + x0;
          Eigen::Isometry3d tf(tf0);
          tf.translation().block<2,1>(0,0) = x;
          tf.translation()[2] = z;
          tf.rotate(Eigen::AngleAxisd(angle, aa.axis()));

          std::cout << "a: " << a << " | b: " << b << " | c: " << c << " | " << "d: "
                    << d << " | z: " << z << std::endl;
//          std::cout << "angle: " << angle << " | " << aa.axis().transpose() << std::endl;

          makeMinimalCopy(tf);
        }


        return true;
      }

      if( ea.getKey() == 'b' )
      {
        if(minimalSweep.empty())
        {
//          makeMinimalCopyWithFeet();
          return true;
        }

        const double rho = 1e-2;
        Eigen::Isometry3d tf = minimalSweep.front();
        makeMinimalCopy(tf);
        for(size_t i=1; i < minimalSweep.size(); ++i)
        {
          Eigen::Isometry3d next_tf = minimalSweep[i];
          while(diff(next_tf, tf) > 1e-8)
          {
            tf = stepTowards(next_tf, tf, rho);
            makeMinimalCopy(tf);
          }
        }

        return true;
      }

      if( ea.getKey() == 'B' )
      {
        for(size_t i=0; i < minimalCopies.size(); ++i)
        {
          const std::vector<SkeletonPtr>& copies = minimalCopies[i];
          for(size_t j=0; j < copies.size(); ++j)
            mWorld->removeSkeleton(copies[j]);
        }

        minimalCopies.clear();
        return true;
      }

      if( ea.getKey() == 'i' )
      {
//        makeMinimalCopyWithFeet();
        minimalSweep.push_back(mHubo->getBodyNode(0)->getWorldTransform());
        return true;
      }

      if( ea.getKey() == 'I' )
      {
        minimalSweep.pop_back();
        return true;
      }

      if( '1' <= ea.getKey() && ea.getKey() <= '9' )
      {
        std::size_t index = ea.getKey() - '1';
        if(index < mConstraintActive.size())
        {
          EndEffector* ee = mHubo->getEndEffector(mEndEffectorIndex[index]);
          const InverseKinematicsPtr& ik = ee->getIK();
          if(ik && mConstraintActive[index])
          {
            mConstraintActive[index] = false;

            ik->getErrorMethod().setBounds(mDefaultBounds[index]);
            ik->getTarget()->setRelativeTransform(mDefaultTargetTf[index]);

            mWorld->removeSimpleFrame(ik->getTarget());
          }
          else if(ik)
          {
            mConstraintActive[index] = true;

            // Use the standard default bounds instead of our custom default
            // bounds
            ik->getErrorMethod().setBounds();
            ik->getTarget()->setTransform(ee->getTransform());

            mWorld->addSimpleFrame(ik->getTarget());
          }
        }
        return true;
      }

//      if( 'x' == ea.getKey() )
//      {
//        EndEffector* ee = mHubo->getEndEffector("l_foot");
//        ee->getSupport()->setActive(!ee->getSupport()->isActive());
//        return true;
//      }

      if( 'x' == ea.getKey() )
      {
        for(size_t i=0; i < clones.size(); ++i)
        {
          mWorld->removeSkeleton(clones[i]);
        }

        clones.clear();
        return true;
      }

//      if( 'c' == ea.getKey() )
//      {
//        EndEffector* ee = mHubo->getEndEffector("r_foot");
//        ee->getSupport()->setActive(!ee->getSupport()->isActive());
//        return true;
//      }

      if( 'c' == ea.getKey() )
      {
        SkeletonPtr newClone = mHubo->clone("hubo_#"+clones.size());
        newClone->setPositions(mHubo->getPositions());
        setAlpha(newClone, 1.0);

        const double a0 = 0.3;
        const double a1 = 0.95;
        for(size_t i=0; i < clones.size(); ++i)
        {
          const double a = (a1-a0)/clones.size()*i+a0;
          setAlpha(clones[i], a);
        }

        for(size_t i=0; i < clones.size(); ++i)
        {
          mWorld->removeSkeleton(clones[i]);
        }

        clones.push_back(newClone);

        for(size_t i=0; i < clones.size(); ++i)
        {
          mWorld->addSkeleton(clones[i]);
        }

        return true;
      }

      if( 'C' == ea.getKey() )
      {
        for(size_t i=0; i < clones.size(); ++i)
        {
          mWorld->removeSkeleton(clones[i]);
        }

//        clones.clear();
        return true;
      }

      if(ea.getKey() == ::osgGA::GUIEventAdapter::KEY_Shift_L)
        mTeleop->mAmplifyMovement = true;

      switch(ea.getKey())
      {
        case 'w': case 'W': mMoveComponents[TeleoperationWorld::MOVE_W] = true; break;
        case 'a': case 'A': mMoveComponents[TeleoperationWorld::MOVE_A] = true; break;
        case 's': case 'S': mMoveComponents[TeleoperationWorld::MOVE_S] = true; break;
        case 'd': case 'D': mMoveComponents[TeleoperationWorld::MOVE_D] = true; break;
        case 'q': case 'Q': mMoveComponents[TeleoperationWorld::MOVE_Q] = true; break;
        case 'e': case 'E': mMoveComponents[TeleoperationWorld::MOVE_E] = true; break;
        case 'f': case 'F': mMoveComponents[TeleoperationWorld::MOVE_F] = true; break;
        case 'z': case 'Z': mMoveComponents[TeleoperationWorld::MOVE_Z] = true; break;
      }

      switch(ea.getKey())
      {
        case 'w': case 'a': case 's': case 'd': case 'q': case 'e': case 'f': case 'z':
        case 'W': case 'A': case 'S': case 'D': case 'Q': case 'E': case 'F': case 'Z':
        {
          mTeleop->setMovement(mMoveComponents);
          return true;
        }
      }

      if(mOptimizationKey == ea.getKey())
      {
        if(mPosture)
          mPosture->enforceIdealPosture = true;

        if(mBalance)
          mBalance->setErrorMethod(dart::constraint::BalanceConstraint::OPTIMIZE_BALANCE);

        return true;
      }
    }

    if( ::osgGA::GUIEventAdapter::KEYUP == ea.getEventType() )
    {
      if(ea.getKey() == mOptimizationKey)
      {
        if(mPosture)
          mPosture->enforceIdealPosture = false;

        if(mBalance)
          mBalance->setErrorMethod(dart::constraint::BalanceConstraint::FROM_CENTROID);

        return true;
      }

      if(ea.getKey() == ::osgGA::GUIEventAdapter::KEY_Shift_L)
        mTeleop->mAmplifyMovement = false;

      switch(ea.getKey())
      {
        case 'w': case 'W': mMoveComponents[TeleoperationWorld::MOVE_W] = false; break;
        case 'a': case 'A': mMoveComponents[TeleoperationWorld::MOVE_A] = false; break;
        case 's': case 'S': mMoveComponents[TeleoperationWorld::MOVE_S] = false; break;
        case 'd': case 'D': mMoveComponents[TeleoperationWorld::MOVE_D] = false; break;
        case 'q': case 'Q': mMoveComponents[TeleoperationWorld::MOVE_Q] = false; break;
        case 'e': case 'E': mMoveComponents[TeleoperationWorld::MOVE_E] = false; break;
        case 'f': case 'F': mMoveComponents[TeleoperationWorld::MOVE_F] = false; break;
        case 'z': case 'Z': mMoveComponents[TeleoperationWorld::MOVE_Z] = false; break;
      }

      switch(ea.getKey())
      {
        case 'w': case 'a': case 's': case 'd': case 'q': case'e': case 'f': case 'z':
        case 'W': case 'A': case 'S': case 'D': case 'Q': case 'E': case 'F': case 'Z':
        {
          mTeleop->setMovement(mMoveComponents);
          return true;
        }
      }
    }

    return false;
  }

  void setScale(size_t id, double x)
  {
    if(!currentBox)
      return;

    Eigen::Vector3d scale = std::dynamic_pointer_cast<BoxShape>(
          currentBox->node->getShape())->getSize();
    scale[id] += x;

    if(scale[id] <= 0)
      return;

    std::dynamic_pointer_cast<BoxShape>(currentBox->node->getShape())->
        setSize(scale);
  }

  void selectBox()
  {
    currentBox = nullptr;
    if(currentDnD)
    {
      mViewer->disableDragAndDrop(currentDnD);
      currentDnD = nullptr;
    }

    for(size_t i=0; i < boxes.size(); ++i)
    {
      SimpleFramePtr sn = boxes[i]->node;
      VisualAspect* visual = sn->getVisualAspect();
      if(boxes[i]->left)
        visual->setColor(LeftColor);
      else
        visual->setColor(RightColor);
    }

    if(mSelectedBox >= 0)
    {
      currentBox = boxes[mSelectedBox];
      currentBox->node->getVisualAspect()->setColor(SelectedColor);
      currentDnD = mViewer->enableDragAndDrop(currentBox->node.get());
      currentDnD->setRotationOption(dart::gui::osg::DragAndDrop::RotationOption::ALWAYS_OFF);
    }
  }

  void selectNextBox()
  {
    ++mSelectedBox;
    if(mSelectedBox >= boxes.size())
    {
      if(boxes.empty())
        mSelectedBox = -1;
      else
        mSelectedBox = 0;
    }

    selectBox();
  }

  void selectPrevBox()
  {
    --mSelectedBox;
    if(mSelectedBox < 0)
      mSelectedBox = boxes.size()-1;

    selectBox();
  }

//protected:

  Viewer* mViewer;

  TeleoperationWorld* mTeleop;

  SkeletonPtr mHubo;

  WorldPtr mWorld;

  Eigen::VectorXd mRestConfig;

  std::vector<bool> mConstraintActive;

  std::vector<std::size_t> mEndEffectorIndex;

  std::vector< std::pair<Eigen::Vector6d, Eigen::Vector6d> > mDefaultBounds;

  Eigen::aligned_vector<Eigen::Isometry3d> mDefaultTargetTf;

  std::shared_ptr<RelaxedPosture> mPosture;

  std::shared_ptr<dart::constraint::BalanceConstraint> mBalance;

  char mOptimizationKey;

  std::vector<bool> mMoveComponents;

  size_t mLighting;

  int mSelectedBox;

  std::vector<std::shared_ptr<Box>> boxes;

  std::vector<std::vector<SimpleFramePtr>> leftCopies;
  std::vector<std::vector<SimpleFramePtr>> rightCopies;
  std::vector<std::vector<SkeletonPtr>> minimalCopies;
  std::vector<Eigen::Isometry3d> minimalSweep;

  std::vector<SkeletonPtr> minimals;

  std::shared_ptr<Box> currentBox;
  dart::gui::osg::SimpleFrameDnD* currentDnD;

  size_t numBoxes;

  double ds;
  double dx;

  ::osg::Vec3 eye, center, up;

  std::vector<Eigen::VectorXd> configs;
  std::vector<Eigen::VectorXd> jumpConfigs;
  int c_index;

  std::vector<SkeletonPtr> clones;

  Eigen::Isometry3d mInitialLeftTf;
};


void TeleoperationWorld::customPreRefresh()
{
  if(mAnyMovement)
  {
    Eigen::Isometry3d old_tf;

    if(mInput->currentBox)
      old_tf = mInput->currentBox->node->getWorldTransform();
    else
      old_tf = mHubo->getBodyNode(0)->getWorldTransform();

    Eigen::Isometry3d new_tf = Eigen::Isometry3d::Identity();
    Eigen::Vector3d forward = old_tf.linear().col(0);
    forward[2] = 0.0;
    if(forward.norm() > 1e-10)
      forward.normalize();
    else
      forward.setZero();

    Eigen::Vector3d left = old_tf.linear().col(1);
    left[2] = 0.0;
    if(left.norm() > 1e-10)
      left.normalize();
    else
      left.setZero();

    const Eigen::Vector3d& up = Eigen::Vector3d::UnitZ();

    double linearStep = mInput->dx;
    double elevationStep = 0.2*linearStep;
    double rotationalStep = 2.0*M_PI/180.0/0.01*linearStep;

    if(mAmplifyMovement)
    {
      linearStep *= 2.0;
      elevationStep *= 2.0;
      rotationalStep *= 2.0;
    }

    if(mMoveComponents[MOVE_W])
      new_tf.translate( linearStep*forward);

    if(mMoveComponents[MOVE_S])
      new_tf.translate(-linearStep*forward);

    if(mMoveComponents[MOVE_A])
      new_tf.translate( linearStep*left);

    if(mMoveComponents[MOVE_D])
      new_tf.translate(-linearStep*left);

    if(mMoveComponents[MOVE_F])
      new_tf.translate( elevationStep*up);

    if(mMoveComponents[MOVE_Z])
      new_tf.translate(-elevationStep*up);

    if(mMoveComponents[MOVE_Q])
      new_tf.rotate(Eigen::AngleAxisd( rotationalStep, up));

    if(mMoveComponents[MOVE_E])
      new_tf.rotate(Eigen::AngleAxisd(-rotationalStep, up));

    new_tf.pretranslate(old_tf.translation());
    new_tf.rotate(old_tf.rotation());


    if(mInput->currentBox)
    {
      mInput->currentBox->node->setTransform(new_tf);
    }
    else
    {
      mHubo->getJoint(0)->setPositions(FreeJoint::convertToPositions(new_tf));
    }
  }

  mHubo->getIK(true)->solve();
}



SkeletonPtr createGround()
{
  // Create a Skeleton to represent the ground
  SkeletonPtr ground = Skeleton::create("ground");
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  double thickness = 0.01;
  tf.translation() = Eigen::Vector3d(0,0,-thickness/2.0);
  WeldJoint::Properties joint;
  joint.mT_ParentBodyToJoint = tf;
  ground->createJointAndBodyNodePair<WeldJoint>(nullptr, joint);
  ShapePtr groundShape =
      std::make_shared<BoxShape>(Eigen::Vector3d(10,10,thickness));

  auto shapeNode = ground->getBodyNode(0)->createShapeNodeWith<
      VisualAspect, CollisionAspect, DynamicsAspect>(groundShape);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue(0.2));

  return ground;
}

SkeletonPtr createHubo()
{
  dart::utils::DartLoader loader;
  loader.addPackageDirectory("drchubo", DART_DATA_PATH"/urdf/drchubo");
  SkeletonPtr hubo =
      loader.parseSkeleton(DART_DATA_PATH"/urdf/drchubo/drchubo.urdf");

  for(std::size_t i = 0; i < hubo->getNumBodyNodes(); ++i)
  {
    BodyNode* bn = hubo->getBodyNode(i);
    if(bn->getName().substr(0, 7) == "Body_LF"
       || bn->getName().substr(0, 7) == "Body_RF")
    {
      bn->remove();
      --i;
    }
  }

  return hubo;
}

void setStartupConfiguration(const SkeletonPtr& hubo)
{
  hubo->getDof("LHP")->setPosition(-45.0*M_PI/180.0);
  hubo->getDof("LKP")->setPosition( 90.0*M_PI/180.0);
  hubo->getDof("LAP")->setPosition(-45.0*M_PI/180.0);

  hubo->getDof("RHP")->setPosition(-45.0*M_PI/180.0);
  hubo->getDof("RKP")->setPosition( 90.0*M_PI/180.0);
  hubo->getDof("RAP")->setPosition(-45.0*M_PI/180.0);

  hubo->getDof("LSP")->setPosition(  30.0*M_PI/180.0);
  hubo->getDof("LEP")->setPosition(-120.0*M_PI/180.0);

  hubo->getDof("RSP")->setPosition(  30.0*M_PI/180.0);
  hubo->getDof("REP")->setPosition(-120.0*M_PI/180.0);


  hubo->getDof("LSY")->setPositionLowerLimit(-90.0*M_PI/180.0);
  hubo->getDof("LSY")->setPositionUpperLimit( 90.0*M_PI/180.0);
  hubo->getDof("LWY")->setPositionLowerLimit(-90.0*M_PI/180.0);
  hubo->getDof("LWY")->setPositionUpperLimit( 90.0*M_PI/180.0);

  hubo->getDof("RSY")->setPositionLowerLimit(-90.0*M_PI/180.0);
  hubo->getDof("RSY")->setPositionUpperLimit( 90.0*M_PI/180.0);
  hubo->getDof("RWY")->setPositionLowerLimit(-90.0*M_PI/180.0);
  hubo->getDof("RWY")->setPositionUpperLimit( 90.0*M_PI/180.0);
}

void setupEndEffectors(const SkeletonPtr& hubo)
{
  Eigen::VectorXd rootjoint_weights = Eigen::VectorXd::Ones(7);
  rootjoint_weights = 0.01*rootjoint_weights;

  double extra_error_clamp = 0.1;

  Eigen::Vector3d linearBounds =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());

  Eigen::Vector3d angularBounds =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());

  Eigen::Isometry3d tf_hand(Eigen::Isometry3d::Identity());
  tf_hand.translate(Eigen::Vector3d(0.0, 0.0, -0.09));

  EndEffector* l_hand = hubo->getBodyNode("Body_LWR")->
      createEndEffector("l_hand");
  l_hand->setDefaultRelativeTransform(tf_hand, true);

  dart::gui::osg::InteractiveFramePtr lh_target(new dart::gui::osg::InteractiveFrame(
                                           Frame::World(), "lh_target"));

  l_hand->getIK(true)->setTarget(lh_target);
  l_hand->getIK()->useWholeBody();

  l_hand->getIK()->setGradientMethod<HuboArmIK>("Body_LSP");

  l_hand->getIK()->getAnalytical()->setExtraDofUtilization(
        IK::Analytical::POST_ANALYTICAL);

  l_hand->getIK()->getAnalytical()->setExtraErrorLengthClamp(extra_error_clamp);

  l_hand->getIK()->getGradientMethod().setComponentWeights(rootjoint_weights);

  l_hand->getIK()->getErrorMethod().setLinearBounds(
        -linearBounds, linearBounds);

  l_hand->getIK()->getErrorMethod().setAngularBounds(
        -angularBounds, angularBounds);


  EndEffector* r_hand = hubo->getBodyNode("Body_RWR")->
      createEndEffector("r_hand");
  r_hand->setDefaultRelativeTransform(tf_hand, true);

  dart::gui::osg::InteractiveFramePtr rh_target(new dart::gui::osg::InteractiveFrame(
                                           Frame::World(), "rh_target"));

  r_hand->getIK(true)->setTarget(rh_target);
  r_hand->getIK()->useWholeBody();

  r_hand->getIK()->setGradientMethod<HuboArmIK>("Body_RSP");

  r_hand->getIK()->getAnalytical()->setExtraDofUtilization(
        IK::Analytical::POST_ANALYTICAL);

  r_hand->getIK()->getAnalytical()->setExtraErrorLengthClamp(extra_error_clamp);

  r_hand->getIK()->getGradientMethod().setComponentWeights(rootjoint_weights);

  r_hand->getIK()->getErrorMethod().setLinearBounds(
        -linearBounds, linearBounds);

  r_hand->getIK()->getErrorMethod().setAngularBounds(
        -angularBounds, angularBounds);


  dart::math::SupportGeometry foot_support;
  foot_support.push_back(Eigen::Vector3d(-0.08,  0.05, 0.0));
  foot_support.push_back(Eigen::Vector3d(-0.18,  0.05, 0.0));
  foot_support.push_back(Eigen::Vector3d(-0.18, -0.05, 0.0));
  foot_support.push_back(Eigen::Vector3d(-0.08, -0.05, 0.0));

  Eigen::Isometry3d tf_foot(Eigen::Isometry3d::Identity());
  double ground_dist = 0.01;
  tf_foot.translation() = Eigen::Vector3d(0.14, 0.0, -0.136+ground_dist);

//  linearBounds[2] = 1e-8;
  Eigen::Vector3d ground_offset = ground_dist * Eigen::Vector3d::UnitZ();

//  angularBounds[0] = 1e-8;
//  angularBounds[1] = 1e-8;

  EndEffector* l_foot = hubo->getBodyNode("Body_LAR")->
      createEndEffector("l_foot");
  l_foot->setDefaultRelativeTransform(tf_foot, true);

  dart::gui::osg::InteractiveFramePtr lf_target(new dart::gui::osg::InteractiveFrame(
                                           Frame::World(), "lf_target"));

  l_foot->getIK(true)->setTarget(lf_target);

  l_foot->getIK()->setHierarchyLevel(1);

  l_foot->getIK()->getErrorMethod().setLinearBounds(
        -linearBounds + ground_offset, linearBounds + ground_offset);
  l_foot->getIK()->getErrorMethod().setAngularBounds(
        -angularBounds, angularBounds);

  l_foot->getIK()->setGradientMethod<HuboLegIK>("Body_LHY");

  l_foot->getSupport(true)->setGeometry(foot_support);
  l_foot->getSupport()->setActive();


  EndEffector* r_foot = hubo->getBodyNode("Body_RAR")->
      createEndEffector("r_foot");
  r_foot->setDefaultRelativeTransform(tf_foot, true);

  dart::gui::osg::InteractiveFramePtr rf_target(new dart::gui::osg::InteractiveFrame(
                                           Frame::World(), "rf_target"));

  r_foot->getIK(true)->setTarget(rf_target);

  r_foot->getIK()->setHierarchyLevel(1);

  r_foot->getIK()->getErrorMethod().setLinearBounds(
        -linearBounds + ground_offset, linearBounds + ground_offset);
  r_foot->getIK()->getErrorMethod().setAngularBounds(
        -angularBounds, angularBounds);

  r_foot->getIK()->setGradientMethod<HuboLegIK>("Body_RHY");

  r_foot->getSupport(true)->setGeometry(foot_support);
  r_foot->getSupport()->setActive();


  dart::math::SupportGeometry peg_support;
  peg_support.push_back(Eigen::Vector3d::Zero());

  linearBounds = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  angularBounds = linearBounds;

  Eigen::Isometry3d tf_peg(Eigen::Isometry3d::Identity());
  tf_peg.translation() = Eigen::Vector3d(0.0, 0.0, 0.09);

  EndEffector* l_peg = hubo->getBodyNode("Body_LWP")->createEndEffector("l_peg");
  l_peg->setDefaultRelativeTransform(tf_peg, true);

  dart::gui::osg::InteractiveFramePtr lp_target(new dart::gui::osg::InteractiveFrame(
                                           Frame::World(), "lp_target"));

  l_peg->getIK(true)->setTarget(lp_target);

  l_peg->getIK()->setGradientMethod<HuboArmIK>("Body_LSP");

  l_peg->getIK()->getErrorMethod().setLinearBounds(
        -linearBounds, linearBounds);
  l_peg->getIK()->getErrorMethod().setAngularBounds(
        -angularBounds, angularBounds);

  l_peg->getSupport(true)->setGeometry(peg_support);

  EndEffector* r_peg = hubo->getBodyNode("Body_RWP")->createEndEffector("r_peg");
  r_peg->setDefaultRelativeTransform(tf_peg, true);

  dart::gui::osg::InteractiveFramePtr rp_target(new dart::gui::osg::InteractiveFrame(
                                           Frame::World(), "rp_target"));

  r_peg->getIK(true)->setTarget(rp_target);

  r_peg->getIK()->setGradientMethod<HuboArmIK>("Body_RSP");

  r_peg->getIK()->getErrorMethod().setLinearBounds(
        -linearBounds, linearBounds);
  r_peg->getIK()->getErrorMethod().setAngularBounds(
        -angularBounds, angularBounds);

  r_peg->getSupport(true)->setGeometry(peg_support);

  double heightChange = -r_foot->getWorldTransform().translation()[2]+ground_dist;
  hubo->getDof("rootJoint_pos_z")->setPosition(heightChange);

  l_foot->getIK()->getTarget()->setTransform(l_foot->getTransform());
  r_foot->getIK()->getTarget()->setTransform(r_foot->getTransform());
}

void enableDragAndDrops(dart::gui::osg::Viewer& viewer, const SkeletonPtr& hubo)
{
  // Turn on drag-and-drop for the whole Skeleton
  for(std::size_t i=0; i < hubo->getNumBodyNodes(); ++i)
    viewer.enableDragAndDrop(hubo->getBodyNode(i), false, false);

  for(std::size_t i=0; i < hubo->getNumEndEffectors(); ++i)
  {
    EndEffector* ee = hubo->getEndEffector(i);
    if(!ee->getIK())
      continue;

    // Check whether the target is an interactive frame, and add it if it is
    if(const auto& frame = std::dynamic_pointer_cast<dart::gui::osg::InteractiveFrame>(
         ee->getIK()->getTarget()))
      viewer.enableDragAndDrop(frame.get());
  }
}

void setupWholeBodySolver(const SkeletonPtr& hubo)
{
  std::shared_ptr<dart::optimizer::GradientDescentSolver> solver =
      std::dynamic_pointer_cast<dart::optimizer::GradientDescentSolver>(
        hubo->getIK(true)->getSolver());

  std::size_t nDofs = hubo->getNumDofs();

  double default_weight = 0.01;
  Eigen::VectorXd weights = default_weight * Eigen::VectorXd::Ones(nDofs);
  weights[2] = 0.0;
  weights[3] = 0.0;
  weights[4] = 0.0;

  Eigen::VectorXd lower_posture = Eigen::VectorXd::Constant(nDofs,
        -std::numeric_limits<double>::infinity());
  lower_posture[0] = -0.35;
  lower_posture[1] = -0.35;
  lower_posture[5] =  0.55;

  Eigen::VectorXd upper_posture = Eigen::VectorXd::Constant(nDofs,
         std::numeric_limits<double>::infinity());
  upper_posture[0] =  0.35;
  upper_posture[1] =  0.50;
  upper_posture[5] =  0.95;

//  std::shared_ptr<RelaxedPosture> objective = std::make_shared<RelaxedPosture>(
//        hubo->getPositions(), lower_posture, upper_posture, weights);

//  hubo->getIK()->setObjective(objective);

//  std::shared_ptr<dart::constraint::BalanceConstraint> balance =
//      std::make_shared<dart::constraint::BalanceConstraint>(hubo->getIK());
//  hubo->getIK()->getProblem()->addEqConstraint(balance);

//  balance->setErrorMethod(dart::constraint::BalanceConstraint::FROM_CENTROID);
//  balance->setBalanceMethod(dart::constraint::BalanceConstraint::SHIFT_SUPPORT);

  solver->setNumMaxIterations(5);
}

int main()
{
  dart::simulation::WorldPtr world(new dart::simulation::World);

  SkeletonPtr hubo = createHubo();
  setStartupConfiguration(hubo);
  setupEndEffectors(hubo);

  Eigen::VectorXd positions = hubo->getPositions();
  // We make a clone to test whether the cloned version behaves the exact same
  // as the original version.
//  hubo = hubo->clone("hubo_copy");
  hubo->setPositions(positions);

  world->addSkeleton(hubo);
//  world->addSkeleton(createGround());


//  dart::dynamics::SkeletonPtr env = Skeleton::create("scene");
//  dart::dynamics::BodyNode* bn = env->createJointAndBodyNodePair<
//      dart::dynamics::WeldJoint>().second;

//  const std::string path = "/home/grey/projects/posgraph/data/models/UnevenEnvObj/";
//  const std::string file = path+"UnevenEnv.obj";
////  const std::string path = "/home/grey/projects/posgraph/data/models/DoubleJumpEnvObj/";
////  const std::string file = path+"DoubleJumpEnv.obj";
//  dart::common::ResourceRetrieverPtr relative =
//      std::make_shared<dart::common::RelativeResourceRetriever>(path);
//  const aiScene* scene = dart::dynamics::MeshShape::loadMesh(file, relative);
//  dart::dynamics::MeshShapePtr mesh = std::make_shared<MeshShape>(
//        0.0254*Eigen::Vector3d::Ones(), scene, file);
//  bn->createShapeNodeWith<dart::dynamics::VisualAspect,
//                          dart::dynamics::CollisionAspect>(mesh);
//  world->addSkeleton(env);


//  limbo(world);


  setupWholeBodySolver(hubo);

  ::osg::ref_ptr<TeleoperationWorld> node = new TeleoperationWorld(world, hubo);

  Viewer viewer;
  viewer.getCamera()->setClearColor(osg::Vec4(1.0, 1.0, 1.0, 1.0));
  viewer.allowSimulation(false);
  viewer.addWorldNode(node);

  enableDragAndDrops(viewer, hubo);

  viewer.addEventHandler(new InputHandler(&viewer, node, hubo, world));

//  double display_elevation = 0.05;
//  viewer.addAttachment(new dart::gui::osg::SupportPolygonVisual(
//                         hubo, display_elevation));

  std::cout << viewer.getInstructions() << std::endl;

  std::cout << "Alt + Click:   Try to translate a body without changing its orientation\n"
            << "Ctrl + Click:  Try to rotate a body without changing its translation\n"
            << "Shift + Click: Move a body using only its parent joint\n"
            << "1 -> 6:        Toggle the interactive target of an EndEffector\n"
            << "W A S D:       Move the robot around the scene\n"
            << "Q E:           Rotate the robot counter-clockwise and clockwise\n"
            << "F Z:           Shift the robot's elevation up and down\n"
            << "X C:           Toggle support on the left and right foot\n"
            << "R:             Optimize the robot's posture\n"
            << "T:             Reset the robot to its relaxed posture\n\n"
            << "  The green polygon is the support polygon of the robot, and the blue/red ball is\n"
            << "  the robot's center of mass. The green ball is the centroid of the polygon.\n\n"
            << "Note that this is purely kinematic. Physical simulation is not allowed in this app.\n"
            << std::endl;

  // Set up the window
  viewer.setUpViewInWindow(0, 0, 1280, 960);

  // Set up the default viewing position
  viewer.getCameraManipulator()->setHomePosition(::osg::Vec3( 5.34,  3.00, 1.91),
                                                 ::osg::Vec3( 0.00,  0.00, 0.50),
                                                 ::osg::Vec3(-0.20, -0.08, 0.98));

  // Reset the camera manipulator so that it starts in the new viewing position
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  viewer.run();
}

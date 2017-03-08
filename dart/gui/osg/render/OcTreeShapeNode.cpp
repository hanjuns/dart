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

#include <osg/Geode>
#include <osg/ShapeDrawable>

#include "dart/gui/osg/render/OcTreeShapeNode.hpp"
#include "dart/gui/osg/Utils.hpp"

#include "dart/dynamics/OcTreeShape.hpp"
#include "dart/dynamics/ShapeFrame.hpp"

namespace dart {
namespace gui {
namespace osg {
namespace render {

//==============================================================================
class OcTreeShapeGeode : public ShapeNode, public ::osg::Geode
{
public:

  OcTreeShapeGeode(const std::shared_ptr<dart::dynamics::OcTreeShape>& shape,
                   ShapeFrameNode* parentShapeFrame);

  void refresh();
  void extractData(bool firstTime);

protected:

  virtual ~OcTreeShapeGeode() { }

  std::shared_ptr<dart::dynamics::OcTreeShape> mOcTreeShape;

};

//==============================================================================
OcTreeShapeNode::OcTreeShapeNode(
    std::shared_ptr<dynamics::OcTreeShape> shape,
    ShapeFrameNode *parent)
  : ShapeNode(shape, parent, this),
    mOcTreeShape(shape),
    mGeode(nullptr)
{
  extractData(true);
  setNodeMask(mVisualAspect->isHidden()? 0x0 : ~0x0);
}

//==============================================================================
void OcTreeShapeNode::refresh()
{
  mUtilized = true;

  setNodeMask(mVisualAspect->isHidden()? 0x0 : ~0x0);

  if(mShape->getDataVariance() == dart::dynamics::Shape::STATIC)
    return;

  extractData(false);
}

//==============================================================================
void OcTreeShapeNode::extractData(bool /*firstTime*/)
{
  if(nullptr == mGeode)
  {
    mGeode = new OcTreeShapeGeode(mOcTreeShape, mParentShapeFrameNode);
    addChild(mGeode);
    return;
  }

  mGeode->refresh();
}

//==============================================================================
OcTreeShapeNode::~OcTreeShapeNode()
{
  // Do nothing
}

//==============================================================================
OcTreeShapeGeode::OcTreeShapeGeode(
    const std::shared_ptr<dynamics::OcTreeShape>& shape,
    ShapeFrameNode* parentShapeFrame)
  : ShapeNode(shape, parentShapeFrame, this),
    mOcTreeShape(shape)
{
  getOrCreateStateSet()->setMode(GL_BLEND, ::osg::StateAttribute::ON);
  getOrCreateStateSet()->setRenderingHint(::osg::StateSet::TRANSPARENT_BIN);
  extractData(true);
}

//==============================================================================
void OcTreeShapeGeode::refresh()
{
  mUtilized = true;
  extractData(false);
}

//==============================================================================
void OcTreeShapeGeode::extractData(bool firstTime)
{
  int dv = mOcTreeShape->getDataVariance();
  const bool resetBoxes =
         ( (dart::dynamics::Shape::DYNAMIC_ELEMENTS & dv) != 0)
      || ( (dart::dynamics::Shape::DYNAMIC_VERTICES & dv) != 0)
      || ( (dart::dynamics::Shape::DYNAMIC_PRIMITIVE & dv) != 0);

  const ::osg::Vec4 color = eigToOsgVec4(mVisualAspect->getRGBA());

  if(firstTime || resetBoxes)
  {
    removeDrawables(0, getNumDrawables());

    std::vector<Eigen::Vector4d> boxes;
//    std::vector<Eigen::Vector3d> boxes;
    boxes.reserve(mOcTreeShape->size()/2);
    octomap::OcTree::iterator it =
            mOcTreeShape->begin(mOcTreeShape->getTreeDepth());
    octomap::OcTree::iterator end = mOcTreeShape->end();
    for(; it != end; ++it)
    {
      if(mOcTreeShape->isNodeOccupied(*it))
//      if(it->getLogOdds() > 0)
      {
        Eigen::Vector4d c(it.getX(), it.getY(), it.getZ(), it.getSize());
//        Eigen::Vector4d c(it.getX(), it.getY(), it.getZ());
        boxes.push_back(c);
      }
    }

//    const double resolution = mOcTreeShape->getResolution();

//    for(const Eigen::Vector3d& c : boxes)
    for(const Eigen::Vector4d& c : boxes)
    {
      ::osg::ref_ptr< ::osg::ShapeDrawable> drawable =
          new ::osg::ShapeDrawable(
//            new ::osg::Box(::osg::Vec3(c[0], c[1], c[2]), resolution));
            new ::osg::Box(::osg::Vec3(c[0], c[1], c[2]), c[3]));
      drawable->setColor(color);

      addDrawable(drawable);
    }
  }

  const bool resetColor = dart::dynamics::Shape::DYNAMIC_COLOR == dv;
  if(firstTime || resetColor)
  {
    const size_t numChildren = getNumDrawables();
    for(size_t i=0; i < numChildren; ++i)
    {
      ::osg::ShapeDrawable* drawable =
          dynamic_cast< ::osg::ShapeDrawable*>(getDrawable(i));
      drawable->setColor(color);
    }
  }
}

} // namespace render
} // namespace osg
} // namespace gui
} // namespace dart

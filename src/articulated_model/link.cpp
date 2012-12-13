/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Dalibor Matura, Jia Pan */

#include "fcl/articulated_model/link.h"

#include "fcl/articulated_model/joint.h"

namespace fcl
{

Link::Link(const std::string& name) : name_(name)
{}

const std::string& Link::getName() const
{
  return name_;
}

void Link::setName(const std::string& name)
{
  name_ = name;
}

void Link::addChildJoint(boost::shared_ptr<Joint> joint)
{
  children_joints_.push_back(joint);
}

void Link::setParentJoint(boost::shared_ptr<Joint> joint)
{
  parent_joint_ = joint;
}

void Link::addGeometry(boost::shared_ptr<CollisionGeometry> geometry)
{
  geometries_.push_back(geometry);
}

std::size_t Link::getNumChildJoints() const
{
  return children_joints_.size();
}

std::size_t Link::getNumGeometries() const
{
  return geometries_.size();
}

std::vector<boost::shared_ptr<Joint> > Link::getChildJoints() const
{
  return children_joints_;
}

boost::shared_ptr<Joint> Link::getParentJoint() const
{
	return parent_joint_;
}

bool Link::operator==(const Link& link) const
{
	return name_ == link.name_ &&
		geometries_ == link.geometries_ &&
		children_joints_ == link.children_joints_ &&
		parent_joint_ == parent_joint_;
}

bool Link::operator!=(const Link& link) const
{
	return !((*this) == link);
}

}

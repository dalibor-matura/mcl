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

#include "fcl/articulated_model/joint.h"
#include "fcl/articulated_model/link.h"
#include "fcl/articulated_model/joint_config.h"

#include <boost/assert.hpp>

namespace fcl
{

#ifndef NDEBUG

bool isNormalized(const Vec3f& vec)
{
	return vec.length() == 1;
}

#endif

Joint::Joint(const boost::shared_ptr<Link>& link_parent, const boost::shared_ptr<Link>& link_child,
			 const Transform3f& transform_to_parent,
			 const std::string& name) :
  link_parent_(link_parent), link_child_(link_child),
  transform_to_parent_(transform_to_parent),
  type_(JT_UNKNOWN),
  name_(name)
{}

Joint::Joint(const std::string& name) :
  name_(name)
{
}

const std::string& Joint::getName() const
{
  return name_;
}

void Joint::setName(const std::string& name)
{
  name_ = name;
}

boost::shared_ptr<Link> Joint::getParentLink() const
{
  return link_parent_.lock();
}

boost::shared_ptr<Link> Joint::getChildLink() const
{
  return link_child_.lock();
}

void Joint::setParentLink(const boost::shared_ptr<Link>& link)
{
  link_parent_ = link;
}

void Joint::setChildLink(const boost::shared_ptr<Link>& link)
{
  link_child_ = link;
}

JointType Joint::getJointType() const
{
  return type_;
}

const Transform3f& Joint::getTransformToParent() const
{
  return transform_to_parent_;
}

void Joint::setTransformToParent(const Transform3f& t)
{
  transform_to_parent_ = t;
}

const Vec3f& Joint::getAxis() const
{
  static Vec3f vec;

  return vec;
}

void Joint::setAxis(const Vec3f& axis)
{
	BOOST_ASSERT(isNormalized(axis) && "Axis is not normalized.");

	axis_ = axis;
}

bool Joint::operator==(const Joint& joint) const
{
	return name_ == joint.name_ &&
		type_ == joint.type_;
		//transform_to_parent_ == joint.transform_to_parent_ &&
		//link_parent_.lock() == joint.link_parent_.lock() &&
		//link_child_.lock() == joint.link_child_.lock();
}

bool Joint::operator!=(const Joint& joint) const
{
	return !((*this) == joint);
}

PrismaticJoint::PrismaticJoint(const boost::shared_ptr<Link>& link_parent, const boost::shared_ptr<Link>& link_child,
							   const Transform3f& transform_to_parent,
							   const std::string& name,
							   const Vec3f& axis) :
  Joint(link_parent, link_child, transform_to_parent, name)
{
  BOOST_ASSERT(isNormalized(axis) && "Axis is not normalized.");

  setAxis(axis);

  init();
}

void PrismaticJoint::init()
{
  type_ = JT_PRISMATIC;
}


PrismaticJoint::PrismaticJoint(const std::string& name) :
	Joint(name)
{
  init();
}

const Vec3f& PrismaticJoint::getAxis() const
{
  return axis_;
}

std::size_t PrismaticJoint::getNumDofs() const
{
  return 1;
}

Transform3f PrismaticJoint::getLocalTransform(const JointConfig& cfg) const
{
  const Quaternion3f& quat = transform_to_parent_.getQuatRotation();
  const Vec3f& transl = transform_to_parent_.getTranslation();
  return Transform3f(quat, quat.transform(axis_ * cfg[0]) + transl);
}


RevoluteJoint::RevoluteJoint(const boost::shared_ptr<Link>& link_parent, const boost::shared_ptr<Link>& link_child,
							 const Transform3f& transform_to_parent,
							 const std::string& name,
							 const Vec3f& axis) :
  Joint(link_parent, link_child, transform_to_parent, name)
{
  BOOST_ASSERT(isNormalized(axis) && "Axis is not normalized.");

  setAxis(axis); 

  init();
}

void RevoluteJoint::init()
{
  type_ = JT_REVOLUTE;
}

RevoluteJoint::RevoluteJoint(const std::string& name) :
Joint(name)
{
  init();
}

const Vec3f& RevoluteJoint::getAxis() const
{
  return axis_;
}

std::size_t RevoluteJoint::getNumDofs() const
{
  return 1;
}

Transform3f RevoluteJoint::getLocalTransform(const JointConfig& cfg) const
{
  Quaternion3f quat;
  quat.fromAxisAngle(axis_, cfg[0]);
  return Transform3f(transform_to_parent_.getQuatRotation() * quat, transform_to_parent_.getTranslation());
}


BallEulerJoint::BallEulerJoint(const boost::shared_ptr<Link>& link_parent, const boost::shared_ptr<Link>& link_child,
							   const Transform3f& transform_to_parent,
							   const std::string& name) :
  Joint(link_parent, link_child, transform_to_parent, name)
{}

std::size_t BallEulerJoint::getNumDofs() const
{
  return 3;
}

Transform3f BallEulerJoint::getLocalTransform(const JointConfig& cfg) const
{
  Matrix3f rot;
  rot.setEulerYPR(cfg[0], cfg[1], cfg[2]);
  return transform_to_parent_ * Transform3f(rot);
}

}

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

#include "fcl/articulated_model/model.h"
#include "fcl/articulated_model/model_config.h"
#include <boost/assert.hpp>

namespace fcl
{


boost::shared_ptr<Link> Model::getRoot() const
{
  return root_link_;
}

boost::shared_ptr<Link> Model::getLink(const std::string& name) const
{
  boost::shared_ptr<Link> ptr;
  std::map<std::string, boost::shared_ptr<Link> >::const_iterator it = links_.find(name);
  if(it == links_.end())
    ptr.reset();
  else
    ptr = it->second;
  return ptr;
}

boost::shared_ptr<Joint> Model::getJoint(const std::string& name) const
{
  boost::shared_ptr<Joint> ptr;
  std::map<std::string, boost::shared_ptr<Joint> >::const_iterator it = joints_.find(name);
  if(it == joints_.end())
    ptr.reset();
  else
    ptr = it->second;
  return ptr;
}

InterpolationType Model::getJointInterpolationType(const std::string& name) const
{
  InterpolationType interpolation_type;

  std::map<std::string, InterpolationType>::const_iterator it = joints_interpolation_types_.find(name);
  if(it != joints_interpolation_types_.end())
  {
    interpolation_type = it->second;; 
  }

  return interpolation_type;
}

std::map<std::string, boost::shared_ptr<Joint> > Model::getJointsMap() const
{
  return joints_;
}

const std::string& Model::getName() const
{
  return name_;
}

std::vector<boost::shared_ptr<const Link> > Model::getLinks() const
{
  std::vector<boost::shared_ptr<const Link> > links;
  for(std::map<std::string, boost::shared_ptr<Link> >::const_iterator it = links_.begin(); it != links_.end(); ++it)
  {
    links.push_back(it->second);
  }

  return links;
}

std::vector<boost::shared_ptr<const Joint> > Model::getJoints() const
{
    std::vector<boost::shared_ptr<const Joint> > joints;
    for(std::map<std::string, boost::shared_ptr<Joint> >::const_iterator it = joints_.begin(); it != joints_.end(); ++it)
    {
        joints.push_back(it->second);
    }

    return joints;
}

std::size_t Model::getNumLinks() const
{
  return links_.size();
}

std::size_t Model::getNumJoints() const
{
  return joints_.size();
}

std::size_t Model::getNumDofs() const
{
  std::size_t dof = 0;
  for(std::map<std::string, boost::shared_ptr<Joint> >::const_iterator it = joints_.begin(); it != joints_.end(); ++it)
  {
    dof += it->second->getNumDofs();
  }

  return dof;
}

void Model::addLink(const boost::shared_ptr<Link>& link)
{
  links_[link->getName()] = link;
}

void Model::addJoint(const boost::shared_ptr<Joint>& joint,const InterpolationType& interpolation_type)
{
  joints_[joint->getName()] = joint;
  joints_interpolation_types_[joint->getName()] = interpolation_type; 
}

void Model::initRoot(const std::map<std::string, std::string>& link_parent_tree)
{
  root_link_.reset();

  /// find the links that have no parent in the tree
  for(std::map<std::string, boost::shared_ptr<Link> >::const_iterator it = links_.begin(); it != links_.end(); ++it)
  {
    std::map<std::string, std::string>::const_iterator parent = link_parent_tree.find(it->first);
    if(parent == link_parent_tree.end())
    {
      if(!root_link_)
      {
        root_link_ = getLink(it->first);
      }
      else
      {
        throw ModelParseError("Two root links found: [" + root_link_->getName() + "] and [" + it->first + "]");
      }
    }
  }

  if(!root_link_)
    throw ModelParseError("No root link found.");
}

void Model::initTree()
{
  std::map<std::string, std::string> link_parent_tree;

  initTree(link_parent_tree);
  initRoot(link_parent_tree);	

  constructJointParentTree(getRoot() );
}

void Model::initTree(std::map<std::string, std::string>& link_parent_tree)
{
  for(std::map<std::string, boost::shared_ptr<Joint> >::iterator it = joints_.begin(); it != joints_.end(); ++it)
  {
    std::string parent_link_name = it->second->getParentLink()->getName();
    std::string child_link_name = it->second->getChildLink()->getName();

    it->second->getParentLink()->addChildJoint(it->second);
    it->second->getChildLink()->setParentJoint(it->second);

    link_parent_tree[child_link_name] = parent_link_name;
  }
}

void Model::constructJointParentTree(boost::shared_ptr<const Link> link)
{	
  std::vector<boost::shared_ptr<Joint> > child_joints = link->getChildJoints();
  boost::shared_ptr<Joint> parent_joint = link->getParentJoint();
  std::vector<boost::shared_ptr<Joint> >::const_iterator it;

  for (it = child_joints.begin(); it != child_joints.end(); ++it)
  {
    boost::shared_ptr<const Joint> joint = (*it);
    boost::shared_ptr<const Link> child_link = joint->getChildLink();

    if (parent_joint.use_count() != 0)
    {
      joint_parents_[joint->getName()] = parent_joint;
    }		

    if (child_link.use_count() != 0)
    {
      constructJointParentTree(child_link);
    }
  }
}

boost::shared_ptr<Joint> Model::getJointParent(const boost::shared_ptr<const Joint>& joint) const
{
  boost::shared_ptr<Joint> parent;

  std::map<std::string, boost::shared_ptr<Joint> >::const_iterator it = joint_parents_.find(joint->getName() );

  if (it != joint_parents_.end() )
  {
    parent = it->second;
  }

  return parent;
}

}
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

#ifndef FCL_ARTICULATED_MODEL_LINK_H
#define FCL_ARTICULATED_MODEL_LINK_H

#include "fcl/data_types.h"
#include "fcl/collision_object.h"

#include <boost/shared_ptr.hpp>
#include <vector>

namespace fcl
{

class Joint;

class Link
{
public:
  Link(const std::string& name);

  const std::string& getName() const;
  
  void setName(const std::string& name);
  
  void addChildJoint(boost::shared_ptr<Joint> joint);
  
  void setParentJoint(boost::shared_ptr<Joint> joint);
  
  void addObject(boost::shared_ptr<CollisionObject> object);
  
  std::size_t getNumChildJoints() const;
  
  std::size_t getNumObjects() const;

  std::vector<boost::shared_ptr<Joint> > getChildJoints() const;

  boost::shared_ptr<Joint> getParentJoint() const;

  bool operator==(const Link& link) const;
  bool operator!=(const Link& link) const;
  
protected:
  std::string name_;

  std::vector<boost::shared_ptr<CollisionObject> > objects_;

  std::vector<boost::shared_ptr<Joint> > children_joints_;

  boost::shared_ptr<Joint> parent_joint_;
};

}

#endif
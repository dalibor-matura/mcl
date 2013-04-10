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

/** \author Jia Pan */

#include "fcl/collision_data.h"

namespace fcl
{

bool CollisionRequest::isSatisfied(const CollisionResult& result) const
{
  return (!enable_cost) && result.isCollision() && (num_max_contacts <= result.numContacts());
}

void CollisionRequest::assign(const CollisionRequest& request)
{
	operator=(request);
}


bool DistanceRequest::isSatisfied(const DistanceResult& result) const
{
  return (result.min_distance <= 0);
}


bool CollisionResult::isCollision() const
{
	return contacts_.size() > 0;
}

size_t CollisionResult::numContacts() const
{
	return contacts_.size();
}

size_t CollisionResult::numCostSources() const
{
	return cost_sources_.size();
}

const Contact& CollisionResult::getContact(size_t i) const
{
	if(i < contacts_.size()) 
		return contacts_[i];
	else
		return contacts_.back();
}

void CollisionResult::getContacts(std::vector<Contact>& contacts)
{
	contacts.resize(contacts_.size());
	std::copy(contacts_.begin(), contacts_.end(), contacts.begin());
}

void CollisionResult::getCostSources(std::vector<CostSource>& cost_sources)
{
	cost_sources.resize(cost_sources_.size());
	std::copy(cost_sources_.begin(), cost_sources_.end(), cost_sources.begin());
}

void CollisionResult::clear()
{
	contacts_.clear();
	cost_sources_.clear();
}


void ContinuousCollisionResult::clear()
{
	CollisionResult::clear();

	time_of_contact_ = 0.0;
}

}
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

#include "fcl/ccd/conservative_advancement.h"

#include <boost/make_shared.hpp>

namespace fcl
{

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
int conservativeAdvancement(const CollisionGeometry* o1,
							const MotionBase* motion1,
							const CollisionGeometry* o2,
							const MotionBase* motion2,
							const CollisionRequest& request,
							CollisionResult& result,
							FCL_REAL& toc)
{
	typedef ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode> ConservativeAdvancementType;

	boost::shared_ptr<ConservativeAdvancementType> advancement = 
		boost::make_shared<ConservativeAdvancementType>(o1, motion1, o2, motion2);

	ContinuousCollisionRequest continuous_request;
	ContinuousCollisionResult continuous_result;

	continuous_request.assign(request);
	advancement->collide(continuous_request, continuous_result);
	result = continuous_result;

	toc = continuous_result.getTimeOfContact();

	return result.numContacts();
}


template
int conservativeAdvancement<RSS, MeshConservativeAdvancementTraversalNodeRSS, MeshCollisionTraversalNodeRSS>(const CollisionGeometry* o1,
																											 const MotionBase* motion1,
																											 const CollisionGeometry* o2,
																											 const MotionBase* motion2,
																											 const CollisionRequest& request,
																											 CollisionResult& result,
																											 FCL_REAL& toc);

template int conservativeAdvancement<OBBRSS, MeshConservativeAdvancementTraversalNodeOBBRSS, MeshCollisionTraversalNodeOBBRSS>(const CollisionGeometry* o1,
																															   const MotionBase* motion1,
																															   const CollisionGeometry* o2,
																															   const MotionBase* motion2,
																															   const CollisionRequest& request,
																															   CollisionResult& result,
																															   FCL_REAL& toc);

}

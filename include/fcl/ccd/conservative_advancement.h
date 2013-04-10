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

#ifndef FCL_CONSERVATIVE_ADVANCEMENT_H
#define FCL_CONSERVATIVE_ADVANCEMENT_H

#include "fcl/collision_object.h"
#include "fcl/collision_data.h"
#include "fcl/ccd/motion_base.h"
#include "fcl/ccd/motion.h"
#include "fcl/collision_node.h"
#include "fcl/traversal/traversal_node_bvhs.h"
#include "fcl/traversal/traversal_node_setup.h"
#include "fcl/traversal/traversal_recurse.h"
#include "fcl/collision.h"

namespace fcl
{

class CollisionGeometry;
class ContinuousCollisionObject;
class MotionBase;

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
class ConservativeAdvancement
{
public:
	ConservativeAdvancement(const ContinuousCollisionObject* o1, const ContinuousCollisionObject* o2);
	ConservativeAdvancement(const CollisionGeometry* geometry1, const MotionBase* motion1, 
		const CollisionGeometry* geometry2, const MotionBase* motion2);

	// returns number of contacts
	int collide(const ContinuousCollisionRequest& request, ContinuousCollisionResult& result);

private:
	void setRequest(const ContinuousCollisionRequest& request);
	void setResult(ContinuousCollisionResult& result);

	bool areInvariantsSatisfied() const;
	bool areRequestInvariantsSatisfied() const;
	bool areGeometriesInvariantsSatisfied() const;

	bool isObjectTypeBVH(const CollisionGeometry* geometry) const;
	bool isNodeTypeRSS(const CollisionGeometry* geometry) const;

	void initCollisionDetection();
	void setStartConfigurations();

	void integrateTimeToMotions(FCL_REAL time);

	void clearResult();

	void performDiscreteCollision();
	void initDiscreteCollision(CollisionNode& node);
	Transform3f getCurrentTransform(const MotionBase* motion);
	
	bool isCollisionFree() const;

	void performContinuousCollision();
	void initContinuousCollision(ConservativeAdvancementNode& node);
	void performConservativeSteps(ConservativeAdvancementNode& node);
	bool stepsRemains(ConservativeAdvancementNode& node) const;
	void performOneConservativeStep(ConservativeAdvancementNode& node);
	void initDistanceRecurse(ConservativeAdvancementNode& node);

	bool isColliding(ConservativeAdvancementNode& node) const;	

	void processRecurseResult(ConservativeAdvancementNode& node);

	void addContact(ConservativeAdvancementNode& node);
	void doTimeStep(ConservativeAdvancementNode& node);

private:
	const CollisionGeometry* geometry1_;
	const CollisionGeometry* geometry2_;

	const MotionBase* motion1_;	
	const MotionBase* motion2_;

	const BVHModel<BV>* model1_;
	const BVHModel<BV>* model2_;

	const ContinuousCollisionRequest* request_;
	ContinuousCollisionResult* result_;
};

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::ConservativeAdvancement(
	const ContinuousCollisionObject* o1, const ContinuousCollisionObject* o2)
	:
	geometry1_(o1->getCollisionGeometry() ),
	geometry2_(o2->getCollisionGeometry() ),
	motion1_(o1->getMotion() ),
	motion2_(o2->getMotion() ),
	model1_(static_cast<const BVHModel<BV>*>(o1->getCollisionGeometry() ) ),
	model2_(static_cast<const BVHModel<BV>*>(o2->getCollisionGeometry() ) )
{
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::ConservativeAdvancement(
	const CollisionGeometry* geometry1, const MotionBase* motion1, 
	const CollisionGeometry* geometry2, const MotionBase* motion2)
	:
	geometry1_(geometry1),
	geometry2_(geometry2),
	motion1_(motion1),
	motion2_(motion2),
	model1_(static_cast<const BVHModel<BV>*>(geometry1) ),
	model2_(static_cast<const BVHModel<BV>*>(geometry2) )
{
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
int ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::collide(
	const ContinuousCollisionRequest& request, ContinuousCollisionResult& result)
{
	setRequest(request);
	setResult(result);

	if (!areInvariantsSatisfied() )
	{
		return 0;
	}

	initCollisionDetection();
	performDiscreteCollision();	

	if (isCollisionFree() )
	{
		performContinuousCollision();
	}

	return result_->numContacts();
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
void ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::setRequest(
	const ContinuousCollisionRequest& request)
{
	request_ = &request;
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
void ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::setResult(
	ContinuousCollisionResult& result)
{
	result_ = &result;
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
bool ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::areInvariantsSatisfied() const
{
	return areRequestInvariantsSatisfied() && areGeometriesInvariantsSatisfied();
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
bool ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::areRequestInvariantsSatisfied() const
{
	if (request_->num_max_contacts > 0)
	{
		return true;
	}
	else
	{
		std::cerr << "Warning: should stop early as num_max_contact is " << request_->num_max_contacts << " !" << std::endl;
		return false;
	}
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
bool ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::areGeometriesInvariantsSatisfied() const
{
	return 
		isObjectTypeBVH(geometry1_) && 
		isObjectTypeBVH(geometry2_) &&
		isNodeTypeRSS(geometry1_) && 
		isNodeTypeRSS(geometry2_);
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
bool ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::isObjectTypeBVH(const CollisionGeometry* geometry) const
{
	const OBJECT_TYPE object_type = geometry->getObjectType();

	return object_type == OT_BVH;
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
bool ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::isNodeTypeRSS(const CollisionGeometry* geometry) const
{
	const NODE_TYPE node_type = geometry->getNodeType();

	return node_type == BV_RSS;
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
void ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::initCollisionDetection()
{
	setStartConfigurations();
	clearResult();
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
void ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::setStartConfigurations()
{
	integrateTimeToMotions(request_->start_time);
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
void ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::integrateTimeToMotions(FCL_REAL time)
{
	motion1_->integrate(time);
	motion2_->integrate(time);
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
void ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::clearResult()
{
	result_->clear();
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
void ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::performDiscreteCollision()
{
	CollisionNode node;
	initDiscreteCollision(node);

	::fcl::collide(&node);
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
void ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::initDiscreteCollision(CollisionNode& node)
{	
	bool succeded = initialize(
		node,
		*model1_, getCurrentTransform(motion1_),
		*model2_, getCurrentTransform(motion2_),
		*request_,
		*result_
	);

	if (!succeded)
	{
		std::cout << "initialize error" << std::endl;
	}

	node.enable_statistics = false;
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
Transform3f ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::getCurrentTransform(const MotionBase* motion)
{	
	Transform3f transformation;
	motion->getCurrentTransform(transformation);

	return transformation;
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
bool ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::isCollisionFree() const
{
	return (result_->numContacts() == 0);
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
void ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::performContinuousCollision()
{
	ConservativeAdvancementNode node;
	initContinuousCollision(node);

	performConservativeSteps(node);	

	result_->setTimeOfContact(node.toc);
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
void ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::initContinuousCollision(ConservativeAdvancementNode& node)
{
	initialize(
		node, 
		*model1_, getCurrentTransform(motion1_),
		*model2_, getCurrentTransform(motion2_)
		);

	node.motion1 = motion1_;
	node.motion2 = motion2_;
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
void ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::performConservativeSteps(ConservativeAdvancementNode& node)
{
	while (isCollisionFree() && stepsRemains(node) )
	{
		performOneConservativeStep(node);
	}
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
bool ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::stepsRemains(ConservativeAdvancementNode& node) const
{
	return (node.toc < request_->end_time);
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
void ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::performOneConservativeStep(ConservativeAdvancementNode& node)
{
	initDistanceRecurse(node);
	distanceRecurse(&node, 0, 0, NULL);
	processRecurseResult(node);	
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
void ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::initDistanceRecurse(ConservativeAdvancementNode& node)
{
	Matrix3f R1_t, R2_t;
	Vec3f T1_t, T2_t;

	node.motion1->getCurrentTransform(R1_t, T1_t);
	node.motion2->getCurrentTransform(R2_t, T2_t);

	relativeTransform(R1_t, T1_t, R2_t, T2_t, node.R, node.T);

	node.delta_t = request_->end_time;
	node.min_distance = std::numeric_limits<FCL_REAL>::max();
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
bool ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::isColliding(ConservativeAdvancementNode& node) const
{
	return (node.delta_t <= node.t_err);
}


template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
void ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::processRecurseResult(ConservativeAdvancementNode& node)
{
	if(isColliding(node) )
	{
		addContact(node);
	}
	else
	{
		doTimeStep(node);		
	}	
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
void ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::addContact(ConservativeAdvancementNode& node)
{
	result_->addContact(Contact(model1_, model2_, node.last_tri_id1, node.last_tri_id2) );
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
void ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::doTimeStep(ConservativeAdvancementNode& node)
{
	node.toc += node.delta_t;
	if(node.toc > request_->end_time)
	{
		node.toc = request_->end_time;
	}
	else
	{
		integrateTimeToMotions(node.toc);
	}
}


template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
int conservativeAdvancement(const CollisionGeometry* o1,
	const MotionBase* motion1,
	const CollisionGeometry* o2,
	const MotionBase* motion2,
	const CollisionRequest& request,
	CollisionResult& result,
	FCL_REAL& toc);

}
#endif

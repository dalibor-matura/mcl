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

#include <vector>
#include <utility>

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

	bool collideBoolean(const ContinuousCollisionRequest& request, ContinuousCollisionResult& result);

	int discreteDetectionInTime(FCL_REAL time, CollisionResult& result);

private:
	void setRequest(const ContinuousCollisionRequest& request);
	void setResult(ContinuousCollisionResult& result);

	bool areInvariantsSatisfied() const;
	bool areRequestInvariantsSatisfied() const;
	bool areGeometriesInvariantsSatisfied() const;

	bool isObjectTypeBVH(const CollisionGeometry* geometry) const;
	bool isNodeTypeRSS(const CollisionGeometry* geometry) const;

	void initCollisionDetection();

	void integrateTimeToMotions(FCL_REAL start_time, FCL_REAL end_time = -1.0);

	void clearResult();

	void performDiscreteCollision(FCL_REAL time);
	void initDiscreteCollision(FCL_REAL time);
	void initDiscreteCollisionNode(CollisionNode& node);
	Transform3f getCurrentTransform(const MotionBase* motion);
	
	bool isCollisionFree() const;

	void performContinuousCollision();
	void initContinuousCollision(ConservativeAdvancementNode& node);
	void performConservativeSteps(ConservativeAdvancementNode& node);
	bool stepsRemains(ConservativeAdvancementNode& node) const;
	void performOneConservativeStep(ConservativeAdvancementNode& node);
	void initDistanceRecurse(ConservativeAdvancementNode& node);
	void distanceRecurse(ConservativeAdvancementNode& node) const;

	bool isColliding(ConservativeAdvancementNode& node) const;	

	void processRecurseResult(ConservativeAdvancementNode& node);

	void addContact(ConservativeAdvancementNode& node);
	void doTimeStep(ConservativeAdvancementNode& node);

	bool performBooleanContinuousCollision();

	void calculateVelocityBound();
	FCL_REAL getMaxRadius(const BVHModel<BV>* model, const MotionBase* motion);

	FCL_REAL getVelocityBound() const;

	FCL_REAL getLeftTimeStep(ConservativeAdvancementNode& node,
		const FCL_REAL& start_time, const FCL_REAL& end_time);
	FCL_REAL getRightTimeStep(ConservativeAdvancementNode& node,
		const FCL_REAL& start_time, const FCL_REAL& end_time);

	FCL_REAL getTimeStep(ConservativeAdvancementNode& node) const;

	FCL_REAL getMinDistance(ConservativeAdvancementNode& node) const;

	FCL_REAL moveForward();
	void booleanCoolisionDetectionStep();

private:
	const CollisionGeometry* geometry1_;
	const CollisionGeometry* geometry2_;

	const MotionBase* motion1_;	
	const MotionBase* motion2_;

	const BVHModel<BV>* model1_;
	const BVHModel<BV>* model2_;

	const ContinuousCollisionRequest* request_;
	ContinuousCollisionResult* result_;	

	DistanceRequest distance_request_;
	DistanceResult distance_result_;

	FCL_REAL velocity_bound_;
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
	model2_(static_cast<const BVHModel<BV>*>(o2->getCollisionGeometry() ) ),
	velocity_bound_(0.0)
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
	model2_(static_cast<const BVHModel<BV>*>(geometry2) ),
	velocity_bound_(0.0)
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

	performDiscreteCollision(request_->start_time);	

	if (isCollisionFree() )
	{
		performContinuousCollision();
	}

	return static_cast<int>(result_->numContacts() );
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
int ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::discreteDetectionInTime(
	FCL_REAL time, CollisionResult& result)
{
	ContinuousCollisionResult continuous_result;	
	setResult(continuous_result);

	ContinuousCollisionRequest continuous_request;
	setRequest(continuous_request);

	performDiscreteCollision(time);	

	result = *result_;

	return static_cast<int>(result_->numContacts() );
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
	clearResult();
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
void ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::clearResult()
{
	result_->clear();
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
void ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::performDiscreteCollision
	(FCL_REAL time)
{
	initDiscreteCollision(time);

	CollisionNode node;
	initDiscreteCollisionNode(node);

	::fcl::collide(&node);
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
void ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::initDiscreteCollision
	(FCL_REAL time)
{
	integrateTimeToMotions(time);
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
void ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::integrateTimeToMotions
	(FCL_REAL start_time, FCL_REAL end_time = -1.0)
{
	motion1_->integrate(start_time, end_time);
	motion2_->integrate(start_time, end_time);
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
void ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::initDiscreteCollisionNode(CollisionNode& node)
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
		*model2_, getCurrentTransform(motion2_),
		distance_request_,
		distance_result_
		);

	//node.motion1 = motion1_;
	//node.motion2 = motion2_;
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

	//distanceRecurse(node);
	calculateVelocityBound();
	node.delta_t = getTimeStep(node);	

	processRecurseResult(node);	
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
void ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::initDistanceRecurse(ConservativeAdvancementNode& node)
{
	Matrix3f R1_t, R2_t;
	Vec3f T1_t, T2_t;

	motion1_->getCurrentTransform(R1_t, T1_t);
	motion2_->getCurrentTransform(R2_t, T2_t);

	relativeTransform(R1_t, T1_t, R2_t, T2_t, node.R, node.T);

	node.delta_t = request_->end_time;
	//node.min_distance = std::numeric_limits<FCL_REAL>::max();
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
void ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::distanceRecurse(
	ConservativeAdvancementNode& node) const
{
	//distanceRecurse(&node, 0, 0, NULL);

	//BVHFrontList front_list;
	int queue_size = 1000;
	//distanceQueueRecurse(&node, 0, 0, &front_list, queue_size);
	distanceQueueRecurse(&node, 0, 0, NULL, queue_size);
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
bool ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::isColliding(ConservativeAdvancementNode& node) const
{
	return (node.delta_t <= (FCL_REAL)0.00001); //node.t_err);
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
void ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::addContact(ConservativeAdvancementNode& node)
{
	//result_->addContact(Contact(model1_, model2_, node.last_tri_id1, node.last_tri_id2) );
	result_->addContact(Contact(model1_, model2_, 0, 0) );
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
bool ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::collideBoolean(
	const ContinuousCollisionRequest& request, ContinuousCollisionResult& result)
{
	setRequest(request);
	setResult(result);

	if (!areInvariantsSatisfied() )
	{
		return false;
	}

	initCollisionDetection();

	return performBooleanContinuousCollision();		
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
bool ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::performBooleanContinuousCollision()
{
	performDiscreteCollision(request_->start_time);	
	performDiscreteCollision(request_->end_time);	

	if (!isCollisionFree() )
	{
		return true;
	}	

	ConservativeAdvancementNode node;
	initContinuousCollision(node);

	// Prepare time interval pairs
	std::vector<std::pair<FCL_REAL, FCL_REAL> > time_pairs;
	time_pairs.push_back(std::make_pair(request_->start_time, request_->end_time) );

	while (!time_pairs.empty() )
	{
		std::pair<FCL_REAL, FCL_REAL> one_time_pair = time_pairs.back();
		time_pairs.pop_back();

		const FCL_REAL& left_time = one_time_pair.first;
		const FCL_REAL& right_time = one_time_pair.second;		

		integrateTimeToMotions(left_time, right_time);
		calculateVelocityBound();

		FCL_REAL left_time_step = getLeftTimeStep(node, left_time, right_time);
		FCL_REAL right_time_step = getRightTimeStep(node, left_time, right_time);

		if ( (left_time_step + right_time_step) < (right_time - left_time) ) 		
		{
			FCL_REAL new_left_time = left_time + left_time_step;
			FCL_REAL new_right_time = right_time - right_time_step;

			FCL_REAL middle_time = (new_left_time + new_right_time) / 2.0;

			performDiscreteCollision(middle_time);	

			if (!isCollisionFree() )
			{
				return true;
			}	

			time_pairs.push_back(std::make_pair(new_left_time, middle_time) );
			time_pairs.push_back(std::make_pair(middle_time, new_right_time) );
		}
	}

	return false;
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
void ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::calculateVelocityBound()
{
	// FAKE computation
	// velocity_bound_ = 0.000001;
	// return;

	FCL_REAL model1_max_radius = getMaxRadius(model1_, motion1_);
	FCL_REAL model2_max_radius = getMaxRadius(model2_, motion2_);	

	velocity_bound_ = 0.0;

	Vec3f nullPoint, distancePoint_1, distancePoint_2;

	distancePoint_1[0] = model1_max_radius;
	distancePoint_2[0] = model2_max_radius;

	TriangleMotionBoundVisitor mb_visitor1(distancePoint_1, nullPoint, nullPoint, Vec3f(0.0, 0.0, 0.0) );
	TriangleMotionBoundVisitor mb_visitor2(distancePoint_2, nullPoint, nullPoint, Vec3f(0.0, 0.0, 0.0) );

	velocity_bound_ += motion1_->computeMotionBound(mb_visitor1);
	velocity_bound_ += motion2_->computeMotionBound(mb_visitor2);

	//velocity_bound_ += motion1_->getMotionBound(Vec3f(0.0, 0.0, 0.0), model1_max_radius);
	//velocity_bound_ += motion2_->getMotionBound(Vec3f(0.0, 0.0, 0.0), model2_max_radius);
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
FCL_REAL ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::getMaxRadius(
	const BVHModel<BV>* model, const MotionBase* motion)
{
	// Expects RSS as BV template variable

	FCL_REAL proj_max = 0;
	FCL_REAL tmp = 0;

	if (model1_->getNumBVs() <= 0)
	{
		return 0.0;
	}

	BV bv = model->getBV(0).bv;

	proj_max = (bv.Tr).sqrLength();

	tmp = (bv.Tr + bv.axis[0] * bv.l[0]).sqrLength();
	if(tmp > proj_max) proj_max = tmp;
	tmp = (bv.Tr + bv.axis[1] * bv.l[1]).sqrLength();
	if(tmp > proj_max) proj_max = tmp;
	tmp = (bv.Tr + bv.axis[0] * bv.l[0] + bv.axis[1] * bv.l[1]).sqrLength();
	if(tmp > proj_max) proj_max = tmp;

	proj_max = std::sqrt(proj_max);

	return proj_max;
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
FCL_REAL ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::getVelocityBound() const
{
	return velocity_bound_;
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
FCL_REAL ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::getLeftTimeStep(
	ConservativeAdvancementNode& node, const FCL_REAL& start_time, const FCL_REAL& end_time)
{
	//integrateTimeToMotions(start_time, end_time);

	return getTimeStep(node);
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
FCL_REAL ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::getTimeStep(
	ConservativeAdvancementNode& node) const
{
	FCL_REAL min_distance = getMinDistance(node);
	FCL_REAL velocity_bound = getVelocityBound();

	return min_distance / velocity_bound;
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
FCL_REAL ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::getMinDistance(
	ConservativeAdvancementNode& node) const
{
	Matrix3f R1_t, R2_t;
	Vec3f T1_t, T2_t;

	motion1_->getCurrentTransform(R1_t, T1_t);
	motion2_->getCurrentTransform(R2_t, T2_t);

	relativeTransform(R1_t, T1_t, R2_t, T2_t, node.R, node.T);

	node.delta_t = 1.0;
	//node.min_distance = std::numeric_limits<FCL_REAL>::max();

	distanceRecurse(node);

	return node.result->min_distance;
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
FCL_REAL ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::getRightTimeStep(
	ConservativeAdvancementNode& node, const FCL_REAL& start_time, const FCL_REAL& end_time)
{
	integrateTimeToMotions(end_time, start_time);

	return getTimeStep(node);
}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
FCL_REAL ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::moveForward()
{

}

template<typename BV, typename ConservativeAdvancementNode, typename CollisionNode>
void ConservativeAdvancement<BV, ConservativeAdvancementNode, CollisionNode>::booleanCoolisionDetectionStep()
{

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

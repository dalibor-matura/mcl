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


#ifndef FCL_COLLISION_DATA_H
#define FCL_COLLISION_DATA_H

#include "fcl/collision_object.h"
#include "fcl/math/vec_3f.h"
#include <vector>
#include <set>
#include <limits>


namespace fcl
{

/// @brief Contact information returned by collision
struct Contact
{
  /// @brief collision object 1
  const CollisionGeometry* o1;

  /// @brief collision object 2
  const CollisionGeometry* o2;

  /// @brief contact primitive in object 1
  /// if object 1 is mesh or point cloud, it is the triangle or point id
  /// if object 1 is geometry shape, it is NONE (-1),
  /// if object 1 is octree, it is the id of the cell
  int b1;


  /// @brief contact primitive in object 2
  /// if object 2 is mesh or point cloud, it is the triangle or point id
  /// if object 2 is geometry shape, it is NONE (-1),
  /// if object 2 is octree, it is the id of the cell
  int b2;
 
  /// @brief contact normal, pointing from o1 to o2
  Vec3f normal;

  /// @brief contact position, in world space
  Vec3f pos;

  /// @brief penetration depth
  FCL_REAL penetration_depth;

 
  /// @brief invalid contact primitive information
  static const int NONE = -1;

  Contact() : o1(NULL),
			  o2(NULL),
			  b1(NONE),
			  b2(NONE)
  {}

  Contact(const CollisionGeometry* o1_, const CollisionGeometry* o2_, int b1_, int b2_) : o1(o1_),
																						  o2(o2_),
																						  b1(b1_),
																						  b2(b2_)
  {}

  Contact(const CollisionGeometry* o1_, const CollisionGeometry* o2_, int b1_, int b2_,
		  const Vec3f& pos_, const Vec3f& normal_, FCL_REAL depth_) : o1(o1_),
																	  o2(o2_),
																	  b1(b1_),
																	  b2(b2_),
																	  normal(normal_),
																	  pos(pos_),
																	  penetration_depth(depth_)
  {}

  bool operator < (const Contact& other) const
  {
	if(b1 == other.b1)
	  return b2 < other.b2;
	return b1 < other.b1;
  }
};

/// @brief Cost source describes an area with a cost. The area is described by an AABB region.
struct CostSource
{
  /// @brief aabb lower bound
  Vec3f aabb_min;

  /// @brief aabb upper bound
  Vec3f aabb_max;

  /// @brief cost density in the AABB region
  FCL_REAL cost_density;

  FCL_REAL total_cost;

  CostSource(const Vec3f& aabb_min_, const Vec3f& aabb_max_, FCL_REAL cost_density_) : aabb_min(aabb_min_),
																					   aabb_max(aabb_max_),
																					   cost_density(cost_density_)
  {
	total_cost = cost_density * (aabb_max[0] - aabb_min[0]) * (aabb_max[1] - aabb_min[1]) * (aabb_max[2] - aabb_min[2]);
  }

  CostSource(const AABB& aabb, FCL_REAL cost_density_) : aabb_min(aabb.min_),
														 aabb_max(aabb.max_),
														 cost_density(cost_density_)
  {
	total_cost = cost_density * (aabb_max[0] - aabb_min[0]) * (aabb_max[1] - aabb_min[1]) * (aabb_max[2] - aabb_min[2]);
  }

  CostSource() {}

  bool operator < (const CostSource& other) const
  {
	if(total_cost < other.total_cost) 
	  return false;
	if(total_cost > other.total_cost)
	  return true;
	
	if(cost_density < other.cost_density)
	  return false;
	if(cost_density > other.cost_density)
	  return true;
  
	for(size_t i = 0; i < 3; ++i)
	  if(aabb_min[i] != other.aabb_min[i])
	return aabb_min[i] < other.aabb_min[i];
 
	return false;
  }
};

struct CollisionResult;

/// @brief request to the collision algorithm
struct CollisionRequest
{
  /// @brief The maximum number of contacts_ will return
  size_t num_max_contacts;

  /// @brief whether the contact information (normal, penetration depth and contact position) will return
  bool enable_contact;

  /// @brief The maximum number of cost sources will return
  size_t num_max_cost_sources;

  /// @brief whether the cost sources will be computed
  bool enable_cost;

  /// @brief whether the cost computation is approximated
  bool use_approximate_cost;

  CollisionRequest(size_t num_max_contacts_ = 1,
				   bool enable_contact_ = false,
				   size_t num_max_cost_sources_ = 1,
				   bool enable_cost_ = false,
				   bool use_approximate_cost_ = true)
	:
	num_max_contacts(num_max_contacts_),
	enable_contact(enable_contact_),
	num_max_cost_sources(num_max_cost_sources_),
	enable_cost(enable_cost_),
	use_approximate_cost(use_approximate_cost_)
  {
  }

  bool isSatisfied(const CollisionResult& result) const;

  void assign(const CollisionRequest& request);
};

/// @brief request to the continuous collision algorithm
struct ContinuousCollisionRequest
	: CollisionRequest
{
	/// @brief what part (interval) of the movement should be tested
	FCL_REAL start_time;
	FCL_REAL end_time;

	ContinuousCollisionRequest()
		:
		start_time(0.0),
		end_time(1.0)
	{
	}
};

/// @brief collision result
struct CollisionResult
{
public:
	/// @brief add one contact into result structure
	inline void addContact(const Contact& c) 
	{
		contacts_.push_back(c);
	}

	/// @brief add one cost source into result structure
	inline void addCostSource(const CostSource& c, std::size_t num_max_cost_sources)
	{
		cost_sources_.insert(c);
		while (cost_sources_.size() > num_max_cost_sources)
			cost_sources_.erase(--cost_sources_.end());
	}

	/// @brief return binary collision result
	bool isCollision() const;

	/// @brief number of contacts_ found
	size_t numContacts() const;

	/// @brief number of cost sources found
	size_t numCostSources() const;

	/// @brief get the i-th contact calculated
	const Contact& getContact(size_t i) const;

	/// @brief get all the contacts_
	void getContacts(std::vector<Contact>& contacts);

	/// @brief get all the cost sources 
	void getCostSources(std::vector<CostSource>& cost_sources);

	/// @brief clear the results obtained
	void clear();

private:
	/// @brief contact information
	std::vector<Contact> contacts_;

	std::set<CostSource> cost_sources_;
};

/// @brief continuous collision result
struct ContinuousCollisionResult 
	: public CollisionResult
{
public:
	ContinuousCollisionResult()
		:
		time_of_contact_(0.0)
	  {
	  }

	  inline void setTimeOfContact(FCL_REAL time_of_contact)
	  {
		  time_of_contact_ = time_of_contact;
	  }

	  inline FCL_REAL getTimeOfContact() const
	  {
		  return time_of_contact_;
	  }	  

	  /// @brief clear the results obtained
	  void clear();

private:
	FCL_REAL time_of_contact_;
};

struct DistanceResult;

/// @brief request to the distance computation
struct DistanceRequest
{
  /// @brief whether to return the nearest points
  bool enable_nearest_points;

  DistanceRequest(bool enable_nearest_points_ = false) : enable_nearest_points(enable_nearest_points_)
  {
  }

  bool isSatisfied(const DistanceResult& result) const;
};

/// @brief distance result
struct DistanceResult
{

public:

  /// @brief minimum distance between two objects. if two objects are in collision, min_distance <= 0.
  FCL_REAL min_distance;

  /// @brief nearest points
  Vec3f nearest_points[2];

  /// @brief collision object 1
  const CollisionGeometry* o1;

  /// @brief collision object 2
  const CollisionGeometry* o2;

  /// @brief information about the nearest point in object 1
  /// if object 1 is mesh or point cloud, it is the triangle or point id
  /// if object 1 is geometry shape, it is NONE (-1),
  /// if object 1 is octree, it is the id of the cell
  int b1;

  /// @brief information about the nearest point in object 2
  /// if object 2 is mesh or point cloud, it is the triangle or point id
  /// if object 2 is geometry shape, it is NONE (-1),
  /// if object 2 is octree, it is the id of the cell
  int b2;

  /// @brief invalid contact primitive information
  static const int NONE = -1;
  
  DistanceResult(FCL_REAL min_distance_ = std::numeric_limits<FCL_REAL>::max()) : min_distance(min_distance_), 
																				  o1(NULL),
																				  o2(NULL),
																				  b1(NONE),
																				  b2(NONE)
  {
  }


  /// @brief add distance information into the result
  void update(FCL_REAL distance, const CollisionGeometry* o1_, const CollisionGeometry* o2_, int b1_, int b2_)
  {
	if(min_distance > distance)
	{
	  min_distance = distance;
	  o1 = o1_;
	  o2 = o2_;
	  b1 = b1_;
	  b2 = b2_;
	}
  }

  /// @brief add distance information into the result
  void update(FCL_REAL distance, const CollisionGeometry* o1_, const CollisionGeometry* o2_, int b1_, int b2_, const Vec3f& p1, const Vec3f& p2)
  {
	if(min_distance > distance)
	{
	  min_distance = distance;
	  o1 = o1_;
	  o2 = o2_;
	  b1 = b1_;
	  b2 = b2_;
	  nearest_points[0] = p1;
	  nearest_points[1] = p2;
	}
  }

  /// @brief add distance information into the result
  void update(const DistanceResult& other_result)
  {
	if(min_distance > other_result.min_distance)
	{
	  min_distance = other_result.min_distance;
	  o1 = other_result.o1;
	  o2 = other_result.o2;
	  b1 = other_result.b1;
	  b2 = other_result.b2;
	  nearest_points[0] = other_result.nearest_points[0];
	  nearest_points[1] = other_result.nearest_points[1];
	}
  }

  /// @brief clear the result
  void clear()
  {
	min_distance = std::numeric_limits<FCL_REAL>::max();
	o1 = NULL;
	o2 = NULL;
	b1 = NONE;
	b2 = NONE;
  }
};




}

#endif
